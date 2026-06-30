#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;
use egui_plot::{Plot, Line, HLine, PlotPoints};
use std::sync::{Arc, Mutex};
use engen_core::cfd::solver::{
    Solver, Tube, Junction, RadiusProfile, LimiterType, BoundaryType, TubeSide,
    P_ATM, RHO_ATM, bezier_point, bezier_length
};
use engen_audio::AudioFilterParams;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Pressure,
    Velocity,
    Density,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PresetType {
    Straight,
    Taper,
    ExpansionChamber,
    YJunction,
}

impl std::fmt::Display for PresetType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PresetType::Straight => write!(f, "Straight Tube"),
            PresetType::Taper => write!(f, "Tapered Tube"),
            PresetType::ExpansionChamber => write!(f, "Expansion Chamber"),
            PresetType::YJunction => write!(f, "Y-Junction Exhaust"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DragHandle {
    Junction { id: usize },
    ControlPoint1 { tube_id: usize },
    ControlPoint2 { tube_id: usize },
    FreeStart { tube_id: usize },
    FreeEnd { tube_id: usize },
}

struct Particle {
    tube_id: usize,
    t: f32, // Progress along the tube (0.0 to 1.0)
}

struct SharedState {
    tubes: Vec<Tube>,
    junctions: Vec<Junction>,
    time: f32,
    
    limiter: LimiterType,
    friction: f32,
    heat_transfer: f32,
    speed_multiplier: f32,
    
    inject_pulse: bool,
    pulse_amplitude: f32,
    pulse_width: f32,
    
    preset_type: Option<PresetType>,
    reset_trigger: bool,
    
    steps_per_second: f32,
    real_time_factor: f32,
    
    audio_volume: f32,
    reflection_filter: f32,
    jones_history: Vec<f32>,
    
    // Audio filter cutoffs (Hz) - synced to AudioFilterParams
    lp_cutoff_hz: f32,
    hp_cutoff_hz: f32,
}

fn spawn_solver_thread(
    shared_state: Arc<Mutex<SharedState>>,
    audio_buffer: Arc<Mutex<Vec<f32>>>,
    _filter_params: Arc<Mutex<AudioFilterParams>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut solver = Solver::new_y_junction(); // default Y-junction preset

        // Write initial solver state back to UI
        {
            let mut state = shared_state.lock().unwrap();
            state.tubes = solver.tubes.clone();
            state.junctions = solver.junctions.clone();
            state.time = solver.t;
        }
        
        let mut steps_this_second = 0;
        let mut steps_timer = std::time::Instant::now();
        let mut last_ui_update = std::time::Instant::now();
        
        let sim_dt = 1.0 / 64000.0; // 64kHz
        
        let mut local_jones_history = Vec::new();
        let mut local_audio_buf: Vec<f32> = Vec::with_capacity(128);
        
        loop {
            let mut inject = false;
            let mut amp = 20000.0;
            let mut width = 0.08;
            let mut reset = false;
            let speed;
            let mut preset_to_load = None;
            let mut force_ui_update = false;
            let audio_volume;
            let reflection_filter;
            
            {
                if let Ok(mut state) = shared_state.lock() {
                    if state.preset_type.is_some() {
                        preset_to_load = state.preset_type.take();
                    }
                    if state.reset_trigger {
                        state.reset_trigger = false;
                        reset = true;
                    }
                    if state.inject_pulse {
                        state.inject_pulse = false;
                        inject = true;
                        amp = state.pulse_amplitude;
                        width = state.pulse_width;
                    }
                    speed = state.speed_multiplier;
                    
                    audio_volume = state.audio_volume;
                    reflection_filter = state.reflection_filter;
                    
                    // Sync runtime editable fields
                    solver.limiter = state.limiter;
                    solver.friction = state.friction;
                    solver.heat_transfer = state.heat_transfer;
                    solver.reflection_filter = reflection_filter;
                    
                    // Sync valve lifts dynamically without resetting
                    for (k, tube) in solver.tubes.iter_mut().enumerate() {
                        if k < state.tubes.len() {
                            if let BoundaryType::Valve { lift } = state.tubes[k].left_bc {
                                if let BoundaryType::Valve { lift: ref mut l_lift } = tube.left_bc {
                                    *l_lift = lift;
                                }
                            }
                            if let BoundaryType::Valve { lift } = state.tubes[k].right_bc {
                                if let BoundaryType::Valve { lift: ref mut r_lift } = tube.right_bc {
                                    *r_lift = lift;
                                }
                            }
                        }
                    }
                } else {
                    break; // exit thread
                }
            }
            
            if let Some(preset) = preset_to_load {
                solver = match preset {
                    PresetType::Straight => Solver::new_single_tube(RadiusProfile::Linear, BoundaryType::Closed, BoundaryType::Closed),
                    PresetType::Taper => {
                        let mut s = Solver::new_single_tube(RadiusProfile::Linear, BoundaryType::Closed, BoundaryType::Closed);
                        s.tubes[0].r_start = 0.015;
                        s.tubes[0].r_end = 0.04;
                        s.tubes[0].rebuild_geometry();
                        s.tubes[0].reset_state();
                        s
                    }
                    PresetType::ExpansionChamber => Solver::new_single_tube(RadiusProfile::ExpansionChamber, BoundaryType::Closed, BoundaryType::Closed),
                    PresetType::YJunction => Solver::new_y_junction(),
                };
                if let Ok(mut state) = shared_state.lock() {
                    state.tubes = solver.tubes.clone();
                    state.junctions = solver.junctions.clone();
                    state.time = solver.t;
                }
                force_ui_update = true;
            }
            
            if reset {
                // Sync geometry edited by the UI back to the solver
                if let Ok(state) = shared_state.lock() {
                    for (k, tube) in solver.tubes.iter_mut().enumerate() {
                        if k < state.tubes.len() {
                            let ui_tube = &state.tubes[k];
                            tube.p0 = ui_tube.p0;
                            tube.p1 = ui_tube.p1;
                            tube.p2 = ui_tube.p2;
                            tube.p3 = ui_tube.p3;
                            tube.r_start = ui_tube.r_start;
                            tube.r_end = ui_tube.r_end;
                            tube.r_mid = ui_tube.r_mid;
                            tube.radius_profile = ui_tube.radius_profile;
                            tube.num_cells = ui_tube.num_cells;
                            tube.left_bc = ui_tube.left_bc;
                            tube.right_bc = ui_tube.right_bc;
                            tube.rebuild_geometry();
                            tube.reset_state();
                        }
                    }
                    
                    for (k, j) in solver.junctions.iter_mut().enumerate() {
                        if k < state.junctions.len() {
                            j.pos = state.junctions[k].pos;
                            
                            // Re-calculate Y-junction volume dynamically
                            let mut vol = 0.0;
                            for conn in &j.connections {
                                let tube = &solver.tubes[conn.tube_id];
                                match conn.side {
                                    TubeSide::Left => vol += tube.area[1] * tube.dx,
                                    TubeSide::Right => vol += tube.area[tube.num_cells] * tube.dx,
                                }
                            }
                            j.volume = vol * 0.5;
                        }
                    }
                    solver.t = 0.0;
                }
                force_ui_update = true;
            }
            
            if inject {
                solver.inject_pulse(amp, width);
                force_ui_update = true;
            }
            
            let current_len = {
                if let Ok(buf) = audio_buffer.lock() {
                    buf.len()
                } else {
                    0
                }
            };
            
            let target_len = 3000;
            if current_len < target_len && speed > 0.0 {
                let missing_samples = target_len - current_len;
                let steps_to_run = ((missing_samples as f32) / speed) as usize;
                let steps_to_run = steps_to_run.min(500).max(1);
                
                local_audio_buf.clear();
                let mut steps_run = 0;
                
                for _ in 0..steps_to_run {
                    let jones_val = solver.step(sim_dt);
                    
                    let scaled_jones = jones_val * 5.0e-2;
                    let scaled = scaled_jones * audio_volume;
                    let audio_sample = scaled / (1.0 + scaled.abs());
                    local_audio_buf.push(audio_sample);
                    
                    local_jones_history.push(scaled_jones);
                    if local_jones_history.len() > 4096 {
                        let excess = local_jones_history.len() - 4096;
                        local_jones_history.drain(0..excess);
                    }
                    steps_run += 1;
                    steps_this_second += 1;
                }
                
                if !local_audio_buf.is_empty() {
                    if let Ok(mut audio_buf) = audio_buffer.lock() {
                        audio_buf.extend_from_slice(&local_audio_buf);
                    }
                }
                
                let now_ui = std::time::Instant::now();
                if force_ui_update || (steps_run > 0 && now_ui.duration_since(last_ui_update) >= std::time::Duration::from_millis(16)) {
                    if let Ok(mut state) = shared_state.lock() {
                        state.tubes = solver.tubes.clone();
                        state.junctions = solver.junctions.clone();
                        state.time = solver.t;
                        state.jones_history = local_jones_history.clone();
                    }
                    last_ui_update = now_ui;
                }
            } else {
                if force_ui_update {
                    if let Ok(mut state) = shared_state.lock() {
                        state.tubes = solver.tubes.clone();
                        state.junctions = solver.junctions.clone();
                        state.time = solver.t;
                        state.jones_history = local_jones_history.clone();
                    }
                    last_ui_update = std::time::Instant::now();
                }
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
            
            if steps_timer.elapsed().as_secs_f32() >= 1.0 {
                if let Ok(mut state) = shared_state.lock() {
                    state.steps_per_second = steps_this_second as f32;
                    state.real_time_factor = (steps_this_second as f32 * sim_dt) / 1.0;
                }
                steps_this_second = 0;
                steps_timer = std::time::Instant::now();
            }
            
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    })
}

struct EngenApp {
    shared_state: Arc<Mutex<SharedState>>,
    filter_params: Arc<Mutex<AudioFilterParams>>,
    active_tab: Tab,
    amplitude_input: f32,
    width_input: f32,
    speed_input: f32,
    friction_input: f32,
    heat_transfer_input: f32,
    limiter_input: LimiterType,
    fixed_scale: bool,
    
    preset_selection: PresetType,
    selected_tube_id: Option<usize>,
    
    dragging_handle: Option<DragHandle>,
    flow_particles: Vec<Particle>,
    
    volume_input: f32,
    reflection_input: f32,
    
    lp_cutoff_input: f32,
    hp_cutoff_input: f32,
}

impl EngenApp {
    fn new(shared_state: Arc<Mutex<SharedState>>, filter_params: Arc<Mutex<AudioFilterParams>>) -> Self {
        let (limiter, friction, heat_transfer, vol, filter, lp_cutoff, hp_cutoff) = {
            let state = shared_state.lock().unwrap();
            (state.limiter, state.friction, state.heat_transfer, state.audio_volume, state.reflection_filter,
             state.lp_cutoff_hz, state.hp_cutoff_hz)
        };
        
        let mut app = Self {
            shared_state,
            filter_params,
            active_tab: Tab::Pressure,
            amplitude_input: 20000.0,
            width_input: 0.08,
            speed_input: 1.0,
            friction_input: friction,
            heat_transfer_input: heat_transfer,
            limiter_input: limiter,
            fixed_scale: true,
            preset_selection: PresetType::YJunction,
            selected_tube_id: Some(0),
            dragging_handle: None,
            flow_particles: Vec::new(),
            volume_input: vol,
            reflection_input: filter,
            lp_cutoff_input: lp_cutoff,
            hp_cutoff_input: hp_cutoff,
        };
        app.reseed_particles();
        app
    }

    fn reseed_particles(&mut self) {
        self.flow_particles.clear();
        let state = self.shared_state.lock().unwrap();
        for tube in &state.tubes {
            // Spawn 15 particles evenly spaced per tube
            for _ in 0..15 {
                self.flow_particles.push(Particle {
                    tube_id: tube.id,
                    t: 0.0,
                });
            }
        }
    }
}

impl eframe::App for EngenApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint(); // Redraw constantly for smooth physics and flow animation
        
        // Fullscreen hotkey check (F11)
        if ctx.input(|i| i.key_pressed(egui::Key::F11)) {
            let is_fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!is_fullscreen));
        }

        let (tubes, junctions, current_time, rtf, sps) = {
            let mut state = self.shared_state.lock().unwrap();
            
            // Sync control inputs to shared state config
            state.limiter = self.limiter_input;
            state.friction = self.friction_input;
            state.heat_transfer = self.heat_transfer_input;
            state.speed_multiplier = self.speed_input;
            state.audio_volume = self.volume_input;
            state.reflection_filter = self.reflection_input;
            
            (
                state.tubes.clone(),
                state.junctions.clone(),
                state.time,
                state.real_time_factor,
                state.steps_per_second,
            )
        };
        
        // ------------------ Side Panel Controls ------------------
        egui::SidePanel::left("control_panel").width_range(280.0..=350.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.vertical_centered(|ui| {
                    ui.heading("EnGen CFD Visualizer");
                    ui.label(egui::RichText::new("Milestone 2: Y-Junctions & Geometry").weak());
                });
                
                ui.add_space(10.0);
                ui.separator();
                ui.add_space(5.0);
                
                // Scenario Presets
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Preset Scenarios").strong());
                    let old_preset = self.preset_selection;
                    egui::ComboBox::from_id_source("preset_combo")
                        .selected_text(format!("{}", self.preset_selection))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.preset_selection, PresetType::Straight, "Straight Tube");
                            ui.selectable_value(&mut self.preset_selection, PresetType::Taper, "Tapered Tube");
                            ui.selectable_value(&mut self.preset_selection, PresetType::ExpansionChamber, "Expansion Chamber");
                            ui.selectable_value(&mut self.preset_selection, PresetType::YJunction, "Y-Junction Exhaust");
                        });
                    
                    if self.preset_selection != old_preset {
                        let mut state = self.shared_state.lock().unwrap();
                        state.preset_type = Some(self.preset_selection);
                        self.selected_tube_id = Some(0);
                        // Defer reseed particles until next frame once solver recreates tubes
                        ctx.request_repaint();
                    }
                });
                
                ui.add_space(10.0);

                // Simulation Stats
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Performance Metrics").strong());
                    ui.horizontal(|ui| {
                        ui.label("Time Step Rate:");
                        ui.label(egui::RichText::new(format!("{:.0} Hz", sps)).color(egui::Color32::LIGHT_GREEN));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Real-Time Factor:");
                        let color = if rtf >= 0.95 { egui::Color32::LIGHT_GREEN } else if rtf > 0.1 { egui::Color32::LIGHT_YELLOW } else { egui::Color32::LIGHT_RED };
                        ui.label(egui::RichText::new(format!("{:.2}x", rtf)).color(color));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Simulated Time:");
                        ui.label(format!("{:.5} s", current_time));
                    });
                });
                
                ui.add_space(10.0);
                
                // Simulation Controls
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Simulation Controls").strong());
                    ui.horizontal(|ui| {
                        if ui.button("⏸ Pause").clicked() {
                            self.speed_input = 0.0;
                        }
                        if ui.button("▶ Play").clicked() && self.speed_input == 0.0 {
                            self.speed_input = 1.0;
                        }
                        if ui.button("🔄 Reset").clicked() {
                            let mut state = self.shared_state.lock().unwrap();
                            state.reset_trigger = true;
                        }
                    });
                    ui.add(egui::Slider::new(&mut self.speed_input, 0.0..=2.0).text("Speed Multiplier").suffix("x"));
                    
                    ui.horizontal(|ui| {
                        let is_fullscreen = ctx.input(|i| i.viewport().fullscreen.unwrap_or(false));
                        let btn_txt = if is_fullscreen { "🗖 Windowed" } else { "🖵 Fullscreen" };
                        if ui.button(btn_txt).clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(!is_fullscreen));
                        }
                        ui.label(egui::RichText::new("(or F11)").weak().size(11.0));
                    });
                });
                
                ui.add_space(10.0);
                
                // Pulse Injection
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Pulse Injection (Main Tube)").strong());
                    ui.add(egui::Slider::new(&mut self.amplitude_input, 1000.0..=50000.0).text("Amplitude").suffix(" Pa"));
                    ui.add(egui::Slider::new(&mut self.width_input, 0.02..=0.30).text("Width").suffix(" m"));
                    if ui.button("💥 Inject Pressure Pulse").clicked() {
                        let mut state = self.shared_state.lock().unwrap();
                        state.pulse_amplitude = self.amplitude_input;
                        state.pulse_width = self.width_input;
                        state.inject_pulse = true;
                    }
                });
                
                ui.add_space(10.0);
                
                // Solver Configuration
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Global Physics Settings").strong());
                    
                    ui.horizontal(|ui| {
                        ui.label("Limiter:");
                        egui::ComboBox::from_id_source("limiter_box")
                            .selected_text(format!("{}", self.limiter_input))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.limiter_input, LimiterType::None, "None (1st Order)");
                                ui.selectable_value(&mut self.limiter_input, LimiterType::Minmod, "Minmod");
                                ui.selectable_value(&mut self.limiter_input, LimiterType::MC, "Monotonized Central (MC)");
                                ui.selectable_value(&mut self.limiter_input, LimiterType::VanLeer, "Van Leer");
                                ui.selectable_value(&mut self.limiter_input, LimiterType::Superbee, "Superbee");
                            });
                    });
                    
                    ui.add(egui::Slider::new(&mut self.friction_input, 0.0..=50.0).text("Wall Friction"));
                    ui.add(egui::Slider::new(&mut self.heat_transfer_input, 0.0..=200.0).text("Wall Heat Transfer"));
                    ui.add(egui::Slider::new(&mut self.volume_input, 0.0..=1.0).text("Audio Volume"));
                    ui.add(egui::Slider::new(&mut self.reflection_input, 0.5..=1.0).text("Reflection Filter"));
                });
                
                ui.add_space(10.0);
                
                // Audio Filter Controls
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Audio Filters").strong());
                    ui.label(egui::RichText::new("Adjust to trim crackling / rumble").weak().size(11.0));
                    if ui.add(egui::Slider::new(&mut self.lp_cutoff_input, 500.0..=20000.0)
                        .text("LP Cutoff")
                        .suffix(" Hz")
                        .logarithmic(true)
                    ).changed() {
                        if let Ok(mut fp) = self.filter_params.lock() {
                            fp.lp_cutoff_hz = self.lp_cutoff_input;
                        }
                    }
                    if ui.add(egui::Slider::new(&mut self.hp_cutoff_input, 1.0..=5000.0)
                        .text("HP Cutoff")
                        .suffix(" Hz")
                        .logarithmic(true)
                    ).changed() {
                        if let Ok(mut fp) = self.filter_params.lock() {
                            fp.hp_cutoff_hz = self.hp_cutoff_input;
                        }
                    }
                });

                // Geometry editor panel for selected tube
                if let Some(tube_id) = self.selected_tube_id {
                    if tube_id < tubes.len() {
                        ui.add_space(10.0);
                        ui.group(|ui| {
                            ui.label(egui::RichText::new(format!("Tube Inspector: Tube {}", tube_id)).strong());
                            
                            let mut temp_tube = tubes[tube_id].clone();
                            let mut geom_changed = false;
                            let mut name_changed = false;
                            
                            ui.horizontal(|ui| {
                                ui.label("Name:");
                                if ui.text_edit_singleline(&mut temp_tube.name).changed() {
                                    name_changed = true;
                                }
                            });
                            
                            let mut cells = temp_tube.num_cells;
                            if ui.add(egui::Slider::new(&mut cells, 5..=100).text("CFD Cells")).changed() {
                                temp_tube.num_cells = cells;
                                geom_changed = true;
                            }

                            let mut r_start = temp_tube.r_start;
                            if ui.add(egui::Slider::new(&mut r_start, 0.005..=0.08).text("Start Radius").suffix(" m")).changed() {
                                temp_tube.r_start = r_start;
                                geom_changed = true;
                            }

                            let mut r_end = temp_tube.r_end;
                            if ui.add(egui::Slider::new(&mut r_end, 0.005..=0.08).text("End Radius").suffix(" m")).changed() {
                                temp_tube.r_end = r_end;
                                geom_changed = true;
                            }

                            // Boundary Conditions edits (only if not connected to a junction)
                            let connected_left = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Left));
                            let connected_right = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Right));

                            if !connected_left {
                                ui.horizontal(|ui| {
                                    ui.label("Left BC:");
                                    let mut left_type = match temp_tube.left_bc {
                                        BoundaryType::Closed => 0,
                                        BoundaryType::Open => 1,
                                        BoundaryType::Valve { .. } => 2,
                                    };
                                    let old_type = left_type;
                                    egui::ComboBox::from_id_source("left_bc_combo")
                                        .selected_text(match left_type {
                                            0 => "Closed",
                                            1 => "Open",
                                            _ => "Valve",
                                        })
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut left_type, 0, "Closed");
                                            ui.selectable_value(&mut left_type, 1, "Open");
                                            ui.selectable_value(&mut left_type, 2, "Valve");
                                        });
                                    if left_type != old_type {
                                        temp_tube.left_bc = match left_type {
                                            0 => BoundaryType::Closed,
                                            1 => BoundaryType::Open,
                                            _ => BoundaryType::Valve { lift: 1.0 },
                                        };
                                        geom_changed = true;
                                    }
                                });
                                if let BoundaryType::Valve { mut lift } = temp_tube.left_bc {
                                    if ui.add(egui::Slider::new(&mut lift, 0.0..=1.0).text("Left Valve Lift")).changed() {
                                        temp_tube.left_bc = BoundaryType::Valve { lift };
                                        let mut state = self.shared_state.lock().unwrap();
                                        state.tubes[tube_id].left_bc = BoundaryType::Valve { lift };
                                    }
                                }
                            } else {
                                ui.label(egui::RichText::new("Left end connects to Junction").weak().size(11.0));
                            }

                            if !connected_right {
                                ui.horizontal(|ui| {
                                    ui.label("Right BC:");
                                    let mut right_type = match temp_tube.right_bc {
                                        BoundaryType::Closed => 0,
                                        BoundaryType::Open => 1,
                                        BoundaryType::Valve { .. } => 2,
                                    };
                                    let old_type = right_type;
                                    egui::ComboBox::from_id_source("right_bc_combo")
                                        .selected_text(match right_type {
                                            0 => "Closed",
                                            1 => "Open",
                                            _ => "Valve",
                                        })
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut right_type, 0, "Closed");
                                            ui.selectable_value(&mut right_type, 1, "Open");
                                            ui.selectable_value(&mut right_type, 2, "Valve");
                                        });
                                    if right_type != old_type {
                                        temp_tube.right_bc = match right_type {
                                            0 => BoundaryType::Closed,
                                            1 => BoundaryType::Open,
                                            _ => BoundaryType::Valve { lift: 1.0 },
                                        };
                                        geom_changed = true;
                                    }
                                });
                                if let BoundaryType::Valve { mut lift } = temp_tube.right_bc {
                                    if ui.add(egui::Slider::new(&mut lift, 0.0..=1.0).text("Right Valve Lift")).changed() {
                                        temp_tube.right_bc = BoundaryType::Valve { lift };
                                        let mut state = self.shared_state.lock().unwrap();
                                        state.tubes[tube_id].right_bc = BoundaryType::Valve { lift };
                                    }
                                }
                            } else {
                                ui.label(egui::RichText::new("Right end connects to Junction").weak().size(11.0));
                            }
                            
                            if temp_tube.radius_profile == RadiusProfile::ExpansionChamber {
                                let mut r_mid = temp_tube.r_mid;
                                if ui.add(egui::Slider::new(&mut r_mid, 0.01..=0.12).text("Chamber Radius").suffix(" m")).changed() {
                                    temp_tube.r_mid = r_mid;
                                    geom_changed = true;
                                }
                            }

                            if geom_changed || name_changed {
                                let mut state = self.shared_state.lock().unwrap();
                                state.tubes[tube_id] = temp_tube;
                                if geom_changed {
                                    state.tubes[tube_id].rebuild_geometry();
                                    state.reset_trigger = true;
                                }
                            }
                        });
                    }
                }

                ui.add_space(20.0);
                ui.label(egui::RichText::new("Antigravity - EnGen Project").weak().size(9.0));
            });
        });

        // ------------------ Bottom Panel Plots ------------------
        egui::TopBottomPanel::bottom("bottom_panel")
            .resizable(true)
            .default_height(360.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.active_tab, Tab::Pressure, "Pressure graph (Pa)");
                        ui.selectable_value(&mut self.active_tab, Tab::Velocity, "Velocity graph (m/s)");
                        ui.selectable_value(&mut self.active_tab, Tab::Density, "Density graph (kg/m³)");
                        ui.checkbox(&mut self.fixed_scale, "Fixed Y-Scale");
                    });

                    ui.add_space(5.0);

                    // Fetch selected tube to graph
                    if let Some(tube_id) = self.selected_tube_id {
                        if let Some(tube) = tubes.iter().find(|t| t.id == tube_id) {
                            let m = tube.num_cells;
                            let dx = tube.dx;
                            let gamma = 1.4;

                            match self.active_tab {
                                Tab::Pressure => {
                                    let points: PlotPoints = tube.u[1..=m]
                                        .iter()
                                        .enumerate()
                                        .map(|(i, u_cell)| {
                                            let p = u_cell[2] * (gamma - 1.0) - 0.5 * (u_cell[1] * u_cell[1]) / u_cell[0].max(1e-5);
                                            [((i as f32 + 0.5) * dx) as f64, p as f64]
                                        })
                                        .collect();

                                    let line = Line::new(points).color(egui::Color32::from_rgb(0, 180, 255)).width(2.0);
                                    let mut plot = Plot::new("pressure_plot").height(150.0).x_axis_label("Position along tube (m)").show_grid(true);
                                    if self.fixed_scale {
                                        plot = plot.include_y(P_ATM - 25000.0).include_y(P_ATM + 25000.0);
                                    }
                                    plot.show(ui, |plot_ui| {
                                        plot_ui.hline(HLine::new(P_ATM as f64).color(egui::Color32::GRAY).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                                        plot_ui.line(line);
                                    });
                                }
                                Tab::Velocity => {
                                    let points: PlotPoints = tube.u[1..=m]
                                        .iter()
                                        .enumerate()
                                        .map(|(i, u_cell)| {
                                            let v = u_cell[1] / u_cell[0].max(1e-5);
                                            [((i as f32 + 0.5) * dx) as f64, v as f64]
                                        })
                                        .collect();

                                    let line = Line::new(points).color(egui::Color32::from_rgb(255, 150, 0)).width(2.0);
                                    let mut plot = Plot::new("velocity_plot").height(150.0).x_axis_label("Position along tube (m)").show_grid(true);
                                    if self.fixed_scale {
                                        plot = plot.include_y(-100.0).include_y(100.0);
                                    }
                                    plot.show(ui, |plot_ui| {
                                        plot_ui.hline(HLine::new(0.0).color(egui::Color32::GRAY).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                                        plot_ui.line(line);
                                    });
                                }
                                Tab::Density => {
                                    let points: PlotPoints = tube.u[1..=m]
                                        .iter()
                                        .enumerate()
                                        .map(|(i, u_cell)| {
                                            [((i as f32 + 0.5) * dx) as f64, u_cell[0] as f64]
                                        })
                                        .collect();

                                    let line = Line::new(points).color(egui::Color32::from_rgb(50, 205, 50)).width(2.0);
                                    let mut plot = Plot::new("density_plot").height(150.0).x_axis_label("Position along tube (m)").show_grid(true);
                                    if self.fixed_scale {
                                        plot = plot.include_y(RHO_ATM - 0.3).include_y(RHO_ATM + 0.3);
                                    }
                                    plot.show(ui, |plot_ui| {
                                        plot_ui.hline(HLine::new(RHO_ATM as f64).color(egui::Color32::GRAY).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                                        plot_ui.line(line);
                                    });
                                }
                            }
                        }
                    } else {
                        ui.centered_and_justified(|ui| {
                            ui.label("Click on any tube in the layout viewport to plot its values.");
                        });
                    }

                    ui.add_space(5.0);
                    ui.separator();
                    ui.add_space(5.0);
                    ui.label(egui::RichText::new("Jones Audio Outlet Radiation Signal (dP/dt)").strong());

                    let jones_history = {
                        let state = self.shared_state.lock().unwrap();
                        state.jones_history.clone()
                    };

                    let points: PlotPoints = jones_history
                        .iter()
                        .enumerate()
                        .map(|(i, &val)| [i as f64, val as f64])
                        .collect();

                    let line = Line::new(points).color(egui::Color32::from_rgb(255, 105, 180)).width(1.5);
                    let plot = Plot::new("jones_plot")
                        .height(100.0)
                        .show_grid(true)
                        .include_y(-1000.0)
                        .include_y(1000.0);
                    plot.show(ui, |plot_ui| {
                        plot_ui.hline(HLine::new(0.0).color(egui::Color32::GRAY).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                        plot_ui.line(line);
                    });
                });
            });

        // ------------------ Central Canvas / Viewport ------------------
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("2D Network Layout & Flow Viewport (Interactive)").strong());
                ui.label(egui::RichText::new("Drag yellow circles to move junctions. Drag hollow circles to bend tubes. Click tube to select.").weak().size(11.0));
                
                // Allocate painter for 2D graphics
                let (response, painter) = ui.allocate_painter(
                    ui.available_size(),
                    egui::Sense::click_and_drag(),
                );

                let rect = response.rect;
                painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(18, 18, 18));
                painter.rect_stroke(rect, 4.0, egui::Stroke::new(1.0, egui::Color32::from_rgb(45, 45, 45)));

                // Map physics [-2.5, 2.5] x [-1.5, 1.5] space to pixels
                let scale = (rect.width() / 5.0).min(rect.height() / 3.0);
                let center = rect.center();

                let to_screen = |pos: [f32; 2]| -> egui::Pos2 {
                    egui::pos2(
                        center.x + pos[0] * scale,
                        center.y - pos[1] * scale,
                    )
                };

                let to_physics = |pos: egui::Pos2| -> [f32; 2] {
                    [
                        (pos.x - center.x) / scale,
                        -(pos.y - center.y) / scale,
                    ]
                };

                // Auto-reseed particles if the particle list size doesn't match the active tube structure
                if self.flow_particles.is_empty() || self.flow_particles.len() != tubes.len() * 15 {
                    self.reseed_particles();
                }

                // 1. Draw Network Tubes (Heatmap Quads)
                for tube in &tubes {
                    let m = tube.num_cells;
                    let is_selected = self.selected_tube_id == Some(tube.id);
                    
                    // Render heatmaps
                    for i in 1..=m {
                        // Cell geometry coordinates
                        let sa = to_screen(tube.cell_boundaries_left_2d[i - 1]);
                        let sb = to_screen(tube.cell_boundaries_right_2d[i - 1]);
                        let sc = to_screen(tube.cell_boundaries_right_2d[i]);
                        let sd = to_screen(tube.cell_boundaries_left_2d[i]);

                        // Map cell value to color based on active tab
                        let val = match self.active_tab {
                            Tab::Pressure => {
                                let p = tube.u[i][2] * (1.4 - 1.0) - 0.5 * (tube.u[i][1] * tube.u[i][1]) / tube.u[i][0].max(1e-5);
                                (p - P_ATM) / 20000.0
                            }
                            Tab::Velocity => {
                                let v = tube.u[i][1] / tube.u[i][0].max(1e-5);
                                v / 100.0
                            }
                            Tab::Density => {
                                (tube.u[i][0] - RHO_ATM) / 0.3
                            }
                        };

                        let color = if val > 0.0 {
                            let t = val.min(1.0);
                            let r = (35.0 + 220.0 * t) as u8;
                            let g = (35.0 + 60.0 * t) as u8;
                            let b = (35.0 * (1.0 - t)) as u8;
                            egui::Color32::from_rgb(r, g, b)
                        } else {
                            let t = (-val).min(1.0);
                            let r = (35.0 * (1.0 - t)) as u8;
                            let g = (35.0 + 80.0 * t) as u8;
                            let b = (35.0 + 220.0 * t) as u8;
                            egui::Color32::from_rgb(r, g, b)
                        };

                        painter.add(egui::Shape::convex_polygon(
                            vec![sa, sb, sc, sd],
                            color,
                            egui::Stroke::NONE,
                        ));
                    }

                    // Render tube boundary walls
                    let wall_stroke = if is_selected {
                        egui::Stroke::new(2.0, egui::Color32::WHITE)
                    } else {
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 120, 120))
                    };

                    for i in 1..=m {
                        let sa = to_screen(tube.cell_boundaries_left_2d[i - 1]);
                        let sb = to_screen(tube.cell_boundaries_right_2d[i - 1]);
                        let sc = to_screen(tube.cell_boundaries_right_2d[i]);
                        let sd = to_screen(tube.cell_boundaries_left_2d[i]);

                        painter.line_segment([sa, sd], wall_stroke);
                        painter.line_segment([sb, sc], wall_stroke);
                    }
                }

                // 2. Draw Y-Junction virtual bodies
                for j in &junctions {
                    let screen_pos = to_screen(j.pos);
                    // Draw a larger ring showing the boundary junction
                    painter.circle_stroke(screen_pos, 12.0, egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 215, 0)));
                }

                // 3. Update & Draw Flow Particles (Fixed-Slot Local Phase-Flow)
                let dt = ctx.input(|i| i.stable_dt).min(0.05); // Limit delta time spike
                if self.speed_input > 0.0 {
                    for (idx, particle) in self.flow_particles.iter_mut().enumerate() {
                        if let Some(tube) = tubes.iter().find(|t| t.id == particle.tube_id) {
                            let m = tube.num_cells;
                            let slot_idx = idx % 15;
                            let t_pos = (slot_idx as f32 + particle.t) / 15.0;
                            let cell_idx = (t_pos * m as f32).floor() as usize;
                            let cell_idx = cell_idx.clamp(1, m);
                            
                            // Get local velocity in cell
                            let u_cell = tube.u[cell_idx];
                            let velocity = u_cell[1] / u_cell[0].max(1e-5);
                            let length = bezier_length(tube.p0, tube.p1, tube.p2, tube.p3).max(0.1);

                            // Update particle phase offset (15 slots)
                            let delta_phase = 15.0 * (velocity * dt * self.speed_input) / length;
                            particle.t = (particle.t + delta_phase).rem_euclid(1.0);
                        }
                    }
                }

                for (idx, particle) in self.flow_particles.iter().enumerate() {
                    if let Some(tube) = tubes.iter().find(|t| t.id == particle.tube_id) {
                        let slot_idx = idx % 15;
                        let t_draw = (slot_idx as f32 + particle.t) / 15.0;
                        let p = bezier_point(t_draw, tube.p0, tube.p1, tube.p2, tube.p3);
                        let s_pos = to_screen(p);
                        painter.circle_filled(s_pos, 2.0, egui::Color32::from_rgb(255, 255, 224));
                    }
                }

                // 4. Handle drag controls and selection clicks
                let mouse_pos = ctx.input(|i| i.pointer.interact_pos());
                let mouse_pressed = ctx.input(|i| i.pointer.primary_pressed());
                let mouse_released = ctx.input(|i| i.pointer.any_released());

                if mouse_pressed {
                    if let Some(m_pos) = mouse_pos {
                        let mut clicked_handle = None;
                        
                        // Check junctions first
                        for j in &junctions {
                            let s_pos = to_screen(j.pos);
                            if s_pos.distance(m_pos) < 12.0 {
                                clicked_handle = Some(DragHandle::Junction { id: j.id });
                                break;
                            }
                        }

                        // Check selected tube control handles
                        if clicked_handle.is_none() {
                            if let Some(tube_id) = self.selected_tube_id {
                                if let Some(tube) = tubes.iter().find(|t| t.id == tube_id) {
                                    let s_p1 = to_screen(tube.p1);
                                    let s_p2 = to_screen(tube.p2);
                                    
                                    if s_p1.distance(m_pos) < 10.0 {
                                        clicked_handle = Some(DragHandle::ControlPoint1 { tube_id });
                                    } else if s_p2.distance(m_pos) < 10.0 {
                                        clicked_handle = Some(DragHandle::ControlPoint2 { tube_id });
                                    } else {
                                        // Check if free ends are clicked
                                        let connected_left = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Left));
                                        let connected_right = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Right));

                                        if !connected_left && to_screen(tube.p0).distance(m_pos) < 10.0 {
                                            clicked_handle = Some(DragHandle::FreeStart { tube_id });
                                        } else if !connected_right && to_screen(tube.p3).distance(m_pos) < 10.0 {
                                            clicked_handle = Some(DragHandle::FreeEnd { tube_id });
                                        }
                                    }
                                }
                            }
                        }

                        // If no handles clicked, check if user clicked on a tube cell to select it
                        if clicked_handle.is_none() {
                            let mut closest_tube = None;
                            let mut min_dist = f32::MAX;
                            for tube in &tubes {
                                for &pt in &tube.cell_centers_2d {
                                    let s_pt = to_screen(pt);
                                    let d = s_pt.distance(m_pos);
                                    if d < min_dist {
                                        min_dist = d;
                                        closest_tube = Some(tube.id);
                                    }
                                }
                            }
                            if min_dist < 20.0 {
                                self.selected_tube_id = closest_tube;
                            }
                        }

                        self.dragging_handle = clicked_handle;
                    }
                }

                if mouse_released {
                    self.dragging_handle = None;
                }

                // If currently dragging, update coordinates of the handle
                if let Some(dragged) = self.dragging_handle {
                    if let Some(m_pos) = mouse_pos {
                        let new_pos = to_physics(m_pos);
                        let mut state = self.shared_state.lock().unwrap();
                        
                        match dragged {
                            DragHandle::Junction { id } => {
                                state.junctions[id].pos = new_pos;
                                let j_connections = state.junctions[id].connections.clone();
                                
                                // Dragging the junction moves all connected tube endpoints
                                for conn in &j_connections {
                                    let tube_id = conn.tube_id;
                                    let side = conn.side;
                                    let ui_tube = &mut state.tubes[tube_id];
                                    match side {
                                        TubeSide::Left => ui_tube.p0 = new_pos,
                                        TubeSide::Right => ui_tube.p3 = new_pos,
                                    }
                                    ui_tube.rebuild_geometry();
                                }
                                state.reset_trigger = true;
                            }
                            DragHandle::ControlPoint1 { tube_id } => {
                                let ui_tube = &mut state.tubes[tube_id];
                                ui_tube.p1 = new_pos;
                                ui_tube.rebuild_geometry();
                                state.reset_trigger = true;
                            }
                            DragHandle::ControlPoint2 { tube_id } => {
                                let ui_tube = &mut state.tubes[tube_id];
                                ui_tube.p2 = new_pos;
                                ui_tube.rebuild_geometry();
                                state.reset_trigger = true;
                            }
                            DragHandle::FreeStart { tube_id } => {
                                let ui_tube = &mut state.tubes[tube_id];
                                ui_tube.p0 = new_pos;
                                ui_tube.rebuild_geometry();
                                state.reset_trigger = true;
                            }
                            DragHandle::FreeEnd { tube_id } => {
                                let ui_tube = &mut state.tubes[tube_id];
                                ui_tube.p3 = new_pos;
                                ui_tube.rebuild_geometry();
                                state.reset_trigger = true;
                            }
                        }
                    }
                }

                // 5. Draw Selected Tube control points (Bezier lines)
                if let Some(tube_id) = self.selected_tube_id {
                    if let Some(tube) = tubes.iter().find(|t| t.id == tube_id) {
                        let s_p0 = to_screen(tube.p0);
                        let s_p1 = to_screen(tube.p1);
                        let s_p2 = to_screen(tube.p2);
                        let s_p3 = to_screen(tube.p3);
                        
                        // Draw tangent lines
                        let tangent_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 100));
                        painter.line_segment([s_p0, s_p1], tangent_stroke);
                        painter.line_segment([s_p3, s_p2], tangent_stroke);

                        // Draw handles
                        painter.circle_filled(s_p1, 5.0, egui::Color32::from_rgb(0, 191, 255));
                        painter.circle_stroke(s_p1, 5.0, egui::Stroke::new(1.0, egui::Color32::WHITE));
                        painter.circle_filled(s_p2, 5.0, egui::Color32::from_rgb(0, 191, 255));
                        painter.circle_stroke(s_p2, 5.0, egui::Stroke::new(1.0, egui::Color32::WHITE));

                        // Free ends handles
                        let connected_left = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Left));
                        let connected_right = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube_id && c.side == TubeSide::Right));

                        if !connected_left {
                            painter.circle_filled(s_p0, 5.0, egui::Color32::LIGHT_GREEN);
                        }
                        if !connected_right {
                            painter.circle_filled(s_p3, 5.0, egui::Color32::LIGHT_GREEN);
                        }
                    }
                }

                // Draw Junction handles
                for j in &junctions {
                    let screen_pos = to_screen(j.pos);
                    painter.circle_filled(screen_pos, 6.0, egui::Color32::from_rgb(255, 215, 0));
                    painter.circle_stroke(screen_pos, 6.0, egui::Stroke::new(1.0, egui::Color32::WHITE));
                }
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("EnGen - 1D Compressible Network CFD Solver (Milestone 3)")
            .with_inner_size([1200.0, 850.0]),
        ..Default::default()
    };
    
    let audio_buffer = Arc::new(Mutex::new(Vec::new()));
    let filter_params = Arc::new(Mutex::new(AudioFilterParams::default()));
    let _audio_system = match engen_audio::AudioSystem::new(Arc::clone(&audio_buffer), Arc::clone(&filter_params)) {
        Ok(sys) => Some(sys),
        Err(e) => {
            eprintln!("Failed to initialize audio: {}", e);
            None
        }
    };
    
    let default_fp = AudioFilterParams::default();
    let shared_state = Arc::new(Mutex::new(SharedState {
        tubes: Vec::new(),
        junctions: Vec::new(),
        time: 0.0,
        limiter: LimiterType::VanLeer,
        friction: 15.0,
        heat_transfer: 2.0,
        speed_multiplier: 1.0,
        inject_pulse: false,
        pulse_amplitude: 20000.0,
        pulse_width: 0.08,
        preset_type: None,
        reset_trigger: false,
        steps_per_second: 0.0,
        real_time_factor: 0.0,
        audio_volume: 0.8,
        reflection_filter: 0.75,
        jones_history: Vec::new(),
        lp_cutoff_hz: default_fp.lp_cutoff_hz,
        hp_cutoff_hz: default_fp.hp_cutoff_hz,
    }));
    
    let _solver_handle = spawn_solver_thread(Arc::clone(&shared_state), Arc::clone(&audio_buffer), Arc::clone(&filter_params));
    
    eframe::run_native(
        "engen_ui",
        options,
        Box::new(|cc| {
            let mut style = (*cc.egui_ctx.style()).clone();
            style.visuals.dark_mode = true;
            cc.egui_ctx.set_style(style);
            
            Ok(Box::new(EngenApp::new(shared_state, filter_params)))
        }),
    )
}
