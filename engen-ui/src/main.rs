#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;
use egui_plot::{Plot, Line, HLine, PlotPoints};
use std::sync::{Arc, Mutex};
use engen_core::cfd::solver::{
    Solver, Tube, Junction, RadiusProfile, LimiterType, BoundaryType, TubeSide,
    P_ATM, RHO_ATM, bezier_point, bezier_length
};

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
    speed_multiplier: f32,
    
    inject_pulse: bool,
    pulse_amplitude: f32,
    pulse_width: f32,
    
    preset_type: Option<PresetType>,
    reset_trigger: bool,
    
    steps_per_second: f32,
    real_time_factor: f32,
}

fn spawn_solver_thread(shared_state: Arc<Mutex<SharedState>>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut solver = Solver::new_y_junction(); // default Y-junction preset

        // Write initial solver state back to UI
        {
            let mut state = shared_state.lock().unwrap();
            state.tubes = solver.tubes.clone();
            state.junctions = solver.junctions.clone();
            state.time = solver.t;
        }
        
        let mut last_real_time = std::time::Instant::now();
        let mut step_accumulator = 0.0;
        let mut steps_this_second = 0;
        let mut steps_timer = std::time::Instant::now();
        let mut last_ui_update = std::time::Instant::now();
        
        let sim_dt = 1.0 / 64000.0; // 64kHz
        
        loop {
            let mut inject = false;
            let mut amp = 20000.0;
            let mut width = 0.08;
            let mut reset = false;
            let speed;
            let mut preset_to_load = None;
            let mut force_ui_update = false;
            
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
                    
                    // Sync runtime editable fields
                    solver.limiter = state.limiter;
                    solver.friction = state.friction;
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
            
            let now = std::time::Instant::now();
            let elapsed_sec = now.duration_since(last_real_time).as_secs_f32();
            last_real_time = now;
            
            if speed > 0.0 {
                let target_sim_adv = elapsed_sec * speed;
                step_accumulator += target_sim_adv;
                
                if step_accumulator > 0.1 {
                    step_accumulator = 0.1;
                }
                
                let mut steps_run = 0;
                while step_accumulator >= sim_dt {
                    solver.step(sim_dt);
                    step_accumulator -= sim_dt;
                    steps_run += 1;
                    steps_this_second += 1;
                }
                
                let now_ui = std::time::Instant::now();
                if force_ui_update || (steps_run > 0 && now_ui.duration_since(last_ui_update) >= std::time::Duration::from_millis(16)) {
                    if let Ok(mut state) = shared_state.lock() {
                        state.tubes = solver.tubes.clone();
                        state.junctions = solver.junctions.clone();
                        state.time = solver.t;
                    }
                    last_ui_update = now_ui;
                }
            } else {
                step_accumulator = 0.0;
                if force_ui_update {
                    if let Ok(mut state) = shared_state.lock() {
                        state.tubes = solver.tubes.clone();
                        state.junctions = solver.junctions.clone();
                        state.time = solver.t;
                    }
                    last_ui_update = std::time::Instant::now();
                }
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
    active_tab: Tab,
    amplitude_input: f32,
    width_input: f32,
    speed_input: f32,
    friction_input: f32,
    limiter_input: LimiterType,
    fixed_scale: bool,
    
    preset_selection: PresetType,
    selected_tube_id: Option<usize>,
    
    dragging_handle: Option<DragHandle>,
    flow_particles: Vec<Particle>,
}

impl EngenApp {
    fn new(shared_state: Arc<Mutex<SharedState>>) -> Self {
        let (limiter, friction) = {
            let state = shared_state.lock().unwrap();
            (state.limiter, state.friction)
        };
        
        let mut app = Self {
            shared_state,
            active_tab: Tab::Pressure,
            amplitude_input: 20000.0,
            width_input: 0.08,
            speed_input: 1.0,
            friction_input: friction,
            limiter_input: limiter,
            fixed_scale: true,
            preset_selection: PresetType::YJunction,
            selected_tube_id: Some(0),
            dragging_handle: None,
            flow_particles: Vec::new(),
        };
        app.reseed_particles();
        app
    }

    fn reseed_particles(&mut self) {
        self.flow_particles.clear();
        let state = self.shared_state.lock().unwrap();
        for tube in &state.tubes {
            // Spawn 15 particles evenly spaced per tube
            for i in 0..15 {
                self.flow_particles.push(Particle {
                    tube_id: tube.id,
                    t: i as f32 / 15.0,
                });
            }
        }
    }
}

impl eframe::App for EngenApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint(); // Redraw constantly for smooth physics and flow animation
        
        let (tubes, junctions, current_time, rtf, sps) = {
            let mut state = self.shared_state.lock().unwrap();
            
            // Sync control inputs to shared state config
            state.limiter = self.limiter_input;
            state.friction = self.friction_input;
            state.speed_multiplier = self.speed_input;
            
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
                                if ui.radio_value(&mut temp_tube.left_bc, BoundaryType::Closed, "Closed").clicked()
                                   || ui.radio_value(&mut temp_tube.left_bc, BoundaryType::Open, "Open").clicked() {
                                    geom_changed = true;
                                }
                            });
                        } else {
                            ui.label(egui::RichText::new("Left end connects to Junction").weak().size(11.0));
                        }

                        if !connected_right {
                            ui.horizontal(|ui| {
                                ui.label("Right BC:");
                                if ui.radio_value(&mut temp_tube.right_bc, BoundaryType::Closed, "Closed").clicked()
                                   || ui.radio_value(&mut temp_tube.right_bc, BoundaryType::Open, "Open").clicked() {
                                    geom_changed = true;
                                }
                            });
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

        // ------------------ Central Canvas / Viewport ------------------
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.label(egui::RichText::new("2D Network Layout & Flow Viewport (Interactive)").strong());
                ui.label(egui::RichText::new("Drag yellow circles to move junctions. Drag hollow circles to bend tubes. Click tube to select.").weak().size(11.0));
                
                // Allocate painter for 2D graphics
                let (response, painter) = ui.allocate_painter(
                    egui::vec2(ui.available_width(), ui.available_height() - 250.0), // leaves space for bottom graph
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

                // 3. Update & Draw Flow Particles
                let dt = ctx.input(|i| i.stable_dt).min(0.05); // Limit delta time spike
                if self.speed_input > 0.0 {
                    for particle in &mut self.flow_particles {
                        if let Some(tube) = tubes.iter().find(|t| t.id == particle.tube_id) {
                            let m = tube.num_cells;
                            let cell_idx = (particle.t * m as f32).floor() as usize;
                            let cell_idx = cell_idx.clamp(1, m);
                            
                            // Get local velocity in cell
                            let u_cell = tube.u[cell_idx];
                            let velocity = u_cell[1] / u_cell[0].max(1e-5);
                            let length = bezier_length(tube.p0, tube.p1, tube.p2, tube.p3).max(0.1);

                            // Update particle progress: dt * velocity / length
                            let delta_t = (velocity * dt * self.speed_input) / length;
                            particle.t = (particle.t + delta_t).rem_euclid(1.0);
                        }
                    }
                }

                for particle in &self.flow_particles {
                    if let Some(tube) = tubes.iter().find(|t| t.id == particle.tube_id) {
                        let p = bezier_point(particle.t, tube.p0, tube.p1, tube.p2, tube.p3);
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

            // ------------------ Bottom Plots ------------------
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
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("EnGen - 1D Compressible Network CFD Solver (Milestone 2)")
            .with_inner_size([1200.0, 750.0]),
        ..Default::default()
    };
    
    let shared_state = Arc::new(Mutex::new(SharedState {
        tubes: Vec::new(),
        junctions: Vec::new(),
        time: 0.0,
        limiter: LimiterType::MC,
        friction: 0.0,
        speed_multiplier: 1.0,
        inject_pulse: false,
        pulse_amplitude: 20000.0,
        pulse_width: 0.08,
        preset_type: None,
        reset_trigger: false,
        steps_per_second: 0.0,
        real_time_factor: 0.0,
    }));
    
    let _solver_handle = spawn_solver_thread(Arc::clone(&shared_state));
    
    eframe::run_native(
        "engen_ui",
        options,
        Box::new(|cc| {
            let mut style = (*cc.egui_ctx.style()).clone();
            style.visuals.dark_mode = true;
            cc.egui_ctx.set_style(style);
            
            Ok(Box::new(EngenApp::new(shared_state)))
        }),
    )
}
