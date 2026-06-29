#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;
use egui_plot::{Plot, Line, HLine, PlotPoints};
use std::sync::{Arc, Mutex};
use engen_core::cfd::solver::{
    Solver, SolverConfig, LimiterType, BoundaryType, P_ATM, RHO_ATM
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Pressure,
    Velocity,
    Density,
}

struct SharedState {
    pressure: Vec<f32>,
    velocity: Vec<f32>,
    density: Vec<f32>,
    time: f32,
    
    config: SolverConfig,
    speed_multiplier: f32,
    inject_pulse: bool,
    pulse_amplitude: f32,
    pulse_width: f32,
    
    reset_trigger: bool,
    
    steps_per_second: f32,
    real_time_factor: f32,
}

fn spawn_solver_thread(shared_state: Arc<Mutex<SharedState>>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut solver = {
            let state = shared_state.lock().unwrap();
            Solver::new(state.config.clone())
        };
        
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
            let mut new_config = None;
            let mut force_ui_update = false;
            
            {
                if let Ok(mut state) = shared_state.lock() {
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
                    solver.config.limiter = state.config.limiter;
                    solver.config.left_bc = state.config.left_bc;
                    solver.config.right_bc = state.config.right_bc;
                    solver.config.friction = state.config.friction;
                    
                    // Check if geometry changed (requires recreation)
                    if state.config.length != solver.config.length || state.config.dx != solver.config.dx {
                        new_config = Some(state.config.clone());
                    }
                } else {
                    break; // Poisoned mutex, exit thread
                }
            }
            
            if reset {
                let state = shared_state.lock().unwrap();
                solver = Solver::new(state.config.clone());
                force_ui_update = true;
            } else if let Some(cfg) = new_config {
                solver = Solver::new(cfg);
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
                
                // Cap accumulator to prevent spiral of death
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
                        state.pressure = solver.get_pressure();
                        state.velocity = solver.get_velocity();
                        state.density = solver.get_density();
                        state.time = solver.t;
                    }
                    last_ui_update = now_ui;
                }
            } else {
                step_accumulator = 0.0;
                if force_ui_update {
                    if let Ok(mut state) = shared_state.lock() {
                        state.pressure = solver.get_pressure();
                        state.velocity = solver.get_velocity();
                        state.density = solver.get_density();
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
    length_input: f32,
    dx_input: f32,
    left_bc_input: BoundaryType,
    right_bc_input: BoundaryType,
    limiter_input: LimiterType,
    fixed_scale: bool,
}

impl EngenApp {
    fn new(shared_state: Arc<Mutex<SharedState>>, _ctx: egui::Context) -> Self {
        let (limiter, left_bc, right_bc, friction, length, dx) = {
            let state = shared_state.lock().unwrap();
            (
                state.config.limiter,
                state.config.left_bc,
                state.config.right_bc,
                state.config.friction,
                state.config.length,
                state.config.dx,
            )
        };
        
        Self {
            shared_state,
            active_tab: Tab::Pressure,
            amplitude_input: 20000.0,
            width_input: 0.08,
            speed_input: 1.0,
            friction_input: friction,
            length_input: length,
            dx_input: dx,
            left_bc_input: left_bc,
            right_bc_input: right_bc,
            limiter_input: limiter,
            fixed_scale: true,
        }
    }
}

impl eframe::App for EngenApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint(); // Keep UI updated in real-time
        
        let mut state = self.shared_state.lock().unwrap();
        
        // Sync UI inputs to shared state configuration
        state.config.limiter = self.limiter_input;
        state.config.left_bc = self.left_bc_input;
        state.config.right_bc = self.right_bc_input;
        state.config.friction = self.friction_input;
        state.speed_multiplier = self.speed_input;
        
        let pressure = state.pressure.clone();
        let velocity = state.velocity.clone();
        let density = state.density.clone();
        let current_time = state.time;
        let dx = state.config.dx;
        let rtf = state.real_time_factor;
        let sps = state.steps_per_second;
        
        egui::SidePanel::left("control_panel").width_range(280.0..=350.0).show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("EnGen CFD visualizer");
                ui.label(egui::RichText::new("Milestone 1: Pressure propagation").weak());
            });
            
            ui.add_space(10.0);
            ui.separator();
            ui.add_space(5.0);
            
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
                        state.reset_trigger = true;
                    }
                });
                ui.add(egui::Slider::new(&mut self.speed_input, 0.0..=2.0).text("Speed Multiplier").suffix("x"));
            });
            
            ui.add_space(10.0);
            
            // Pulse Injection
            ui.group(|ui| {
                ui.label(egui::RichText::new("Pulse Injection").strong());
                ui.add(egui::Slider::new(&mut self.amplitude_input, 1000.0..=50000.0).text("Amplitude").suffix(" Pa"));
                ui.add(egui::Slider::new(&mut self.width_input, 0.02..=0.30).text("Width").suffix(" m"));
                if ui.button("💥 Inject Pressure Pulse").clicked() {
                    state.pulse_amplitude = self.amplitude_input;
                    state.pulse_width = self.width_input;
                    state.inject_pulse = true;
                }
            });
            
            ui.add_space(10.0);
            
            // Boundary Conditions
            ui.group(|ui| {
                ui.label(egui::RichText::new("Boundary Conditions").strong());
                ui.horizontal(|ui| {
                    ui.label("Left end:");
                    ui.radio_value(&mut self.left_bc_input, BoundaryType::Closed, "Closed");
                    ui.radio_value(&mut self.left_bc_input, BoundaryType::Open, "Open");
                });
                ui.horizontal(|ui| {
                    ui.label("Right end:");
                    ui.radio_value(&mut self.right_bc_input, BoundaryType::Closed, "Closed");
                    ui.radio_value(&mut self.right_bc_input, BoundaryType::Open, "Open");
                });
            });
            
            ui.add_space(10.0);
            
            // Solver Configuration
            ui.group(|ui| {
                ui.label(egui::RichText::new("Solver Settings").strong());
                
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
            
            ui.add_space(10.0);
            
            // Tube Geometry Settings
            ui.group(|ui| {
                ui.label(egui::RichText::new("Tube Geometry").strong());
                ui.add(egui::Slider::new(&mut self.length_input, 0.5..=5.0).text("Length").suffix(" m"));
                ui.add(egui::Slider::new(&mut self.dx_input, 0.01..=0.05).text("Cell Size dx").suffix(" m"));
                
                let geometry_changed = self.length_input != state.config.length || self.dx_input != state.config.dx;
                if ui.add_enabled(geometry_changed, egui::Button::new("Apply Geometry (Resets Solver)")).clicked() {
                    state.config.length = self.length_input;
                    state.config.dx = self.dx_input;
                    state.reset_trigger = true;
                }
            });
            
            ui.add_space(20.0);
            ui.label(egui::RichText::new("Antigravity - EnGen Project").weak().size(9.0));
        });
        
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.active_tab, Tab::Pressure, "Pressure (Pa)");
                ui.selectable_value(&mut self.active_tab, Tab::Velocity, "Velocity (m/s)");
                ui.selectable_value(&mut self.active_tab, Tab::Density, "Density (kg/m³)");
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.checkbox(&mut self.fixed_scale, "Fixed Y-Scale");
                });
            });
            
            ui.add_space(5.0);
            
            // Tube Visualization Bar (Heatmap representation of row of cells)
            ui.group(|ui| {
                ui.label(egui::RichText::new("Physical Tube State (Heatmap)").strong());
                let num_cells = pressure.len();
                if num_cells > 0 {
                    let (rect, _response) = ui.allocate_at_least(
                        egui::vec2(ui.available_width(), 26.0),
                        egui::Sense::hover(),
                    );
                    let painter = ui.painter_at(rect);
                    
                    // Draw outer border (tube walls)
                    painter.rect_stroke(rect, 2.0, egui::Stroke::new(1.5, egui::Color32::from_rgb(100, 100, 100)));
                    
                    let cell_width = rect.width() / num_cells as f32;
                    for i in 0..num_cells {
                        let p = pressure[i];
                        let p_rel = (p - P_ATM) / 20000.0; // Scale relative to pulse amplitude
                        
                        // Sleek color mapping:
                        // High pressure: red-orange
                        // Atmospheric: neutral dark grey
                        // Low pressure: blue-cyan
                        let color = if p_rel > 0.0 {
                            let t = p_rel.min(1.0);
                            let r = (45.0 + 210.0 * t) as u8;
                            let g = (45.0 + 70.0 * t) as u8;
                            let b = (45.0 * (1.0 - t)) as u8;
                            egui::Color32::from_rgb(r, g, b)
                        } else {
                            let t = (-p_rel).min(1.0);
                            let r = (45.0 * (1.0 - t)) as u8;
                            let g = (45.0 + 90.0 * t) as u8;
                            let b = (45.0 + 210.0 * t) as u8;
                            egui::Color32::from_rgb(r, g, b)
                        };
                        
                        let cell_rect = egui::Rect::from_min_max(
                            egui::pos2(rect.min.x + i as f32 * cell_width + 1.0, rect.min.y + 1.0),
                            egui::pos2(rect.min.x + (i + 1) as f32 * cell_width - 1.0, rect.max.y - 1.0),
                        );
                        painter.rect_filled(cell_rect, 1.0, color);
                    }
                }
            });
            ui.add_space(10.0);
            
            match self.active_tab {
                Tab::Pressure => {
                    let points: PlotPoints = pressure
                        .iter()
                        .enumerate()
                        .map(|(i, &p)| [((i as f32 + 0.5) * dx) as f64, p as f64])
                        .collect();
                    
                    let line = Line::new(points)
                        .color(egui::Color32::from_rgb(0, 180, 255))
                        .width(2.5);
                    
                    let mut plot = Plot::new("pressure_plot")
                        .x_axis_label("Tube Position (m)")
                        .y_axis_label("Pressure (Pa)")
                        .show_grid(true);
                    
                    if self.fixed_scale {
                        plot = plot.include_y(P_ATM - 25000.0)
                                   .include_y(P_ATM + 25000.0);
                    }
                    
                    plot.show(ui, |plot_ui| {
                        plot_ui.hline(HLine::new(P_ATM as f64).color(egui::Color32::GRAY).width(1.0).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                        plot_ui.line(line);
                    });
                }
                Tab::Velocity => {
                    let points: PlotPoints = velocity
                        .iter()
                        .enumerate()
                        .map(|(i, &v)| [((i as f32 + 0.5) * dx) as f64, v as f64])
                        .collect();
                    
                    let line = Line::new(points)
                        .color(egui::Color32::from_rgb(255, 150, 0))
                        .width(2.5);
                    
                    let mut plot = Plot::new("velocity_plot")
                        .x_axis_label("Tube Position (m)")
                        .y_axis_label("Velocity (m/s)")
                        .show_grid(true);
                    
                    if self.fixed_scale {
                        plot = plot.include_y(-100.0)
                                   .include_y(100.0);
                    }
                    
                    plot.show(ui, |plot_ui| {
                        plot_ui.hline(HLine::new(0.0).color(egui::Color32::GRAY).width(1.0).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                        plot_ui.line(line);
                    });
                }
                Tab::Density => {
                    let points: PlotPoints = density
                        .iter()
                        .enumerate()
                        .map(|(i, &d)| [((i as f32 + 0.5) * dx) as f64, d as f64])
                        .collect();
                    
                    let line = Line::new(points)
                        .color(egui::Color32::from_rgb(50, 205, 50))
                        .width(2.5);
                    
                    let mut plot = Plot::new("density_plot")
                        .x_axis_label("Tube Position (m)")
                        .y_axis_label("Density (kg/m³)")
                        .show_grid(true);
                    
                    if self.fixed_scale {
                        plot = plot.include_y(RHO_ATM - 0.3)
                                   .include_y(RHO_ATM + 0.3);
                    }
                    
                    plot.show(ui, |plot_ui| {
                        plot_ui.hline(HLine::new(RHO_ATM as f64).color(egui::Color32::GRAY).width(1.0).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                        plot_ui.line(line);
                    });
                }
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("EnGen - 1D Compressible CFD Solver (Milestone 1)")
            .with_inner_size([1150.0, 720.0]),
        ..Default::default()
    };
    
    let shared_state = Arc::new(Mutex::new(SharedState {
        pressure: vec![P_ATM; 50],
        velocity: vec![0.0; 50],
        density: vec![RHO_ATM; 50],
        time: 0.0,
        config: SolverConfig::default(),
        speed_multiplier: 1.0,
        inject_pulse: false,
        pulse_amplitude: 20000.0,
        pulse_width: 0.08,
        reset_trigger: false,
        steps_per_second: 0.0,
        real_time_factor: 0.0,
    }));
    
    let _solver_handle = spawn_solver_thread(Arc::clone(&shared_state));
    
    eframe::run_native(
        "engen_ui",
        options,
        Box::new(|cc| {
            // Dark theme style adjustment
            let mut style = (*cc.egui_ctx.style()).clone();
            style.visuals.dark_mode = true;
            cc.egui_ctx.set_style(style);
            
            Ok(Box::new(EngenApp::new(shared_state, cc.egui_ctx.clone())))
        }),
    )
}
