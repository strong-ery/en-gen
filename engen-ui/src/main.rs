// TEMP: disabled for debugging - re-enable once throttle/RPM issue is diagnosed
// #![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui;
use egui_plot::{Plot, Line, HLine, PlotPoints};
use std::sync::{Arc, Mutex};
use engen_core::cfd::solver::{
    Solver, Tube, Junction, RadiusProfile, LimiterType, BoundaryType, TubeSide, SolverProfile,
    P_ATM, RHO_ATM, bezier_point, bezier_length
};
use engen_core::mechanical::get_valve_lift;
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
    SingleCylinder,
    InlineFour,
}

impl std::fmt::Display for PresetType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PresetType::Straight => write!(f, "Straight Tube"),
            PresetType::Taper => write!(f, "Tapered Tube"),
            PresetType::ExpansionChamber => write!(f, "Expansion Chamber"),
            PresetType::YJunction => write!(f, "Y-Junction Exhaust"),
            PresetType::SingleCylinder => write!(f, "Single Cylinder Engine"),
            PresetType::InlineFour => write!(f, "Inline-4 Engine"),
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

    // Milestone 4 mechanical fields
    pub has_engine: bool,
    pub engine_rpm: f32,
    pub engine_crank_angle: f32,
    pub cylinder_pressure: f32,
    pub cylinder_volume: f32,
    
    // Cyl pressure histories for all cylinders
    pub cyl_pressure_histories: Vec<Vec<[f32; 2]>>, // Index = Cylinder ID, elements = [crank_angle_degrees, pressure_pa]
    
    // Engine parameters (editable)
    pub engine_bore: f32,
    pub engine_stroke: f32,
    pub engine_conrod: f32,
    pub engine_compression_ratio: f32,
    pub engine_inertia: f32,
    pub engine_friction: f32,
    
    // Control inputs
    pub spin_rpm: f32,
    pub trigger_spin: bool,
    pub throttle: f32,
    pub ignition_on: bool,
    pub ignition_timing_deg: f32,
    pub target_afr: f32,
    pub audio_on: bool,

    // Starter motor state
    pub starter_engaged: bool,
    pub starter_timer: f32,    // Remaining starter duration in seconds

    // ECU telemetry
    pub ecu_map_pa: f32,
    pub ecu_iac_lift: f32,
    pub ecu_is_cranking: bool,
    pub ecu_rev_limiter_active: bool,

    // Species & Misfires Telemetry
    pub cylinder_afr: f32,
    pub cylinder_lambda: f32,
    pub residual_fraction: f32,
    pub last_misfire: bool,
    pub misfire_count: u32,
    pub profile: SolverProfile,
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
        let mut debug_print_counter: u32 = 0;
        
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
            let state_audio_on;
            
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
                    state_audio_on = state.audio_on;
                    
                    // Sync runtime editable fields
                    solver.limiter = state.limiter;
                    solver.friction = state.friction;
                    solver.heat_transfer = state.heat_transfer;
                    solver.reflection_filter = reflection_filter;
                    solver.ignition_on = state.ignition_on;
                    if solver.crankshaft.is_none() {
                        solver.ignition_timing_deg = state.ignition_timing_deg;
                        solver.target_afr = state.target_afr;
                    }
                    
                    // Legacy Spin Up Trigger (kept for debug button)
                    if state.trigger_spin {
                        state.trigger_spin = false;
                        if let Some(ref mut crank) = solver.crankshaft {
                            crank.omega = ((state.spin_rpm as f64 * 2.0 * std::f64::consts::PI) / 60.0) as f32;
                            solver.ecu.is_cranking = false; // direct running mode
                        }
                    }
                    
                    // Starter motor: apply dynamic cranking torque controlled by solver/ecu
                    solver.starter_active = state.starter_engaged && state.starter_timer > 0.0;
                    
                    // Throttle mapping
                    solver.throttle_input = state.throttle;
                    
                    // Sync engine mechanical parameters
                    if let Some(ref mut crank) = solver.crankshaft {
                        crank.inertia = state.engine_inertia;
                        crank.friction_coeff = state.engine_friction;
                    }
                    if let Some(cyl) = solver.cylinders.first_mut() {
                        if (cyl.bore - state.engine_bore).abs() > 1e-4
                            || (cyl.stroke - state.engine_stroke).abs() > 1e-4
                            || (cyl.conrod_length - state.engine_conrod).abs() > 1e-4
                            || (cyl.compression_ratio - state.engine_compression_ratio).abs() > 1e-4
                        {
                            cyl.update_geometry(
                                state.engine_bore,
                                state.engine_stroke,
                                state.engine_conrod,
                                state.engine_compression_ratio,
                            );
                            state.cyl_pressure_histories.clear();
                        }
                    }

                    // Sync valve lifts dynamically without resetting
                    for (k, tube) in solver.tubes.iter_mut().enumerate() {
                        if k == 0 && solver.crankshaft.is_some() {
                            // Tube 0 is the throttle-controlled intake boundary when an engine
                            // is present; its lift is driven by `state.throttle` above, not by
                            // the UI tube mirror, so skip it here or we'd immediately overwrite
                            // the throttle value with the stale UI copy.
                            continue;
                        }
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
                    PresetType::SingleCylinder => Solver::new_single_cylinder(),
                    PresetType::InlineFour => Solver::new_inline_four(),
                };
                if let Ok(mut state) = shared_state.lock() {
                    state.tubes = solver.tubes.clone();
                    state.junctions = solver.junctions.clone();
                    state.time = solver.t;
                    
                    if let Some(ref cyl) = solver.cylinders.first() {
                        state.engine_bore = cyl.bore;
                        state.engine_stroke = cyl.stroke;
                        state.engine_conrod = cyl.conrod_length;
                        state.engine_compression_ratio = cyl.compression_ratio;
                    }
                    if let Some(ref crank) = solver.crankshaft {
                        state.engine_inertia = crank.inertia;
                        state.engine_friction = crank.friction_coeff;
                    }
                    state.ignition_on = solver.ignition_on;
                    state.ignition_timing_deg = solver.ignition_timing_deg;
                    state.target_afr = solver.target_afr;
                    
                    state.ecu_map_pa = solver.ecu.map_pa;
                    state.ecu_iac_lift = solver.ecu.iac_lift;
                    state.ecu_is_cranking = solver.ecu.is_cranking;
                    state.ecu_rev_limiter_active = solver.ecu.rev_limiter_active;
                    
                    state.cylinder_afr = solver.last_cylinder_afr;
                    state.cylinder_lambda = solver.last_cylinder_lambda;
                    state.residual_fraction = solver.last_residual_fraction;
                    state.last_misfire = solver.last_misfire;
                    state.misfire_count = solver.misfire_count;
                    state.profile = solver.profile;
                    
                    // Sync starter disengagement
                    if !solver.starter_active && state.starter_engaged {
                        state.starter_engaged = false;
                        state.starter_timer = 0.0;
                    }
                    
                    state.cyl_pressure_histories.clear();
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
                    solver.update_connected_flags();
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
                    let audio_sample = if state_audio_on && !scaled.is_nan() && !scaled.is_infinite() {
                        scaled / (1.0 + scaled.abs())
                    } else {
                        0.0
                    };
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
                        
                        // Decrement starter timer by simulated time elapsed
                        if state.starter_engaged && state.starter_timer > 0.0 {
                            state.starter_timer -= steps_run as f32 * sim_dt;
                        }

                        state.ecu_map_pa = solver.ecu.map_pa;
                        state.ecu_iac_lift = solver.ecu.iac_lift;
                        state.ecu_is_cranking = solver.ecu.is_cranking;
                        state.ecu_rev_limiter_active = solver.ecu.rev_limiter_active;
                        state.ignition_timing_deg = solver.ignition_timing_deg;
                        state.target_afr = solver.target_afr;
                        
                        state.cylinder_afr = solver.last_cylinder_afr;
                        state.cylinder_lambda = solver.last_cylinder_lambda;
                        state.residual_fraction = solver.last_residual_fraction;
                        state.last_misfire = solver.last_misfire;
                        state.misfire_count = solver.misfire_count;
                        state.profile = solver.profile;

                        // Sync starter disengagement
                        if !solver.starter_active && state.starter_engaged {
                            state.starter_engaged = false;
                            state.starter_timer = 0.0;
                        }
                        
                        if let Some(ref crank) = solver.crankshaft {
                            state.has_engine = true;
                            state.engine_rpm = (crank.omega * 60.0) / (2.0 * std::f32::consts::PI);
                            state.engine_crank_angle = crank.theta;
                            if let Some(cyl) = solver.cylinders.first() {
                                state.cylinder_pressure = cyl.p;
                                state.cylinder_volume = cyl.volume;
                            }
                            
                            if state.cyl_pressure_histories.len() < solver.cylinders.len() {
                                state.cyl_pressure_histories.resize(solver.cylinders.len(), Vec::new());
                            }
                            for (i, c_cyl) in solver.cylinders.iter().enumerate() {
                                let angle_deg = (crank.theta + c_cyl.crank_pin_offset_radians).to_degrees().rem_euclid(720.0);
                                state.cyl_pressure_histories[i].push([angle_deg, c_cyl.p]);
                                if state.cyl_pressure_histories[i].len() > 360 {
                                    state.cyl_pressure_histories[i].remove(0);
                                }
                            }

                            if let Some(cyl) = solver.cylinders.first() {
                                // TEMP DEBUG: print throttle/lift/rpm/cyl mass once per second
                                // to verify the throttle value is actually reaching tube 0 and
                                // actually changing trapped cylinder mass. Remove once diagnosed.
                                debug_print_counter += 1;
                                if debug_print_counter >= 60 {
                                    debug_print_counter = 0;
                                    let lift = match solver.tubes[0].left_bc {
                                        BoundaryType::Valve { lift } => lift,
                                        _ => -1.0,
                                    };
                                    eprintln!(
                                        "[DEBUG] throttle={:.3} tube0_left_lift={:.3} rpm={:.0} cyl_mass={:.6} cyl_p={:.0}",
                                        state.throttle, lift, state.engine_rpm, cyl.mass, cyl.p
                                    );
                                }
                            }
                        } else {
                            state.has_engine = false;
                            state.cyl_pressure_histories.clear();
                        }
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
                        
                        if let Some(ref crank) = solver.crankshaft {
                            state.has_engine = true;
                            state.engine_rpm = (crank.omega * 60.0) / (2.0 * std::f32::consts::PI);
                            state.engine_crank_angle = crank.theta;
                            if let Some(cyl) = solver.cylinders.first() {
                                state.cylinder_pressure = cyl.p;
                                state.cylinder_volume = cyl.volume;
                            }
                        } else {
                            state.has_engine = false;
                        }
                    }
                    last_ui_update = std::time::Instant::now();
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            
            if steps_timer.elapsed().as_secs_f32() >= 1.0 {
                if let Ok(mut state) = shared_state.lock() {
                    state.steps_per_second = steps_this_second as f32;
                    state.real_time_factor = (steps_this_second as f32 * sim_dt) / 1.0;
                }
                steps_this_second = 0;
                steps_timer = std::time::Instant::now();
            }
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

    // Engine parameters inputs
    engine_bore_input: f32,
    engine_stroke_input: f32,
    engine_conrod_input: f32,
    engine_compression_ratio_input: f32,
    engine_inertia_input: f32,
    engine_friction_input: f32,
    spin_rpm_input: f32,
    throttle_input: f32,

    ignition_on_input: bool,
    ignition_timing_deg_input: f32,
    target_afr_input: f32,
    audio_on_input: bool,
    header_length_input: f32,
}

impl EngenApp {
    fn new(shared_state: Arc<Mutex<SharedState>>, filter_params: Arc<Mutex<AudioFilterParams>>) -> Self {
        let (limiter, friction, heat_transfer, vol, filter, lp_cutoff, hp_cutoff, bore, stroke, conrod, cr, inertia, eng_fric, spin_rpm, throttle, ign, timing, afr, audio) = {
            let state = shared_state.lock().unwrap();
            (state.limiter, state.friction, state.heat_transfer, state.audio_volume, state.reflection_filter,
             state.lp_cutoff_hz, state.hp_cutoff_hz,
             state.engine_bore, state.engine_stroke, state.engine_conrod, state.engine_compression_ratio,
             state.engine_inertia, state.engine_friction, state.spin_rpm, state.throttle,
             state.ignition_on, state.ignition_timing_deg, state.target_afr, state.audio_on)
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
            
            engine_bore_input: bore,
            engine_stroke_input: stroke,
            engine_conrod_input: conrod,
            engine_compression_ratio_input: cr,
            engine_inertia_input: inertia,
            engine_friction_input: eng_fric,
            spin_rpm_input: spin_rpm,
            throttle_input: throttle,

            ignition_on_input: ign,
            ignition_timing_deg_input: timing,
            target_afr_input: afr,
            audio_on_input: audio,
            header_length_input: 0.4,
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

        let (tubes, junctions, current_time, rtf, sps, has_engine, engine_rpm, crank_angle, cyl_p, cyl_v, eng_bore, eng_stroke, eng_conrod, solver_profile) = {
            let mut state = self.shared_state.lock().unwrap();
            
            // Sync control inputs to shared state config
            state.limiter = self.limiter_input;
            state.friction = self.friction_input;
            state.heat_transfer = self.heat_transfer_input;
            state.speed_multiplier = self.speed_input;
            state.audio_volume = self.volume_input;
            state.reflection_filter = self.reflection_input;
            
            // Sync engine sliders from EngenApp to SharedState
            state.throttle = self.throttle_input;
            state.engine_bore = self.engine_bore_input;
            state.engine_stroke = self.engine_stroke_input;
            state.engine_conrod = self.engine_conrod_input;
            state.engine_compression_ratio = self.engine_compression_ratio_input;
            state.engine_inertia = self.engine_inertia_input;
            state.engine_friction = self.engine_friction_input;
            state.spin_rpm = self.spin_rpm_input;
            state.ignition_on = self.ignition_on_input;
            state.audio_on = self.audio_on_input;

            if state.has_engine {
                self.ignition_timing_deg_input = state.ignition_timing_deg;
                self.target_afr_input = state.target_afr;
            } else {
                state.ignition_timing_deg = self.ignition_timing_deg_input;
                state.target_afr = self.target_afr_input;
            }
            
            (
                state.tubes.clone(),
                state.junctions.clone(),
                state.time,
                state.real_time_factor,
                state.steps_per_second,
                state.has_engine,
                state.engine_rpm,
                state.engine_crank_angle,
                state.cylinder_pressure,
                state.cylinder_volume,
                state.engine_bore,
                state.engine_stroke,
                state.engine_conrod,
                state.profile,
            )
        };
        
        // ------------------ Side Panel Controls ------------------
        egui::SidePanel::left("control_panel").width_range(280.0..=350.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.vertical_centered(|ui| {
                    ui.heading("EnGen CFD Visualizer");
                    ui.label(egui::RichText::new("Milestone 4: Piston & Crankshaft Mechanics").weak());
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
                            ui.selectable_value(&mut self.preset_selection, PresetType::SingleCylinder, "Single Cylinder Engine");
                            ui.selectable_value(&mut self.preset_selection, PresetType::InlineFour, "Inline-4 Engine");
                        });
                    
                    if self.preset_selection != old_preset {
                        let mut state = self.shared_state.lock().unwrap();
                        state.preset_type = Some(self.preset_selection);
                        self.selected_tube_id = Some(0);
                        // Defer reseed particles until next frame once solver recreates tubes
                        ctx.request_repaint();
                    }

                    if self.preset_selection == PresetType::InlineFour {
                        ui.add_space(5.0);
                        if ui.add(egui::Slider::new(&mut self.header_length_input, 0.15..=1.0).text("Header Length").suffix(" m")).changed() {
                            if let Ok(mut state) = self.shared_state.lock() {
                                let l = self.header_length_input;
                                let collector_x = -0.2 + l;
                                
                                // Re-scale header tubes 5, 6, 7, 8
                                let y_ports = [0.6, 0.2, -0.2, -0.6];
                                for idx in 0..4 {
                                    let tube_idx = 5 + idx;
                                    let y_port = y_ports[idx];
                                    if tube_idx < state.tubes.len() {
                                        state.tubes[tube_idx].p0 = [-0.2, y_port];
                                        state.tubes[tube_idx].p1 = [-0.2 + 0.3 * l, y_port];
                                        state.tubes[tube_idx].p2 = [collector_x - 0.3 * l, 0.0];
                                        state.tubes[tube_idx].p3 = [collector_x, 0.0];
                                    }
                                }
                                
                                // Re-scale tailpipe tube 9 starting point
                                if state.tubes.len() > 9 {
                                    state.tubes[9].p0 = [collector_x, 0.0];
                                    state.tubes[9].p1 = [collector_x + 0.2, 0.0];
                                    state.tubes[9].p2 = [collector_x + 0.5, 0.0];
                                    state.tubes[9].p3 = [collector_x + 0.8, 0.0];
                                }
                                
                                // Re-scale collector junction position
                                if state.junctions.len() > 1 {
                                    state.junctions[1].pos = [collector_x, 0.0];
                                }
                                
                                state.reset_trigger = true;
                            }
                        }
                    }
                });
                
                ui.add_space(10.0);

                if has_engine {
                    ui.group(|ui| {
                        ui.label(egui::RichText::new("Engine Diagnostics").strong());
                        ui.horizontal(|ui| {
                            ui.label("Engine Speed:");
                            ui.label(egui::RichText::new(format!("{:.0} RPM", engine_rpm)).color(egui::Color32::LIGHT_GREEN));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Crank Angle:");
                            ui.label(format!("{:.1}° BTDC", (crank_angle.to_degrees() % 360.0)));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Cylinder Pressure:");
                            ui.label(format!("{:.1} kPa", cyl_p / 1000.0));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Cylinder Volume:");
                            ui.label(format!("{:.1} cc", cyl_v * 1e6));
                        });
                    });
                    ui.add_space(10.0);

                    ui.group(|ui| {
                        ui.label(egui::RichText::new("Engine Controls").strong());
                        ui.add(egui::Slider::new(&mut self.throttle_input, 0.0..=1.0).text("Throttle").suffix("x"));
                        
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.ignition_on_input, "Ignition ON");
                            ui.checkbox(&mut self.audio_on_input, "Audio Output");
                        });

                        if has_engine {
                            ui.horizontal(|ui| {
                                ui.label("Ignition Timing:");
                                ui.label(format!("{:.1}° BTDC (ECU Map)", self.ignition_timing_deg_input as f64));
                            });
                            ui.horizontal(|ui| {
                                ui.label("Target AFR:");
                                ui.label(format!("{:.1} (λ={:.2}) (ECU Map)", self.target_afr_input as f64, (self.target_afr_input / 14.7) as f64));
                            });
                        } else {
                            ui.add(egui::Slider::new(&mut self.ignition_timing_deg_input, 0.0..=45.0).text("Ignition Timing").suffix("° BTDC"));
                            
                            ui.horizontal(|ui| {
                                ui.add(egui::Slider::new(&mut self.target_afr_input, 9.0..=20.0).text("Target AFR"));
                                let lambda = self.target_afr_input / 14.7;
                                ui.label(format!("(λ={:.2})", lambda as f64));
                            });
                        }

                        let (ecu_map_pa, ecu_iac_lift, ecu_is_cranking, ecu_rev_limiter_active, cyl_afr, cyl_lambda, res_frac, last_misfire, misfire_cnt) = {
                            if let Ok(state) = self.shared_state.lock() {
                                (
                                    state.ecu_map_pa,
                                    state.ecu_iac_lift,
                                    state.ecu_is_cranking,
                                    state.ecu_rev_limiter_active,
                                    state.cylinder_afr,
                                    state.cylinder_lambda,
                                    state.residual_fraction,
                                    state.last_misfire,
                                    state.misfire_count,
                                )
                            } else {
                                (101325.0, 0.0, true, false, 14.7, 1.0, 0.0, false, 0)
                            }
                        };

                        if has_engine {
                            ui.add_space(5.0);
                            ui.separator();
                            ui.label(egui::RichText::new("ECU Telemetry").strong());
                            
                            let mode_str = if ecu_is_cranking { "Cranking" } else { "Running" };
                            ui.horizontal(|ui| {
                                ui.label("ECU Mode:");
                                ui.label(egui::RichText::new(mode_str).color(if ecu_is_cranking { egui::Color32::YELLOW } else { egui::Color32::GREEN }));
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("MAP (Load):");
                                ui.label(format!("{:.1} kPa", (ecu_map_pa / 1000.0) as f64));
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("IAC Valve Lift:");
                                ui.label(format!("{:.1}%", (ecu_iac_lift * 100.0 / 0.015) as f64));
                            });

                            ui.horizontal(|ui| {
                                ui.label("In-Cylinder AFR:");
                                let afr_color = if last_misfire {
                                    egui::Color32::RED
                                } else if (cyl_afr - 14.7).abs() < 1.0 {
                                    egui::Color32::GREEN
                                } else {
                                    egui::Color32::YELLOW
                                };
                                ui.label(egui::RichText::new(format!("{:.1} (λ={:.2})", cyl_afr as f64, cyl_lambda as f64)).color(afr_color));
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Residual Exhaust:");
                                ui.label(format!("{:.1}%", (res_frac * 100.0) as f64));
                            });
                            
                            ui.horizontal(|ui| {
                                ui.label("Misfires:");
                                if last_misfire {
                                    ui.label(egui::RichText::new(format!("{} (💥 MISFIRE)", misfire_cnt)).color(egui::Color32::RED).strong());
                                } else {
                                    ui.label(format!("{}", misfire_cnt));
                                }
                            });

                            if ecu_rev_limiter_active {
                                ui.colored_label(egui::Color32::RED, "⚠️ REV LIMITER ACTIVE");
                            }
                        }

                        ui.add_space(5.0);

                        // Check starter state
                        let starter_active = {
                            if let Ok(state) = self.shared_state.lock() {
                                state.starter_engaged && state.starter_timer > 0.0
                            } else {
                                false
                            }
                        };

                        ui.horizontal(|ui| {
                            if engine_rpm < 100.0 && !starter_active {
                                if ui.button("⚡ Start Engine (Crank)").clicked() {
                                    self.ignition_on_input = true;
                                    if let Ok(mut state) = self.shared_state.lock() {
                                        state.starter_engaged = true;
                                        state.starter_timer = 2.0; // 2 second crank window
                                        state.ignition_on = true;
                                    }
                                }
                            } else if starter_active {
                                ui.label(egui::RichText::new("⚡ Cranking...").color(egui::Color32::YELLOW));
                            } else {
                                if ui.button("🔌 Kill Engine (Stop)").clicked() {
                                    self.ignition_on_input = false;
                                }
                            }
                            
                            // Spin up controls for debugging
                            if ui.button("🌀 Spin (2500 RPM)").clicked() {
                                if let Ok(mut state) = self.shared_state.lock() {
                                    state.trigger_spin = true;
                                    state.spin_rpm = 2500.0;
                                }
                            }
                        });
                    });
                    ui.add_space(10.0);

                    ui.group(|ui| {
                        ui.label(egui::RichText::new("Engine Geometry").strong());
                        ui.add(egui::Slider::new(&mut self.engine_bore_input, 0.04..=0.15).text("Bore").suffix(" m"));
                        ui.add(egui::Slider::new(&mut self.engine_stroke_input, 0.04..=0.15).text("Stroke").suffix(" m"));
                        ui.add(egui::Slider::new(&mut self.engine_conrod_input, 0.10..=0.30).text("Conrod").suffix(" m"));
                        ui.add(egui::Slider::new(&mut self.engine_compression_ratio_input, 4.0..=20.0).text("CR"));
                        ui.add(egui::Slider::new(&mut self.engine_inertia_input, 0.005..=0.2).text("Inertia"));
                        ui.add(egui::Slider::new(&mut self.engine_friction_input, 0.01..=0.5).text("Friction"));
                    });
                    ui.add_space(10.0);
                }

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
                    
                    ui.add_space(5.0);
                    ui.separator();
                    ui.add_space(5.0);
                    ui.label(egui::RichText::new("Solver Profile Breakdown").strong().size(12.0));
                    ui.horizontal(|ui| {
                        ui.label("Tubes RHS:");
                        ui.label(format!("{:.1} µs", solver_profile.tubes_rhs_us));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Tubes Update:");
                        ui.label(format!("{:.1} µs", solver_profile.tubes_update_us));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Boundary Cond:");
                        ui.label(format!("{:.1} µs", solver_profile.boundary_conditions_us));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cylinder Physics:");
                        ui.label(format!("{:.1} µs", solver_profile.cylinder_physics_us));
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
                
                if !has_engine {
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
                }
                
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
                    
                    ui.add(egui::Slider::new(&mut self.friction_input, 0.0..=0.2).text("Wall Friction"));
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

                    if has_engine {
                        ui.add_space(5.0);
                        ui.separator();
                        ui.add_space(5.0);
                        ui.label(egui::RichText::new("Cylinder Pressure vs. Crank Angle (PV/θ Curve)").strong());
                        
                        let cyl_histories = {
                            let state = self.shared_state.lock().unwrap();
                            state.cyl_pressure_histories.clone()
                        };
                        
                        let colors = [
                            egui::Color32::from_rgb(50, 205, 50),   // Green
                            egui::Color32::from_rgb(0, 191, 255),   // Deep Sky Blue
                            egui::Color32::from_rgb(255, 165, 0),   // Orange
                            egui::Color32::from_rgb(255, 105, 180),  // Hot Pink
                        ];
                        
                        let plot = Plot::new("cylinder_pressure_plot")
                            .height(140.0)
                            .show_grid(true)
                            .x_axis_label("Crank Angle (degrees)")
                            .y_axis_label("Pressure (kPa)")
                            .include_x(0.0)
                            .include_x(720.0)
                            .include_y(0.0)
                            .include_y(2500.0); // Show up to 25 bar (2500 kPa)
                        
                        plot.show(ui, |plot_ui| {
                            plot_ui.hline(HLine::new((P_ATM / 1000.0) as f64).color(egui::Color32::GRAY).style(egui_plot::LineStyle::Dashed { length: 6.0 }));
                            for (i, history) in cyl_histories.iter().enumerate() {
                                let points: PlotPoints = history
                                    .iter()
                                    .map(|&[angle, p]| [angle as f64, (p / 1000.0) as f64])
                                    .collect();
                                let line = Line::new(points)
                                    .color(colors[i % colors.len()])
                                    .width(2.0)
                                    .name(format!("Cyl {}", i + 1));
                                plot_ui.line(line);
                            }
                        });
                    }
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

                // Draw Crankshaft & Cylinder if present
                if has_engine {
                    let theta = crank_angle;
                    
                    // Use actual engine geometry scaled for viewport display.
                    // Physics dimensions are in meters (~0.04-0.08m), viewport is ~[-2.5, 2.5].
                    // We apply a visual scale to make it fill the viewport nicely.
                    let vis_scale = 3.5; // meters → viewport units
                    
                    let r_crank = (eng_stroke / 2.0) * vis_scale; // crank radius
                    let l_rod = eng_conrod * vis_scale;            // connecting rod length
                    let piston_w = eng_bore * vis_scale;           // piston width = bore
                    let piston_h = piston_w * 0.5;                 // piston height (visual proportional to bore)
                    
                    // Crankshaft center position — placed so cylinder head lands at y ≈ 0.35
                    // TDC wrist_y = c_center_y + r + l, head_y = TDC_wrist + piston_h/2 + clearance
                    let head_y = 0.35;
                    let tdc_wrist_y = head_y - piston_h / 2.0 - 0.02; // 0.02 clearance gap
                    let c_center_y = tdc_wrist_y - r_crank - l_rod;
                    let c_center = [0.0, c_center_y];
                    
                    // 1. Calculate positions using exact crank-slider kinematics
                    let pin_x = c_center[0] + r_crank * theta.sin();
                    let pin_y = c_center[1] + r_crank * theta.cos();
                    let s_pin = to_screen([pin_x, pin_y]);
                    let s_center = to_screen(c_center);
                    
                    // Wrist pin: exact crank-slider formula (matches physics in crankshaft.rs)
                    let wrist_x = c_center[0];
                    let wrist_y = c_center[1] + r_crank * theta.cos() 
                        + (l_rod * l_rod - r_crank * r_crank * theta.sin().powi(2)).max(0.0).sqrt();
                    let s_wrist = to_screen([wrist_x, wrist_y]);
                    
                    // Piston body centered on wrist pin
                    let p_top_y = wrist_y + piston_h / 2.0;
                    let p_bottom_y = wrist_y - piston_h / 2.0;
                    let s_p_left_top = to_screen([wrist_x - piston_w / 2.0, p_top_y]);
                    let s_p_right_top = to_screen([wrist_x + piston_w / 2.0, p_top_y]);
                    let s_p_right_bottom = to_screen([wrist_x + piston_w / 2.0, p_bottom_y]);
                    let s_p_left_bottom = to_screen([wrist_x - piston_w / 2.0, p_bottom_y]);
                    
                    // 2. Draw Crankshaft counterweight (sized relative to crank radius)
                    let cw_radius = r_crank * 1.4;
                    painter.circle_filled(s_center, scale * cw_radius, egui::Color32::from_rgba_unmultiplied(65, 65, 65, 180));
                    painter.circle_stroke(s_center, scale * cw_radius, egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 100, 100)));
                    
                    // 3. Draw crank pin orbit path
                    painter.circle_stroke(s_center, scale * r_crank, egui::Stroke::new(0.5, egui::Color32::from_rgba_unmultiplied(120, 120, 120, 60)));
                    
                    // 4. Draw Crank Web (arm from center to pin)
                    painter.line_segment([s_center, s_pin], egui::Stroke::new(8.0, egui::Color32::from_rgb(90, 95, 100)));
                    painter.circle_filled(s_pin, 7.0, egui::Color32::from_rgb(130, 130, 130));
                    
                    // 5. Draw Connecting Rod (from crank pin to wrist pin)
                    painter.line_segment([s_pin, s_wrist], egui::Stroke::new(6.0, egui::Color32::from_rgb(175, 180, 185)));
                    
                    // 6. Draw Piston body
                    painter.add(egui::Shape::convex_polygon(
                        vec![s_p_left_top, s_p_right_top, s_p_right_bottom, s_p_left_bottom],
                        egui::Color32::from_rgb(100, 105, 110),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(180, 180, 180)),
                    ));
                    
                    // Piston rings (near crown)
                    let ring1_y = p_top_y - piston_h * 0.15;
                    let ring2_y = p_top_y - piston_h * 0.30;
                    painter.line_segment(
                        [to_screen([wrist_x - piston_w / 2.0, ring1_y]), to_screen([wrist_x + piston_w / 2.0, ring1_y])],
                        egui::Stroke::new(1.5, egui::Color32::BLACK),
                    );
                    painter.line_segment(
                        [to_screen([wrist_x - piston_w / 2.0, ring2_y]), to_screen([wrist_x + piston_w / 2.0, ring2_y])],
                        egui::Stroke::new(1.5, egui::Color32::BLACK),
                    );
                    
                    // Wrist pin
                    painter.circle_filled(s_wrist, 5.0, egui::Color32::from_rgb(220, 220, 220));
                    
                    // 7. Draw Cylinder Bore Walls
                    let wall_gap = 0.005; // small gap between piston and cylinder wall
                    let cyl_wall_x_left = wrist_x - piston_w / 2.0 - wall_gap;
                    let cyl_wall_x_right = wrist_x + piston_w / 2.0 + wall_gap;
                    // Walls extend from below BDC piston skirt to the head
                    let bdc_wrist_y = c_center_y + (l_rod - r_crank); // BDC wrist position
                    let wall_bottom_y = bdc_wrist_y - piston_h * 0.8;
                    let s_wall_left_bottom = to_screen([cyl_wall_x_left, wall_bottom_y]);
                    let s_wall_left_top = to_screen([cyl_wall_x_left, head_y]);
                    let s_wall_right_bottom = to_screen([cyl_wall_x_right, wall_bottom_y]);
                    let s_wall_right_top = to_screen([cyl_wall_x_right, head_y]);
                    
                    let cylinder_wall_stroke = egui::Stroke::new(2.5, egui::Color32::from_rgb(80, 85, 90));
                    painter.line_segment([s_wall_left_bottom, s_wall_left_top], cylinder_wall_stroke);
                    painter.line_segment([s_wall_right_bottom, s_wall_right_top], cylinder_wall_stroke);
                    
                    // Cylinder head plate
                    painter.line_segment(
                        [to_screen([cyl_wall_x_left, head_y]), to_screen([cyl_wall_x_right, head_y])],
                        egui::Stroke::new(4.0, egui::Color32::from_rgb(80, 85, 90)),
                    );
                    
                    // 8. Draw Intake and Exhaust Ports (connecting to tube endpoints at head_y)
                    let port_half = piston_w / 2.0 + wall_gap;
                    painter.line_segment(
                        [to_screen([-0.12, head_y]), to_screen([-port_half * 0.3, head_y])],
                        egui::Stroke::new(6.0, egui::Color32::from_rgb(50, 50, 50)),
                    );
                    painter.line_segment(
                        [to_screen([0.12, head_y]), to_screen([port_half * 0.3, head_y])],
                        egui::Stroke::new(6.0, egui::Color32::from_rgb(50, 50, 50)),
                    );
                    
                    // 9. Draw Intake and Exhaust Valves in the Head
                    let lift_in = get_valve_lift(theta, 350.0f32.to_radians(), 570.0f32.to_radians());
                    let lift_ex = get_valve_lift(theta, 140.0f32.to_radians(), 380.0f32.to_radians());
                    
                    // Intake Valve (moves down from head when open)
                    let valve_drop = 0.03; // max visual displacement when fully open
                    let v_in_y = head_y - lift_in * valve_drop;
                    let v_in_cx = -piston_w * 0.18; // centered on intake side
                    let valve_head_w = piston_w * 0.22;
                    painter.line_segment(
                        [to_screen([v_in_cx, v_in_y + 0.05]), to_screen([v_in_cx, v_in_y])],
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 140, 0)),
                    );
                    painter.line_segment(
                        [to_screen([v_in_cx - valve_head_w, v_in_y]), to_screen([v_in_cx + valve_head_w, v_in_y])],
                        egui::Stroke::new(2.5, egui::Color32::from_rgb(255, 140, 0)),
                    );
                    
                    // Exhaust Valve
                    let v_ex_y = head_y - lift_ex * valve_drop;
                    let v_ex_cx = piston_w * 0.18; // centered on exhaust side
                    painter.line_segment(
                        [to_screen([v_ex_cx, v_ex_y + 0.05]), to_screen([v_ex_cx, v_ex_y])],
                        egui::Stroke::new(2.0, egui::Color32::from_rgb(0, 191, 255)),
                    );
                    painter.line_segment(
                        [to_screen([v_ex_cx - valve_head_w, v_ex_y]), to_screen([v_ex_cx + valve_head_w, v_ex_y])],
                        egui::Stroke::new(2.5, egui::Color32::from_rgb(0, 191, 255)),
                    );
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
            .with_title("EnGen - 1D Compressible Network CFD Solver (Milestone 5)")
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
        friction: 0.025,
        heat_transfer: 30.0,
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
        
        has_engine: false,
        engine_rpm: 0.0,
        engine_crank_angle: 0.0,
        cylinder_pressure: P_ATM,
        cylinder_volume: 0.0,
        cyl_pressure_histories: Vec::new(),
        engine_bore: 0.052,
        engine_stroke: 0.036,
        engine_conrod: 0.072,
        engine_compression_ratio: 10.0,
        engine_inertia: 0.01,
        engine_friction: 0.001,
        spin_rpm: 1200.0,
        trigger_spin: false,
        throttle: 1.0,
        ignition_on: true,
        ignition_timing_deg: 15.0,
        target_afr: 14.7,
        audio_on: true,
        starter_engaged: false,
        starter_timer: 0.0,
        ecu_map_pa: 101325.0,
        ecu_iac_lift: 0.0,
        ecu_is_cranking: true,
        ecu_rev_limiter_active: false,
        cylinder_afr: 14.7,
        cylinder_lambda: 1.0,
        residual_fraction: 0.0,
        last_misfire: false,
        misfire_count: 0,
        profile: SolverProfile::default(),
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
