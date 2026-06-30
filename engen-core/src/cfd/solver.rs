use std::fmt;
use crate::mechanical::{Crankshaft, get_valve_lift, piston_dy_dtheta, ValveType};
use crate::combustion::{Cylinder, CylinderConnection};
use crate::ecu::Ecu;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimiterType {
    None,
    Minmod,
    Superbee,
    MC, // Monotonized Central
    VanLeer,
}

impl fmt::Display for LimiterType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LimiterType::None => write!(f, "None (1st Order)"),
            LimiterType::Minmod => write!(f, "Minmod"),
            LimiterType::Superbee => write!(f, "Superbee"),
            LimiterType::MC => write!(f, "Monotonized Central (MC)"),
            LimiterType::VanLeer => write!(f, "Van Leer"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoundaryType {
    Closed, // Rigid reflecting wall
    Open,   // Atmospheric boundary
    Valve { lift: f32 },
}

impl fmt::Display for BoundaryType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BoundaryType::Closed => write!(f, "Closed (Wall)"),
            BoundaryType::Open => write!(f, "Open (Atmosphere)"),
            BoundaryType::Valve { lift } => write!(f, "Valve (Lift: {:.2})", lift),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadiusProfile {
    Linear,
    ExpansionChamber,
}

impl fmt::Display for RadiusProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RadiusProfile::Linear => write!(f, "Linear Taper"),
            RadiusProfile::ExpansionChamber => write!(f, "Expansion Chamber"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TubeSide {
    Left,
    Right,
}

#[derive(Debug, Clone)]
pub struct JunctionConnection {
    pub tube_id: usize,
    pub side: TubeSide,
}

#[derive(Debug, Clone)]
pub struct Junction {
    pub id: usize,
    pub pos: [f32; 2],
    pub connections: Vec<JunctionConnection>,
    // Conserved variable equivalent for zero-dimensional junction cell [rho, E] (no momentum vector)
    pub rho: f32,
    pub e: f32,
    pub volume: f32,
    pub species: [f32; 4],
    pub species_temp: [f32; 4],
}

use crate::cfd::species::AIR_Y;

impl Junction {
    /// Copies only the mutable simulation state from `other`, leaving
    /// `connections` (and other static fields) untouched. Used instead of
    /// `Vec<Junction>::clone_from`, since derived Clone doesn't override
    /// clone_from and falls back to full per-field clone() on each element,
    /// reallocating `connections` every substep even though it never changes.
    #[inline]
    fn copy_state_from(&mut self, other: &Junction) {
        self.rho = other.rho;
        self.e = other.e;
        self.species = other.species;
        self.species_temp = other.species_temp;
    }
}

#[derive(Debug, Clone)]
pub struct Tube {
    pub id: usize,
    pub name: String,
    pub num_cells: usize,
    pub u: Vec<[f32; 3]>,     // Conserved variables: [rho, rho*v, E]
    pub dx: f32,             // Cell width (m)
    pub area: Vec<f32>,       // Cell cross-sectional area (m^2)
    pub dadx: Vec<f32>,       // Area derivative dA/dx (m)
    
    // 2D Bezier Geometry
    pub p0: [f32; 2],
    pub p1: [f32; 2],
    pub p2: [f32; 2],
    pub p3: [f32; 2],
    pub r_start: f32,
    pub r_end: f32,
    pub r_mid: f32,           // Mid radius for expansion chambers
    pub radius_profile: RadiusProfile,
    
    pub left_bc: BoundaryType,   // Active if not connected to a junction
    pub right_bc: BoundaryType,  // Active if not connected to a junction
    
    // Boundary fluxes computed on last step (used to update junctions)
    pub left_boundary_flux: [f32; 3],
    pub right_boundary_flux: [f32; 3],
    
    pub left_last_r_out: f32,
    pub right_last_r_out: f32,
    pub connected_left: bool,
    pub connected_right: bool,
    
    // Jones mass flow derivative variables
    pub left_last_mass_flow: f32,
    pub right_last_mass_flow: f32,
    
    // Species Tracking
    pub species: Vec<[f32; 4]>,
    pub species_temp: Vec<[f32; 4]>,
    pub left_species_flux: [f32; 4],
    pub right_species_flux: [f32; 4],
    
    // Pre-allocated temporary workspaces for solver to avoid heap allocations in hot path
    pub u_temp: Vec<[f32; 3]>,
    pub w: Vec<[f32; 3]>,
    pub slopes: Vec<[f32; 3]>,
    pub fluxes: Vec<[f32; 3]>,
    pub rhs: Vec<[f32; 3]>,
    pub species_rhs: Vec<[f32; 4]>,
    
    // Pre-calculated 2D graphics vectors for rendering the heatmap layout
    pub cell_centers_2d: Vec<[f32; 2]>,
    pub cell_tangents_2d: Vec<[f32; 2]>,
    pub cell_boundaries_left_2d: Vec<[f32; 2]>,
    pub cell_boundaries_right_2d: Vec<[f32; 2]>,
}

impl Tube {
    pub fn new(
        id: usize,
        name: String,
        num_cells: usize,
        p0: [f32; 2],
        p1: [f32; 2],
        p2: [f32; 2],
        p3: [f32; 2],
        r_start: f32,
        r_end: f32,
        r_mid: f32,
        radius_profile: RadiusProfile,
        left_bc: BoundaryType,
        right_bc: BoundaryType,
    ) -> Self {
        let mut tube = Self {
            id,
            name,
            num_cells: num_cells.max(5),
            u: Vec::new(),
            dx: 0.0,
            area: Vec::new(),
            dadx: Vec::new(),
            p0,
            p1,
            p2,
            p3,
            r_start,
            r_end,
            r_mid,
            radius_profile,
            left_bc,
            right_bc,
            left_boundary_flux: [0.0; 3],
            right_boundary_flux: [0.0; 3],
            left_last_r_out: atmospheric_r_plus(1.4),
            right_last_r_out: atmospheric_r_minus(1.4),
            connected_left: false,
            connected_right: false,
            left_last_mass_flow: 0.0,
            right_last_mass_flow: 0.0,
            species: Vec::new(),
            species_temp: Vec::new(),
            left_species_flux: [0.0; 4],
            right_species_flux: [0.0; 4],
            u_temp: Vec::new(),
            w: Vec::new(),
            slopes: Vec::new(),
            fluxes: Vec::new(),
            rhs: Vec::new(),
            species_rhs: Vec::new(),
            cell_centers_2d: Vec::new(),
            cell_tangents_2d: Vec::new(),
            cell_boundaries_left_2d: Vec::new(),
            cell_boundaries_right_2d: Vec::new(),
        };
        tube.rebuild_geometry();
        tube.reset_state();
        tube
    }

    /// Quiescent-atmosphere R+ invariant stored at a left open/valve end.
    pub fn reset_reflection_filters(&mut self) {
        self.left_last_r_out = atmospheric_r_plus(1.4);
        self.right_last_r_out = atmospheric_r_minus(1.4);
    }

    pub fn reset_state(&mut self) {
        let w_init = [RHO_ATM, 0.0, P_ATM];
        let u_init = primitive_to_conserved(&w_init, 1.4);
        self.u = vec![u_init; self.num_cells + 2];
        self.left_boundary_flux = [0.0; 3];
        self.right_boundary_flux = [0.0; 3];
        self.reset_reflection_filters();
        self.left_last_mass_flow = 0.0;
        self.right_last_mass_flow = 0.0;
        
        self.species = vec![AIR_Y; self.num_cells + 2];
        self.species_temp = vec![AIR_Y; self.num_cells + 2];
        self.left_species_flux = [0.0; 4];
        self.right_species_flux = [0.0; 4];
        
        // Reset temporary workspaces
        for x in &mut self.u_temp { *x = [0.0; 3]; }
        for x in &mut self.w { *x = [0.0; 3]; }
        for x in &mut self.slopes { *x = [0.0; 3]; }
        for x in &mut self.fluxes { *x = [0.0; 3]; }
        for x in &mut self.rhs { *x = [0.0; 3]; }
        for x in &mut self.species_rhs { *x = [0.0; 4]; }
    }

    pub fn get_radius_at(&self, t: f32) -> f32 {
        match self.radius_profile {
            RadiusProfile::Linear => self.r_start + t * (self.r_end - self.r_start),
            RadiusProfile::ExpansionChamber => {
                if t < 0.25 {
                    let u = t / 0.25;
                    self.r_start + u * (self.r_mid - self.r_start)
                } else if t < 0.75 {
                    self.r_mid
                } else {
                    let u = (t - 0.75) / 0.25;
                    self.r_mid + u * (self.r_end - self.r_mid)
                }
            }
        }
    }

    pub fn rebuild_geometry(&mut self) {
        let length = bezier_length(self.p0, self.p1, self.p2, self.p3);
        let m = self.num_cells;
        self.dx = length / m as f32;

        self.area = vec![0.0; m + 2];
        self.dadx = vec![0.0; m + 2];
        
        // Resize temporary workspaces
        self.u_temp = vec![[0.0; 3]; m + 2];
        self.w = vec![[0.0; 3]; m + 2];
        self.slopes = vec![[0.0; 3]; m + 2];
        self.fluxes = vec![[0.0; 3]; m + 1];
        self.rhs = vec![[0.0; 3]; m + 2];
        self.species = vec![AIR_Y; m + 2];
        self.species_temp = vec![AIR_Y; m + 2];
        self.species_rhs = vec![[0.0; 4]; m + 2];
        
        self.cell_centers_2d = vec![[0.0; 2]; m + 1];
        self.cell_tangents_2d = vec![[0.0; 2]; m + 1];
        self.cell_boundaries_left_2d = vec![[0.0; 2]; m + 2];
        self.cell_boundaries_right_2d = vec![[0.0; 2]; m + 2];

        // Evaluate properties at active cell centers (1..=m)
        for i in 1..=m {
            let t = (i as f32 - 0.5) / m as f32;
            let r = self.get_radius_at(t);
            self.area[i] = std::f32::consts::PI * r * r;

            // Area derivative dA/dx = dA/dt * dt/dx = 2*pi*r* dr/dt / |B'(t)|
            let deriv = bezier_derivative(t, self.p0, self.p1, self.p2, self.p3);
            let speed = (deriv[0] * deriv[0] + deriv[1] * deriv[1]).sqrt().max(1e-5);
            
            let drdt = match self.radius_profile {
                RadiusProfile::Linear => self.r_end - self.r_start,
                RadiusProfile::ExpansionChamber => {
                    if t < 0.25 {
                        (self.r_mid - self.r_start) / 0.25
                    } else if t < 0.75 {
                        0.0
                    } else {
                        (self.r_end - self.r_mid) / 0.25
                    }
                }
            };
            let dadt = 2.0 * std::f32::consts::PI * r * drdt;
            self.dadx[i] = dadt / speed;

            // Pre-calculate center and velocity direction vectors for particles
            self.cell_centers_2d[i] = bezier_point(t, self.p0, self.p1, self.p2, self.p3);
            self.cell_tangents_2d[i] = [deriv[0] / speed, deriv[1] / speed];
        }

        // Ghost cell area boundary padding
        self.area[0] = self.area[1];
        self.dadx[0] = 0.0;
        self.area[m + 1] = self.area[m];
        self.dadx[m + 1] = 0.0;

        // Evaluate boundary coordinates for rendering quad layout strip
        for j in 0..=m {
            let t = j as f32 / m as f32;
            let p = bezier_point(t, self.p0, self.p1, self.p2, self.p3);
            let deriv = bezier_derivative(t, self.p0, self.p1, self.p2, self.p3);
            let speed = (deriv[0] * deriv[0] + deriv[1] * deriv[1]).sqrt().max(1e-5);
            let normal = [-deriv[1] / speed, deriv[0] / speed];
            let r = self.get_radius_at(t);

            // Scale factor to map coordinates to a readable viewport scale
            self.cell_boundaries_left_2d[j] = [p[0] + normal[0] * r, p[1] + normal[1] * r];
            self.cell_boundaries_right_2d[j] = [p[0] - normal[0] * r, p[1] - normal[1] * r];
        }
        // Pad boundaries list
        self.cell_boundaries_left_2d[m + 1] = self.cell_boundaries_left_2d[m];
        self.cell_boundaries_right_2d[m + 1] = self.cell_boundaries_right_2d[m];
    }
}

pub const P_ATM: f32 = 101325.0; // Pa
pub const T_ATM: f32 = 293.15;   // K
pub const R_AIR: f32 = 287.05;   // J/(kg*K)
pub const RHO_ATM: f32 = P_ATM / (R_AIR * T_ATM);

fn atmospheric_sound_speed(gamma: f32) -> f32 {
    (gamma * R_AIR * T_ATM).sqrt()
}

/// R+ invariant at a quiescent open boundary (v=0, c=c_res).
fn atmospheric_r_plus(gamma: f32) -> f32 {
    2.0 * atmospheric_sound_speed(gamma) / (gamma - 1.0)
}

/// R- invariant at a quiescent open boundary (v=0, c=c_res).
fn atmospheric_r_minus(gamma: f32) -> f32 {
    -atmospheric_r_plus(gamma)
}

/// Build a ghost-cell primitive state from filtered Riemann invariants.
/// Thermodynamics are tied to c_ghost via isentropic relations from the reservoir.
fn ghost_from_riemann(r_plus: f32, r_minus: f32, _gamma: f32, c_res: f32) -> [f32; 3] {
    let v_ghost = 0.5 * (r_plus + r_minus);
    let c_ghost = (0.1 * (r_plus - r_minus)).max(1.0);
    let rho_ghost = RHO_ATM * (c_ghost / c_res).powi(5);
    let p_ghost = P_ATM * (c_ghost / c_res).powi(7);
    [rho_ghost.max(1e-4), v_ghost, p_ghost.max(1e-2)]
}

fn ghost_from_riemann_res(r_plus: f32, r_minus: f32, _gamma: f32, c_res: f32, p_res: f32, rho_res: f32) -> [f32; 3] {
    let v_ghost = 0.5 * (r_plus + r_minus);
    let c_ghost = (0.1 * (r_plus - r_minus)).max(1.0);
    let rho_ghost = rho_res * (c_ghost / c_res).powi(5);
    let p_ghost = p_res * (c_ghost / c_res).powi(7);
    [rho_ghost.max(1e-4), v_ghost, p_ghost.max(1e-2)]
}


/// 0 = quiescent atmosphere, 1 = fully active Benson/open-port coupling.
fn open_bc_activity(_w: &[f32; 3]) -> f32 {
    1.0
}

fn lerp_primitive(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    let s = 1.0 - t;
    [a[0] * s + b[0] * t, a[1] * s + b[1] * t, a[2] * s + b[2] * t]
}

fn is_radiating_open_bc(bc: BoundaryType) -> bool {
    match bc {
        BoundaryType::Open => true,
        BoundaryType::Valve { lift } => lift > 1e-4,
        BoundaryType::Closed => false,
    }
}

#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct SolverProfile {
    pub tubes_rhs_us: f32,
    pub tubes_update_us: f32,
    pub boundary_conditions_us: f32,
    pub cylinder_physics_us: f32,
}

pub struct Solver {
    pub tubes: Vec<Tube>,
    pub junctions: Vec<Junction>,
    pub junctions_temp: Vec<Junction>,
    pub limiter: LimiterType,
    pub friction: f32,
    pub heat_transfer: f32,
    pub t: f32,
    pub reflection_filter: f32,
    pub step_counter: usize,
    pub cached_dt_stable: f32,
    pub crankshaft: Option<Crankshaft>,
    pub cylinders: Vec<Cylinder>,
    pub ignition_on: bool,
    pub ignition_timing_deg: f32,
    pub target_afr: f32,
    pub throttle_input: f32,
    pub starter_active: bool,
    pub ecu: Ecu,
    
    // Telemetry for species & misfires
    pub last_cylinder_afr: f32,
    pub last_cylinder_lambda: f32,
    pub last_residual_fraction: f32,
    pub last_misfire: bool,
    pub misfire_count: u32,
    pub misfire_model: crate::combustion::MisfireModel,
    pub profile: SolverProfile,
    initial_mass_flows: Vec<(f32, f32)>,
}

impl Solver {
    pub fn update_connected_flags(&mut self) {
        for tube in &mut self.tubes {
            tube.connected_left = false;
            tube.connected_right = false;
        }
        for j in &self.junctions {
            for conn in &j.connections {
                if conn.tube_id < self.tubes.len() {
                    match conn.side {
                        TubeSide::Left => self.tubes[conn.tube_id].connected_left = true,
                        TubeSide::Right => self.tubes[conn.tube_id].connected_right = true,
                    }
                }
            }
        }
        for cyl in &self.cylinders {
            for conn in &cyl.connections {
                if conn.tube_id < self.tubes.len() {
                    match conn.side {
                        TubeSide::Left => self.tubes[conn.tube_id].connected_left = true,
                        TubeSide::Right => self.tubes[conn.tube_id].connected_right = true,
                    }
                }
            }
        }
    }

    pub fn new_single_tube(config: RadiusProfile, left_bc: BoundaryType, right_bc: BoundaryType) -> Self {
        let r_mid = if config == RadiusProfile::ExpansionChamber { 0.05 } else { 0.02 };
        let tube = Tube::new(
            0,
            "Main Tube".to_string(),
            50,
            [-2.0, 0.0],
            [-0.66, 0.0],
            [0.66, 0.0],
            [2.0, 0.0],
            0.02,
            0.02,
            r_mid,
            config,
            left_bc,
            right_bc,
        );
        let mut solver = Self {
            tubes: vec![tube],
            junctions: Vec::new(),
            junctions_temp: Vec::new(),
            limiter: LimiterType::VanLeer,
            friction: 0.0,
            heat_transfer: 0.0,
            t: 0.0,
            reflection_filter: 0.75,
            step_counter: 0,
            cached_dt_stable: 0.0,
            crankshaft: None,
            cylinders: Vec::new(),
            ignition_on: false,
            ignition_timing_deg: 15.0,
            target_afr: 14.7,
            throttle_input: 1.0,
            starter_active: false,
            ecu: Ecu::new(),
            last_cylinder_afr: 14.7,
            last_cylinder_lambda: 1.0,
            last_residual_fraction: 0.0,
            last_misfire: false,
            misfire_count: 0,
            misfire_model: crate::combustion::MisfireModel::new(),
            profile: SolverProfile::default(),
            initial_mass_flows: Vec::new(),
        };
        solver.update_connected_flags();
        solver
    }

    pub fn new_y_junction() -> Self {
        // Main Tube (Main Exhaust Pipe)
        let main_tube = Tube::new(
            0,
            "Main".to_string(),
            40,
            [-1.5, 0.0],
            [-0.5, 0.0],
            [-0.2, 0.0],
            [0.0, 0.0],
            0.025,
            0.025,
            0.025,
            RadiusProfile::Linear,
            BoundaryType::Open,
            BoundaryType::Closed, // connecting to junction
        );
        
        // Branch A (Top Curve)
        let branch_a = Tube::new(
            1,
            "Branch A".to_string(),
            30,
            [0.0, 0.0],
            [0.4, 0.15],
            [0.8, 0.45],
            [1.2, 0.5],
            0.018,
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed, // connecting to junction
            BoundaryType::Closed, // end wall
        );

        // Branch B (Bottom Curve)
        let branch_b = Tube::new(
            2,
            "Branch B".to_string(),
            30,
            [0.0, 0.0],
            [0.4, -0.15],
            [0.8, -0.45],
            [1.2, -0.5],
            0.018,
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed, // connecting to junction
            BoundaryType::Closed, // end wall
        );

        // Single Y-junction at (0.0, 0.0) connecting right end of main (index 0) and left end of branches (1 and 2)
        let conn_main = JunctionConnection { tube_id: 0, side: TubeSide::Right };
        let conn_a = JunctionConnection { tube_id: 1, side: TubeSide::Left };
        let conn_b = JunctionConnection { tube_id: 2, side: TubeSide::Left };
        
        // Volume calculation based on sum of endpoint area times 0.5 * cell length
        let vol = (main_tube.area[main_tube.num_cells] * main_tube.dx +
                   branch_a.area[1] * branch_a.dx +
                   branch_b.area[1] * branch_b.dx) * 0.5;

        let junction = Junction {
            id: 0,
            pos: [0.0, 0.0],
            connections: vec![conn_main, conn_a, conn_b],
            rho: RHO_ATM,
            e: P_ATM / (1.4 - 1.0),
            volume: vol,
            species: AIR_Y,
            species_temp: AIR_Y,
        };

        let mut solver = Self {
            tubes: vec![main_tube, branch_a, branch_b],
            junctions: vec![junction],
            junctions_temp: Vec::new(),
            limiter: LimiterType::VanLeer,
            friction: 0.0,
            heat_transfer: 0.0,
            t: 0.0,
            reflection_filter: 0.75,
            step_counter: 0,
            cached_dt_stable: 0.0,
            crankshaft: None,
            cylinders: Vec::new(),
            ignition_on: false,
            ignition_timing_deg: 15.0,
            target_afr: 14.7,
            throttle_input: 1.0,
            starter_active: false,
            ecu: Ecu::new(),
            last_cylinder_afr: 14.7,
            last_cylinder_lambda: 1.0,
            last_residual_fraction: 0.0,
            last_misfire: false,
            misfire_count: 0,
            misfire_model: crate::combustion::MisfireModel::new(),
            profile: SolverProfile::default(),
            initial_mass_flows: Vec::new(),
        };
        solver.update_connected_flags();
        solver
    }

    pub fn new_single_cylinder() -> Self {
        // Intake Runner (Atmosphere/throttle inlet -> cylinder port)
        let intake_tube = Tube::new(
            0,
            "Intake".to_string(),
            30,
            [-1.2, 0.35],
            [-0.8, 0.35],
            [-0.4, 0.35],
            [-0.12, 0.35],
            0.02,
            0.02,
            0.02,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 1.0 }, // Throttle valve
            BoundaryType::Valve { lift: 0.0 }, // connected to intake valve
        );
        
        // Exhaust Runner (cylinder port -> Atmosphere outlet)
        let exhaust_tube = Tube::new(
            1,
            "Exhaust".to_string(),
            40,
            [0.12, 0.35],
            [0.48, 0.35],
            [0.84, 0.35],
            [1.2, 0.35],
            0.012, // r_start (12mm)
            0.012, // r_end (12mm)
            0.035, // r_mid (35mm muffler chamber)
            RadiusProfile::ExpansionChamber,
            BoundaryType::Valve { lift: 0.0 }, // connected to exhaust valve
            BoundaryType::Open,   // radiates sound pressure
        );

        let conn_in = CylinderConnection {
            tube_id: 0,
            side: TubeSide::Right,
            valve_type: ValveType::Intake,
        };
        let conn_ex = CylinderConnection {
            tube_id: 1,
            side: TubeSide::Left,
            valve_type: ValveType::Exhaust,
        };

        // Single cylinder with 52mm bore, 36mm stroke, 72mm conrod, 10.0 CR (chainsaw specs)
        let cylinder = Cylinder::new(
            0,
            0.052, // bore (m)
            0.036, // stroke (m)
            0.072, // conrod (m)
            10.0,  // CR
            vec![conn_in, conn_ex],
        );

        // Crankshaft with 0.01 kg*m^2 inertia (suitable for small 4-stroke) and 0.001 viscous friction
        let crankshaft = Crankshaft::new(0.01, 0.001);

        let mut solver = Self {
            tubes: vec![intake_tube, exhaust_tube],
            junctions: Vec::new(),
            junctions_temp: Vec::new(),
            limiter: LimiterType::VanLeer,
            friction: 0.025,
            heat_transfer: 30.0,
            t: 0.0,
            reflection_filter: 0.75,
            step_counter: 0,
            cached_dt_stable: 0.0,
            crankshaft: Some(crankshaft),
            cylinders: vec![cylinder],
            ignition_on: true,
            ignition_timing_deg: 15.0,
            target_afr: 14.7,
            throttle_input: 1.0,
            starter_active: false,
            ecu: Ecu::new(),
            last_cylinder_afr: 14.7,
            last_cylinder_lambda: 1.0,
            last_residual_fraction: 0.0,
            last_misfire: false,
            misfire_count: 0,
            misfire_model: crate::combustion::MisfireModel::new(),
            profile: SolverProfile::default(),
            initial_mass_flows: Vec::new(),
        };
        solver.update_connected_flags();
        solver
    }

    pub fn new_inline_four() -> Self {
        let config = match crate::config::FullConfig::load_from_file("config/engine_preset.yaml") {
            Ok(c) => c,
            Err(_) => {
                let default_cfg = crate::config::FullConfig::default();
                let _ = std::fs::create_dir_all("config");
                let _ = default_cfg.save_to_file("config/engine_preset.yaml");
                default_cfg
            }
        };

        let pi = std::f32::consts::PI;

        // Tube 0: Main Intake Pipe (Atmosphere -> Plenum Junction)
        let tube0 = Tube::new(
            0,
            "Intake Main".to_string(),
            20,
            [-1.2, 0.0],
            [-1.0, 0.0],
            [-0.8, 0.0],
            [-0.7, 0.0],
            0.025, // 25mm radius
            0.025,
            0.025,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 1.0 }, // Throttle
            BoundaryType::Closed,
        );

        // Tube 1: Intake Runner 1
        let tube1 = Tube::new(
            1,
            "Intake Runner 1".to_string(),
            10,
            [-0.7, 0.0],
            [-0.6, 0.2],
            [-0.5, 0.4],
            [-0.4, 0.6],
            0.018, // 18mm radius
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed,
            BoundaryType::Valve { lift: 0.0 }, // Intake valve 1
        );

        // Tube 2: Intake Runner 2
        let tube2 = Tube::new(
            2,
            "Intake Runner 2".to_string(),
            10,
            [-0.7, 0.0],
            [-0.6, 0.1],
            [-0.5, 0.15],
            [-0.4, 0.2],
            0.018,
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed,
            BoundaryType::Valve { lift: 0.0 }, // Intake valve 2
        );

        // Tube 3: Intake Runner 3
        let tube3 = Tube::new(
            3,
            "Intake Runner 3".to_string(),
            10,
            [-0.7, 0.0],
            [-0.6, -0.1],
            [-0.5, -0.15],
            [-0.4, -0.2],
            0.018,
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed,
            BoundaryType::Valve { lift: 0.0 }, // Intake valve 3
        );

        // Tube 4: Intake Runner 4
        let tube4 = Tube::new(
            4,
            "Intake Runner 4".to_string(),
            10,
            [-0.7, 0.0],
            [-0.6, -0.2],
            [-0.5, -0.4],
            [-0.4, -0.6],
            0.018,
            0.018,
            0.018,
            RadiusProfile::Linear,
            BoundaryType::Closed,
            BoundaryType::Valve { lift: 0.0 }, // Intake valve 4
        );

        // Tube 5: Exhaust Header 1
        let tube5 = Tube::new(
            5,
            "Header 1".to_string(),
            15,
            [-0.2, 0.6],
            [0.0, 0.4],
            [0.2, 0.2],
            [0.4, 0.0],
            0.016, // 16mm radius
            0.016,
            0.016,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 0.0 }, // Exhaust valve 1
            BoundaryType::Closed,
        );

        // Tube 6: Exhaust Header 2
        let tube6 = Tube::new(
            6,
            "Header 2".to_string(),
            15,
            [-0.2, 0.2],
            [0.0, 0.1],
            [0.2, 0.05],
            [0.4, 0.0],
            0.016,
            0.016,
            0.016,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 0.0 }, // Exhaust valve 2
            BoundaryType::Closed,
        );

        // Tube 7: Exhaust Header 3
        let tube7 = Tube::new(
            7,
            "Header 3".to_string(),
            15,
            [-0.2, -0.2],
            [0.0, -0.1],
            [0.2, -0.05],
            [0.4, 0.0],
            0.016,
            0.016,
            0.016,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 0.0 }, // Exhaust valve 3
            BoundaryType::Closed,
        );

        // Tube 8: Exhaust Header 4
        let tube8 = Tube::new(
            8,
            "Header 4".to_string(),
            15,
            [-0.2, -0.6],
            [0.0, -0.4],
            [0.2, -0.2],
            [0.4, 0.0],
            0.016,
            0.016,
            0.016,
            RadiusProfile::Linear,
            BoundaryType::Valve { lift: 0.0 }, // Exhaust valve 4
            BoundaryType::Closed,
        );

        // Tube 9: Main Exhaust Pipe (Collector -> Atmosphere)
        let tube9 = Tube::new(
            9,
            "Main Exhaust".to_string(),
            30,
            [0.4, 0.0],
            [0.6, 0.0],
            [0.9, 0.0],
            [1.2, 0.0],
            0.022, // 22mm radius
            0.022,
            0.022,
            RadiusProfile::Linear,
            BoundaryType::Closed,
            BoundaryType::Open,
        );

        // --- Junction 0 (Intake Plenum) ---
        let vol_0 = (tube0.area[tube0.num_cells] * tube0.dx +
                     tube1.area[1] * tube1.dx +
                     tube2.area[1] * tube2.dx +
                     tube3.area[1] * tube3.dx +
                     tube4.area[1] * tube4.dx) * 0.5;

        let j0 = Junction {
            id: 0,
            pos: [-0.7, 0.0],
            connections: vec![
                JunctionConnection { tube_id: 0, side: TubeSide::Right },
                JunctionConnection { tube_id: 1, side: TubeSide::Left },
                JunctionConnection { tube_id: 2, side: TubeSide::Left },
                JunctionConnection { tube_id: 3, side: TubeSide::Left },
                JunctionConnection { tube_id: 4, side: TubeSide::Left },
            ],
            rho: RHO_ATM,
            e: P_ATM / (1.4 - 1.0),
            volume: vol_0,
            species: AIR_Y,
            species_temp: AIR_Y,
        };

        // --- Junction 1 (Exhaust Collector) ---
        let vol_1 = (tube5.area[tube5.num_cells] * tube5.dx +
                     tube6.area[tube6.num_cells] * tube6.dx +
                     tube7.area[tube7.num_cells] * tube7.dx +
                     tube8.area[tube8.num_cells] * tube8.dx +
                     tube9.area[1] * tube9.dx) * 0.5;

        let j1 = Junction {
            id: 1,
            pos: [0.4, 0.0],
            connections: vec![
                JunctionConnection { tube_id: 5, side: TubeSide::Right },
                JunctionConnection { tube_id: 6, side: TubeSide::Right },
                JunctionConnection { tube_id: 7, side: TubeSide::Right },
                JunctionConnection { tube_id: 8, side: TubeSide::Right },
                JunctionConnection { tube_id: 9, side: TubeSide::Left },
            ],
            rho: RHO_ATM,
            e: P_ATM / (1.4 - 1.0),
            volume: vol_1,
            species: AIR_Y,
            species_temp: AIR_Y,
        };

        // --- Cylinders (ZX6R typical specs, loaded from config) ---
        let bore = config.engine.bore;
        let stroke = config.engine.stroke;
        let conrod = config.engine.conrod_length;
        let cr = config.engine.compression_ratio;

        // Firing order 1-3-4-2: offsets are 0, 3*PI, PI, 2*PI
        let cyl1 = Cylinder::new(
            0, bore, stroke, conrod, cr,
            vec![
                CylinderConnection { tube_id: 1, side: TubeSide::Right, valve_type: ValveType::Intake },
                CylinderConnection { tube_id: 5, side: TubeSide::Left, valve_type: ValveType::Exhaust },
            ]
        ).with_offset(0.0);

        let cyl2 = Cylinder::new(
            1, bore, stroke, conrod, cr,
            vec![
                CylinderConnection { tube_id: 2, side: TubeSide::Right, valve_type: ValveType::Intake },
                CylinderConnection { tube_id: 6, side: TubeSide::Left, valve_type: ValveType::Exhaust },
            ]
        ).with_offset(3.0 * pi);

        let cyl3 = Cylinder::new(
            2, bore, stroke, conrod, cr,
            vec![
                CylinderConnection { tube_id: 3, side: TubeSide::Right, valve_type: ValveType::Intake },
                CylinderConnection { tube_id: 7, side: TubeSide::Left, valve_type: ValveType::Exhaust },
            ]
        ).with_offset(1.0 * pi);

        let cyl4 = Cylinder::new(
            3, bore, stroke, conrod, cr,
            vec![
                CylinderConnection { tube_id: 4, side: TubeSide::Right, valve_type: ValveType::Intake },
                CylinderConnection { tube_id: 8, side: TubeSide::Left, valve_type: ValveType::Exhaust },
            ]
        ).with_offset(2.0 * pi);

        // Crankshaft with configuration parameters
        let crankshaft = Crankshaft::new(config.engine.inertia, config.engine.viscous_friction);

        let mut ecu = Ecu::new();
        ecu.redline_rpm = config.ecu.redline_rpm;
        ecu.target_idle_rpm = config.ecu.target_idle_rpm;
        ecu.target_afr = config.ecu.target_afr;
        ecu.manual_afr_control = config.ecu.manual_afr_control;

        let mut solver = Self {
            tubes: vec![tube0, tube1, tube2, tube3, tube4, tube5, tube6, tube7, tube8, tube9],
            junctions: vec![j0, j1],
            junctions_temp: Vec::new(),
            limiter: LimiterType::VanLeer,
            friction: config.engine.wall_friction,
            heat_transfer: config.engine.heat_transfer,
            t: 0.0,
            reflection_filter: 0.75,
            step_counter: 0,
            cached_dt_stable: 0.0,
            crankshaft: Some(crankshaft),
            cylinders: vec![cyl1, cyl2, cyl3, cyl4],
            ignition_on: true,
            ignition_timing_deg: config.ecu.ignition_timing_deg,
            target_afr: config.ecu.target_afr,
            throttle_input: 1.0,
            starter_active: false,
            ecu,
            last_cylinder_afr: config.ecu.target_afr,
            last_cylinder_lambda: config.ecu.target_afr / 14.7,
            last_residual_fraction: 0.0,
            last_misfire: false,
            misfire_count: 0,
            misfire_model: crate::combustion::MisfireModel::new(),
            profile: SolverProfile::default(),
            initial_mass_flows: Vec::new(),
        };
        solver.update_connected_flags();
        solver
    }

    pub fn inject_pulse(&mut self, amplitude_pa: f32, width_m: f32) {
        // Inject a pulse into the main tube (index 0)
        if self.tubes.is_empty() { return; }
        
        let gamma = 1.4;
        let main = &mut self.tubes[0];
        let dx = main.dx;
        
        for i in 1..=main.num_cells {
            let x = (i as f32 - 0.5) * dx;
            let p_pulse = amplitude_pa * (-0.5 * (x / width_m).powi(2)).exp();
            
            let mut w = conserved_to_primitive(&main.u[i], gamma);
            w[2] += p_pulse;
            main.u[i] = primitive_to_conserved(&w, gamma);
        }
        
        self.apply_boundary_conditions();
    }

    pub fn step(&mut self, dt: f32) -> f32 {
        let gamma = 1.4;
        
        // 1. Calculate the maximum stable timestep based on CFL = 0.8
        // Recalculate stable timestep on every step to capture transient shock waves (blowdown)
        if true {
            let mut max_speed = 10.0f32;
            let mut min_dx = f32::MAX;
            
            for tube in &self.tubes {
                min_dx = min_dx.min(tube.dx);
                for i in 1..=tube.num_cells {
                    let rho = tube.u[i][0].max(1e-4);
                    let momentum = tube.u[i][1];
                    let energy = tube.u[i][2];
                    let velocity = (momentum / rho).abs();
                    let kinetic = 0.5 * rho * velocity * velocity;
                    let internal = (energy - kinetic).max(1e-4);
                    let p = internal * (gamma - 1.0);
                    let speed_of_sound = (gamma * p / rho).sqrt();
                    max_speed = max_speed.max(velocity + speed_of_sound);
                }
            }
            let cfl = 0.8f32;
            if max_speed.is_nan() || max_speed.is_infinite() || max_speed <= 0.0 {
                max_speed = 340.0;
            }
            self.cached_dt_stable = cfl * (min_dx / max_speed);
        }
        
        self.step_counter += 1;
        let dt_stable = self.cached_dt_stable;
        
        // Determine number of sub-steps
        let n_steps = ((dt / dt_stable).ceil().max(1.0) as usize).min(200);
        //eprintln!("n_steps={} dt={} dt_stable={}", n_steps, dt, dt_stable);
        let sub_dt = dt / n_steps as f32;
        
        // Store starting mass flows for correct derivative calculation over the full dt.
        // Reuses a persistent buffer instead of allocating a new Vec every step().
        self.initial_mass_flows.clear();
        for tube in &self.tubes {
            let left_flow = {
                let w1 = conserved_to_primitive(&tube.u[1], gamma);
                -w1[0] * w1[1] * tube.area[1]
            };
            let right_flow = {
                let wm = conserved_to_primitive(&tube.u[tube.num_cells], gamma);
                wm[0] * wm[1] * tube.area[tube.num_cells]
            };
            self.initial_mass_flows.push((left_flow, right_flow));
        }
        
        // Evolve system by sub-stepping
        for _ in 0..n_steps {
            self.single_substep(sub_dt);
        }
        
        // Compute final fluxes and calculate net Jones dP/dt
        let mut net_jones = 0.0;
        let limiter = self.limiter;
        let friction = self.friction;
        let heat_transfer = self.heat_transfer;
        
        for (k, tube) in self.tubes.iter_mut().enumerate() {
            Self::compute_rhs_internal(limiter, friction, heat_transfer, tube, false);
            
            let connected_left = self.junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube.id && c.side == TubeSide::Left));
            let connected_right = self.junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube.id && c.side == TubeSide::Right));
            let left_open = !connected_left && is_radiating_open_bc(tube.left_bc);
            let right_open = !connected_right && is_radiating_open_bc(tube.right_bc);

            let mut left_flow = 0.0;
            let mut right_flow = 0.0;
            if left_open {
                let w1 = conserved_to_primitive(&tube.u[1], gamma);
                left_flow = -w1[0] * w1[1] * tube.area[1];
            }
            if right_open {
                let wm = conserved_to_primitive(&tube.u[tube.num_cells], gamma);
                right_flow = wm[0] * wm[1] * tube.area[tube.num_cells];
            }

            if left_open && right_open {
                // Net boundary mass flux avoids catastrophic cancellation between endpoints.
                let net_flow = left_flow + right_flow;
                let net_initial = self.initial_mass_flows[k].0 + self.initial_mass_flows[k].1;
                net_jones += (net_flow - net_initial) / dt;
            } else {
                if left_open {
                    net_jones += (left_flow - self.initial_mass_flows[k].0) / dt;
                }
                if right_open {
                    net_jones += (right_flow - self.initial_mass_flows[k].1) / dt;
                }
            }
        }
        
        net_jones
    }

    fn single_substep(&mut self, dt: f32) {
        let gamma = 1.4;
        let num_tubes = self.tubes.len();

        // Only pay for Instant::now()/QueryPerformanceCounter syscalls on a
        // sampled subset of substeps -- profiling every substep (up to 200x
        // per step()) was dominating the profile itself.
        let do_profile = self.step_counter % 64 == 0;

        let mut t_rhs = 0.0;
        let mut t_update = 0.0;
        let mut t_bc = 0.0;
        let mut t_cyl = 0.0;

        let t_start_cyl = if do_profile { Some(std::time::Instant::now()) } else { None };

        // 0. Update ECU State
        if let Some(ref crank) = self.crankshaft {
            let rpm = crank.omega * 60.0 / (2.0 * std::f32::consts::PI);
            let map_pa = if !self.tubes.is_empty() {
                let last_cell = self.tubes[0].num_cells;
                let w = conserved_to_primitive(&self.tubes[0].u[last_cell], gamma);
                w[2]
            } else {
                P_ATM
            };
            self.ecu.update(rpm, map_pa, self.throttle_input, dt);

            // Sync public fields for interface
            self.target_afr = self.ecu.target_afr;
            self.ignition_timing_deg = self.ecu.ignition_timing_deg;

            // Set the intake boundary valve lift
            if !self.tubes.is_empty() {
                let lift = (self.throttle_input + self.ecu.iac_lift).clamp(0.001, 1.0);
                self.tubes[0].left_bc = BoundaryType::Valve { lift };
            }
        }

        // 1. Update Crankshaft and Piston Volumes
        if let Some(ref mut crank) = self.crankshaft {
            let theta_old = crank.theta;
            
            let mut pressure_torque = 0.0;
            for cyl in &self.cylinders {
                let force = (cyl.p - P_ATM) * cyl.piston_area;
                let dy_dtheta = piston_dy_dtheta(crank.theta + cyl.crank_pin_offset_radians, cyl.stroke, cyl.conrod_length);
                pressure_torque += force * dy_dtheta;
            }
            
            // Set starter torque from ECU
            let starter_torque = self.ecu.get_starter_torque(self.starter_active);
            crank.starter_torque = starter_torque;
            // If ECU is no longer cranking, automatically turn off starter active flag
            if !self.ecu.is_cranking {
                self.starter_active = false;
            }

            // Advance crankshaft dynamics
            crank.step(pressure_torque, dt);
            let theta_new = crank.theta;
            
            // Calculate new volumes and volume increments (delta_vol)
            for cyl in &mut self.cylinders {
                let v_old = cyl.volume;
                let v_new = cyl.volume_at(crank.theta + cyl.crank_pin_offset_radians);
                cyl.volume = v_new;
                cyl.delta_vol = v_new - v_old;
            }

            // Fuel Injection Model
            if !self.ecu.fuel_cut {
                for cyl in &self.cylinders {
                    for conn in &cyl.connections {
                        if conn.valve_type == ValveType::Intake {
                            let (open_a, close_a) = (350.0f32.to_radians(), 570.0f32.to_radians());
                            let lift = get_valve_lift(crank.theta + cyl.crank_pin_offset_radians, open_a, close_a);
                            if lift > 0.0 {
                                let tube = &mut self.tubes[conn.tube_id];
                                let m = tube.num_cells;
                                let lambda = self.ecu.target_afr / crate::cfd::species::AFR_STOICH;
                                let cells_to_inject: [usize; 3] = if conn.side == TubeSide::Left {
                                    [1, 2, 3]
                                } else {
                                    [m, m - 1, m - 2]
                                };
                                for cell_idx in cells_to_inject {
                                    let y_o2 = tube.species[cell_idx][crate::cfd::species::I_O2];
                                    let y_fuel_target = y_o2 / (crate::cfd::species::STOICH_O2_PER_FUEL * lambda);
                                    tube.species[cell_idx][crate::cfd::species::I_FUEL] = tube.species[cell_idx][crate::cfd::species::I_FUEL].max(y_fuel_target);
                                    crate::cfd::species::clamp_species(&mut tube.species[cell_idx]);
                                }
                            }
                        }
                    }
                }
            }

            // Ignition check and combustion energy release (Wiebe Function Curve)
            for cyl in &mut self.cylinders {
                if self.ignition_on && !self.ecu.spark_cut {
                    let spark_angle = (4.0 * std::f32::consts::PI - self.ecu.ignition_timing_deg.to_radians()).rem_euclid(4.0 * std::f32::consts::PI);
                    
                    // Check if spark_angle was crossed this step to trigger combustion (using local crank angle with pin offset)
                    let theta_old_cyl = (theta_old + cyl.crank_pin_offset_radians).rem_euclid(4.0 * std::f32::consts::PI);
                    let theta_new_cyl = (theta_new + cyl.crank_pin_offset_radians).rem_euclid(4.0 * std::f32::consts::PI);
                    
                    let diff_target = (spark_angle - theta_old_cyl).rem_euclid(4.0 * std::f32::consts::PI);
                    let diff_step = (theta_new_cyl - theta_old_cyl).rem_euclid(4.0 * std::f32::consts::PI);
                    
                    if diff_target < diff_step {
                        let lambda = crate::combustion::burn::lambda_from_species(cyl.species[crate::cfd::species::I_O2], cyl.species[crate::cfd::species::I_FUEL]);
                        let viable = crate::combustion::burn::combustion_viable(cyl.species[crate::cfd::species::I_O2], cyl.species[crate::cfd::species::I_FUEL], cyl.residual_fraction);
                        let misfire = self.misfire_model.should_misfire(cyl.residual_fraction, lambda);
                        
                        // Sync telemetry fields for interface
                        self.last_cylinder_afr = crate::combustion::burn::afr_from_lambda(lambda);
                        self.last_cylinder_lambda = lambda;
                        self.last_residual_fraction = cyl.residual_fraction;
                        self.last_misfire = !viable || misfire;
                        self.misfire_count = self.misfire_model.misfire_count;
                        
                        if viable && !misfire {
                            cyl.combustion_phase = 0.0;
                            cyl.initial_fuel_fraction = cyl.species[crate::cfd::species::I_FUEL];
                        } else {
                            cyl.combustion_phase = -1.0;
                        }
                    }
                } else {
                    cyl.combustion_phase = -1.0;
                }
                
                // If combustion is active, apply Wiebe heat release
                if cyl.combustion_phase >= 0.0 {
                    let d_theta = (theta_new - theta_old).rem_euclid(4.0 * std::f32::consts::PI);
                    let comb_duration = 50.0f32.to_radians(); // 50 degrees combustion duration
                    
                    let phase_old = cyl.combustion_phase;
                    let phase_new = cyl.combustion_phase + d_theta;
                    cyl.combustion_phase = phase_new;
                    
                    // Wiebe parameters: efficiency exponent a=5.0, form factor m=2.0 (so exponent is m+1=3.0)
                    let xb_old = crate::combustion::burn::wiebe_fraction(phase_old, comb_duration, 5.0, 3);
                    let xb_new = crate::combustion::burn::wiebe_fraction(phase_new, comb_duration, 5.0, 3);
                    
                    let d_xb = (xb_new - xb_old).max(0.0);
                    
                    let q_lhv = 44.0e6; // J/kg
                    let lambda = crate::combustion::burn::lambda_from_species(cyl.species[crate::cfd::species::I_O2], cyl.species[crate::cfd::species::I_FUEL]);
                    let eta = crate::combustion::burn::afr_efficiency(lambda);
                    let cv = R_AIR / (gamma - 1.0);
                    
                    // Energy release uses the actual fuel mass in the cylinder
                    let m_fuel = cyl.mass * cyl.initial_fuel_fraction;
                    let delta_u = d_xb * m_fuel * q_lhv * eta;
                    
                    cyl.energy += delta_u;
                    
                    // Consume species in the cylinder proportionally to d_xb
                    crate::combustion::burn::consume_species(&mut cyl.species, d_xb, cyl.initial_fuel_fraction);
                    
                    // Peak temperature cap at 2800 K
                    let max_temp = 2800.0;
                    let max_energy = cyl.mass * cv * max_temp;
                    if cyl.energy > max_energy {
                        cyl.energy = max_energy;
                    }
                    
                    cyl.update_thermodynamics(gamma);
                    
                    if phase_new >= comb_duration {
                        cyl.combustion_phase = -1.0; // finished
                    }
                }
            }
        }
        if let Some(t) = t_start_cyl { t_cyl += t.elapsed().as_micros() as f32; }

        // ------------------ SSP-RK2 Step 1 ------------------
        let t_start_bc = if do_profile { Some(std::time::Instant::now()) } else { None };
        self.apply_boundary_conditions();
        if let Some(t) = t_start_bc { t_bc += t.elapsed().as_micros() as f32; }
        
        let t_start_rhs = if do_profile { Some(std::time::Instant::now()) } else { None };
        let limiter = self.limiter;
        let friction = self.friction;
        let heat_transfer = self.heat_transfer;

        for tube in &mut self.tubes {
            Self::compute_rhs_internal(limiter, friction, heat_transfer, tube, false);
        }
        if let Some(t) = t_start_rhs { t_rhs += t.elapsed().as_micros() as f32; }

        let t_start_update = if do_profile { Some(std::time::Instant::now()) } else { None };
        for tube in &mut self.tubes {
            for i in 1..=tube.num_cells {
                for c in 0..3 {
                    tube.u_temp[i][c] = tube.u[i][c] + dt * tube.rhs[i][c];
                }
                for k in 0..4 {
                    tube.species_temp[i][k] = (tube.species[i][k] + dt * tube.species_rhs[i][k]).clamp(0.0, 1.0);
                }
            }
        }

        // Update state fields only -- avoids re-cloning each Junction's
        // `connections` Vec (which never changes) every substep. Falls back
        // to a full clone once if lengths don't match yet (e.g. first call).
        if self.junctions_temp.len() != self.junctions.len() {
            self.junctions_temp.clone_from(&self.junctions);
        } else {
            for (dst, src) in self.junctions_temp.iter_mut().zip(self.junctions.iter()) {
                dst.copy_state_from(src);
            }
        }
        for (j_idx, junction) in self.junctions.iter().enumerate() {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            let mut dmass_species = [0.0f32; 4];
            
            for conn in &junction.connections {
                let tube = &self.tubes[conn.tube_id];
                let left_flux = tube.left_boundary_flux;
                let right_flux = tube.right_boundary_flux;
                match conn.side {
                    TubeSide::Left => {
                        let area = tube.area[1];
                        let flow = -area * left_flux[0];
                        dmass += flow;
                        denergy -= area * left_flux[2];
                        for k in 0..4 {
                            let y_upwind = if flow >= 0.0 { tube.species[1][k] } else { junction.species[k] };
                            dmass_species[k] += flow * y_upwind;
                        }
                    }
                    TubeSide::Right => {
                        let area = tube.area[tube.num_cells];
                        let flow = area * right_flux[0];
                        dmass += flow;
                        denergy += area * right_flux[2];
                        for k in 0..4 {
                            let y_upwind = if flow >= 0.0 { tube.species[tube.num_cells][k] } else { junction.species[k] };
                            dmass_species[k] += flow * y_upwind;
                        }
                    }
                }
            }

            let new_rho = (junction.rho + (dt / junction.volume) * dmass).max(1e-4);
            self.junctions_temp[j_idx].rho = new_rho;
            self.junctions_temp[j_idx].e = (junction.e + (dt / junction.volume) * denergy).max(1e-2);
            
            let junction_mass_old = junction.rho * junction.volume;
            let junction_mass_new = new_rho * junction.volume;
            for k in 0..4 {
                let m_species_old = junction_mass_old * junction.species[k];
                let m_species_new = (m_species_old + dt * dmass_species[k]).max(0.0);
                self.junctions_temp[j_idx].species_temp[k] = m_species_new / junction_mass_new.max(1e-6);
            }
            crate::cfd::species::clamp_species(&mut self.junctions_temp[j_idx].species_temp);
        }

        // Step 1: Update Cylinder Temp States
        for cyl in &mut self.cylinders {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            let mut dmass_species = [0.0f32; 4];
            
            for conn in &cyl.connections {
                let tube = &self.tubes[conn.tube_id];
                let area = if conn.side == TubeSide::Left { tube.area[1] } else { tube.area[tube.num_cells] };
                let flux = if conn.side == TubeSide::Left { tube.left_boundary_flux } else { tube.right_boundary_flux };
                
                let flow = match conn.side {
                    TubeSide::Left => -area * flux[0],
                    TubeSide::Right => area * flux[0],
                };
                dmass += flow;
                denergy += match conn.side {
                    TubeSide::Left => -area * flux[2],
                    TubeSide::Right => area * flux[2],
                };
                
                for k in 0..4 {
                    let y_upwind = if flow >= 0.0 {
                        let boundary_cell = if conn.side == TubeSide::Left { 1 } else { tube.num_cells };
                        tube.species[boundary_cell][k]
                    } else {
                        cyl.species[k]
                    };
                    dmass_species[k] += flow * y_upwind;
                }
            }
            
            let v_new = cyl.volume;
            let v_old = (v_new - cyl.delta_vol).max(1e-6);
            let v_ratio = (v_old / v_new).powf(gamma - 1.0);
            
            let max_drain_mass = 0.5 * cyl.mass;
            let mass_change = (dt * dmass).max(-max_drain_mass);
            let cyl_mass_new = (cyl.mass + mass_change).max(1e-6);
            cyl.mass_temp = cyl_mass_new;
            
            let max_drain_energy = 0.5 * cyl.energy;
            let energy_change = (dt * denergy).max(-max_drain_energy);
            cyl.energy_temp = (cyl.energy * v_ratio + energy_change).max(1e-4);
            cyl.update_thermodynamics_temp(gamma);
            
            for k in 0..4 {
                let m_species_old = cyl.mass * cyl.species[k];
                let m_species_new = (m_species_old + dt * dmass_species[k]).max(0.0);
                cyl.species_temp[k] = m_species_new / cyl_mass_new.max(1e-6);
            }
            crate::cfd::species::clamp_species(&mut cyl.species_temp);
        }
        if let Some(t) = t_start_update { t_update += t.elapsed().as_micros() as f32; }

        // ------------------ SSP-RK2 Step 2 ------------------
        let t_start_bc = if do_profile { Some(std::time::Instant::now()) } else { None };
        let filter_strength = self.reflection_filter;
        let crank_theta = self.crankshaft.as_ref().map(|c| c.theta);
        Self::apply_network_bc_cylinders(
            &mut self.tubes,
            &self.junctions,
            true,
            &self.junctions_temp,
            &self.cylinders,
            crank_theta,
            gamma,
            filter_strength,
        );
        if let Some(t) = t_start_bc { t_bc += t.elapsed().as_micros() as f32; }
        
        let t_start_rhs = if do_profile { Some(std::time::Instant::now()) } else { None };
        for tube in &mut self.tubes {
            Self::compute_rhs_internal(limiter, friction, heat_transfer, tube, true);
        }
        if let Some(t) = t_start_rhs { t_rhs += t.elapsed().as_micros() as f32; }

        let t_start_update = if do_profile { Some(std::time::Instant::now()) } else { None };
        for tube in &mut self.tubes {
            for i in 1..=tube.num_cells {
                for c in 0..3 {
                    tube.u[i][c] = 0.5 * tube.u[i][c] + 0.5 * tube.u_temp[i][c] + 0.5 * dt * tube.rhs[i][c];
                }
                for k in 0..4 {
                    tube.species[i][k] = (0.5 * tube.species[i][k] + 0.5 * tube.species_temp[i][k] + 0.5 * dt * tube.species_rhs[i][k]).clamp(0.0, 1.0);
                }
            }
            
            // Safety clamp
            for i in 1..=tube.num_cells {
                tube.u[i][0] = tube.u[i][0].max(1e-4);
                let rho = tube.u[i][0];
                let v = tube.u[i][1] / rho;
                let ke = 0.5 * rho * v * v;
                let mut e_tot = tube.u[i][2];
                if e_tot - ke < 1e-4 {
                    let p_min = 1e-2;
                    e_tot = p_min / (gamma - 1.0) + ke;
                    tube.u[i][2] = e_tot;
                }
                
                if tube.u[i][0].is_nan() || tube.u[i][1].is_nan() || tube.u[i][2].is_nan() {
                    let w_atm = [RHO_ATM, 0.0, P_ATM];
                    tube.u[i] = primitive_to_conserved(&w_atm, gamma);
                }
            }
        }

        for (j_idx, junction) in self.junctions.iter_mut().enumerate() {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            let mut dmass_species = [0.0f32; 4];
            
            for conn in &junction.connections {
                let tube = &self.tubes[conn.tube_id];
                let left_flux = tube.left_boundary_flux;
                let right_flux = tube.right_boundary_flux;
                match conn.side {
                    TubeSide::Left => {
                        let area = tube.area[1];
                        let flow = -area * left_flux[0];
                        dmass += flow;
                        denergy -= area * left_flux[2];
                        for k in 0..4 {
                            let y_upwind = if flow >= 0.0 { tube.species_temp[1][k] } else { self.junctions_temp[j_idx].species_temp[k] };
                            dmass_species[k] += flow * y_upwind;
                        }
                    }
                    TubeSide::Right => {
                        let area = tube.area[tube.num_cells];
                        let flow = area * right_flux[0];
                        dmass += flow;
                        denergy += area * right_flux[2];
                        for k in 0..4 {
                            let y_upwind = if flow >= 0.0 { tube.species_temp[tube.num_cells][k] } else { self.junctions_temp[j_idx].species_temp[k] };
                            dmass_species[k] += flow * y_upwind;
                        }
                    }
                }
            }

            let new_rho = (0.5 * junction.rho + 0.5 * self.junctions_temp[j_idx].rho + 0.5 * (dt / junction.volume) * dmass).max(1e-4);
            junction.rho = new_rho;
            junction.e = (0.5 * junction.e + 0.5 * self.junctions_temp[j_idx].e + 0.5 * (dt / junction.volume) * denergy).max(1e-2);
            
            let junction_mass = new_rho * junction.volume;
            let junction_mass_old = junction.rho * junction.volume;
            let junction_mass_temp = self.junctions_temp[j_idx].rho * junction.volume;
            for k in 0..4 {
                let m_species_old = junction_mass_old * junction.species[k];
                let m_species_temp = junction_mass_temp * self.junctions_temp[j_idx].species_temp[k];
                let m_species_new = (0.5 * m_species_old + 0.5 * m_species_temp + 0.5 * dt * dmass_species[k]).max(0.0);
                junction.species[k] = m_species_new / junction_mass.max(1e-6);
            }
            crate::cfd::species::clamp_species(&mut junction.species);
        }

        // Step 2: Update Cylinder Final States
        for cyl in &mut self.cylinders {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            let mut dmass_species = [0.0f32; 4];
            
            for conn in &cyl.connections {
                let tube = &self.tubes[conn.tube_id];
                let area = if conn.side == TubeSide::Left { tube.area[1] } else { tube.area[tube.num_cells] };
                let flux = if conn.side == TubeSide::Left { tube.left_boundary_flux } else { tube.right_boundary_flux };
                
                let flow = match conn.side {
                    TubeSide::Left => -area * flux[0],
                    TubeSide::Right => area * flux[0],
                };
                dmass += flow;
                denergy += match conn.side {
                    TubeSide::Left => -area * flux[2],
                    TubeSide::Right => area * flux[2],
                };
                
                for k in 0..4 {
                    let y_upwind = if flow >= 0.0 {
                        let boundary_cell = if conn.side == TubeSide::Left { 1 } else { tube.num_cells };
                        tube.species_temp[boundary_cell][k]
                    } else {
                        cyl.species_temp[k]
                    };
                    dmass_species[k] += flow * y_upwind;
                }
            }
            
            let v_new = cyl.volume;
            let v_old = (v_new - cyl.delta_vol).max(1e-6);
            let v_ratio = (v_old / v_new).powf(gamma - 1.0);
            
            let max_drain_mass = 0.5 * cyl.mass;
            let mass_change = (0.5 * dt * dmass).max(-max_drain_mass);
            let cyl_mass_new = (0.5 * cyl.mass + 0.5 * cyl.mass_temp + mass_change).max(1e-6);
            cyl.mass = cyl_mass_new;
            
            let max_drain_energy = 0.5 * cyl.energy;
            let energy_change = (0.5 * dt * denergy).max(-max_drain_energy);
            cyl.energy = (0.5 * cyl.energy * v_ratio + 0.5 * cyl.energy_temp + energy_change).max(1e-4);
            cyl.update_thermodynamics(gamma);
            
            let cyl_mass_old = cyl.mass;
            let cyl_mass_temp = cyl.mass_temp;
            for k in 0..4 {
                let m_species_old = cyl_mass_old * cyl.species[k];
                let m_species_temp = cyl_mass_temp * cyl.species_temp[k];
                let m_species_new = (0.5 * m_species_old + 0.5 * m_species_temp + 0.5 * dt * dmass_species[k]).max(0.0);
                cyl.species[k] = m_species_new / cyl_mass_new.max(1e-6);
            }
            crate::cfd::species::clamp_species(&mut cyl.species);
            
            // Derive residual fraction: Y_CO2 + Y_EXHAUST
            cyl.residual_fraction = cyl.species[crate::cfd::species::I_CO2] + cyl.species[crate::cfd::species::I_EXHAUST];
        }
        if let Some(t) = t_start_update { t_update += t.elapsed().as_micros() as f32; }

        let t_start_bc = if do_profile { Some(std::time::Instant::now()) } else { None };
        self.apply_boundary_conditions();
        if let Some(t) = t_start_bc { t_bc += t.elapsed().as_micros() as f32; }

        let alpha = 0.05;
        if do_profile {
            self.profile.tubes_rhs_us = self.profile.tubes_rhs_us * (1.0 - alpha) + alpha * t_rhs;
            self.profile.tubes_update_us = self.profile.tubes_update_us * (1.0 - alpha) + alpha * t_update;
            self.profile.boundary_conditions_us = self.profile.boundary_conditions_us * (1.0 - alpha) + alpha * t_bc;
            self.profile.cylinder_physics_us = self.profile.cylinder_physics_us * (1.0 - alpha) + alpha * t_cyl;
        }
    }

    pub fn apply_boundary_conditions(&mut self) {
        let gamma = 1.4;
        let filter_strength = self.reflection_filter;
        let crank_theta = self.crankshaft.as_ref().map(|c| c.theta);
        Self::apply_network_bc_cylinders(
            &mut self.tubes,
            &self.junctions,
            false,
            &self.junctions,
            &self.cylinders,
            crank_theta,
            gamma,
            filter_strength,
        );
    }

    fn open_left_ghost(
        w1: [f32; 3],
        gamma: f32,
        c_res: f32,
        filter_strength: f32,
        lift: f32,
        last_r_out: f32,
    ) -> ([f32; 3], f32) {
        let w_atm = [RHO_ATM, 0.0, P_ATM];
        let r_atm = atmospheric_r_plus(gamma);
        let activity = open_bc_activity(&w1);
        if activity <= 0.0 {
            return (w_atm, r_atm);
        }

        let r_minus = w1[1] - 2.0 * (gamma * w1[2] / w1[0]).sqrt() / (gamma - 1.0);
        let r_in = atmospheric_r_plus(gamma);
        let c1 = (gamma * w1[2] / w1[0]).sqrt();
        let v_out = Self::solve_bisection_valve(r_in, w1[2], w1[0], c1, lift, c_res);
        let c_bc = 0.2 * (r_in - v_out);
        let r_plus_reflected = (-v_out) + 2.0 * c_bc / (gamma - 1.0);
        let alpha = if (lift - 1.0).abs() < 1e-4 {
            filter_strength
        } else {
            1.0 - (1.0 - filter_strength) * lift
        };
        let r_out_filtered = alpha * r_plus_reflected + (1.0 - alpha) * last_r_out;
        let w_benson = ghost_from_riemann(r_out_filtered, r_minus, gamma, c_res);

        if activity >= 1.0 {
            (w_benson, r_out_filtered)
        } else {
            (
                lerp_primitive(w_atm, w_benson, activity),
                r_atm * (1.0 - activity) + r_out_filtered * activity,
            )
        }
    }

    fn open_right_ghost(
        wm: [f32; 3],
        gamma: f32,
        c_res: f32,
        filter_strength: f32,
        lift: f32,
        last_r_out: f32,
    ) -> ([f32; 3], f32) {
        let w_atm = [RHO_ATM, 0.0, P_ATM];
        let r_atm = atmospheric_r_minus(gamma);
        let activity = open_bc_activity(&wm);
        if activity <= 0.0 {
            return (w_atm, r_atm);
        }

        let r_plus = wm[1] + 2.0 * (gamma * wm[2] / wm[0]).sqrt() / (gamma - 1.0);
        let r_in = atmospheric_r_plus(gamma);
        let cm = (gamma * wm[2] / wm[0]).sqrt();
        let v_out = Self::solve_bisection_valve(r_in, wm[2], wm[0], cm, lift, c_res);
        let c_bc = 0.2 * (r_in - v_out);
        let r_minus_reflected = v_out - 2.0 * c_bc / (gamma - 1.0);
        let alpha = if (lift - 1.0).abs() < 1e-4 {
            filter_strength
        } else {
            1.0 - (1.0 - filter_strength) * lift
        };
        let r_out_filtered = alpha * r_minus_reflected + (1.0 - alpha) * last_r_out;
        let w_benson = ghost_from_riemann(r_plus, r_out_filtered, gamma, c_res);

        if activity >= 1.0 {
            (w_benson, r_out_filtered)
        } else {
            (
                lerp_primitive(w_atm, w_benson, activity),
                r_atm * (1.0 - activity) + r_out_filtered * activity,
            )
        }
    }

    fn valve_left_ghost(
        w1: [f32; 3],
        gamma: f32,
        p_res: f32,
        rho_res: f32,
        c_res: f32,
        filter_strength: f32,
        lift: f32,
        last_r_out: f32,
    ) -> ([f32; 3], f32) {
        let w_res = [rho_res, 0.0, p_res];
        let r_res = 2.0 * c_res / (gamma - 1.0);
        let activity = open_bc_activity(&w1);
        if activity <= 0.0 {
            return (w_res, r_res);
        }

        let r_minus = w1[1] - 2.0 * (gamma * w1[2] / w1[0]).sqrt() / (gamma - 1.0);
        let r_in = r_res;
        let c1 = (gamma * w1[2] / w1[0]).sqrt();
        let v_out = Self::solve_bisection_valve_res(r_in, w1[2], w1[0], c1, lift, c_res, p_res, rho_res);
        let c_bc = 0.2 * (r_in - v_out);
        let r_plus_reflected = (-v_out) + 2.0 * c_bc / (gamma - 1.0);
        let alpha = if (lift - 1.0).abs() < 1e-4 {
            filter_strength
        } else {
            1.0 - (1.0 - filter_strength) * lift
        };
        let r_out_filtered = alpha * r_plus_reflected + (1.0 - alpha) * last_r_out;
        let w_benson = ghost_from_riemann_res(r_out_filtered, r_minus, gamma, c_res, p_res, rho_res);

        if activity >= 1.0 {
            (w_benson, r_out_filtered)
        } else {
            (
                lerp_primitive(w_res, w_benson, activity),
                r_res * (1.0 - activity) + r_out_filtered * activity,
            )
        }
    }

    fn valve_right_ghost(
        wm: [f32; 3],
        gamma: f32,
        p_res: f32,
        rho_res: f32,
        c_res: f32,
        filter_strength: f32,
        lift: f32,
        last_r_out: f32,
    ) -> ([f32; 3], f32) {
        let w_res = [rho_res, 0.0, p_res];
        let r_res = -2.0 * c_res / (gamma - 1.0);
        let activity = open_bc_activity(&wm);
        if activity <= 0.0 {
            return (w_res, r_res);
        }

        let r_plus = wm[1] + 2.0 * (gamma * wm[2] / wm[0]).sqrt() / (gamma - 1.0);
        let r_in = 2.0 * c_res / (gamma - 1.0);
        let cm = (gamma * wm[2] / wm[0]).sqrt();
        let v_out = Self::solve_bisection_valve_res(r_in, wm[2], wm[0], cm, lift, c_res, p_res, rho_res);
        let c_bc = 0.2 * (r_in - v_out);
        let r_minus_reflected = v_out - 2.0 * c_bc / (gamma - 1.0);
        let alpha = if (lift - 1.0).abs() < 1e-4 {
            filter_strength
        } else {
            1.0 - (1.0 - filter_strength) * lift
        };
        let r_out_filtered = alpha * r_minus_reflected + (1.0 - alpha) * last_r_out;
        let w_benson = ghost_from_riemann_res(r_plus, r_out_filtered, gamma, c_res, p_res, rho_res);

        if activity >= 1.0 {
            (w_benson, r_out_filtered)
        } else {
            (
                lerp_primitive(w_res, w_benson, activity),
                r_res * (1.0 - activity) + r_out_filtered * activity,
            )
        }
    }

    #[inline(always)]
    fn isentropic_phi(pr: f32) -> f32 {
        if pr <= 0.52828 {
            0.5787
        } else {
            let poly = 0.174087330145
                     + pr * (1.845079102463
                     + pr * (-1.619439906917
                     + pr * (1.268381247447
                     + pr * (-0.593403052591
                     + pr * 0.120537101468))));
            (1.0 - pr).max(0.0).sqrt() * poly
        }
    }

    fn solve_bisection_valve_res(
        r_in: f32,
        p_interior: f32,
        _rho_interior: f32,
        c_interior: f32,
        lift: f32,
        c_res: f32,
        p_res: f32,
        rho_res: f32,
    ) -> f32 {
        let psi = lift.max(1e-4);
        let c_int = c_interior.max(1e-2);

        // Hoisted out of the per-iteration closure: these don't depend on
        // v_out, so computing them once per solve instead of once per
        // eval_residual call saves a division + 7th/5th power every iteration.
        let inv_c_int7 = 1.0 / {
            let c2 = c_int * c_int;
            let c4 = c2 * c2;
            let c6 = c4 * c2;
            c6 * c_int
        };
        let inv_c_res5 = {
            let c2 = c_res * c_res;
            let c4 = c2 * c2;
            1.0 / (c4 * c_res)
        };

        let eval_residual = |v_out: f32| -> f32 {
            let c_bc = 0.2 * (r_in - v_out);
            if c_bc <= 0.0 {
                return 1e6;
            }

            let c_bc2 = c_bc * c_bc;
            let c_bc4 = c_bc2 * c_bc2;
            let c_bc6 = c_bc4 * c_bc2;
            let c_bc7 = c_bc6 * c_bc;
            let p_bc = p_interior * c_bc7 * inv_c_int7;

            if p_bc >= p_res - 25.0 {
                // Outflow
                let pr = (p_res / p_bc.max(1e-3)).clamp(0.0, 1.0);
                let phi = Self::isentropic_phi(pr);
                let target_v = psi * c_bc * phi;
                v_out - target_v
            } else {
                // Inflow
                let pr = (p_bc / p_res.max(1e-3)).clamp(0.0, 1.0);
                let phi = Self::isentropic_phi(pr);
                let c_bc5 = c_bc4 * c_bc;
                let rho_bc = rho_res * c_bc5 * inv_c_res5;
                let g = psi * rho_res * c_res * phi;
                let target_v = -g / rho_bc.max(1e-3);
                v_out - target_v
            }
        };

        let mut low = -c_res - 200.0;
        let mut high = (r_in - 1e-3).max(low + 10.0);

        let mut r_low = eval_residual(low);
        let mut r_high = eval_residual(high);

        if r_low * r_high > 0.0 {
            if r_low > 0.0 {
                for _ in 0..15 {
                    low -= 300.0;
                    r_low = eval_residual(low);
                    if r_low < 0.0 {
                        break;
                    }
                }
            } else {
                for _ in 0..15 {
                    high += 300.0;
                    if high >= r_in {
                        high = r_in - 1e-5;
                        break;
                    }
                    r_high = eval_residual(high);
                    if r_high > 0.0 {
                        break;
                    }
                }
            }
        }

        let mut result = 0.5 * (low + high);
        let mut side = 0i32;
        for _ in 0..12 {
            let denom = r_high - r_low;
            if denom.abs() < 1e-6 {
                break;
            }
            let mid = (low * r_high - high * r_low) / denom;
            let r_mid = eval_residual(mid);
            result = mid;

            if r_mid.abs() < 1e-3 {
                break;
            }

            if r_low * r_mid > 0.0 {
                low = mid;
                r_low = r_mid;
                if side == -1 {
                    r_high *= 0.5;
                }
                side = -1;
            } else {
                high = mid;
                r_high = r_mid;
                if side == 1 {
                    r_low *= 0.5;
                }
                side = 1;
            }
        }

        result
    }

    #[allow(non_snake_case)]
    fn apply_network_bc_cylinders(
        tubes: &mut Vec<Tube>,
        junctions: &Vec<Junction>,
        use_temp: bool,
        j_states: &Vec<Junction>,
        cyl_states: &Vec<Cylinder>,
        crank_theta: Option<f32>,
        gamma: f32,
        filter_strength: f32,
    ) {
        // First call the standard network BC to set junctions and standard boundaries
        Self::apply_network_bc(tubes, junctions, use_temp, j_states, gamma, filter_strength);
        
        // If we have cylinders, apply valve boundary conditions to overwrite standard BCs on connected ends
        for cyl in cyl_states {
            let p_res = if use_temp { cyl.p_temp } else { cyl.p };
            let rho_res = if use_temp { cyl.rho_temp } else { cyl.rho };
            let c_res = (gamma * p_res / rho_res.max(1e-4)).sqrt();
            
            for conn in &cyl.connections {
                let tube = &mut tubes[conn.tube_id];
                let m = tube.num_cells;
                let lift = if let Some(theta) = crank_theta {
                    let (open_a, close_a) = match conn.valve_type {
                        ValveType::Intake => (350.0f32.to_radians(), 570.0f32.to_radians()),
                        ValveType::Exhaust => (140.0f32.to_radians(), 380.0f32.to_radians()),
                    };
                    get_valve_lift(theta + cyl.crank_pin_offset_radians, open_a, close_a)
                } else {
                    // Default to fully open if no crank is present (e.g. static tests)
                    1.0
                };
                
                let tube_filter = filter_strength;
                
                match conn.side {
                    TubeSide::Left => {
                        let w1 = {
                            let state = if use_temp { &tube.u_temp } else { &tube.u };
                            conserved_to_primitive(&state[1], gamma)
                        };
                        
                        let w0;
                        let new_r_out;
                        
                        if lift < 1e-6 {
                            w0 = [w1[0], -w1[1], w1[2]];
                            let c1 = (gamma * w1[2] / w1[0]).sqrt();
                            let r_wall = -w1[1] + 2.0 * c1 / (gamma - 1.0);
                            new_r_out = Some(r_wall);
                        } else {
                            let (ghost, r_out) = Self::valve_left_ghost(
                                w1,
                                gamma,
                                p_res,
                                rho_res,
                                c_res,
                                tube_filter,
                                lift,
                                tube.left_last_r_out,
                            );
                            
                            if lift < 0.02 {
                                let w_wall = [w1[0], -w1[1], w1[2]];
                                let alpha = (lift - 1e-6) / (0.02 - 1e-6);
                                let alpha = alpha.clamp(0.0, 1.0);
                                w0 = lerp_primitive(w_wall, ghost, alpha);
                                
                                let c1 = (gamma * w1[2] / w1[0]).sqrt();
                                let r_wall = -w1[1] + 2.0 * c1 / (gamma - 1.0);
                                new_r_out = Some(alpha * r_out + (1.0 - alpha) * r_wall);
                            } else {
                                w0 = ghost;
                                new_r_out = Some(r_out);
                            }
                        }
                        
                        let state = if use_temp { &mut tube.u_temp } else { &mut tube.u };
                        state[0] = primitive_to_conserved(&w0, gamma);
                        if let Some(r) = new_r_out {
                            tube.left_last_r_out = r;
                        }
                        
                        let species_state = if use_temp { &mut tube.species_temp } else { &mut tube.species };
                        species_state[0] = if use_temp { cyl.species_temp } else { cyl.species };
                    }
                    TubeSide::Right => {
                        let wm = {
                            let state = if use_temp { &tube.u_temp } else { &tube.u };
                            conserved_to_primitive(&state[m], gamma)
                        };
                        
                        let w_m1;
                        let new_r_out;
                        
                        if lift < 1e-6 {
                            w_m1 = [wm[0], -wm[1], wm[2]];
                            let cm = (gamma * wm[2] / wm[0]).sqrt();
                            let r_wall = wm[1] - 2.0 * cm / (gamma - 1.0);
                            new_r_out = Some(r_wall);
                        } else {
                            let (ghost, r_out) = Self::valve_right_ghost(
                                wm,
                                gamma,
                                p_res,
                                rho_res,
                                c_res,
                                tube_filter,
                                lift,
                                tube.right_last_r_out,
                            );
                            
                            if lift < 0.02 {
                                let w_wall = [wm[0], -wm[1], wm[2]];
                                let alpha = (lift - 1e-6) / (0.02 - 1e-6);
                                let alpha = alpha.clamp(0.0, 1.0);
                                w_m1 = lerp_primitive(w_wall, ghost, alpha);
                                
                                let cm = (gamma * wm[2] / wm[0]).sqrt();
                                let r_wall = wm[1] - 2.0 * cm / (gamma - 1.0);
                                new_r_out = Some(alpha * r_out + (1.0 - alpha) * r_wall);
                            } else {
                                w_m1 = ghost;
                                new_r_out = Some(r_out);
                            }
                        }
                        
                        let state = if use_temp { &mut tube.u_temp } else { &mut tube.u };
                        state[m + 1] = primitive_to_conserved(&w_m1, gamma);
                        if let Some(r) = new_r_out {
                            tube.right_last_r_out = r;
                        }
                        
                        let species_state = if use_temp { &mut tube.species_temp } else { &mut tube.species };
                        species_state[m + 1] = if use_temp { cyl.species_temp } else { cyl.species };
                    }
                }
            }
        }
    }

    #[allow(non_snake_case)]
    fn apply_network_bc(
        tubes: &mut Vec<Tube>,
        _junctions: &Vec<Junction>,
        use_temp: bool,
        j_states: &Vec<Junction>,
        gamma: f32,
        filter_strength: f32,
    ) {
        let c_res = atmospheric_sound_speed(gamma);
        
        for k in 0..tubes.len() {
            let m = tubes[k].num_cells;
            let left_bc = tubes[k].left_bc;
            let right_bc = tubes[k].right_bc;
            let connected_left = tubes[k].connected_left;
            let connected_right = tubes[k].connected_right;
            let left_radiating = !connected_left && is_radiating_open_bc(left_bc);
            let right_radiating = !connected_right && is_radiating_open_bc(right_bc);
            // Dual-open ends form a resonant cavity; absorb reflections more aggressively at idle.
            let tube_filter = if left_radiating && right_radiating {
                filter_strength.max(0.94)
            } else {
                filter_strength
            };
            
            // 1. Check Left End
            if !connected_left {
                let w1 = {
                    let state = if use_temp { &tubes[k].u_temp } else { &tubes[k].u };
                    conserved_to_primitive(&state[1], gamma)
                };
                
                let w0;
                let new_r_out;
                
                match left_bc {
                    BoundaryType::Closed => {
                        w0 = [w1[0], -w1[1], w1[2]];
                        let c1 = (gamma * w1[2] / w1[0]).sqrt();
                        let r_wall = -w1[1] + 2.0 * c1 / (gamma - 1.0);
                        new_r_out = Some(r_wall);
                    }
                    BoundaryType::Open => {
                        let (ghost, r_out) = Self::open_left_ghost(w1, gamma, c_res, tube_filter, 1.0, tubes[k].left_last_r_out);
                        w0 = ghost;
                        new_r_out = Some(r_out);
                    }
                    BoundaryType::Valve { lift } => {
                        if lift < 1e-4 {
                            w0 = [w1[0], -w1[1], w1[2]];
                            let c1 = (gamma * w1[2] / w1[0]).sqrt();
                            let r_wall = -w1[1] + 2.0 * c1 / (gamma - 1.0);
                            new_r_out = Some(r_wall);
                        } else {
                            let (ghost, r_out) = Self::open_left_ghost(w1, gamma, c_res, tube_filter, lift, tubes[k].left_last_r_out);
                            if lift < 0.04 {
                                let w_wall = [w1[0], -w1[1], w1[2]];
                                let alpha = (lift - 1e-4) / (0.04 - 1e-4);
                                let alpha = alpha.clamp(0.0, 1.0);
                                w0 = lerp_primitive(w_wall, ghost, alpha);
                                
                                let c1 = (gamma * w1[2] / w1[0]).sqrt();
                                let r_wall = -w1[1] + 2.0 * c1 / (gamma - 1.0);
                                new_r_out = Some(alpha * r_out + (1.0 - alpha) * r_wall);
                            } else {
                                w0 = ghost;
                                new_r_out = Some(r_out);
                            }
                        }
                    }
                }
                
                let state = if use_temp { &mut tubes[k].u_temp } else { &mut tubes[k].u };
                state[0] = primitive_to_conserved(&w0, gamma);
                if let Some(r) = new_r_out {
                    tubes[k].left_last_r_out = r;
                }
                
                let species_state = if use_temp { &mut tubes[k].species_temp } else { &mut tubes[k].species };
                match left_bc {
                    BoundaryType::Closed => {
                        species_state[0] = species_state[1];
                    }
                    _ => {
                        if w0[1] >= 0.0 {
                            species_state[0] = AIR_Y;
                        } else {
                            species_state[0] = species_state[1];
                        }
                    }
                }
            }
            
            // 2. Check Right End
            if !connected_right {
                let wm = {
                    let state = if use_temp { &tubes[k].u_temp } else { &tubes[k].u };
                    conserved_to_primitive(&state[m], gamma)
                };
                
                let w_m1;
                let new_r_out;
                
                match right_bc {
                    BoundaryType::Closed => {
                        w_m1 = [wm[0], -wm[1], wm[2]];
                        let cm = (gamma * wm[2] / wm[0]).sqrt();
                        let r_wall = wm[1] - 2.0 * cm / (gamma - 1.0);
                        new_r_out = Some(r_wall);
                    }
                    BoundaryType::Open => {
                        let (ghost, r_out) = Self::open_right_ghost(wm, gamma, c_res, tube_filter, 1.0, tubes[k].right_last_r_out);
                        w_m1 = ghost;
                        new_r_out = Some(r_out);
                    }
                    BoundaryType::Valve { lift } => {
                        if lift < 1e-4 {
                            w_m1 = [wm[0], -wm[1], wm[2]];
                            let cm = (gamma * wm[2] / wm[0]).sqrt();
                            let r_wall = wm[1] - 2.0 * cm / (gamma - 1.0);
                            new_r_out = Some(r_wall);
                        } else {
                            let (ghost, r_out) = Self::open_right_ghost(wm, gamma, c_res, tube_filter, lift, tubes[k].right_last_r_out);
                            if lift < 0.04 {
                                let w_wall = [wm[0], -wm[1], wm[2]];
                                let alpha = (lift - 1e-4) / (0.04 - 1e-4);
                                let alpha = alpha.clamp(0.0, 1.0);
                                w_m1 = lerp_primitive(w_wall, ghost, alpha);
                                
                                let cm = (gamma * wm[2] / wm[0]).sqrt();
                                let r_wall = wm[1] - 2.0 * cm / (gamma - 1.0);
                                new_r_out = Some(alpha * r_out + (1.0 - alpha) * r_wall);
                            } else {
                                w_m1 = ghost;
                                new_r_out = Some(r_out);
                            }
                        }
                    }
                }
                
                let state = if use_temp { &mut tubes[k].u_temp } else { &mut tubes[k].u };
                state[m + 1] = primitive_to_conserved(&w_m1, gamma);
                if let Some(r) = new_r_out {
                    tubes[k].right_last_r_out = r;
                }
                
                let species_state = if use_temp { &mut tubes[k].species_temp } else { &mut tubes[k].species };
                match right_bc {
                    BoundaryType::Closed => {
                        species_state[m + 1] = species_state[m];
                    }
                    _ => {
                        if w_m1[1] <= 0.0 {
                            species_state[m + 1] = AIR_Y;
                        } else {
                            species_state[m + 1] = species_state[m];
                        }
                    }
                }
            }
        }
        
        // Apply constant-pressure junction boundary values
        for j in j_states {
            let p_junction = j.e * (gamma - 1.0);
            let rho_junction = j.rho;
            let species_j = if use_temp { j.species_temp } else { j.species };
            
            for conn in &j.connections {
                let tube = &mut tubes[conn.tube_id];
                let num_cells = tube.num_cells;
                let state = if use_temp { &mut tube.u_temp } else { &mut tube.u };
                let species_state = if use_temp { &mut tube.species_temp } else { &mut tube.species };
                
                match conn.side {
                    TubeSide::Left => {
                        let w1 = conserved_to_primitive(&state[1], gamma);
                        let w0 = [rho_junction, w1[1], p_junction];
                        state[0] = primitive_to_conserved(&w0, gamma);
                        species_state[0] = species_j;
                    }
                    TubeSide::Right => {
                        let wm = conserved_to_primitive(&state[num_cells], gamma);
                        let w_m1 = [rho_junction, wm[1], p_junction];
                        state[num_cells + 1] = primitive_to_conserved(&w_m1, gamma);
                        species_state[num_cells + 1] = species_j;
                    }
                }
            }
        }
    }

    fn solve_bisection_valve(
        r_in: f32,
        p_interior: f32,
        _rho_interior: f32,
        c_interior: f32,
        lift: f32,
        c_res: f32,
    ) -> f32 {
        let psi = lift.max(1e-4);
        let c_int = c_interior.max(1e-2);

        let inv_c_int7 = 1.0 / {
            let c2 = c_int * c_int;
            let c4 = c2 * c2;
            let c6 = c4 * c2;
            c6 * c_int
        };
        let inv_c_res5 = {
            let c2 = c_res * c_res;
            let c4 = c2 * c2;
            1.0 / (c4 * c_res)
        };

        let eval_residual = |v_out: f32| -> f32 {
            let c_bc = 0.2 * (r_in - v_out);
            if c_bc <= 0.0 {
                return 1e6; // Penalty for non-physical negative sound speeds
            }

            let c_bc2 = c_bc * c_bc;
            let c_bc4 = c_bc2 * c_bc2;
            let c_bc6 = c_bc4 * c_bc2;
            let c_bc7 = c_bc6 * c_bc;
            let p_bc = p_interior * c_bc7 * inv_c_int7;

            // Small hysteresis band avoids inflow/outflow branch chatter near atmospheric pressure.
            if p_bc >= P_ATM - 25.0 {
                // Outflow: gas leaves pipe, expands into reservoir
                let pr = (P_ATM / p_bc.max(1e-3)).clamp(0.0, 1.0);
                let phi = Self::isentropic_phi(pr);
                let target_v = psi * c_bc * phi;
                v_out - target_v
            } else {
                // Inflow: gas enters pipe from atmospheric reservoir
                let pr = (p_bc / P_ATM).clamp(0.0, 1.0);
                let phi = Self::isentropic_phi(pr);
                let c_bc5 = c_bc4 * c_bc;
                let rho_bc = RHO_ATM * c_bc5 * inv_c_res5;
                let g = psi * RHO_ATM * c_res * phi;
                let target_v = -g / rho_bc.max(1e-3);
                v_out - target_v
            }
        };

        let mut low = -c_res - 200.0;
        let mut high = (r_in - 1e-3).max(low + 10.0);

        let mut r_low = eval_residual(low);
        let mut r_high = eval_residual(high);

        if r_low * r_high > 0.0 {
            if r_low > 0.0 {
                for _ in 0..15 {
                    low -= 300.0;
                    r_low = eval_residual(low);
                    if r_low < 0.0 {
                        break;
                    }
                }
            } else {
                for _ in 0..15 {
                    high += 300.0;
                    if high >= r_in {
                        high = r_in - 1e-5;
                        break;
                    }
                    r_high = eval_residual(high);
                    if r_high > 0.0 {
                        break;
                    }
                }
            }
        }

        let mut result = 0.5 * (low + high);
        let mut side = 0i32;
        for _ in 0..12 {
            let denom = r_high - r_low;
            if denom.abs() < 1e-6 {
                break;
            }
            let mid = (low * r_high - high * r_low) / denom;
            let r_mid = eval_residual(mid);
            result = mid;

            if r_mid.abs() < 1e-3 {
                break;
            }

            if r_low * r_mid > 0.0 {
                low = mid;
                r_low = r_mid;
                if side == -1 {
                    r_high *= 0.5;
                }
                side = -1;
            } else {
                high = mid;
                r_high = r_mid;
                if side == 1 {
                    r_low *= 0.5;
                }
                side = 1;
            }
        }

        result
    }



/// Internal FVM scheme solver
    fn compute_rhs_internal(
        limiter: LimiterType,
        friction: f32,
        heat_transfer: f32,
        tube: &mut Tube,
        use_temp: bool,
    ) {
        let num_cells = tube.num_cells;
        let dx = tube.dx;
        let gamma = 1.4;

        let state = if use_temp { &tube.u_temp } else { &tube.u };

        // 1. Primitive variables
        for i in 0..=(num_cells + 1) {
            unsafe {
                *tube.w.get_unchecked_mut(i) = conserved_to_primitive(state.get_unchecked(i), gamma);
            }
        }

        // 2. MUSCL slopes
        unsafe {
            match limiter {
                LimiterType::None => {
                    for i in 1..=num_cells {
                        *tube.slopes.get_unchecked_mut(i) = [0.0; 3];
                    }
                }
                LimiterType::Minmod => {
                    for i in 1..=num_cells {
                        for k in 0..3 {
                            let dw_l = tube.w.get_unchecked(i)[k] - tube.w.get_unchecked(i - 1)[k];
                            let dw_r = tube.w.get_unchecked(i + 1)[k] - tube.w.get_unchecked(i)[k];
                            tube.slopes.get_unchecked_mut(i)[k] = limit_slope_minmod(dw_l, dw_r);
                        }
                    }
                }
                LimiterType::Superbee => {
                    for i in 1..=num_cells {
                        for k in 0..3 {
                            let dw_l = tube.w.get_unchecked(i)[k] - tube.w.get_unchecked(i - 1)[k];
                            let dw_r = tube.w.get_unchecked(i + 1)[k] - tube.w.get_unchecked(i)[k];
                            tube.slopes.get_unchecked_mut(i)[k] = limit_slope_superbee(dw_l, dw_r);
                        }
                    }
                }
                LimiterType::MC => {
                    for i in 1..=num_cells {
                        for k in 0..3 {
                            let dw_l = tube.w.get_unchecked(i)[k] - tube.w.get_unchecked(i - 1)[k];
                            let dw_r = tube.w.get_unchecked(i + 1)[k] - tube.w.get_unchecked(i)[k];
                            tube.slopes.get_unchecked_mut(i)[k] = limit_slope_mc(dw_l, dw_r);
                        }
                    }
                }
                LimiterType::VanLeer => {
                    for i in 1..=num_cells {
                        for k in 0..3 {
                            let dw_l = tube.w.get_unchecked(i)[k] - tube.w.get_unchecked(i - 1)[k];
                            let dw_r = tube.w.get_unchecked(i + 1)[k] - tube.w.get_unchecked(i)[k];
                            tube.slopes.get_unchecked_mut(i)[k] = limit_slope_vanleer(dw_l, dw_r);
                        }
                    }
                }
            }
        }

        // 3. Interface Fluxes
        for j in 0..=num_cells {
            unsafe {
                let mut w_l = *tube.w.get_unchecked(j);
                if j >= 1 {
                    let slope_l = tube.slopes.get_unchecked(j);
                    for k in 0..3 {
                        w_l[k] += 0.5 * slope_l[k];
                    }
                }
                w_l[0] = w_l[0].max(1e-4);
                w_l[2] = w_l[2].max(1e-2);

                let mut w_r = *tube.w.get_unchecked(j + 1);
                if j + 1 <= num_cells {
                    let slope_r = tube.slopes.get_unchecked(j + 1);
                    for k in 0..3 {
                        w_r[k] -= 0.5 * slope_r[k];
                    }
                }
                w_r[0] = w_r[0].max(1e-4);
                w_r[2] = w_r[2].max(1e-2);

                let u_l = primitive_to_conserved(&w_l, gamma);
                let u_r = primitive_to_conserved(&w_r, gamma);

                let flux_l = euler_flux(&u_l, w_l[2]);
                let flux_r = euler_flux(&u_r, w_r[2]);

                let a_l = (gamma * w_l[2] / w_l[0]).sqrt();
                let a_r = (gamma * w_r[2] / w_r[0]).sqrt();
                let max_wave_speed = (w_l[1].abs() + a_l).max(w_r[1].abs() + a_r);

                let mut interface_flux = [0.0; 3];
                for k in 0..3 {
                    interface_flux[k] = 0.5 * (flux_l[k] + flux_r[k]) - 0.5 * max_wave_speed * (u_r[k] - u_l[k]);
                }
                *tube.fluxes.get_unchecked_mut(j) = interface_flux;
            }
        }

        // Save boundary interface fluxes
        let left_flux = tube.fluxes[0];
        let right_flux = tube.fluxes[num_cells];

        // 4. Update cells with source terms (including variable area)
        let idx = 1.0 / dx;

        for i in 1..=num_cells {
            unsafe {
                let flux_i = *tube.fluxes.get_unchecked(i);
                let flux_im1 = *tube.fluxes.get_unchecked(i - 1);
                let rhs_i = tube.rhs.get_unchecked_mut(i);
                rhs_i[0] = -idx * (flux_i[0] - flux_im1[0]);
                rhs_i[1] = -idx * (flux_i[1] - flux_im1[1]);
                rhs_i[2] = -idx * (flux_i[2] - flux_im1[2]);

                // Variable cross-sectional area source term:
                // S_A = - dA/dx * 1/A * [rho*v, rho*v^2, (E + P)*v]
                let a = *tube.area.get_unchecked(i);
                let dadx = *tube.dadx.get_unchecked(i);

                if a > 1e-6 && dadx.abs() > 1e-6 {
                    let s = state.get_unchecked(i);
                    let rho = s[0];
                    let v = s[1] / rho.max(1e-5);
                    let p = tube.w.get_unchecked(i)[2];
                    let e = s[2];

                    let s_area_mass = -(dadx / a) * rho * v;
                    let s_area_momentum = -(dadx / a) * rho * v * v;
                    let s_area_energy = -(dadx / a) * (e + p) * v;

                    let rhs_i = tube.rhs.get_unchecked_mut(i);
                    rhs_i[0] += s_area_mass;
                    rhs_i[1] += s_area_momentum;
                    rhs_i[2] += s_area_energy;
                }

                // Friction source term
                if friction > 0.0 {
                    let s = state.get_unchecked(i);
                    let rho = s[0];
                    let v = s[1] / rho.max(1e-5);

                    // Physical Darcy-Weisbach quadratic wall friction
                    // D = 2 * radius = 2 * sqrt(Area / PI)
                    let radius = (a / std::f32::consts::PI).sqrt().max(0.001);
                    let d = 2.0 * radius;
                    let f_factor = friction / (2.0 * d);
                    let s_momentum = -f_factor * rho * v * v.abs();
                    let s_energy = -f_factor * rho * v * v * v.abs();

                    let rhs_i = tube.rhs.get_unchecked_mut(i);
                    rhs_i[1] += s_momentum;
                    rhs_i[2] += s_energy;
                }

                // Wall heat transfer: relaxes gas temperature toward wall temperature (T_ATM)
                if heat_transfer > 0.0 {
                    let s = state.get_unchecked(i);
                    let rho = s[0].max(1e-5);
                    let v = s[1] / rho;
                    let ke = 0.5 * rho * v * v;
                    let internal_e = (s[2] - ke).max(1e-4);
                    let p = internal_e * (gamma - 1.0);
                    let t_gas = p / (rho * R_AIR);
                    let dt_wall = t_gas - T_ATM;

                    let radius = (a / std::f32::consts::PI).sqrt().max(0.001);
                    let s_energy_heat = -heat_transfer * rho * R_AIR * dt_wall * (2.0 / radius);

                    tube.rhs.get_unchecked_mut(i)[2] += s_energy_heat;
                }
            }
        }

        // Species upwind advection
        let species_state = if use_temp { &tube.species_temp } else { &tube.species };
        for i in 1..=num_cells {
            unsafe {
                let s = state.get_unchecked(i);
                let rho = s[0].max(1e-5);
                let v = s[1] / rho;
                let sp_i = species_state.get_unchecked(i);
                let sp_im1 = species_state.get_unchecked(i - 1);
                let sp_ip1 = species_state.get_unchecked(i + 1);
                let rhs_i = tube.species_rhs.get_unchecked_mut(i);
                for k in 0..4 {
                    let dy_dx = if v >= 0.0 {
                        (sp_i[k] - sp_im1[k]) / dx
                    } else {
                        (sp_ip1[k] - sp_i[k]) / dx
                    };
                    rhs_i[k] = -v * dy_dx;
                }
            }
        }

        // Save boundary species fluxes (mass flux * upwind species)
        let left_mass_flux = tube.fluxes[0][0];
        let right_mass_flux = tube.fluxes[num_cells][0];
        let left_upwind = if left_mass_flux >= 0.0 { species_state[0] } else { species_state[1] };
        let right_upwind = if right_mass_flux >= 0.0 { species_state[num_cells] } else { species_state[num_cells + 1] };
        for k in 0..4 {
            tube.left_species_flux[k] = left_mass_flux * left_upwind[k];
            tube.right_species_flux[k] = right_mass_flux * right_upwind[k];
        }

        tube.left_boundary_flux = left_flux;
        tube.right_boundary_flux = right_flux;
    }
}

// ------------------ Cubic Bezier Core Math ------------------

pub fn bezier_point(t: f32, p0: [f32; 2], p1: [f32; 2], p2: [f32; 2], p3: [f32; 2]) -> [f32; 2] {
    let mt = 1.0 - t;
    let mt2 = mt * mt;
    let mt3 = mt2 * mt;
    let t2 = t * t;
    let t3 = t2 * t;
    
    [
        mt3 * p0[0] + 3.0 * mt2 * t * p1[0] + 3.0 * mt * t2 * p2[0] + t3 * p3[0],
        mt3 * p0[1] + 3.0 * mt2 * t * p1[1] + 3.0 * mt * t2 * p2[1] + t3 * p3[1],
    ]
}

pub fn bezier_derivative(t: f32, p0: [f32; 2], p1: [f32; 2], p2: [f32; 2], p3: [f32; 2]) -> [f32; 2] {
    let mt = 1.0 - t;
    let mt2 = mt * mt;
    let t2 = t * t;
    
    let c0 = 3.0 * mt2;
    let c1 = 6.0 * mt * t;
    let c2 = 3.0 * t2;
    
    [
        c0 * (p1[0] - p0[0]) + c1 * (p2[0] - p1[0]) + c2 * (p3[0] - p2[0]),
        c0 * (p1[1] - p0[1]) + c1 * (p2[1] - p1[1]) + c2 * (p3[1] - p2[1]),
    ]
}

pub fn bezier_length(p0: [f32; 2], p1: [f32; 2], p2: [f32; 2], p3: [f32; 2]) -> f32 {
    let mut len = 0.0;
    let mut prev = bezier_point(0.0, p0, p1, p2, p3);
    let steps = 100;
    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let curr = bezier_point(t, p0, p1, p2, p3);
        let dx = curr[0] - prev[0];
        let dy = curr[1] - prev[1];
        len += (dx * dx + dy * dy).sqrt();
        prev = curr;
    }
    len
}

// ------------------ Conversions ------------------

#[inline(always)]
pub fn primitive_to_conserved(w: &[f32; 3], gamma: f32) -> [f32; 3] {
    let rho = w[0];
    let v = w[1];
    let p = w[2];
    
    let kinetic = 0.5 * rho * v * v;
    let internal = p / (gamma - 1.0);
    let energy = internal + kinetic;
    
    [rho, rho * v, energy]
}

#[inline(always)]
pub fn conserved_to_primitive(u: &[f32; 3], gamma: f32) -> [f32; 3] {
    let rho = u[0].max(1e-4);
    let momentum = u[1];
    let energy = u[2];
    
    let v = momentum / rho;
    let kinetic = 0.5 * rho * v * v;
    let internal = (energy - kinetic).max(1e-4);
    let p = internal * (gamma - 1.0);
    
    [rho, v, p]
}

#[inline(always)]
pub fn euler_flux(u: &[f32; 3], p: f32) -> [f32; 3] {
    let rho = u[0];
    let momentum = u[1];
    let energy = u[2];
    
    let v = momentum / rho.max(1e-5);
    
    [
        momentum,
        momentum * v + p,
        (energy + p) * v,
    ]
}
// ------------------ Slopes ------------------

fn limit_slope_minmod(a: f32, b: f32) -> f32 {
    if a * b <= 0.0 {
        0.0
    } else if a.abs() < b.abs() {
        a
    } else {
        b
    }
}

fn limit_slope_superbee(a: f32, b: f32) -> f32 {
    if a * b <= 0.0 {
        0.0
    } else {
        let s = a.signum();
        let abs_a = a.abs();
        let abs_b = b.abs();
        
        let limit1 = (2.0 * abs_a).min(abs_b);
        let limit2 = abs_a.min(2.0 * abs_b);
        
        s * limit1.max(limit2)
    }
}

fn limit_slope_mc(a: f32, b: f32) -> f32 {
    if a * b <= 0.0 {
        0.0
    } else {
        let s = a.signum();
        let abs_a = a.abs();
        let abs_b = b.abs();
        let avg = 0.5 * (abs_a + abs_b);
        
        s * (2.0 * abs_a).min(2.0 * abs_b).min(avg)
    }
}

fn limit_slope_vanleer(a: f32, b: f32) -> f32 {
    if a * b <= 0.0 {
        0.0
    } else {
        (2.0 * a * b) / (a + b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier_length() {
        let p0 = [0.0, 0.0];
        let p1 = [0.5, 0.0];
        let p2 = [1.0, 0.0];
        let p3 = [1.5, 0.0];
        let len = bezier_length(p0, p1, p2, p3);
        assert!((len - 1.5).abs() < 1e-3);
    }

    #[test]
    fn test_y_junction_solver() {
        let mut solver = Solver::new_y_junction();
        assert_eq!(solver.tubes.len(), 3);
        assert_eq!(solver.junctions.len(), 1);
        
        // Inject pulse
        solver.inject_pulse(20000.0, 0.1);
        
        // Take a step
        solver.step(1e-5);
        
        // Verify states aren't NaN
        for tube in &solver.tubes {
            for cell in &tube.u {
                assert!(!cell[0].is_nan());
                assert!(!cell[1].is_nan());
                assert!(!cell[2].is_nan());
            }
        }
    }

    #[test]
    fn test_open_open_quiescent_jones() {
        let mut solver = Solver::new_single_tube(RadiusProfile::Linear, BoundaryType::Open, BoundaryType::Open);
        solver.friction = 0.02;
        solver.heat_transfer = 2.0;

        let sim_dt = 1.0 / 64000.0;
        let mut max_jones = 0.0f32;
        let mut max_velocity = 0.0f32;

        for _ in 0..8000 {
            let jones_val = solver.step(sim_dt);
            max_jones = max_jones.max(jones_val.abs());

            for tube in &solver.tubes {
                for i in 1..=tube.num_cells {
                    let w = conserved_to_primitive(&tube.u[i], 1.4);
                    max_velocity = max_velocity.max(w[1].abs());
                }
            }
        }

        assert!(max_jones < 5.0, "Quiescent Jones noise too high: {}", max_jones);
        assert!(max_velocity < 5.0, "Quiescent velocity drift too high: {}", max_velocity);
    }

    #[test]
    fn test_boundary_stability() {
        // Create a solver with a single tube, open at both ends to test inflow stability
        let mut solver = Solver::new_single_tube(RadiusProfile::Linear, BoundaryType::Open, BoundaryType::Open);
        
        // Set a high reflection filter and low friction to make stability harder
        solver.reflection_filter = 0.85;
        solver.friction = 0.01;
        
        // Inject multiple strong pulses
        for _ in 0..5 {
            solver.inject_pulse(150000.0, 0.05);
            
            // Step the solver for some time
            let sim_dt = 1.0 / 64000.0;
            for _ in 0..100 {
                let jones_val = solver.step(sim_dt);
                assert!(!jones_val.is_nan());
                assert!(jones_val.abs() < 1e7, "Jones signal blew up: {}", jones_val);
            }
        }
        
        // Verify all cell states are physically reasonable and non-NaN
        for tube in &solver.tubes {
            for cell in &tube.u {
                assert!(!cell[0].is_nan());
                assert!(!cell[1].is_nan());
                assert!(!cell[2].is_nan());
                
                let w = conserved_to_primitive(cell, 1.4);
                assert!(w[0] > 1e-3, "Density went below physical threshold: {}", w[0]);
                assert!(w[2] > 1e-1, "Pressure went below physical threshold: {}", w[2]);
            }
        }
    }

    #[test]
    fn test_crank_slider_kinematics() {
        use crate::mechanical::crankshaft::{piston_displacement, piston_dy_dtheta};
        let stroke = 0.08;
        let conrod = 0.16;
        
        // Piston displacement at TDC (0) should be 0
        let tdc_disp = piston_displacement(0.0, stroke, conrod);
        assert!(tdc_disp.abs() < 1e-5);
        
        // Piston displacement at BDC (PI) should be stroke
        let bdc_disp = piston_displacement(std::f32::consts::PI, stroke, conrod);
        assert!((bdc_disp - stroke).abs() < 1e-5);
        
        // dy/dtheta should be 0 at TDC (0) and BDC (PI)
        let tdc_dy = piston_dy_dtheta(0.0, stroke, conrod);
        let bdc_dy = piston_dy_dtheta(std::f32::consts::PI, stroke, conrod);
        assert!(tdc_dy.abs() < 1e-5);
        assert!(bdc_dy.abs() < 1e-5);
    }

    #[test]
    fn test_cylinder_isentropic() {
        // Run with closed valves to verify isentropic compression / expansion
        let mut solver = Solver::new_single_cylinder();
        
        // Disconnect tubes by setting connections empty, effectively closing all valves
        if let Some(ref mut cyl) = solver.cylinders.get_mut(0) {
            cyl.connections.clear();
            cyl.reset_state();
        }
        
        // Spin the crankshaft up to 1000 RPM (approx 104.7 rad/s)
        if let Some(ref mut crank) = solver.crankshaft {
            crank.omega = 104.7;
        }
        
        // Step the engine through a compression stroke (from BDC to TDC, e.g. theta goes from PI to 2*PI)
        if let Some(ref mut crank) = solver.crankshaft {
            crank.theta = std::f32::consts::PI; // start at BDC
        }
        // Force state reset at current angle
        let initial_vol = solver.cylinders[0].volume_at(std::f32::consts::PI);
        let initial_p = 101325.0;
        solver.cylinders[0].volume = initial_vol;
        solver.cylinders[0].mass = RHO_ATM * initial_vol;
        solver.cylinders[0].energy = initial_p * initial_vol / (1.4 - 1.0);
        solver.cylinders[0].update_thermodynamics(1.4);
        
        let initial_p_v_gamma = solver.cylinders[0].p * solver.cylinders[0].volume.powf(1.4);
        
        let sim_dt = 1.0 / 64000.0;
        for _ in 0..100 {
            solver.step(sim_dt);
            
            // Check that P * V^gamma is conserved
            let current_p_v_gamma = solver.cylinders[0].p * solver.cylinders[0].volume.powf(1.4);
            let rel_err = (current_p_v_gamma - initial_p_v_gamma).abs() / initial_p_v_gamma;
            assert!(rel_err < 1e-3, "Isentropic relation violated: rel_err = {}, p = {}, v = {}", rel_err, solver.cylinders[0].p, solver.cylinders[0].volume);
        }
    }

    #[test]
    fn test_engine_spindown() {
        let mut solver = Solver::new_single_cylinder();
        
        // Set initial high RPM
        if let Some(ref mut crank) = solver.crankshaft {
            crank.omega = 300.0; // ~3000 RPM
        }
        
        let initial_omega = solver.crankshaft.as_ref().unwrap().omega;
        
        // Step solver for some time
        let sim_dt = 1.0 / 64000.0;
        for _ in 0..500 {
            solver.step(sim_dt);
        }
        
        let final_omega = solver.crankshaft.as_ref().unwrap().omega;
        assert!(final_omega < initial_omega, "Engine did not slow down: initial = {}, final = {}", initial_omega, final_omega);
    }

    #[test]
    fn test_engine_combustion_run() {
        // Run with throttle = 1.0 (WOT)
        let mut solver_wot = Solver::new_single_cylinder();
        solver_wot.ignition_on = true;
        solver_wot.throttle_input = 1.0;
        if let Some(ref mut crank) = solver_wot.crankshaft {
            crank.omega = 890.0; // ~8500 RPM
        }
        
        // Run with throttle = 0.0 (Idle/Closed throttle)
        let mut solver_idle = Solver::new_single_cylinder();
        solver_idle.ignition_on = true;
        solver_idle.throttle_input = 0.0;
        if let Some(ref mut crank) = solver_idle.crankshaft {
            crank.omega = 890.0; // ~8500 RPM
        }
        
        let sim_dt = 1.0 / 64000.0;
        for _ in 0..64000 {
            solver_wot.step(sim_dt);
            solver_idle.step(sim_dt);
        }
        
        let final_omega_wot = solver_wot.crankshaft.as_ref().unwrap().omega;
        let last_cell_wot = solver_wot.tubes[0].num_cells;
        let w_wot = conserved_to_primitive(&solver_wot.tubes[0].u[last_cell_wot], 1.4);
        let final_map_wot = w_wot[2];

        let final_omega_idle = solver_idle.crankshaft.as_ref().unwrap().omega;
        let last_cell_idle = solver_idle.tubes[0].num_cells;
        let w_idle = conserved_to_primitive(&solver_idle.tubes[0].u[last_cell_idle], 1.4);
        let final_map_idle = w_idle[2];

        println!("[TEST OUTPUT] WOT: final_omega = {:.2} ({:.0} RPM), final_map = {:.0} Pa", final_omega_wot, final_omega_wot * 60.0 / (2.0 * std::f32::consts::PI), final_map_wot);
        println!("[TEST OUTPUT] IDLE: final_omega = {:.2} ({:.0} RPM), final_map = {:.0} Pa", final_omega_idle, final_omega_idle * 60.0 / (2.0 * std::f32::consts::PI), final_map_idle);
        
        // WOT should have significantly higher RPM and MAP than IDLE
        assert!(final_omega_wot > final_omega_idle);
        assert!(final_map_wot > final_map_idle);
    }

    #[test]
    fn test_species_advection() {
        // Create a single tube connected to open atmosphere
        let mut solver = Solver::new_single_tube(RadiusProfile::Linear, BoundaryType::Open, BoundaryType::Open);
        let tube = &mut solver.tubes[0];
        
        // Initial state is AIR_Y
        assert_eq!(tube.species[1], AIR_Y);
        
        // Set cell 1 to a rich fuel mixture: 10% fuel, 15% O2
        let rich_mixture = [0.15, 0.10, 0.0, 0.0];
        tube.species[1] = rich_mixture;
        
        // Establish a flow to the right: positive velocity in all cells
        for i in 1..=tube.num_cells {
            let mut w = conserved_to_primitive(&tube.u[i], 1.4);
            w[1] = 50.0; // 50 m/s velocity to the right
            tube.u[i] = primitive_to_conserved(&w, 1.4);
        }
        
        // Take a few steps of the simulation
        let sim_dt = 1.0 / 64000.0;
        for _ in 0..10 {
            solver.step(sim_dt);
        }
        
        // Verify that species advected to downstream cells (e.g. cell 2 should have non-zero fuel)
        let tube_after = &solver.tubes[0];
        assert!(tube_after.species[2][crate::cfd::species::I_FUEL] > 0.0,
            "Fuel did not advect to cell 2: Y_fuel = {}", tube_after.species[2][crate::cfd::species::I_FUEL]);
    }

    #[test]
    fn test_combustion_afr_efficiency() {
        let mut runs = vec![];
        
        // Test three AFR settings: stoichiometric (14.7), extremely rich (6.0), and extremely lean (25.0)
        for target_afr in [14.7, 6.0, 25.0] {
            let mut solver = Solver::new_single_cylinder();
            solver.ignition_on = true;
            solver.throttle_input = 1.0;
            solver.ecu.manual_afr_control = true;
            solver.ecu.target_afr = target_afr;
            solver.target_afr = target_afr;
            
            if let Some(ref mut crank) = solver.crankshaft {
                crank.omega = 890.0; // start at ~8500 RPM
            }
            
            // Run for 3000 steps
            let sim_dt = 1.0 / 64000.0;
            for _ in 0..3000 {
                solver.step(sim_dt);
            }
            
            let final_omega = solver.crankshaft.as_ref().unwrap().omega;
            runs.push((target_afr, final_omega));
        }
        
        println!("[AFR TEST OUTPUT] Runs: {:?}", runs);
        
        // Stoichiometric (14.7) should produce more power and have higher final omega than rich (6.0) and lean (25.0)
        let omega_stoich = runs[0].1;
        let omega_rich = runs[1].1;
        let omega_lean = runs[2].1;
        
        assert!(omega_stoich > omega_rich, "Stoich RPM ({}) should be higher than Rich RPM ({})", omega_stoich, omega_rich);
        assert!(omega_stoich > omega_lean, "Stoich RPM ({}) should be higher than Lean RPM ({})", omega_stoich, omega_lean);
    }

    #[test]
    fn test_misfire_at_high_residual() {
        let mut solver = Solver::new_single_cylinder();
        solver.ignition_on = true;
        
        // Disconnect the cylinder so advection doesn't wash out the species
        solver.cylinders[0].connections.clear();
        
        // Artificially dilute the cylinder with 70% exhaust products (residual fraction = 0.70)
        let high_residual = [0.05, 0.05, 0.35, 0.35];
        solver.cylinders[0].species = high_residual;
        solver.cylinders[0].residual_fraction = 0.70;
        
        // Set combustion phase to trigger spark
        solver.cylinders[0].combustion_phase = -1.0;
        let init_energy = solver.cylinders[0].energy;
        
        // Advance crankshaft to spark angle (with 5.0 deg timing during cranking) to trigger spark
        if let Some(ref mut crank) = solver.crankshaft {
            crank.omega = 100.0; // rotate so that it triggers
            crank.theta = (4.0 * std::f32::consts::PI - 5.0f32.to_radians()) - 0.005;
        }
        
        // Take multiple steps to cross the spark timing
        let sim_dt = 1.0 / 64000.0;
        for i in 0..10 {
            if let Some(ref crank) = solver.crankshaft {
                let spark_angle = (4.0 * std::f32::consts::PI - solver.ecu.ignition_timing_deg.to_radians()).rem_euclid(4.0 * std::f32::consts::PI);
                println!("[STEP {}] crank.theta = {}, spark_angle = {}, timing = {}, viable = {}, last_misfire = {}",
                    i, crank.theta, spark_angle, solver.ecu.ignition_timing_deg,
                    crate::combustion::burn::combustion_viable(solver.cylinders[0].species[0], solver.cylinders[0].species[1], solver.cylinders[0].residual_fraction),
                    solver.last_misfire
                );
            }
            solver.step(sim_dt);
        }
        
        // Verify that a misfire was recorded, and energy change is small (purely compression work, no combustion)
        assert!(solver.last_misfire, "Engine did not misfire under high residual fraction");
        assert!(solver.misfire_count >= 1, "Misfire count did not increment");
        let energy_diff = (solver.cylinders[0].energy - init_energy).abs();
        assert!(energy_diff < 0.05, "Cylinder energy changed significantly: energy_diff = {}", energy_diff);
    }

    #[test]
    fn test_inline_four_firing_order() {
        let solver = Solver::new_inline_four();
        assert_eq!(solver.cylinders.len(), 4);
        
        // Firing order is 1-3-4-2. Pin offsets should match:
        assert_eq!(solver.cylinders[0].crank_pin_offset_radians, 0.0);
        assert_eq!(solver.cylinders[1].crank_pin_offset_radians, 3.0 * std::f32::consts::PI);
        assert_eq!(solver.cylinders[2].crank_pin_offset_radians, 1.0 * std::f32::consts::PI);
        assert_eq!(solver.cylinders[3].crank_pin_offset_radians, 2.0 * std::f32::consts::PI);
    }

    #[test]
    fn test_yaml_config_loading() {
        use crate::config::FullConfig;
        
        // Save a temporary config file
        let mut config = FullConfig::default();
        config.engine.bore = 0.075;
        config.engine.stroke = 0.055;
        config.ecu.target_idle_rpm = 1800.0;
        
        let path = "config/temp_test_config.yaml";
        let _ = std::fs::create_dir_all("config");
        config.save_to_file(path).expect("Failed to save YAML config");
        
        // Load it back
        let loaded = FullConfig::load_from_file(path).expect("Failed to load YAML config");
        assert_eq!(loaded.engine.bore, 0.075);
        assert_eq!(loaded.engine.stroke, 0.055);
        assert_eq!(loaded.ecu.target_idle_rpm, 1800.0);
        
        // Cleanup
        let _ = std::fs::remove_file(path);
    }
}