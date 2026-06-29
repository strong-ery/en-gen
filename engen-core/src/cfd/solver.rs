use std::fmt;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundaryType {
    Closed, // Rigid reflecting wall
    Open,   // Atmospheric boundary
}

impl fmt::Display for BoundaryType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BoundaryType::Closed => write!(f, "Closed (Wall)"),
            BoundaryType::Open => write!(f, "Open (Atmosphere)"),
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
            cell_centers_2d: Vec::new(),
            cell_tangents_2d: Vec::new(),
            cell_boundaries_left_2d: Vec::new(),
            cell_boundaries_right_2d: Vec::new(),
        };
        tube.rebuild_geometry();
        tube.reset_state();
        tube
    }

    pub fn reset_state(&mut self) {
        let w_init = [RHO_ATM, 0.0, P_ATM];
        let u_init = primitive_to_conserved(&w_init, 1.4);
        self.u = vec![u_init; self.num_cells + 2];
        self.left_boundary_flux = [0.0; 3];
        self.right_boundary_flux = [0.0; 3];
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

#[derive(Debug, Clone)]
pub struct Solver {
    pub tubes: Vec<Tube>,
    pub junctions: Vec<Junction>,
    pub limiter: LimiterType,
    pub friction: f32,
    pub t: f32,
}

impl Solver {
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
        Self {
            tubes: vec![tube],
            junctions: Vec::new(),
            limiter: LimiterType::MC,
            friction: 0.0,
            t: 0.0,
        }
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
        };

        Self {
            tubes: vec![main_tube, branch_a, branch_b],
            junctions: vec![junction],
            limiter: LimiterType::MC,
            friction: 0.0,
            t: 0.0,
        }
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

    pub fn step(&mut self, dt: f32) {
        let gamma = 1.4;
        let num_tubes = self.tubes.len();

        // ------------------ SSP-RK2 Step 1 ------------------
        self.apply_boundary_conditions();
        
        // Compute RHS and boundary fluxes for all tubes at u^n
        let mut rhs1 = Vec::with_capacity(num_tubes);
        let mut step1_left_fluxes = Vec::with_capacity(num_tubes);
        let mut step1_right_fluxes = Vec::with_capacity(num_tubes);
        
        let limiter = self.limiter;
        let friction = self.friction;

        for tube in &self.tubes {
            let (rhs, left_flux, right_flux) = Self::compute_rhs_internal(limiter, friction, tube, &tube.u);
            rhs1.push(rhs);
            step1_left_fluxes.push(left_flux);
            step1_right_fluxes.push(right_flux);
        }

        // Temporary state storage u1
        let mut u1 = Vec::with_capacity(num_tubes);
        for (k, tube) in self.tubes.iter().enumerate() {
            let mut u1_tube = tube.u.clone();
            for i in 1..=tube.num_cells {
                for c in 0..3 {
                    u1_tube[i][c] = tube.u[i][c] + dt * rhs1[k][i][c];
                }
            }
            u1.push(u1_tube);
        }

        // Evolve junctions to state 1
        let mut j1 = self.junctions.clone();
        for (j_idx, junction) in self.junctions.iter().enumerate() {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            
            for conn in &junction.connections {
                let left_flux = step1_left_fluxes[conn.tube_id];
                let right_flux = step1_right_fluxes[conn.tube_id];
                let tube = &self.tubes[conn.tube_id];
                match conn.side {
                    TubeSide::Left => {
                        let area = tube.area[1];
                        dmass -= area * left_flux[0];
                        denergy -= area * left_flux[2];
                    }
                    TubeSide::Right => {
                        let area = tube.area[tube.num_cells];
                        dmass += area * right_flux[0];
                        denergy += area * right_flux[2];
                    }
                }
            }

            j1[j_idx].rho = (junction.rho + (dt / junction.volume) * dmass).max(1e-4);
            j1[j_idx].e = (junction.e + (dt / junction.volume) * denergy).max(1e-2);
        }

        // ------------------ SSP-RK2 Step 2 ------------------
        // Apply BC on intermediate states u1 and j1
        Self::apply_network_bc(&mut u1, &j1, &self.tubes, &self.junctions, gamma);
        
        let mut rhs2 = Vec::with_capacity(num_tubes);
        let mut step2_left_fluxes = Vec::with_capacity(num_tubes);
        let mut step2_right_fluxes = Vec::with_capacity(num_tubes);
        
        for (k, tube) in self.tubes.iter().enumerate() {
            let (rhs, left_flux, right_flux) = Self::compute_rhs_internal(limiter, friction, tube, &u1[k]);
            rhs2.push(rhs);
            step2_left_fluxes.push(left_flux);
            step2_right_fluxes.push(right_flux);
        }

        // Final state update
        for (k, tube) in self.tubes.iter_mut().enumerate() {
            for i in 1..=tube.num_cells {
                for c in 0..3 {
                    tube.u[i][c] = 0.5 * tube.u[i][c] + 0.5 * u1[k][i][c] + 0.5 * dt * rhs2[k][i][c];
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

        // Update junctions to next timestep
        for (j_idx, junction) in self.junctions.iter_mut().enumerate() {
            let mut dmass = 0.0;
            let mut denergy = 0.0;
            
            for conn in &junction.connections {
                let left_flux = step2_left_fluxes[conn.tube_id];
                let right_flux = step2_right_fluxes[conn.tube_id];
                let tube = &self.tubes[conn.tube_id];
                match conn.side {
                    TubeSide::Left => {
                        let area = tube.area[1];
                        dmass -= area * left_flux[0];
                        denergy -= area * left_flux[2];
                    }
                    TubeSide::Right => {
                        let area = tube.area[tube.num_cells];
                        dmass += area * right_flux[0];
                        denergy += area * right_flux[2];
                    }
                }
            }

            junction.rho = (0.5 * junction.rho + 0.5 * j1[j_idx].rho + 0.5 * (dt / junction.volume) * dmass).max(1e-4);
            junction.e = (0.5 * junction.e + 0.5 * j1[j_idx].e + 0.5 * (dt / junction.volume) * denergy).max(1e-2);
        }

        self.apply_boundary_conditions();
        
        // Save the final boundary fluxes at the end of the step for UI queries
        for tube in &mut self.tubes {
            let (_, left_flux, right_flux) = Self::compute_rhs_internal(limiter, friction, tube, &tube.u);
            tube.left_boundary_flux = left_flux;
            tube.right_boundary_flux = right_flux;
        }

        self.t += dt;
    }

    pub fn apply_boundary_conditions(&mut self) {
        let gamma = 1.4;
        let mut u_temp = self.tubes.iter().map(|t| t.u.clone()).collect::<Vec<_>>();
        Self::apply_network_bc(&mut u_temp, &self.junctions, &self.tubes, &self.junctions, gamma);
        for (k, tube) in self.tubes.iter_mut().enumerate() {
            tube.u = u_temp[k].clone();
        }
    }

    fn apply_network_bc(
        u_states: &mut Vec<Vec<[f32; 3]>>,
        j_states: &Vec<Junction>,
        tubes: &Vec<Tube>,
        junctions: &Vec<Junction>,
        gamma: f32,
    ) {
        // First set standard (non-connected) boundaries
        for (k, tube) in tubes.iter().enumerate() {
            let state = &mut u_states[k];
            let m = tube.num_cells;
            
            // Check Left End
            let connected_left = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube.id && c.side == TubeSide::Left));
            if !connected_left {
                match tube.left_bc {
                    BoundaryType::Closed => {
                        let w1 = conserved_to_primitive(&state[1], gamma);
                        let w0 = [w1[0], -w1[1], w1[2]];
                        state[0] = primitive_to_conserved(&w0, gamma);
                    }
                    BoundaryType::Open => {
                        let w1 = conserved_to_primitive(&state[1], gamma);
                        let rho0 = if w1[1] > 0.0 { RHO_ATM } else { w1[0] };
                        let w0 = [rho0, w1[1], P_ATM];
                        state[0] = primitive_to_conserved(&w0, gamma);
                    }
                }
            }

            // Check Right End
            let connected_right = junctions.iter().any(|j| j.connections.iter().any(|c| c.tube_id == tube.id && c.side == TubeSide::Right));
            if !connected_right {
                match tube.right_bc {
                    BoundaryType::Closed => {
                        let wm = conserved_to_primitive(&state[m], gamma);
                        let w_m1 = [wm[0], -wm[1], wm[2]];
                        state[m + 1] = primitive_to_conserved(&w_m1, gamma);
                    }
                    BoundaryType::Open => {
                        let wm = conserved_to_primitive(&state[m], gamma);
                        let rho_m1 = if wm[1] < 0.0 { RHO_ATM } else { wm[0] };
                        let w_m1 = [rho_m1, wm[1], P_ATM];
                        state[m + 1] = primitive_to_conserved(&w_m1, gamma);
                    }
                }
            }
        }

        // Apply constant-pressure junction boundary values
        for j in j_states {
            let p_junction = j.e * (gamma - 1.0);
            let rho_junction = j.rho;
            
            for conn in &j.connections {
                let state = &mut u_states[conn.tube_id];
                let num_cells = tubes[conn.tube_id].num_cells;
                
                match conn.side {
                    TubeSide::Left => {
                        // Ghost cell 0 is junction
                        let w1 = conserved_to_primitive(&state[1], gamma);
                        let w0 = [rho_junction, w1[1], p_junction]; // copy P_j, rho_j, extrapolate velocity
                        state[0] = primitive_to_conserved(&w0, gamma);
                    }
                    TubeSide::Right => {
                        // Ghost cell num_cells + 1 is junction
                        let wm = conserved_to_primitive(&state[num_cells], gamma);
                        let w_m1 = [rho_junction, wm[1], p_junction];
                        state[num_cells + 1] = primitive_to_conserved(&w_m1, gamma);
                    }
                }
            }
        }
    }



    /// Internal FVM scheme solver
    fn compute_rhs_internal(
        limiter: LimiterType,
        friction: f32,
        tube: &Tube,
        state: &Vec<[f32; 3]>,
    ) -> (Vec<[f32; 3]>, [f32; 3], [f32; 3]) {
        let num_cells = tube.num_cells;
        let dx = tube.dx;
        let gamma = 1.4;

        // 1. Primitive variables
        let mut w = vec![[0.0; 3]; num_cells + 2];
        for i in 0..=(num_cells + 1) {
            w[i] = conserved_to_primitive(&state[i], gamma);
        }

        // 2. MUSCL slopes
        let mut slopes = vec![[0.0; 3]; num_cells + 2];
        for i in 1..=num_cells {
            let dw_l = [w[i][0] - w[i - 1][0], w[i][1] - w[i - 1][1], w[i][2] - w[i - 1][2]];
            let dw_r = [w[i + 1][0] - w[i][0], w[i + 1][1] - w[i][1], w[i + 1][2] - w[i][2]];

            for k in 0..3 {
                slopes[i][k] = match limiter {
                    LimiterType::None => 0.0,
                    LimiterType::Minmod => limit_slope_minmod(dw_l[k], dw_r[k]),
                    LimiterType::Superbee => limit_slope_superbee(dw_l[k], dw_r[k]),
                    LimiterType::MC => limit_slope_mc(dw_l[k], dw_r[k]),
                    LimiterType::VanLeer => limit_slope_vanleer(dw_l[k], dw_r[k]),
                };
            }
        }

        // 3. Interface Fluxes
        let mut fluxes = vec![[0.0; 3]; num_cells + 1];
        for j in 0..=num_cells {
            let mut w_l = w[j];
            if j >= 1 {
                for k in 0..3 {
                    w_l[k] += 0.5 * slopes[j][k];
                }
            }
            w_l[0] = w_l[0].max(1e-4);
            w_l[2] = w_l[2].max(1e-2);

            let mut w_r = w[j + 1];
            if j + 1 <= num_cells {
                for k in 0..3 {
                    w_r[k] -= 0.5 * slopes[j + 1][k];
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
            fluxes[j] = interface_flux;
        }

        // Save boundary interface fluxes
        let left_flux = fluxes[0];
        let right_flux = fluxes[num_cells];

        // 4. Update cells with source terms (including variable area)
        let mut rhs = vec![[0.0; 3]; num_cells + 2];
        let idx = 1.0 / dx;

        for i in 1..=num_cells {
            rhs[i][0] = -idx * (fluxes[i][0] - fluxes[i - 1][0]);
            rhs[i][1] = -idx * (fluxes[i][1] - fluxes[i - 1][1]);
            rhs[i][2] = -idx * (fluxes[i][2] - fluxes[i - 1][2]);

            // Variable cross-sectional area source term:
            // S_A = - dA/dx * 1/A * [rho*v, rho*v^2, (E + P)*v]
            let a = tube.area[i];
            let dadx = tube.dadx[i];
            
            if a > 1e-6 && dadx.abs() > 1e-6 {
                let rho = state[i][0];
                let v = state[i][1] / rho.max(1e-5);
                let p = w[i][2];
                let e = state[i][2];

                let s_area_mass = -(dadx / a) * rho * v;
                let s_area_momentum = -(dadx / a) * rho * v * v;
                let s_area_energy = -(dadx / a) * (e + p) * v;

                rhs[i][0] += s_area_mass;
                rhs[i][1] += s_area_momentum;
                rhs[i][2] += s_area_energy;
            }

            // Friction source term
            if friction > 0.0 {
                let rho = state[i][0];
                let v = state[i][1] / rho.max(1e-5);
                
                let s_momentum = -friction * rho * v;
                let s_energy = -friction * rho * v * v;
                
                rhs[i][1] += s_momentum;
                rhs[i][2] += s_energy;
            }
        }

        (rhs, left_flux, right_flux)
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

pub fn primitive_to_conserved(w: &[f32; 3], gamma: f32) -> [f32; 3] {
    let rho = w[0];
    let v = w[1];
    let p = w[2];
    
    let kinetic = 0.5 * rho * v * v;
    let internal = p / (gamma - 1.0);
    let energy = internal + kinetic;
    
    [rho, rho * v, energy]
}

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
}
