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

#[derive(Debug, Clone)]
pub struct SolverConfig {
    pub length: f32, // Tube length in meters
    pub dx: f32,     // Cell size in meters
    pub gamma: f32,  // Specific heat ratio (1.4 for air)
    pub limiter: LimiterType,
    pub left_bc: BoundaryType,
    pub right_bc: BoundaryType,
    pub friction: f32, // Friction coefficient
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            length: 1.0,
            dx: 0.02,
            gamma: 1.4,
            limiter: LimiterType::MC,
            left_bc: BoundaryType::Closed,
            right_bc: BoundaryType::Closed,
            friction: 0.0,
        }
    }
}

pub const P_ATM: f32 = 101325.0; // Pa
pub const T_ATM: f32 = 293.15;   // K (20 C)
pub const R_AIR: f32 = 287.05;   // J/(kg*K)
pub const RHO_ATM: f32 = P_ATM / (R_AIR * T_ATM); // ~1.204 kg/m^3

#[derive(Debug, Clone)]
pub struct Solver {
    pub config: SolverConfig,
    pub num_cells: usize, // Number of active cells
    pub u: Vec<[f32; 3]>,  // Conserved variables: [rho, rho*v, E] (size: num_cells + 2)
    pub t: f32,           // Current time in seconds
}

impl Solver {
    pub fn new(config: SolverConfig) -> Self {
        let num_cells = (config.length / config.dx).round() as usize;
        let num_cells = num_cells.max(5); // Minimum 5 cells

        // Initial atmospheric conditions in conserved variables
        let w_init = [RHO_ATM, 0.0, P_ATM];
        let u_init = primitive_to_conserved(&w_init, config.gamma);

        // Include ghost cells at index 0 and num_cells + 1
        let u = vec![u_init; num_cells + 2];

        let mut solver = Self {
            config,
            num_cells,
            u,
            t: 0.0,
        };
        solver.apply_boundary_conditions();
        solver
    }

    /// Inject a Gaussian pressure pulse at the left end of the tube
    pub fn inject_pulse(&mut self, amplitude_pa: f32, width_m: f32) {
        let gamma = self.config.gamma;
        let dx = self.config.dx;

        for i in 1..=self.num_cells {
            let x = (i as f32 - 0.5) * dx; // center of cell
            let p_pulse = amplitude_pa * (-0.5 * (x / width_m).powi(2)).exp();
            
            // Get current primitive variables
            let mut w = conserved_to_primitive(&self.u[i], gamma);
            w[2] += p_pulse; // Add pressure pulse
            
            self.u[i] = primitive_to_conserved(&w, gamma);
        }
        self.apply_boundary_conditions();
    }

    /// Take a simulation step using SSP-RK2 time integration
    pub fn step(&mut self, dt: f32) {
        let num_cells = self.num_cells;
        let gamma = self.config.gamma;
        let left_bc = self.config.left_bc;
        let right_bc = self.config.right_bc;

        // Step 1: Compute RHS at u^n
        let rhs1 = self.compute_rhs(&self.u);
        let mut u1 = self.u.clone();
        for i in 1..=num_cells {
            for k in 0..3 {
                u1[i][k] = self.u[i][k] + dt * rhs1[i][k];
            }
        }
        Self::apply_bc_on_state(left_bc, right_bc, gamma, num_cells, &mut u1);

        // Step 2: Compute RHS at u1
        let rhs2 = self.compute_rhs(&u1);
        let mut u_next = self.u.clone();
        for i in 1..=num_cells {
            for k in 0..3 {
                u_next[i][k] = 0.5 * self.u[i][k] + 0.5 * u1[i][k] + 0.5 * dt * rhs2[i][k];
            }
        }
        
        // Safety guard: clamp values to prevent NaN / blow up
        for i in 1..=num_cells {
            // Clamp density
            u_next[i][0] = u_next[i][0].max(1e-4);
            // Clamp internal energy
            let rho = u_next[i][0];
            let v = u_next[i][1] / rho;
            let ke = 0.5 * rho * v * v;
            let mut e_tot = u_next[i][2];
            
            // If total energy is less than kinetic energy, reset internal energy to baseline
            if e_tot - ke < 1e-4 {
                let p_min = 1e-2;
                e_tot = p_min / (gamma - 1.0) + ke;
                u_next[i][2] = e_tot;
            }
            
            // Final check for NaN
            if u_next[i][0].is_nan() || u_next[i][1].is_nan() || u_next[i][2].is_nan() {
                // If NaN occurred, reset cell to atmosphere
                let w_atm = [RHO_ATM, 0.0, P_ATM];
                u_next[i] = primitive_to_conserved(&w_atm, gamma);
            }
        }

        Self::apply_bc_on_state(left_bc, right_bc, gamma, num_cells, &mut u_next);
        self.u = u_next;
        self.t += dt;
    }

    /// Compute RHS: -1/dx * (F_{i+1/2} - F_{i-1/2}) + Sources
    fn compute_rhs(&self, state: &Vec<[f32; 3]>) -> Vec<[f32; 3]> {
        let num_cells = self.num_cells;
        let dx = self.config.dx;
        let gamma = self.config.gamma;
        let limiter = self.config.limiter;
        let friction = self.config.friction;

        // 1. Convert all cells to primitive variables W = [rho, v, p]
        let mut w = vec![[0.0; 3]; num_cells + 2];
        for i in 0..=(num_cells + 1) {
            w[i] = conserved_to_primitive(&state[i], gamma);
        }

        // 2. Compute slopes for active cells (1..=num_cells)
        let mut slopes = vec![[0.0; 3]; num_cells + 2];
        for i in 1..=num_cells {
            let dw_l = [
                w[i][0] - w[i - 1][0],
                w[i][1] - w[i - 1][1],
                w[i][2] - w[i - 1][2],
            ];
            let dw_r = [
                w[i + 1][0] - w[i][0],
                w[i + 1][1] - w[i][1],
                w[i + 1][2] - w[i][2],
            ];

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

        // 3. Compute interface fluxes (0..=num_cells interfaces)
        let mut fluxes = vec![[0.0; 3]; num_cells + 1];

        for j in 0..=num_cells {
            // Left state at interface j (from cell j)
            let mut w_l = w[j];
            if j >= 1 {
                for k in 0..3 {
                    w_l[k] += 0.5 * slopes[j][k];
                }
            }
            // Stability clamp
            w_l[0] = w_l[0].max(1e-4);
            w_l[2] = w_l[2].max(1e-2);

            // Right state at interface j (from cell j+1)
            let mut w_r = w[j + 1];
            if j + 1 <= num_cells {
                for k in 0..3 {
                    w_r[k] -= 0.5 * slopes[j + 1][k];
                }
            }
            // Stability clamp
            w_r[0] = w_r[0].max(1e-4);
            w_r[2] = w_r[2].max(1e-2);

            // Convert primitive to conserved at interface
            let u_l = primitive_to_conserved(&w_l, gamma);
            let u_r = primitive_to_conserved(&w_r, gamma);

            // Compute standard Euler fluxes
            let flux_l = euler_flux(&u_l, w_l[2]);
            let flux_r = euler_flux(&u_r, w_r[2]);

            // Local wave speeds (eigenvalues: v - a, v, v + a)
            let a_l = (gamma * w_l[2] / w_l[0]).sqrt();
            let a_r = (gamma * w_r[2] / w_r[0]).sqrt();
            
            let max_wave_speed = (w_l[1].abs() + a_l).max(w_r[1].abs() + a_r);

            // Rusanov (Local Lax-Friedrichs) flux
            let mut interface_flux = [0.0; 3];
            for k in 0..3 {
                interface_flux[k] = 0.5 * (flux_l[k] + flux_r[k]) - 0.5 * max_wave_speed * (u_r[k] - u_l[k]);
            }
            fluxes[j] = interface_flux;
        }

        // 4. Update cells with flux differences and source terms
        let mut rhs = vec![[0.0; 3]; num_cells + 2];
        let idx = 1.0 / dx;

        for i in 1..=num_cells {
            rhs[i][0] = -idx * (fluxes[i][0] - fluxes[i - 1][0]);
            rhs[i][1] = -idx * (fluxes[i][1] - fluxes[i - 1][1]);
            rhs[i][2] = -idx * (fluxes[i][2] - fluxes[i - 1][2]);

            // Friction source term (linear damping)
            if friction > 0.0 {
                let rho = state[i][0];
                let v = state[i][1] / rho.max(1e-5);
                
                let s_momentum = -friction * rho * v;
                let s_energy = -friction * rho * v * v;
                
                rhs[i][1] += s_momentum;
                rhs[i][2] += s_energy;
            }
        }

        rhs
    }

    pub fn apply_boundary_conditions(&mut self) {
        Self::apply_bc_on_state(
            self.config.left_bc,
            self.config.right_bc,
            self.config.gamma,
            self.num_cells,
            &mut self.u,
        );
    }

    fn apply_bc_on_state(
        left_bc: BoundaryType,
        right_bc: BoundaryType,
        gamma: f32,
        num_cells: usize,
        state: &mut Vec<[f32; 3]>,
    ) {
        // Left Boundary (index 0, connected to cell 1)
        match left_bc {
            BoundaryType::Closed => {
                let w1 = conserved_to_primitive(&state[1], gamma);
                let w0 = [w1[0], -w1[1], w1[2]]; // Mirror velocity, copy P & rho
                state[0] = primitive_to_conserved(&w0, gamma);
            }
            BoundaryType::Open => {
                let w1 = conserved_to_primitive(&state[1], gamma);
                let rho0 = if w1[1] > 0.0 { RHO_ATM } else { w1[0] };
                let w0 = [rho0, w1[1], P_ATM];
                state[0] = primitive_to_conserved(&w0, gamma);
            }
        }

        // Right Boundary (index num_cells+1, connected to cell num_cells)
        match right_bc {
            BoundaryType::Closed => {
                let wm = conserved_to_primitive(&state[num_cells], gamma);
                let w_m1 = [wm[0], -wm[1], wm[2]]; // Mirror velocity
                state[num_cells + 1] = primitive_to_conserved(&w_m1, gamma);
            }
            BoundaryType::Open => {
                let wm = conserved_to_primitive(&state[num_cells], gamma);
                let rho_m1 = if wm[1] < 0.0 { RHO_ATM } else { wm[0] };
                let w_m1 = [rho_m1, wm[1], P_ATM];
                state[num_cells + 1] = primitive_to_conserved(&w_m1, gamma);
            }
        }
    }

    // Helper data-extraction methods
    pub fn get_pressure(&self) -> Vec<f32> {
        let gamma = self.config.gamma;
        self.u[1..=self.num_cells]
            .iter()
            .map(|u_cell| conserved_to_primitive(u_cell, gamma)[2])
            .collect()
    }

    pub fn get_velocity(&self) -> Vec<f32> {
        let gamma = self.config.gamma;
        self.u[1..=self.num_cells]
            .iter()
            .map(|u_cell| conserved_to_primitive(u_cell, gamma)[1])
            .collect()
    }

    pub fn get_density(&self) -> Vec<f32> {
        let gamma = self.config.gamma;
        self.u[1..=self.num_cells]
            .iter()
            .map(|u_cell| conserved_to_primitive(u_cell, gamma)[0])
            .collect()
    }
}

// Convert primitive state [rho, v, p] to conserved state [rho, rho*v, E]
pub fn primitive_to_conserved(w: &[f32; 3], gamma: f32) -> [f32; 3] {
    let rho = w[0];
    let v = w[1];
    let p = w[2];
    
    let kinetic = 0.5 * rho * v * v;
    let internal = p / (gamma - 1.0);
    let energy = internal + kinetic;
    
    [rho, rho * v, energy]
}

// Convert conserved state [rho, rho*v, E] to primitive state [rho, v, p]
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

// Compute the Euler physical flux vector for a conserved state U and pressure P
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

// Limiters implementation
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
    fn test_primitive_conserved_conversion() {
        let w = [1.2, 10.0, 101325.0];
        let gamma = 1.4;
        let u = primitive_to_conserved(&w, gamma);
        let w_back = conserved_to_primitive(&u, gamma);
        assert!((w[0] - w_back[0]).abs() < 1e-4);
        assert!((w[1] - w_back[1]).abs() < 1e-4);
        assert!((w[2] - w_back[2]).abs() < 1.0);
    }

    #[test]
    fn test_solver_step() {
        let config = SolverConfig::default();
        let mut solver = Solver::new(config);
        
        let p_init = solver.get_pressure();
        assert!((p_init[0] - P_ATM).abs() < 1.0);
        
        solver.inject_pulse(20000.0, 0.1);
        let p_after = solver.get_pressure();
        assert!(p_after[0] > P_ATM);
        
        solver.step(1e-5);
        let p_step = solver.get_pressure();
        for p in p_step {
            assert!(!p.is_nan());
            assert!(p > 0.0);
        }
    }
}

