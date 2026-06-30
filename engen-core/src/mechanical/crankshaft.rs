#[derive(Debug, Clone)]
pub struct Crankshaft {
    pub theta: f32,              // Crank angle in radians [0, 4*PI]
    pub omega: f32,              // Angular velocity in rad/s
    pub inertia: f32,            // Rotational inertia in kg*m^2
    pub friction_coeff: f32,     // Viscous friction coefficient
    pub coulomb_friction: f32,   // Coulomb (static/dry) friction torque in N*m
    pub starter_torque: f32,     // Starter motor torque applied when engaged (N*m)
}

impl Crankshaft {
    pub fn new(inertia: f32, friction_coeff: f32) -> Self {
        Self {
            theta: 0.0,
            omega: 0.0,
            inertia: inertia.max(1e-4),
            friction_coeff,
            coulomb_friction: 0.5, // Default: 0.5 N*m dry friction from piston rings/bearings
            starter_torque: 0.0,   // No starter torque by default
        }
    }

    /// Step the crankshaft dynamics forward in time.
    /// Returns the angular velocity `omega`.
    pub fn step(&mut self, pressure_torque: f32, dt: f32) -> f32 {
        // Viscous friction (proportional to speed)
        let viscous_torque = -self.friction_coeff * self.omega;
        
        // Coulomb (dry) friction: constant magnitude, opposes motion direction
        let coulomb_torque = if self.omega.abs() > 1e-3 {
            -self.coulomb_friction * self.omega.signum()
        } else {
            // At near-zero speed, Coulomb friction opposes net driving torque (stiction)
            let driving = pressure_torque + viscous_torque + self.starter_torque;
            if driving.abs() > self.coulomb_friction {
                -self.coulomb_friction * driving.signum()
            } else {
                // Not enough torque to overcome static friction — hold at zero
                -driving
            }
        };
        
        let net_torque = pressure_torque + viscous_torque + coulomb_torque + self.starter_torque;
        
        // Update angular velocity (I * d_omega/dt = torque)
        self.omega += (net_torque / self.inertia) * dt;
        
        // Prevent reverse rotation: engine only spins forward
        if self.omega < 0.0 {
            self.omega = 0.0;
        }
        
        // Clamp RPM to avoid numerical runaway (max ~11,500 RPM = 1200 rad/s)
        let max_omega = 1200.0;
        if self.omega > max_omega {
            self.omega = max_omega;
        }
        
        // Update crank angle (modulo 4*PI for a full four-stroke cycle)
        let four_pi = 4.0 * std::f32::consts::PI;
        self.theta = (self.theta + self.omega * dt).rem_euclid(four_pi);
        
        self.omega
    }
}

/// Compute piston displacement from TDC (Top Dead Center) using exact crank-slider kinematics.
/// Displacement ranges from 0.0 (at TDC) to `stroke` (at BDC).
pub fn piston_displacement(theta: f32, stroke: f32, conrod_length: f32) -> f32 {
    let r = stroke / 2.0;
    let l = conrod_length;
    let x = r * theta.cos() + (l * l - r * r * theta.sin().powi(2)).max(0.0).sqrt();
    let max_x = r + l;
    (max_x - x).max(0.0)
}

/// Compute the derivative dy/dtheta of piston displacement with respect to crank angle.
/// This is used to compute volume rate-of-change and torque:
/// Torque = Force * dy/dtheta,  dV/dt = Area * dy/dtheta * omega.
pub fn piston_dy_dtheta(theta: f32, stroke: f32, conrod_length: f32) -> f32 {
    let r = stroke / 2.0;
    let l = conrod_length;
    let sin_t = theta.sin();
    let cos_t = theta.cos();
    
    let sq = (l * l - r * r * sin_t * sin_t).max(0.0);
    if sq > 1e-6 {
        r * sin_t + (r * r * sin_t * cos_t) / sq.sqrt()
    } else {
        r * sin_t
    }
}
