
#[derive(Debug, Clone)]
pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub last_error: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    pub fn step(&mut self, error: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }
        self.integral += error * dt;
        // Limit integral windup
        self.integral = self.integral.clamp(-0.1, 0.1);
        let derivative = (error - self.last_error) / dt;
        self.last_error = error;
        
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
    
    pub fn reset(&mut self, current_error: f32) {
        self.integral = 0.0;
        self.last_error = current_error;
    }
}

// 2D Bilinear Interpolation Helper
fn lookup_2d(x: f32, y: f32, xs: &[f32], ys: &[f32], table: &[Vec<f32>]) -> f32 {
    let x_idx = match xs.iter().position(|&val| val >= x) {
        Some(0) => 0,
        Some(i) => i - 1,
        None => xs.len() - 2,
    };
    let y_idx = match ys.iter().position(|&val| val >= y) {
        Some(0) => 0,
        Some(i) => i - 1,
        None => ys.len() - 2,
    };

    let x0 = xs[x_idx];
    let x1 = xs[x_idx + 1];
    let y0 = ys[y_idx];
    let y1 = ys[y_idx + 1];

    let q00 = table[x_idx][y_idx];
    let q01 = table[x_idx][y_idx + 1];
    let q10 = table[x_idx + 1][y_idx];
    let q11 = table[x_idx + 1][y_idx + 1];

    let r1 = ((x1 - x) / (x1 - x0)) * q00 + ((x - x0) / (x1 - x0)) * q10;
    let r2 = ((x1 - x) / (x1 - x0)) * q01 + ((x - x0) / (x1 - x0)) * q11;

    ((y1 - y) / (y1 - y0)) * r1 + ((y - y0) / (y1 - y0)) * r2
}

#[derive(Debug, Clone)]
pub struct Ecu {
    // Inputs (from sensors)
    pub map_pa: f32,
    pub engine_rpm: f32,
    pub tps: f32, // 0.0 to 1.0
    
    // Outputs (calculated by ECU)
    pub iac_lift: f32,
    pub ignition_timing_deg: f32,
    pub target_afr: f32,
    pub rev_limiter_active: bool,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    
    // Internal state
    pub is_cranking: bool,
    pub crank_to_run_timer: f32, // to delay transition until engine stabilizes
    pub iac_pid: PidController,
    
    // Calibration parameters
    pub target_idle_rpm: f32,
    pub redline_rpm: f32,
    pub manual_afr_control: bool,
    
    // Configurable maps and limits
    pub throttle_dashpot_lift: f32,
    pub overrun_dashpot_lift: f32,
    pub min_iac_lift: f32,
    pub max_iac_lift: f32,
    pub rpm_bins: Vec<f32>,
    pub map_bins_kpa: Vec<f32>,
    pub ve_table: Vec<Vec<f32>>,
    pub spark_table: Vec<Vec<f32>>,
}

impl Default for Ecu {
    fn default() -> Self {
        Self::from_config(&crate::config::EcuConfig::default())
    }
}

impl Ecu {
    pub fn new() -> Self {
        Self::default()
    }
    
    pub fn from_config(config: &crate::config::EcuConfig) -> Self {
        Self {
            map_pa: 101325.0,
            engine_rpm: 0.0,
            tps: 0.0,
            iac_lift: config.max_iac_lift / 2.0,
            ignition_timing_deg: config.ignition_timing_deg,
            target_afr: config.target_afr,
            rev_limiter_active: false,
            fuel_cut: false,
            spark_cut: false,
            is_cranking: true,
            crank_to_run_timer: 0.0,
            iac_pid: PidController::new(config.iac_kp, config.iac_ki, config.iac_kd),
            target_idle_rpm: config.target_idle_rpm,
            redline_rpm: config.redline_rpm,
            manual_afr_control: config.manual_afr_control,
            throttle_dashpot_lift: config.throttle_dashpot_lift,
            overrun_dashpot_lift: config.overrun_dashpot_lift,
            min_iac_lift: config.min_iac_lift,
            max_iac_lift: config.max_iac_lift,
            rpm_bins: config.rpm_bins.clone(),
            map_bins_kpa: config.map_bins_kpa.clone(),
            ve_table: config.ve_table.clone(),
            spark_table: config.spark_table.clone(),
        }
    }

    /// Update sensor values and run ECU logic (fueling, timing, idle, rev limits)
    pub fn update(&mut self, rpm: f32, map_pa: f32, tps: f32, dt: f32) {
        // Simple first-order sensor lag simulation
        let rpm_alpha = 0.2;
        let map_alpha = 0.2;
        self.engine_rpm = self.engine_rpm * (1.0 - rpm_alpha) + rpm * rpm_alpha;
        self.map_pa = self.map_pa * (1.0 - map_alpha) + map_pa * map_alpha;
        self.tps = tps;

        // Cranking / Running Detection
        if self.is_cranking {
            if self.engine_rpm > 350.0 {
                self.crank_to_run_timer += dt;
                if self.crank_to_run_timer > 0.3 { // wait 300ms of > 350rpm
                    self.is_cranking = false;
                }
            } else {
                self.crank_to_run_timer = 0.0;
            }
        } else {
            // Re-engage cranking only if engine drops below 150 RPM
            if self.engine_rpm < 150.0 {
                self.is_cranking = true;
                self.crank_to_run_timer = 0.0;
                let error = self.target_idle_rpm - self.engine_rpm;
                self.iac_pid.reset(error);
            }
        }

        // --- Rev Limiter (Hard Spark / Fuel cut with hysteresis) ---
        if self.engine_rpm > self.redline_rpm {
            self.rev_limiter_active = true;
            self.fuel_cut = true;
            self.spark_cut = true;
        } else if self.engine_rpm < self.redline_rpm - 200.0 {
            self.rev_limiter_active = false;
            self.fuel_cut = false;
            self.spark_cut = false;
        }

        // --- Idle Speed Control ---
        if self.is_cranking {
            // Cranking IAC lift is high to assist starting
            self.iac_lift = 0.010; 
            let error = self.target_idle_rpm - self.engine_rpm;
            self.iac_pid.reset(error);
        } else if self.tps > 0.02 {
            // Dashpot mode: hold IAC slightly open during throttle application
            self.iac_lift = self.throttle_dashpot_lift;
            let error = self.target_idle_rpm - self.engine_rpm;
            self.iac_pid.reset(error);
        } else if self.engine_rpm > self.target_idle_rpm + 500.0 {
            // Dashpot / overrun mode: engine is coasting down to idle.
            // Hold IAC at a high baseline to catch the engine and prevent PID integral windup.
            self.iac_lift = self.overrun_dashpot_lift;
            let error = self.target_idle_rpm - self.engine_rpm;
            self.iac_pid.reset(error);
        } else {
            // Closed-loop Idle Speed Control
            let error = self.target_idle_rpm - self.engine_rpm;
            let iac_adj = self.iac_pid.step(error, dt);
            self.iac_lift = (self.iac_lift + iac_adj).clamp(self.min_iac_lift, self.max_iac_lift);
        }

        // --- Target AFR Map ---
        if !self.manual_afr_control {
            if self.is_cranking {
                // Rich AFR for cold starting/cranking
                self.target_afr = 11.5;
            } else {
                // Base stoichiometric, slightly rich under high load (high MAP), lean at low load
                let map_kpa = self.map_pa / 1000.0;
                if map_kpa > 85.0 {
                    self.target_afr = 12.8; // Rich at wide-open-throttle
                } else if map_kpa < 30.0 {
                    self.target_afr = 15.5; // Lean on overrun / high vacuum
                } else {
                    self.target_afr = 14.7; // Perfect stoichiometric
                }
            }
        }

        // --- Spark Timing Map ---
        if self.is_cranking {
            // Low fixed advance during cranking to prevent starter kickback
            self.ignition_timing_deg = 5.0;
        } else {
            let min_rpm = *self.rpm_bins.first().unwrap_or(&1000.0);
            let max_rpm = *self.rpm_bins.last().unwrap_or(&16000.0);
            let clamped_rpm = self.engine_rpm.clamp(min_rpm, max_rpm);
            let clamped_map_kpa = (self.map_pa / 1000.0).clamp(20.0, 100.0);
            self.ignition_timing_deg = lookup_2d(clamped_rpm, clamped_map_kpa, &self.rpm_bins, &self.map_bins_kpa, &self.spark_table);
        }
    }

    /// Speed-density model: Predict incoming air mass per cylinder cycle, and return commanded fuel mass.
    /// V_disp: Cylinder displacement volume in m^3
    pub fn get_fuel_mass(&self, v_disp: f32, air_temp_k: f32) -> f32 {
        if self.fuel_cut {
            return 0.0;
        }

        let min_rpm = *self.rpm_bins.first().unwrap_or(&1000.0);
        let max_rpm = *self.rpm_bins.last().unwrap_or(&16000.0);
        let clamped_rpm = self.engine_rpm.clamp(min_rpm, max_rpm);
        let clamped_map_kpa = (self.map_pa / 1000.0).clamp(20.0, 100.0);
        let ve = lookup_2d(clamped_rpm, clamped_map_kpa, &self.rpm_bins, &self.map_bins_kpa, &self.ve_table);

        // Speed-density equation: m_air = (P * V * VE) / (R * T)
        let r_air = 287.05; // J/(kg*K)
        let m_air = (self.map_pa * v_disp * ve) / (r_air * air_temp_k);

        m_air / self.target_afr
    }

    /// Starter Motor Torque Model
    /// A real starter has high holding torque at 0 RPM, falling off linearly.
    pub fn get_starter_torque(&self, active: bool) -> f32 {
        if !active {
            return 0.0;
        }
        // If engine has caught (running mode), starter automatically disengages
        if !self.is_cranking {
            return 0.0;
        }

        // 120 N*m peak cranking torque, zero torque at 600 RPM
        let max_starter_torque = 120.0;
        let max_starter_rpm = 600.0;
        (max_starter_torque * (1.0 - self.engine_rpm / max_starter_rpm)).max(0.0)
    }
}
