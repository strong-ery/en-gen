/// Probabilistic misfire model based on residual fraction and air-fuel ratio.

#[derive(Debug, Clone)]
pub struct MisfireModel {
    pub rng_state: u32,
    pub last_misfire: bool,
    pub misfire_count: u32,
}

impl Default for MisfireModel {
    fn default() -> Self {
        Self {
            rng_state: 0xACE1, // non-zero seed
            last_misfire: false,
            misfire_count: 0,
        }
    }
}

impl MisfireModel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Simple 32-bit xorshift PRNG to keep the core library free of external dependencies.
    fn next_random_float(&mut self) -> f32 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        // Map to [0.0, 1.0]
        (x as f32) / (u32::MAX as f32)
    }

    /// Evaluate if cylinder combustion should result in a misfire.
    ///
    /// * `residual_frac` — Y_CO2 + Y_exhaust mass fractions (0.0 to 1.0)
    /// * `lambda` — relative AFR (1.0 = stoichiometric)
    pub fn should_misfire(&mut self, residual_frac: f32, lambda: f32) -> bool {
        // Base misfire probability increases from 0% at 22% residual to 100% at 67% residual
        let mut prob = ((residual_frac - 0.22).max(0.0) / 0.45).clamp(0.0, 1.0);

        // Double probability penalty outside [0.8, 1.2] lambda range (rich/lean instability)
        if lambda < 0.8 || lambda > 1.2 {
            prob = (prob * 2.0).clamp(0.0, 1.0);
            // Even if residual is zero, lean/rich extremes can misfire
            if prob < 0.05 {
                prob = 0.05;
            }
        }

        let roll = self.next_random_float();
        let misfire = roll < prob;

        self.last_misfire = misfire;
        if misfire {
            self.misfire_count += 1;
        }

        misfire
    }
}
