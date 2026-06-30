#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValveType {
    Intake,
    Exhaust,
}

/// Compute the valve lift as a function of crank angle `theta`, `open_angle`, and `close_angle`.
/// The profile is triangular, starting at 0.0 at `open_angle`, peaking at 1.0, and returning to 0.0 at `close_angle`.
/// All angles are specified in radians.
pub fn get_valve_lift(theta: f32, open_angle: f32, close_angle: f32) -> f32 {
    let four_pi = 4.0 * std::f32::consts::PI;
    
    // Normalise angles to [0, 4*PI]
    let theta_norm = theta.rem_euclid(four_pi);
    let open_norm = open_angle.rem_euclid(four_pi);
    let close_norm = close_angle.rem_euclid(four_pi);
    
    let duration = (close_norm - open_norm).rem_euclid(four_pi);
    if duration < 1e-4 {
        return 0.0;
    }
    
    let relative = (theta_norm - open_norm).rem_euclid(four_pi);
    if relative < duration {
        let z = relative / duration;
        if z < 0.5 {
            2.0 * z
        } else {
            2.0 * (1.0 - z)
        }
    } else {
        0.0
    }
}
