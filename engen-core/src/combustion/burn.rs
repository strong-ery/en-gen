/// Wiebe burn model and AFR-dependent combustion efficiency.

use crate::cfd::species::{I_O2, I_FUEL, I_CO2, I_EXHAUST, STOICH_O2_PER_FUEL, AFR_STOICH};

/// Wiebe cumulative burn fraction.
///
/// x_b(θ) = 1 - exp(-a * (θ / θ_d)^(m+1))
///
/// * `phase` — current crank angle progress since spark (radians)
/// * `duration` — total combustion duration (radians)
/// * `a` — efficiency parameter (typically 5.0 for 99.3% burn)
/// * `m` — form factor (typically 2 → exponent = 3)
#[inline]
pub fn wiebe_fraction(phase: f32, duration: f32, a: f32, m_plus_1: usize) -> f32 {
    let x = (phase / duration).min(1.5); // clamp to avoid huge exponents
    1.0 - (-a * x.powi(m_plus_1 as i32)).exp()
}

/// Combustion thermal efficiency as a function of lambda (relative AFR).
///
/// Peak efficiency ~0.38 at λ ≈ 1.05 (slightly lean of stoich).
/// Falls off smoothly to 0 at λ < 0.5 (very rich) and λ > 1.5 (very lean).
/// Uses a Gaussian-like bell curve centred at λ = 1.05.
pub fn afr_efficiency(lambda: f32) -> f32 {
    let peak_lambda = 1.05;
    let sigma = 0.30; // width of the efficiency window
    let peak_eta = 0.38;

    let x = (lambda - peak_lambda) / sigma;
    let eta = peak_eta * (-0.5 * x * x).exp();

    // Hard cutoff outside combustible range
    if lambda < 0.4 || lambda > 1.6 {
        0.0
    } else {
        eta
    }
}

/// Check whether combustion is physically viable given in-cylinder species.
///
/// Returns `false` (misfire) if:
/// - O2 mass fraction < 0.02 (insufficient oxidiser)
/// - Fuel mass fraction < 0.001 (no fuel)
/// - Residual exhaust fraction > 0.60 (too much dilution)
pub fn combustion_viable(y_o2: f32, y_fuel: f32, residual_frac: f32) -> bool {
    if y_o2 < 0.02 {
        return false;
    }
    if y_fuel < 0.001 {
        return false;
    }
    if residual_frac > 0.60 {
        return false;
    }
    true
}

/// Compute the actual lambda (relative AFR) from in-cylinder species mass fractions.
///
/// lambda = (Y_O2 / Y_fuel) / stoich_O2_per_fuel
/// lambda = 1.0 at stoichiometric
/// lambda > 1.0 = lean, lambda < 1.0 = rich
pub fn lambda_from_species(y_o2: f32, y_fuel: f32) -> f32 {
    if y_fuel < 1e-6 {
        return 99.0; // infinitely lean (no fuel)
    }
    (y_o2 / y_fuel) / STOICH_O2_PER_FUEL
}

/// Compute the actual AFR from lambda.
pub fn afr_from_lambda(lambda: f32) -> f32 {
    lambda * AFR_STOICH
}

/// Consume species during a combustion step.
///
/// Given the burn fraction increment `d_xb` and the total fuel mass fraction
/// available at spark, consume O2 and fuel, produce CO2 and exhaust products.
///
/// Returns the updated species array.
pub fn consume_species(
    species: &mut [f32; 4],
    d_xb: f32,
    initial_fuel_fraction: f32,
) {
    // Amount of fuel burned this step (as mass fraction of total cylinder gas)
    let fuel_burned = d_xb * initial_fuel_fraction;

    // Stoichiometric consumption
    let o2_consumed = fuel_burned * STOICH_O2_PER_FUEL;
    let co2_produced = fuel_burned * super::burn::co2_per_fuel();
    let exhaust_produced = fuel_burned * super::burn::exhaust_per_fuel();

    species[I_FUEL] = (species[I_FUEL] - fuel_burned).max(0.0);
    species[I_O2] = (species[I_O2] - o2_consumed).max(0.0);
    species[I_CO2] = (species[I_CO2] + co2_produced).min(1.0);
    species[I_EXHAUST] = (species[I_EXHAUST] + exhaust_produced).min(1.0);

    // Renormalise to ensure total <= 1.0
    crate::cfd::species::clamp_species(species);
}

/// CO2 produced per kg of fuel burned.
pub fn co2_per_fuel() -> f32 {
    crate::cfd::species::STOICH_CO2_PER_FUEL
}

/// H2O/exhaust produced per kg of fuel burned.
pub fn exhaust_per_fuel() -> f32 {
    crate::cfd::species::STOICH_EXHAUST_PER_FUEL
}
