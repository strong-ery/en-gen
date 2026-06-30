/// Species tracking for gas composition through the CFD network.
///
/// Four species are tracked as mass fractions Y_i (kg_species / kg_total):
///   0 = O2       (oxygen)
///   1 = Fuel     (gasoline vapour, approximated as C8H18)
///   2 = CO2      (carbon dioxide — combustion product)
///   3 = Exhaust  (other combustion products: H2O, trace gases)
///
/// The remainder (1 - sum(Y_i)) is implicitly N2 (inert nitrogen).
/// Species are passively advected — they do not alter gamma or the Euler equations.

pub const NUM_SPECIES: usize = 4;

pub const I_O2: usize = 0;
pub const I_FUEL: usize = 1;
pub const I_CO2: usize = 2;
pub const I_EXHAUST: usize = 3;

/// Ambient air composition by mass fraction.
/// O2 = 23.3%, N2 = 76.7% (simplified dry air, no Ar).
pub const AIR_Y: [f32; NUM_SPECIES] = [0.233, 0.0, 0.0, 0.0];

/// Stoichiometric O2-to-fuel mass ratio for gasoline (C8H18).
///
/// 2 C8H18 + 25 O2 → 16 CO2 + 18 H2O
/// Molar masses: C8H18 = 114, O2 = 32, CO2 = 44, H2O = 18
/// Mass ratio: (25 * 32) / (2 * 114) = 800 / 228 ≈ 3.51
pub const STOICH_O2_PER_FUEL: f32 = 3.51;

/// Stoichiometric CO2 produced per kg of fuel burned.
/// (16 * 44) / (2 * 114) = 704 / 228 ≈ 3.09
pub const STOICH_CO2_PER_FUEL: f32 = 3.09;

/// Stoichiometric H2O (exhaust) produced per kg of fuel burned.
/// (18 * 18) / (2 * 114) = 324 / 228 ≈ 1.42
pub const STOICH_EXHAUST_PER_FUEL: f32 = 1.42;

/// Stoichiometric air-fuel ratio by mass (for gasoline).
/// AFR_stoich = (O2_per_fuel) / (mass_fraction_O2_in_air)
/// = 3.51 / 0.233 ≈ 14.7
pub const AFR_STOICH: f32 = 14.7;

/// Clamp all species mass fractions to [0, 1] and ensure total <= 1.
#[inline]
pub fn clamp_species(y: &mut [f32; NUM_SPECIES]) {
    let mut total = 0.0f32;
    for i in 0..NUM_SPECIES {
        y[i] = y[i].max(0.0);
        total += y[i];
    }
    // If total exceeds 1.0, renormalise proportionally
    if total > 1.0 {
        let inv = 1.0 / total;
        for i in 0..NUM_SPECIES {
            y[i] *= inv;
        }
    }
}
