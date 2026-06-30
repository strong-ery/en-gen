use crate::cfd::solver::{TubeSide, P_ATM, RHO_ATM, R_AIR};
use crate::cfd::species::{AIR_Y, NUM_SPECIES};
use crate::mechanical::valve::ValveType;
use crate::mechanical::crankshaft::piston_displacement;

#[derive(Debug, Clone)]
pub struct CylinderConnection {
    pub tube_id: usize,
    pub side: TubeSide,
    pub valve_type: ValveType,
}

#[derive(Debug, Clone)]
pub struct Cylinder {
    pub id: usize,
    pub bore: f32,              // Cylinder bore in meters
    pub stroke: f32,            // Stroke in meters
    pub conrod_length: f32,     // Connecting rod length in meters
    pub compression_ratio: f32, // Compression ratio (e.g. 9.5)
    pub clearance_volume: f32,  // Clearance volume in m^3
    pub piston_area: f32,       // Piston face area in m^2
    
    // Thermodynamics State (Conserved Variables)
    pub mass: f32,              // Total gas mass in kg
    pub energy: f32,            // Total internal energy in Joules (U)
    
    // Thermodynamics State (Primitive Variables)
    pub p: f32,                 // Pressure in Pa
    pub rho: f32,               // Density in kg/m^3
    pub temp: f32,              // Temperature in K
    
    // Volume tracking
    pub volume: f32,            // Cylinder volume in m^3
    pub delta_vol: f32,         // Volume change in current sub-step (V_new - V_old) in m^3
    
    // Runge-Kutta Step Workspaces
    pub mass_temp: f32,
    pub energy_temp: f32,
    pub rho_temp: f32,
    pub p_temp: f32,
    
    // Species Tracking
    pub species: [f32; NUM_SPECIES],
    pub species_temp: [f32; NUM_SPECIES],
    pub residual_fraction: f32,
    pub initial_fuel_fraction: f32,
    
    // Valve and manifold coupling connections
    pub connections: Vec<CylinderConnection>,
    pub combustion_phase: f32,
}

impl Cylinder {
    pub fn new(
        id: usize,
        bore: f32,
        stroke: f32,
        conrod_length: f32,
        compression_ratio: f32,
        connections: Vec<CylinderConnection>,
    ) -> Self {
        let piston_area = std::f32::consts::PI * 0.25 * bore * bore;
        let stroke_volume = piston_area * stroke;
        let clearance_volume = stroke_volume / (compression_ratio - 1.0);
        
        let mut cyl = Self {
            id,
            bore,
            stroke,
            conrod_length,
            compression_ratio,
            clearance_volume,
            piston_area,
            mass: 0.0,
            energy: 0.0,
            p: P_ATM,
            rho: RHO_ATM,
            temp: 293.15, // T_ATM
            volume: clearance_volume,
            delta_vol: 0.0,
            mass_temp: 0.0,
            energy_temp: 0.0,
            rho_temp: RHO_ATM,
            p_temp: P_ATM,
            species: AIR_Y,
            species_temp: AIR_Y,
            residual_fraction: 0.0,
            initial_fuel_fraction: 0.0,
            connections,
            combustion_phase: -1.0,
        };
        cyl.reset_state();
        cyl
    }

    /// Reset thermodynamics state to quiescent atmosphere at TDC volume.
    pub fn reset_state(&mut self) {
        let init_theta = 0.0; // TDC
        self.volume = self.volume_at(init_theta);
        self.delta_vol = 0.0;
        
        self.rho = RHO_ATM;
        self.p = P_ATM;
        self.temp = 293.15; // T_ATM
        
        self.mass = self.rho * self.volume;
        // Total internal energy: U = P * V / (gamma - 1)
        let gamma = 1.4;
        self.energy = self.p * self.volume / (gamma - 1.0);
        
        self.mass_temp = self.mass;
        self.energy_temp = self.energy;
        self.rho_temp = self.rho;
        self.p_temp = self.p;
        self.species = AIR_Y;
        self.species_temp = AIR_Y;
        self.residual_fraction = 0.0;
        self.initial_fuel_fraction = 0.0;
        self.combustion_phase = -1.0;
    }

    /// Update cylinder dimensions and recalculate geometric parameters.
    pub fn update_geometry(&mut self, bore: f32, stroke: f32, conrod_length: f32, compression_ratio: f32) {
        self.bore = bore;
        self.stroke = stroke;
        self.conrod_length = conrod_length;
        self.compression_ratio = compression_ratio;
        
        self.piston_area = std::f32::consts::PI * 0.25 * bore * bore;
        let stroke_volume = self.piston_area * stroke;
        self.clearance_volume = stroke_volume / (compression_ratio - 1.0);
        
        // After resizing, reset state to avoid pressure jumps or NaN
        self.reset_state();
    }

    /// Calculate cylinder volume in m^3 at a given crank angle.
    pub fn volume_at(&self, theta: f32) -> f32 {
        let y = piston_displacement(theta, self.stroke, self.conrod_length);
        self.clearance_volume + self.piston_area * y
    }

    /// Update primitive thermodynamic variables based on current conserved mass and energy.
    pub fn update_thermodynamics(&mut self, gamma: f32) {
        self.rho = self.mass / self.volume;
        self.p = (gamma - 1.0) * self.energy / self.volume;
        self.p = self.p.max(1e-2);
        self.temp = self.p / (self.rho.max(1e-4) * R_AIR);
    }

    /// Update temp workspaces primitive variables based on temp mass and energy.
    pub fn update_thermodynamics_temp(&mut self, gamma: f32) {
        self.rho_temp = self.mass_temp / self.volume;
        self.p_temp = (gamma - 1.0) * self.energy_temp / self.volume;
        self.p_temp = self.p_temp.max(1e-2);
    }
}
