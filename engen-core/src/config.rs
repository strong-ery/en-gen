use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineConfig {
    pub bore: f32,
    pub stroke: f32,
    pub conrod_length: f32,
    pub compression_ratio: f32,
    pub inertia: f32,
    pub viscous_friction: f32,
    pub wall_friction: f32,
    pub heat_transfer: f32,
    pub intake_valve_open_deg: f32,
    pub intake_valve_close_deg: f32,
    pub exhaust_valve_open_deg: f32,
    pub exhaust_valve_close_deg: f32,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            bore: 0.067,
            stroke: 0.0451,
            conrod_length: 0.083,
            compression_ratio: 12.9,
            inertia: 0.003,
            viscous_friction: 0.0003,
            wall_friction: 0.015,
            heat_transfer: 1.0,
            intake_valve_open_deg: 320.0,
            intake_valve_close_deg: 605.0,
            exhaust_valve_open_deg: 120.0,
            exhaust_valve_close_deg: 395.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcuConfig {
    pub target_idle_rpm: f32,
    pub redline_rpm: f32,
    pub target_afr: f32,
    pub manual_afr_control: bool,
    pub ignition_timing_deg: f32,
    
    // Idle Control Parameters
    pub iac_kp: f32,
    pub iac_ki: f32,
    pub iac_kd: f32,
    pub throttle_dashpot_lift: f32,
    pub overrun_dashpot_lift: f32,
    pub min_iac_lift: f32,
    pub max_iac_lift: f32,
    
    // Maps
    pub rpm_bins: Vec<f32>,
    pub map_bins_kpa: Vec<f32>,
    pub ve_table: Vec<Vec<f32>>,
    pub spark_table: Vec<Vec<f32>>,
}

impl Default for EcuConfig {
    fn default() -> Self {
        Self {
            target_idle_rpm: 1500.0,
            redline_rpm: 16000.0,
            target_afr: 14.7,
            manual_afr_control: false,
            ignition_timing_deg: 15.0,
            
            iac_kp: 0.0005,
            iac_ki: 0.00005,
            iac_kd: 0.00002,
            throttle_dashpot_lift: 0.002,
            overrun_dashpot_lift: 0.012, // High enough to catch falling sportbike engine
            min_iac_lift: 0.002,
            max_iac_lift: 0.020,
            
            rpm_bins: vec![1000.0, 3000.0, 6000.0, 9000.0, 12000.0, 14000.0, 16000.0],
            map_bins_kpa: vec![20.0, 40.0, 60.0, 80.0, 100.0],
            ve_table: vec![
                vec![0.50, 0.55, 0.62, 0.68, 0.70],  // 1000 RPM
                vec![0.60, 0.70, 0.80, 0.86, 0.88],  // 3000 RPM
                vec![0.65, 0.75, 0.85, 0.90, 0.92],  // 6000 RPM
                vec![0.70, 0.80, 0.90, 0.95, 0.98],  // 9000 RPM
                vec![0.75, 0.85, 0.95, 1.05, 0.98],  // 12000 RPM
                vec![0.75, 0.85, 0.95, 1.02, 0.95],  // 14000 RPM
                vec![0.70, 0.80, 0.90, 0.95, 0.92],  // 16000 RPM (keeps breathing)
            ],
            spark_table: vec![
                vec![22.0, 20.0, 18.0, 14.0, 10.0],  // 1000 RPM
                vec![36.0, 34.0, 30.0, 24.0, 18.0],  // 3000 RPM
                vec![40.0, 38.0, 34.0, 28.0, 24.0],  // 6000 RPM
                vec![42.0, 40.0, 36.0, 30.0, 26.0],  // 9000 RPM
                vec![44.0, 42.0, 40.0, 36.0, 34.0],  // 12000 RPM
                vec![46.0, 44.0, 42.0, 38.0, 36.0],  // 14000 RPM
                vec![48.0, 46.0, 44.0, 40.0, 38.0],  // 16000 RPM
            ],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TubeConfig {
    pub id: usize,
    pub name: String,
    pub num_cells: usize,
    pub p0: [f32; 2],
    pub p1: [f32; 2],
    pub p2: [f32; 2],
    pub p3: [f32; 2],
    pub r_start: f32,
    pub r_mid: f32,
    pub r_end: f32,
    pub radius_profile: String, // "Linear" or "ExpansionChamber"
    pub left_bc: String, // "Closed", "Open", "Valve:<lift>"
    pub right_bc: String, // "Closed", "Open", "Valve:<lift>"
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JunctionConnectionConfig {
    pub tube_id: usize,
    pub side: String, // "Left" or "Right"
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JunctionConfig {
    pub id: usize,
    pub pos: [f32; 2],
    pub connections: Vec<JunctionConnectionConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CylinderConnectionConfig {
    pub tube_id: usize,
    pub side: String, // "Left" or "Right"
    pub valve_type: String, // "Intake" or "Exhaust"
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CylinderConfig {
    pub id: usize,
    pub crank_offset_deg: f32,
    pub connections: Vec<CylinderConnectionConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologyConfig {
    pub tubes: Vec<TubeConfig>,
    pub junctions: Vec<JunctionConfig>,
    pub cylinders: Vec<CylinderConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionConfig {
    pub vehicle_mass_kg: f32,
    pub wheel_radius_m: f32,
    pub primary_drive_ratio: f32,
    pub final_drive_ratio: f32,
    pub gear_ratios: [f32; 6],
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            vehicle_mass_kg: 270.0, // Bike wet weight + rider
            wheel_radius_m: 0.315,  // 180/55 ZR17
            primary_drive_ratio: 1.900,
            final_drive_ratio: 2.867,
            gear_ratios: [2.846, 2.200, 1.850, 1.600, 1.421, 1.300],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FullConfig {
    pub engine: EngineConfig,
    pub ecu: EcuConfig,
    pub transmission: TransmissionConfig,
    pub topology: Option<TopologyConfig>,
}

impl Default for FullConfig {
    fn default() -> Self {
        Self {
            engine: EngineConfig::default(),
            ecu: EcuConfig::default(),
            transmission: TransmissionConfig::default(),
            topology: Some(TopologyConfig::default_zx6r()),
        }
    }
}

impl TopologyConfig {
    pub fn default_zx6r() -> Self {
        Self {
            tubes: vec![
                TubeConfig {
                    id: 0,
                    name: "Intake Main".to_string(),
                    num_cells: 10,
                    p0: [-1.2, 0.0],
                    p1: [-1.0, 0.0],
                    p2: [-0.8, 0.0],
                    p3: [-0.7, 0.0],
                    r_start: 0.040,
                    r_mid: 0.040,
                    r_end: 0.040,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Valve:1.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 1,
                    name: "Intake Runner 1".to_string(),
                    num_cells: 5,
                    p0: [-0.3, 0.6],
                    p1: [-0.25, 0.6],
                    p2: [-0.2, 0.6],
                    p3: [-0.15, 0.6],
                    r_start: 0.022,
                    r_mid: 0.022,
                    r_end: 0.022,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Closed".to_string(),
                    right_bc: "Valve:0.0".to_string(),
                },
                TubeConfig {
                    id: 2,
                    name: "Intake Runner 2".to_string(),
                    num_cells: 5,
                    p0: [-0.3, 0.2],
                    p1: [-0.25, 0.2],
                    p2: [-0.2, 0.2],
                    p3: [-0.15, 0.2],
                    r_start: 0.022,
                    r_mid: 0.022,
                    r_end: 0.022,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Closed".to_string(),
                    right_bc: "Valve:0.0".to_string(),
                },
                TubeConfig {
                    id: 3,
                    name: "Intake Runner 3".to_string(),
                    num_cells: 5,
                    p0: [-0.3, -0.2],
                    p1: [-0.25, -0.2],
                    p2: [-0.2, -0.2],
                    p3: [-0.15, -0.2],
                    r_start: 0.022,
                    r_mid: 0.022,
                    r_end: 0.022,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Closed".to_string(),
                    right_bc: "Valve:0.0".to_string(),
                },
                TubeConfig {
                    id: 4,
                    name: "Intake Runner 4".to_string(),
                    num_cells: 5,
                    p0: [-0.3, -0.6],
                    p1: [-0.25, -0.6],
                    p2: [-0.2, -0.6],
                    p3: [-0.15, -0.6],
                    r_start: 0.022,
                    r_mid: 0.022,
                    r_end: 0.022,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Closed".to_string(),
                    right_bc: "Valve:0.0".to_string(),
                },
                TubeConfig {
                    id: 5,
                    name: "Header 1".to_string(),
                    num_cells: 8,
                    p0: [0.15, 0.6],
                    p1: [0.3, 0.6],
                    p2: [0.4, 0.3],
                    p3: [0.5, 0.0],
                    r_start: 0.020,
                    r_mid: 0.020,
                    r_end: 0.020,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Valve:0.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 6,
                    name: "Header 2".to_string(),
                    num_cells: 8,
                    p0: [0.15, 0.2],
                    p1: [0.3, 0.2],
                    p2: [0.4, 0.1],
                    p3: [0.5, 0.0],
                    r_start: 0.020,
                    r_mid: 0.020,
                    r_end: 0.020,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Valve:0.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 7,
                    name: "Header 3".to_string(),
                    num_cells: 8,
                    p0: [0.15, -0.2],
                    p1: [0.3, -0.2],
                    p2: [0.4, -0.1],
                    p3: [0.5, 0.0],
                    r_start: 0.020,
                    r_mid: 0.020,
                    r_end: 0.020,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Valve:0.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 8,
                    name: "Header 4".to_string(),
                    num_cells: 8,
                    p0: [0.15, -0.6],
                    p1: [0.3, -0.6],
                    p2: [0.4, -0.3],
                    p3: [0.5, 0.0],
                    r_start: 0.020,
                    r_mid: 0.020,
                    r_end: 0.020,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Valve:0.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 9,
                    name: "Pre-chamber Resonator".to_string(),
                    num_cells: 10,
                    p0: [0.4, 0.0],
                    p1: [0.5, 0.0],
                    p2: [0.7, 0.0],
                    p3: [0.8, 0.0],
                    r_start: 0.028,
                    r_mid: 0.08,
                    r_end: 0.028,
                    radius_profile: "ExpansionChamber".to_string(),
                    left_bc: "Valve:1.0".to_string(),
                    right_bc: "Closed".to_string(),
                },
                TubeConfig {
                    id: 10,
                    name: "Tailpipe".to_string(),
                    num_cells: 10,
                    p0: [0.8, 0.0],
                    p1: [0.9, 0.0],
                    p2: [1.0, 0.0],
                    p3: [1.2, 0.0],
                    r_start: 0.028,
                    r_mid: 0.028,
                    r_end: 0.028,
                    radius_profile: "Linear".to_string(),
                    left_bc: "Closed".to_string(),
                    right_bc: "Open".to_string(),
                }
            ],
            junctions: vec![
                JunctionConfig {
                    id: 0,
                    pos: [-0.7, 0.0],
                    connections: vec![
                        JunctionConnectionConfig { tube_id: 0, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 1, side: "Left".to_string() },
                        JunctionConnectionConfig { tube_id: 2, side: "Left".to_string() },
                        JunctionConnectionConfig { tube_id: 3, side: "Left".to_string() },
                        JunctionConnectionConfig { tube_id: 4, side: "Left".to_string() },
                    ],
                },
                JunctionConfig {
                    id: 1,
                    pos: [0.4, 0.0],
                    connections: vec![
                        JunctionConnectionConfig { tube_id: 5, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 6, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 7, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 8, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 9, side: "Left".to_string() },
                    ],
                },
                JunctionConfig {
                    id: 2,
                    pos: [0.8, 0.0],
                    connections: vec![
                        JunctionConnectionConfig { tube_id: 9, side: "Right".to_string() },
                        JunctionConnectionConfig { tube_id: 10, side: "Left".to_string() },
                    ],
                }
            ],
            cylinders: vec![
                CylinderConfig {
                    id: 0,
                    crank_offset_deg: 0.0,
                    connections: vec![
                        CylinderConnectionConfig { tube_id: 1, side: "Right".to_string(), valve_type: "Intake".to_string() },
                        CylinderConnectionConfig { tube_id: 5, side: "Left".to_string(), valve_type: "Exhaust".to_string() },
                    ],
                },
                CylinderConfig {
                    id: 1,
                    crank_offset_deg: 180.0, // 1 * 180
                    connections: vec![
                        CylinderConnectionConfig { tube_id: 2, side: "Right".to_string(), valve_type: "Intake".to_string() },
                        CylinderConnectionConfig { tube_id: 6, side: "Left".to_string(), valve_type: "Exhaust".to_string() },
                    ],
                },
                CylinderConfig {
                    id: 2,
                    crank_offset_deg: 540.0, // 3 * 180
                    connections: vec![
                        CylinderConnectionConfig { tube_id: 3, side: "Right".to_string(), valve_type: "Intake".to_string() },
                        CylinderConnectionConfig { tube_id: 7, side: "Left".to_string(), valve_type: "Exhaust".to_string() },
                    ],
                },
                CylinderConfig {
                    id: 3,
                    crank_offset_deg: 360.0, // 2 * 180
                    connections: vec![
                        CylinderConnectionConfig { tube_id: 4, side: "Right".to_string(), valve_type: "Intake".to_string() },
                        CylinderConnectionConfig { tube_id: 8, side: "Left".to_string(), valve_type: "Exhaust".to_string() },
                    ],
                }
            ],
        }
    }
}

impl FullConfig {
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let file = File::open(path)?;
        let config: Self = serde_yaml::from_reader(file)?;
        Ok(config)
    }

    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> Result<(), Box<dyn std::error::Error>> {
        let file = File::create(path)?;
        serde_yaml::to_writer(file, self)?;
        Ok(())
    }
}
