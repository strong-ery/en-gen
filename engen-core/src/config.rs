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
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            bore: 0.067,
            stroke: 0.0425,
            conrod_length: 0.083,
            compression_ratio: 12.9,
            inertia: 0.003,
            viscous_friction: 0.0003,
            wall_friction: 0.025,
            heat_transfer: 30.0,
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
}

impl Default for EcuConfig {
    fn default() -> Self {
        Self {
            target_idle_rpm: 1500.0,
            redline_rpm: 16000.0,
            target_afr: 14.7,
            manual_afr_control: false,
            ignition_timing_deg: 15.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FullConfig {
    pub engine: EngineConfig,
    pub ecu: EcuConfig,
}

impl Default for FullConfig {
    fn default() -> Self {
        Self {
            engine: EngineConfig::default(),
            ecu: EcuConfig::default(),
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
