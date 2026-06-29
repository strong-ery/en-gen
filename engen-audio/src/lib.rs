use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::{Arc, Mutex};

/// Shared filter parameters that the UI can adjust at runtime.
#[derive(Clone, Copy, Debug)]
pub struct AudioFilterParams {
    /// Low-pass filter cutoff frequency in Hz (e.g. 8000.0).
    /// Set to 0.0 or >= Nyquist to effectively bypass.
    pub lp_cutoff_hz: f32,
    /// High-pass filter cutoff frequency in Hz (e.g. 30.0).
    /// Set to 0.0 to effectively bypass.
    pub hp_cutoff_hz: f32,
}

impl Default for AudioFilterParams {
    fn default() -> Self {
        Self {
            lp_cutoff_hz: 5500.0,
            hp_cutoff_hz: 1.0,
        }
    }
}

#[derive(Clone, Copy)]
struct AudioFilter {
    sample_rate: f32,

    // Low-pass state
    lp_state: f32,
    lp_alpha: f32,
    lp_cutoff_hz: f32,

    // High-pass state (first-order IIR)
    hp_state: f32,
    hp_last_in: f32,
    hp_alpha: f32,
    hp_cutoff_hz: f32,
}

impl AudioFilter {
    fn new(sample_rate: f32, lp_cutoff_hz: f32, hp_cutoff_hz: f32) -> Self {
        let lp_alpha = Self::compute_lp_alpha(sample_rate, lp_cutoff_hz);
        let hp_alpha = Self::compute_hp_alpha(sample_rate, hp_cutoff_hz);

        Self {
            sample_rate,
            lp_state: 0.0,
            lp_alpha,
            lp_cutoff_hz,
            hp_state: 0.0,
            hp_last_in: 0.0,
            hp_alpha,
            hp_cutoff_hz,
        }
    }

    /// Compute LP coefficient: exponential smoothing from cutoff frequency.
    fn compute_lp_alpha(sample_rate: f32, cutoff_hz: f32) -> f32 {
        if cutoff_hz <= 0.0 || cutoff_hz >= sample_rate * 0.5 {
            // Bypass: alpha=0 means output = input
            0.0
        } else {
            (-2.0 * std::f32::consts::PI * cutoff_hz / sample_rate).exp()
        }
    }

    /// Compute HP coefficient from cutoff frequency.
    /// alpha ~= RC / (RC + dt) where RC = 1 / (2*pi*fc)
    fn compute_hp_alpha(sample_rate: f32, cutoff_hz: f32) -> f32 {
        if cutoff_hz <= 0.0 {
            // Bypass: alpha=1 means no high-pass filtering
            1.0
        } else {
            let rc = 1.0 / (2.0 * std::f32::consts::PI * cutoff_hz);
            let dt = 1.0 / sample_rate;
            rc / (rc + dt)
        }
    }

    /// Update coefficients if cutoffs have changed.
    fn update_params(&mut self, lp_cutoff_hz: f32, hp_cutoff_hz: f32) {
        if (self.lp_cutoff_hz - lp_cutoff_hz).abs() > 0.01 {
            self.lp_cutoff_hz = lp_cutoff_hz;
            self.lp_alpha = Self::compute_lp_alpha(self.sample_rate, lp_cutoff_hz);
        }
        if (self.hp_cutoff_hz - hp_cutoff_hz).abs() > 0.01 {
            self.hp_cutoff_hz = hp_cutoff_hz;
            self.hp_alpha = Self::compute_hp_alpha(self.sample_rate, hp_cutoff_hz);
        }
    }

    fn process(&mut self, input: f32) -> f32 {
        // High-pass filter (removes subsonic rumble / DC offset)
        let hp_out = self.hp_alpha * (self.hp_state + input - self.hp_last_in);
        self.hp_last_in = input;
        self.hp_state = hp_out;

        // Low-pass filter (removes high-frequency crackling / aliasing artifacts)
        let lp_out = self.lp_alpha * self.lp_state + (1.0 - self.lp_alpha) * hp_out;
        self.lp_state = lp_out;

        lp_out
    }
}

pub struct AudioSystem {
    _stream: cpal::Stream,
    _buffer: Arc<Mutex<Vec<f32>>>,
}

impl AudioSystem {
    pub fn new(
        buffer: Arc<Mutex<Vec<f32>>>,
        filter_params: Arc<Mutex<AudioFilterParams>>,
    ) -> Result<Self, String> {
        let host = cpal::default_host();
        let device = host.default_output_device().ok_or("No default output device found")?;
        
        let config = device.default_output_config().map_err(|e| e.to_string())?;
        let sample_format = config.sample_format();
        let stream_config: cpal::StreamConfig = config.into();
        
        let sample_rate = stream_config.sample_rate as f64;
        let channels = stream_config.channels as usize;
        
        let buffer_clone = Arc::clone(&buffer);
        let mut read_phase = 0.0f64;
        let ratio = 64000.0 / sample_rate;
        
        let initial_params = {
            let p = filter_params.lock().unwrap();
            *p
        };
        let mut filters = vec![
            AudioFilter::new(sample_rate as f32, initial_params.lp_cutoff_hz, initial_params.hp_cutoff_hz);
            channels
        ];

        let filter_params_clone = Arc::clone(&filter_params);
        // Counter for periodic param refresh (avoid locking every sample)
        let mut param_refresh_counter: usize = 0;
        
        let stream = match sample_format {
            cpal::SampleFormat::F32 => {
                device.build_output_stream(
                    stream_config,
                    move |data: &mut [f32], _| {
                        // Refresh filter params every ~256 frames
                        param_refresh_counter += 1;
                        if param_refresh_counter % 256 == 0 {
                            if let Ok(params) = filter_params_clone.try_lock() {
                                for f in &mut filters {
                                    f.update_params(params.lp_cutoff_hz, params.hp_cutoff_hz);
                                }
                            }
                        }

                        let mut buf = buffer_clone.lock().unwrap();
                        let available = buf.len();
                        
                        for frame in data.chunks_mut(channels) {
                            let idx = read_phase.floor() as usize;
                            let fract = read_phase - idx as f64;
                            
                            let val = if idx + 1 < available {
                                (1.0 - fract) as f32 * buf[idx] + fract as f32 * buf[idx + 1]
                            } else if idx < available {
                                buf[idx]
                            } else {
                                0.0
                            };
                            
                            for (ch, sample) in frame.iter_mut().enumerate() {
                                if ch < filters.len() {
                                    *sample = filters[ch].process(val);
                                } else {
                                    *sample = val;
                                }
                            }
                            
                            read_phase += ratio;
                        }
                        
                        let consumed = read_phase.floor() as usize;
                        if consumed > 0 {
                            let buf_len = buf.len();
                            buf.drain(0..consumed.min(buf_len));
                            read_phase -= consumed as f64;
                        }
                        
                        let buf_len = buf.len();
                        if buf_len > 12000 {
                            let excess = buf_len - 3000;
                            buf.drain(0..excess);
                        }
                    },
                    |err| eprintln!("Audio stream error: {}", err),
                    None
                )
            }
            cpal::SampleFormat::I16 => {
                let filter_params_clone2 = Arc::clone(&filter_params);
                let buffer_clone2 = Arc::clone(&buffer);
                let mut filters_i16 = vec![
                    AudioFilter::new(sample_rate as f32, initial_params.lp_cutoff_hz, initial_params.hp_cutoff_hz);
                    channels
                ];
                let mut read_phase_i16 = 0.0f64;
                let mut param_refresh_counter_i16: usize = 0;

                device.build_output_stream(
                    stream_config,
                    move |data: &mut [i16], _| {
                        param_refresh_counter_i16 += 1;
                        if param_refresh_counter_i16 % 256 == 0 {
                            if let Ok(params) = filter_params_clone2.try_lock() {
                                for f in &mut filters_i16 {
                                    f.update_params(params.lp_cutoff_hz, params.hp_cutoff_hz);
                                }
                            }
                        }

                        let mut buf = buffer_clone2.lock().unwrap();
                        let available = buf.len();
                        
                        for frame in data.chunks_mut(channels) {
                            let idx = read_phase_i16.floor() as usize;
                            let fract = read_phase_i16 - idx as f64;
                            
                            let val = if idx + 1 < available {
                                (1.0 - fract) as f32 * buf[idx] + fract as f32 * buf[idx + 1]
                            } else if idx < available {
                                buf[idx]
                            } else {
                                0.0
                            };
                            
                            for (ch, sample) in frame.iter_mut().enumerate() {
                                let filtered = if ch < filters_i16.len() {
                                    filters_i16[ch].process(val)
                                } else {
                                    val
                                };
                                *sample = (filtered.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
                            }
                            
                            read_phase_i16 += ratio;
                        }
                        
                        let consumed = read_phase_i16.floor() as usize;
                        if consumed > 0 {
                            let buf_len = buf.len();
                            buf.drain(0..consumed.min(buf_len));
                            read_phase_i16 -= consumed as f64;
                        }
                        
                        let buf_len = buf.len();
                        if buf_len > 12000 {
                            let excess = buf_len - 3000;
                            buf.drain(0..excess);
                        }
                    },
                    |err| eprintln!("Audio stream error: {}", err),
                    None
                )
            }
            _ => return Err("Unsupported sample format".to_string()),
        }.map_err(|e| e.to_string())?;
        
        stream.play().map_err(|e| e.to_string())?;
        
        Ok(Self {
            _stream: stream,
            _buffer: buffer,
        })
    }
}
