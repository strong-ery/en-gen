# EnGen — Project Context & Architecture

> Engine simulation for audio synthesis. Physics-derived sound only — no samples, no tricks.
> Inspired by Engine Simulator 3D (ES3D) by ange the great.

---

## What This Is

EnGen is a real-time 1D CFD combustion engine simulator whose sole output is audio. The simulation models airflow through an engine as a network of one-dimensional fluid channels. Every sound you hear is derived directly from pressure derivatives at tube outlets — no recordings, no synthesised harmonics, no post-processing.

The two core internal systems (mirroring ES3D's naming) are:

- **Tubular** — 1D CFD fluid simulation (Euler equations, finite-volume cells)
- **Planer** — rigid-body / mechanical physics (crankshaft, pistons, connecting rods)

---

## Workspace Layout

```
en-gen/
├── Cargo.toml              # Workspace root
├── engen-core/             # Tubular + Planer — all physics, no I/O
│   └── src/
│       ├── lib.rs
│       ├── cfd/            # 1D CFD solver (Tubular)
│       │   ├── cell.rs         # Finite-volume cell state
│       │   ├── solver.rs       # Euler equation integrator
│       │   ├── limiter.rs      # Flux/gradient limiters
│       │   ├── boundary.rs     # Benson valve model, atmosphere BC
│       │   ├── junction.rs     # Y/X-pipe, collector merging
│       │   └── species.rs      # O2, fuel, CO2, exhaust tracking
│       ├── combustion/     # Cylinder thermodynamics
│       │   ├── cylinder.rs     # Cylinder volume, isentropic model
│       │   ├── burn.rs         # Wiebe curve, energy release
│       │   └── misfire.rs      # Probabilistic misfire model
│       ├── mechanical/     # Planer
│       │   ├── crankshaft.rs   # Rotational inertia, pin offsets
│       │   ├── piston.rs       # Crank-slider kinematics
│       │   └── valve.rs        # Valve timing, lift profiles
│       └── engine/         # Engine assembly — wires everything together
│           ├── config.rs       # EngineConfig struct (all tuneable params)
│           ├── four_stroke.rs
│           └── two_stroke.rs
├── engen-audio/            # Audio output layer
│   └── src/
│       ├── lib.rs
│       ├── sampler.rs      # Jones formula: dP from d(rho*v*A)/dt
│       ├── spatial.rs      # 3D listener position, per-source volume
│       └── output.rs       # cpal stream, 44.1 kHz output
└── engen-cli/              # Entry point / test runner
    └── src/
        └── main.rs
```

---

## Key Dependencies

| Crate | Crate | Purpose |
|-------|-------|---------|
| `engen-core` | `ndarray` | N-dimensional arrays for CFD cell grids |
| `engen-core` | `rayon` | Data parallelism for tube update loops |
| `engen-audio` | `cpal` | Cross-platform audio I/O |
| `engen-audio` | `hound` | WAV export for offline testing |

---

## Physics — What Actually Matters

### Fluid Simulation (Tubular)

The solver integrates the **1D compressible Euler equations** across a network of finite-volume cells. Each cell is ~2–3 cm. This is the hot path — everything else feeds into or out of this.

**Solver requirements:**
- High-resolution (non-diffusive) PDE solver — first-order upwind is not acceptable
- Gradient/flux limiters to prevent spurious oscillations without smearing shocks
- Minimum 64 kHz internal simulation rate (44.1 kHz is the floor, target higher)
- 32-bit float stability guards — brute-force NaN/Inf checks on every step

**Boundary conditions (hardest part):**
- Benson-derived valve model at all port boundaries
- Custom non-linear solver for valve BCs (better convergence than original Benson)
- Atmospheric outlet BC using the **Jones (1978) formula**: `dP = d(rho * v * A) / dt` — this is the audio signal, not raw pressure
- High-frequency reflection attenuation at outlets

**Dissipation:**
- Wall friction (velocity damping, prevents over-resonance)
- Heat transfer to tube walls (big subjective audio impact)
- Material-specific heat conductance (cast iron vs steel vs aluminium)
- 1D turbulence approximation (produces the characteristic fuzzy pulsing)

**Variable geometry:**
- Variable cross-sectional area along tube length (mufflers, bell mouths, collectors)
- Junction simulation: Y-pipe, X-pipe, 4-into-1 collector
- Pressure waves propagate at finite speed-of-sound through junctions — no shortcuts

**Species tracking:**
- Track O2, fuel vapour, CO2, exhaust products through every cell
- AFR-dependent combustion efficiency — combustion only proceeds within valid lambda window
- High residual exhaust fraction → misfires / partial burns → unsteady idle sound
- Probabilistic misfire model from published data

### Combustion & Thermodynamics

- Cylinder volume linked to piston position at all times
- Isentropic compression/expansion: `P * V^gamma = const`
- Combustion: instantaneous internal-energy increment at TDC as baseline, Wiebe burn curve as upgrade
- Peak temperature cap ~2800 K
- Exhaust blowdown pulse at EVO is a primary audio event — model it correctly

**Four-stroke cycle:** intake → compression → power → exhaust. Firing order via crank pin offsets (e.g. 1-3-4-2 for inline-4).

**Two-stroke specifics:**
- Crankcase is the intake plenum
- Transfer port is a third tube; area controlled by piston position
- Expansion chamber wave timing is critical for two-stroke power band

### Mechanical Simulation (Planer)

- Crankshaft: rotational inertia, configurable crank pin offsets per cylinder
- Connecting rod: crank-slider kinematics, converts linear piston force to torque
- Piston position: `cos(theta)` — not approximated
- Throttle plate: choke point in intake tube, controls idle RPM
- External load: torque/drag applied to crankshaft (for under-load demos)

---

## Audio — The Whole Point

All audio is synthesised directly from simulation state. No samples. No synthetic frequency generation.

**Signal derivation:**
- Primary audio signal: Jones (1978) formula applied at each tube outlet
- `audio_sample = d(rho * v * A) / dt` at the outlet face
- Intake audio sampled separately, layered on exhaust
- Zero non-physical audio parameters — if it has no physical basis it doesn't exist

**Output pipeline:**
- Simulation runs at 64 kHz+ internally
- Downsample to 44.1 kHz for audio output via `cpal`
- Simulation thread is fully decoupled from any render/UI thread

---

## Tuneable Parameters (all should be runtime-editable)

These are the knobs. Changing any of these mid-simulation must be reflected immediately:

- Bore and stroke (physically resizes simulation)
- Compression ratio
- Air-fuel ratio / fuelling
- Ignition timing (advance/retard)
- Exhaust header primary tube length
- Tube radius / diameter
- Tube material (heat conductance)
- Surface finish (friction coefficient)
- Flywheel / rotating assembly inertia
- Throttle plate opening
- Ambient pressure (altitude simulation)
- Valve timing: IVO, IVC, EVO, EVC
- Firing order (crank pin offsets)
- CFD spatial resolution (quality vs performance tradeoff)

---

## Engine Configurations to Support

| Configuration | Priority |
|---|---|
| Single-cylinder 4-stroke | Critical — primary debug target |
| Inline-4 4-stroke | Critical — first multi-cylinder |
| V8 4-stroke | High |
| V12 4-stroke | High — performance ceiling target |
| Harley V-twin 4-stroke | High |
| Single-cylinder 2-stroke (chainsaw) | Critical |
| 2-stroke with expansion chamber | High |

---

## Performance Targets

- Real-time factor < 100% on an i7-8700K class CPU running a V12 with long headers at full quality
- SIMD vectorisation of the tube update loop — batch all tubes into a vectorised struct before each sim step
- Multi-threaded fluid simulation is explicitly **out of scope for v1** (single-threaded only)
- Audio output: 44.1 kHz; internal simulation: 64 kHz+

---

## Known Limits — Don't Fight These

These are physics/math constraints, not bugs:

- Gradient limiters introduce high-frequency artefacts — mathematically unavoidable
- 1D CFD cannot fully model 3D turbulence — 1D approximation is a core design constraint, not a limitation to fix
- Extreme area changes (very large volumes) push the limits of 1D validity
- Low RPM idle is the hardest to simulate convincingly — turbulence and mechanical noise dominate
- The engine will never sound exactly like a real recording — that is fine and expected

---

## Out of Scope (v1)

- Full carburetor simulation (simple approximation only)
- Multi-threaded fluid simulation
- 3D acoustic environment simulation (convolution reverb used as proxy)
- Computational Aero-Acoustics turbulence model
- Mechanical noise sources (piston slap, valve clatter)
- 2D or 3D CFD
- Engine damage / wear

---

## Reference Sources

- **Benson** — valve boundary condition model and algorithm
- **Jones, A.D. (1978)** — sound pressure formula (`dP = d(rho*v*A)/dt`) and exhaust modelling
- **Liljencrants, J.** — end correction at flue pipe mouth, acoustic pressure at pipe exit
- **POWERFIST 212** single-cylinder engine — primary real-world audio reference
- **Stihl MS461** chainsaw (77cc two-stroke) — two-stroke reference engine
