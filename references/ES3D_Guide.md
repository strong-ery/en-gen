**ENGINE SIMULATOR 3D**

Requirements & Feature Specification

_Derived from ange the great devlogs 1-5_

# 1\. Project Overview

Engine Simulator 3D (ES3D) is a real-time airflow machine simulator. At its core, the engine is modelled as a network of one-dimensional fluid channels. The application generates completely synthetic audio directly from the physics - no audio samples, no post-processing tricks. Players design engines (and other air-driven machines) using a 3D graphical interface, hear them run, and tune them in real time.

The two named internal systems are:

- Tubular - the 1D CFD fluid simulation layer
- Planer - the rigid-body / mechanical physics engine

# 2\. Fluid Simulation (Tubular)

## 2.1 Core Solver

The fluid simulation solves the Euler equations for compressible 1D flow across a discretised network of finite-volume cells. Each 2-3 cm segment is an independent cell, making spatial resolution hundreds of times higher than ES2D.

| **Requirement**                                        | **Priority** | **Notes**                                                  |
| ------------------------------------------------------ | ------------ | ---------------------------------------------------------- |
| High-resolution (non-diffusive) PDE solver             | Critical     | Replaces first-order upwind; sharper pressure pulses       |
| Gradient / flux limiters                               | Critical     | Prevent spurious oscillations while preserving sharpness   |
| Multiple solver variants with convergence verification | High         | Confirm theoretical limits, rule out implementation errors |
| 64 kHz+ simulation sample rate                         | Critical     | Devlog mentions 44 kHz iterations/sec as baseline          |
| 32-bit float stability guards                          | High         | Brute-force NaN/Inf checks to prevent runaway instability  |

## 2.2 Boundary Conditions

The ends of tubes (valves, outlets, atmosphere) are the hardest part of the simulation. ES3D uses a Benson-derived valve model with a custom solver for improved convergence.

| **Requirement**                                 | **Priority** | **Notes**                                                   |
| ----------------------------------------------- | ------------ | ----------------------------------------------------------- |
| Benson valve model at all port boundaries       | Critical     | Replaces the simple approximate model used in ES2D          |
| Custom non-linear equation solver for valve BCs | Critical     | Better convergence than Benson's original algorithm         |
| Atmospheric outlet boundary condition           | Critical     | Correctly converts tube-end flow to radiated sound pressure |
| Jones (1978) sound pressure formula at outlet   | High         | dP derived from d(rho\*v\*A)/dt, not raw pressure sampling  |
| High-frequency reflection attenuation at outlet | High         | Prevents unrealistic high-order standing waves              |

## 2.3 Dissipation & Loss Models

| **Requirement**                     | **Priority** | **Notes**                                                       |
| ----------------------------------- | ------------ | --------------------------------------------------------------- |
| Wall friction model                 | Critical     | Dampens velocity to prevent over-resonance                      |
| Heat transfer to tube walls         | Critical     | Significant subjective audio impact even if small energy effect |
| Material-specific heat conductance  | Medium       | Cast iron vs steel vs aluminium manifold behaviour              |
| Surface finish parameter            | Medium       | Affects friction coefficient per tube segment                   |
| Turbulence model (1D approximation) | Medium       | Fuzzy pulsing characteristic of real engines; needs tuning      |

## 2.4 Variable Cross-Section & Junctions

Source terms on the right-hand side of the governing equations handle area changes and junction merging. This enables expansion chambers, collectors, horns, and muffler boxes.

| **Requirement**                                      | **Priority** | **Notes**                                                      |
| ---------------------------------------------------- | ------------ | -------------------------------------------------------------- |
| Variable cross-sectional area along tube length      | Critical     | Required for mufflers, expansion chambers, bell mouths         |
| Tube junction simulation (collector, Y-pipe, X-pipe) | Critical     | Merges multiple tubes; back-pressure and scavenging effects    |
| Pressure wave propagation through junctions          | Critical     | Finite speed-of-sound naturally handles unequal-length headers |
| Expansion chamber tuning support                     | High         | Reflected wave timing for two-stroke power-band shaping        |
| Muffler / baffle box modelling                       | High         | Large volume boxes approximate as inflated tube sections       |

## 2.5 Chemical Tracking

The fluid simulation tracks gas species through the system, not just pressure/temperature/velocity. This enables realistic mixture formation and misfire modelling.

| **Requirement**                                          | **Priority** | **Notes**                                                     |
| -------------------------------------------------------- | ------------ | ------------------------------------------------------------- |
| Species tracking: O2, fuel vapour, CO2, exhaust products | Critical     | Required for realistic AFR in cylinder and misfire            |
| Fuel injection model (theoretically perfect as baseline) | High         | Forces target AFR in intake runner; carburetor replaces later |
| AFR-dependent combustion efficiency                      | Critical     | Combustion only proceeds within valid lambda window           |
| Exhaust product concentration effect on combustion       | Critical     | High residual fraction causes misfires / partial burns        |
| Probabilistic misfire model from published data          | High         | Generates characteristic unsteady idle sound                  |
| Carburetor simulation                                    | Medium       | Complex; acknowledged as difficult; deferred                  |

# 3\. Combustion & Thermodynamics

## 3.1 Cylinder Model

| **Requirement**                                  | **Priority** | **Notes**                                                  |
| ------------------------------------------------ | ------------ | ---------------------------------------------------------- |
| Finite cylinder volume linked to piston position | Critical     | Pressure pulse shape depends on displacement               |
| Isentropic compression / expansion               | Critical     | P\*V^gamma = const during closed valve phases              |
| Combustion energy release model                  | Critical     | Instantaneous internal-energy increment at TDC as baseline |
| Wiebe burn fraction curve                        | High         | Realistic heat-release shape; replaces instantaneous model |
| Peak temperature cap (~2800 K)                   | High         | Prevents runaway; real dissociation limit                  |
| Exhaust blowdown at EVO                          | Critical     | High-pressure pulse into exhaust header on valve opening   |
| Residual gas fraction tracking                   | High         | Couples to species tracking for misfire model              |

## 3.2 Four-Stroke Cycle

| **Requirement**                                          | **Priority** | **Notes**                                    |
| -------------------------------------------------------- | ------------ | -------------------------------------------- |
| Intake stroke: piston draws charge through intake valve  | Critical     |                                              |
| Compression stroke: isentropic compression               | Critical     |                                              |
| Power stroke: combustion + expansion                     | Critical     |                                              |
| Exhaust stroke: blowdown + piston purge                  | Critical     |                                              |
| Correct firing order phasing (e.g. 1-3-4-2 for inline-4) | Critical     | Crank pin offsets per cylinder               |
| Variable valve timing parameters                         | High         | IVO, IVC, EVO, EVC as editable values        |
| Valve lift profile (cam shape)                           | High         | Triangle as baseline; real cam profile later |

## 3.3 Two-Stroke Cycle

| **Requirement**                   | **Priority** | **Notes**                                                       |
| --------------------------------- | ------------ | --------------------------------------------------------------- |
| Crankcase as intake plenum        | Critical     | Intake runner connects to crankcase, not directly to cylinder   |
| Transfer port simulation          | Critical     | Third tube; piston position controls flow area                  |
| Port timing via piston position   | Critical     | Intake, transfer, exhaust ports open/close geometrically        |
| Expansion chamber exhaust support | Critical     | Wave-timing critical for two-stroke power                       |
| Reed valve support (optional)     | Medium       | Some two-strokes use check-valve style reeds                    |
| Centrifugal clutch simulation     | Medium       | Demonstrated in chainsaw devlog; spring/weight/outer-drum model |

# 4\. Mechanical Simulation (Planer)

## 4.1 Rotating Assembly

| **Requirement**                                          | **Priority** | **Notes**                                           |
| -------------------------------------------------------- | ------------ | --------------------------------------------------- |
| Crankshaft rigid-body simulation with rotational inertia | Critical     | Flywheel mass directly affects RPM stability        |
| Connecting rod (crank-slider kinematics)                 | Critical     | Converts linear piston force to crank torque        |
| Piston with correct sinusoidal position from crank angle | Critical     | cos(theta) position; not approximated               |
| Cylinder pressure force applied to piston                | Critical     | Drives the engine; enables self-sustaining run      |
| Multi-cylinder crank with configurable pin offsets       | Critical     | Firing order and balance determined by pin geometry |
| Adjustable bore and stroke at runtime                    | High         | Must physically resize simulation while running     |
| Adjustable flywheel mass at runtime                      | High         |                                                     |

## 4.2 Engine Loading

| **Requirement**                              | **Priority** | **Notes**                                |
| -------------------------------------------- | ------------ | ---------------------------------------- |
| Throttle plate / butterfly valve             | Critical     | Choke point in intake; controls idle RPM |
| External load application (torque/drag)      | High         | Required for under-load audio demos      |
| Bar-and-chain simulation (chainsaw use-case) | Medium       | Sprocket, centrifugal clutch, drag load  |
| Governor simulation                          | Low          | Speed-regulating device; bypass mode     |

# 5\. Audio System

## 5.1 Sound Generation Philosophy

All audio is synthesised directly from simulation state. No engine sample recordings are used. No synthetic frequency generation or post-processing to fake harmonics. The raw CFD output is the audio output.

| **Requirement**                                         | **Priority** | **Notes**                                     |
| ------------------------------------------------------- | ------------ | --------------------------------------------- |
| Raw pressure derivative at tube outlets as audio signal | Critical     | Jones formula; not raw pressure sampling      |
| Intake audio sampling (separate source)                 | High         | Layer on top of exhaust audio                 |
| Mechanical noise sources                                | Medium       | Piston slap, valve train, etc. as future work |
| Zero non-physical audio parameters exposed to user      | Critical     | Everything audible must have a physical basis |

## 5.2 Spatial Audio

| **Requirement**                                            | **Priority** | **Notes**                                             |
| ---------------------------------------------------------- | ------------ | ----------------------------------------------------- |
| 3D microphone / listener position                          | Critical     | Volume of each source changes with distance           |
| Walk-around listener - hear individual components up close | High         | Intake vs exhaust vs mechanical isolation             |
| Convolution reverb for environment                         | High         | Outdoor IR as default; user-selectable or disableable |
| Dry audio mode for asset creators                          | High         | Reverb fully disableable                              |
| Ambient reverb impulse response (user-selectable)          | Medium       | Multiple environment presets                          |

## 5.3 Performance

| **Requirement**                                                | **Priority** | **Notes**                                              |
| -------------------------------------------------------------- | ------------ | ------------------------------------------------------ |
| Simulation runs in dedicated thread (not main/render thread)   | Critical     | Thread-safe comms layer between sim and UI             |
| Real-time factor < 100% on target hardware (i7-8700K)          | Critical     | Goal: V12 with long headers at full quality            |
| Scalable spatial resolution (quality vs performance trade-off) | High         | Lower resolution mode for weaker hardware              |
| SIMD vectorisation of tube update loop                         | High         | Batch all tubes into vectorised struct before sim step |
| Multi-threaded fluid simulation (future)                       | Medium       | Single-thread ceiling hit; acknowledged as non-trivial |
| Audio sample rate: 44.1 kHz output                             | Critical     | Sim runs at higher internal rate, downsampled          |

# 6\. 3D Editor Interface

## 6.1 Viewport & Navigation

| **Requirement**                                  | **Priority** | **Notes**                                        |
| ------------------------------------------------ | ------------ | ------------------------------------------------ |
| 3D grid with multi-level subdivision             | Critical     | Fine + coarse grid levels; professional CAD feel |
| Grid fog / fade at distance                      | High         | Prevents distracting distant lines               |
| Coloured axis lines (X/Y/Z)                      | High         |                                                  |
| Camera orbit, pan, zoom                          | Critical     | Standard 3D viewport controls                    |
| Mouse ray projection onto working plane          | Critical     | For object placement and manipulation            |
| Axis-constrained object movement (Blender-style) | High         | Move along X, Y, or Z independently              |
| Vulkan renderer                                  | Critical     | Custom game engine; not Unity/Unreal             |

## 6.2 Tube / Bezier Curve Editor

Tubes are the primary building block. Each tube is a swept profile along a Bezier curve.

| **Requirement**                               | **Priority** | **Notes**                                          |
| --------------------------------------------- | ------------ | -------------------------------------------------- |
| Bezier curve creation and editing in 3D       | Critical     | Control point placement and dragging               |
| Simplified pipe-bender mode                   | High         | Constrains handles to produce realistic bend radii |
| Circular tube profile sweep                   | Critical     | Standard round pipe; radius is editable            |
| Non-circular profiles (square, rectangular)   | High         | Intake port / plenum shapes                        |
| Profile scale along curve length              | High         | Tapered tubes, collectors, bell mouths             |
| Twist angle control for non-circular profiles | Medium       |                                                    |
| Add / remove control points interactively     | Critical     | Extend curve with new points                       |
| Snap endpoints to other tube endpoints        | High         | Auto-creates junction on snap                      |
| File format for saving / loading curve data   | Critical     | Persistent engine designs                          |

## 6.3 Engine Assembly

| **Requirement**                                        | **Priority** | **Notes**                                                   |
| ------------------------------------------------------ | ------------ | ----------------------------------------------------------- |
| Cylinder head component with intake + exhaust ports    | Critical     | Exposed endpoints snap to runner tubes                      |
| Cylinder / piston visual with animated piston movement | Critical     |                                                             |
| Crankshaft visual with correct crank pin geometry      | High         |                                                             |
| Component library / parts palette                      | High         | User can swap intakes, headers from library                 |
| Downloadable community parts                           | Medium       | Import parts from other users                               |
| Grouping of related components                         | High         | Select and move entire assemblies                           |
| V-engine, inline, flat, radial configurations          | High         | Any cylinder bank angle                                     |
| Real-time editing while engine is running              | Critical     | Parameter changes reflected immediately via scripting layer |

## 6.4 Scripting Interface

A Piranha scripting interpreter is embedded for power users and engine designers. Scripts hot-reload while the engine is running.

| **Requirement**                                              | **Priority** | **Notes**                                    |
| ------------------------------------------------------------ | ------------ | -------------------------------------------- |
| Embedded Piranha scripting interpreter                       | High         | Used internally; may be exposed to users     |
| Hot-reload on script save                                    | Critical     | Immediate feedback; no restart required      |
| Script-driven parameter editing (bore, stroke, timing, etc.) | Critical     | All engine parameters accessible from script |
| Scripting as fallback before full GUI is complete            | High         | Powers devlog prototypes                     |

# 7\. Supported Engine Configurations

The simulation is generic - any airflow network is valid. The following engine types are specifically called out in devlogs as supported or planned:

| **Requirement**                            | **Priority** | **Notes**                                     |
| ------------------------------------------ | ------------ | --------------------------------------------- |
| Single-cylinder 4-stroke                   | Critical     | Primary test and debug target                 |
| Inline-4 4-stroke                          | Critical     | First multi-cylinder implementation           |
| V8 4-stroke                                | High         | Demonstrated in UI devlog with headers        |
| V12 4-stroke                               | High         | Stated performance target (with long headers) |
| Harley-Davidson V-twin (4-stroke)          | High         | Shown in devlog 5                             |
| Inline-4 sportbike (Hayabusa-style)        | High         | Shown in multi-cylinder devlog                |
| Single-cylinder 2-stroke                   | Critical     | Chainsaw (Stihl MS461 reference engine)       |
| 2-stroke with expansion chamber            | High         |                                               |
| Unusual / degenerate engine configurations | Medium       | Community creativity; any valid airflow graph |
| Air pump / non-engine airflow machines     | Medium       | Demonstrated in early prototypes              |

# 8\. Exhaust & Intake Systems

| **Requirement**                                     | **Priority** | **Notes**                                          |
| --------------------------------------------------- | ------------ | -------------------------------------------------- |
| Individual headers per cylinder (zoomies)           | Critical     | Simplest case; one exhaust per cylinder            |
| Collector junctions (4-into-1, 4-into-2-into-1)     | Critical     | Merges primary tubes                               |
| Y-pipe and X-pipe junction types                    | High         |                                                    |
| Unequal-length header support                       | Critical     | Sound propagation handles timing naturally via CFD |
| Muffler / silencer modelling                        | High         | Box + restriction; user-tunable                    |
| Catalytic converter (flow restriction)              | Low          |                                                    |
| Intake plenum (dual-plane, single-plane)            | High         | Volume affects torque curve                        |
| Intake trumpet / velocity stack                     | High         | Variable bell mouth shape                          |
| Throttle body / carb throat restriction             | Critical     | Controls airflow at idle and WOT                   |
| Turbocharger / supercharger (boost pressure source) | Medium       | Elevated intake manifold pressure                  |
| Intercooler (charge temperature reduction)          | Medium       |                                                    |

# 9\. Tuneable Physical Parameters

The following parameters are explicitly mentioned as real-time editable in the devlogs:

- Bore and stroke (physically resizes simulation while running)
- Displacement per cylinder
- Compression ratio
- Air-fuel ratio / fuelling
- Ignition timing (advance/retard)
- Exhaust header primary tube length
- Exhaust system total length
- Tube radius / diameter
- Tube material and surface finish (heat conductance)
- Flywheel / rotating assembly inertia
- Throttle plate opening (idle stop screw)
- Ambient pressure (simulate altitude)
- Valve timing: IVO, IVC, EVO, EVC
- Firing order (via crank pin offsets)
- CFD spatial resolution (performance / quality tradeoff)

# 10\. Platform & Technical Requirements

| **Requirement**                                        | **Priority** | **Notes**                                                   |
| ------------------------------------------------------ | ------------ | ----------------------------------------------------------- |
| Custom Vulkan game engine (not Unity/Unreal)           | Critical     | ange uses his own renderer and physics engine               |
| CMake build system                                     | Critical     |                                                             |
| Simulation thread fully decoupled from render thread   | Critical     | Thread-safe communication layer                             |
| Target platform: Steam (PC)                            | Critical     | Store page live at time of devlogs                          |
| Minimum spec: modern mid-range CPU (i7-8700K class)    | High         | Performance target stated explicitly                        |
| Procedural geometry generation for all engine parts    | Critical     | No pre-made 3D assets; everything generated from parameters |
| Asset file format for curves and engine layouts        | High         | Save/load engine designs                                    |
| Text rendering system (ported from Steam Engine Sim)   | High         | Performance metrics HUD                                     |
| Real-time performance metrics overlay (FPS, RT factor) | High         | Computation time / simulation time ratio display            |

# 11\. Explicitly Out of Scope (Initial Release)

The following were discussed but deferred:

- Full carburetor simulation (too complex for v1; simple approximation only)
- Multi-threaded fluid simulation (acknowledged as non-trivial; single-thread for launch)
- 3D acoustic simulation of environment around engine (convolution reverb used as proxy)
- Computational Aero-Acoustics (CAA) turbulence model (theoretical; identified as long-term goal)
- Mechanical noise sources (piston slap, valve train clatter)
- 2D / 3D CFD (1D is fundamental design constraint; not a limitation to fix)
- Engine damage / wear simulation
- Multiplayer

# 12\. Known Simulation Limits & Audio Artefacts

Ange explicitly acknowledges these as theoretical or practical limits rather than bugs to fix:

- Gradient limiters introduce high-frequency artefacts - unavoidable; proven mathematically
- All solver variants converge to same solution at different rates - confirms correct implementation
- 1D CFD cannot fully model 3D turbulence - 1D approximation used
- Extreme area changes (very large box volumes) push limits of 1D validity
- Low RPM idle remains hardest to simulate convincingly - turbulence and mechanical noise dominate
- Engine will never sound exactly like real life - simulation approximation is inherent

# 13\. Reference Sources Cited in Devlogs

- Benson - valve boundary condition model and algorithm
- Jones, A.D. (1978) - 'Noise Characteristics and Exhaust Process Gas Dynamics of a Small Two-Stroke Engine' - sound pressure formula and exhaust modelling
- Liljencrants, J. - end correction at flue pipe mouth; acoustic pressure at pipe exit
- POWERFIST 212 single-cylinder engine - primary real-world reference recording
- Stihl MS461 chainsaw (77cc two-stroke) - two-stroke reference engine

_Document compiled from ange the great devlogs 1-5. All requirements are derived from stated or demonstrated features._