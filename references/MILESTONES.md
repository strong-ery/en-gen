# EnGen — Milestones

Progress is tracked bottom-up: each milestone produces something you can see or hear that confirms the physics is correct before building on top of it.

---

## ✅ Milestone 1 — Pressure Pulse Propagation
**Status: Complete**

Single fixed-width tube, atmosphere BCs, pressure pulse injected at t=0. Live egui plot of pressure/velocity/density across tube length. Van Leer limiter, wall friction, open/closed BC toggles, heatmap strip.

**Validated:** Pulse travels, reflects, returns. Shape stays sharp. Solver stable.

---

## Milestone 2 — Tube Junctions & Variable Geometry
**Goal:** Confirm pressure waves propagate correctly through geometry changes and multi-tube networks.

- Variable cross-sectional area along a single tube (taper, bell mouth, expansion chamber)
- Source terms on RHS of Euler equations for area change — no shortcuts
- Y-junction: one tube splitting into two (or two merging into one)
- Visualiser shows all tubes in the network simultaneously, colour-coded
- Inject a pulse and watch it split at the junction, travel both branches, reflect back

**Validated:** Wave timing at junction matches `L / c` for each branch. No spurious reflections from area changes.

---

## Milestone 3 — Benson Valve Boundary Condition
**Goal:** Replace the simple open/closed BC with a physically correct valve model.

- Implement Benson valve model at tube endpoints
- Custom non-linear solver for valve BC convergence (Newton or bisection)
- Valve has a lift parameter (0 = fully closed, 1 = fully open)
- Atmospheric outlet BC: correctly converts tube-end flow to sound pressure using Jones (1978) formula: `dP = d(rho * v * A) / dt`
- High-frequency reflection attenuation at open outlets
- UI: valve lift slider per endpoint, real-time waveform of the Jones signal at the outlet

**Validated:** Jones signal at outlet looks like a physically plausible pressure wave. Closed valve reflects pulse cleanly. Open valve radiates and attenuates.

---

## Milestone 4 — Mechanical Simulation (Planer)
**Goal:** A crankshaft driving a piston, volume change only — no combustion yet.

- Crankshaft with rotational inertia, single crank pin
- Connecting rod: crank-slider kinematics, `cos(theta)` piston position — not approximated
- Cylinder volume computed from piston position at every timestep
- Isentropic compression/expansion: `P * V^gamma = const` during closed-valve phases
- Cylinder connected to intake and exhaust tubes via valve endpoints (from M3)
- UI: RPM readout, crank angle indicator, cylinder pressure vs crank angle plot
- Engine spins freely when given an initial RPM (no combustion torque yet — it will decelerate)

**Validated:** Cylinder pressure trace during compression/expansion matches isentropic curve. Intake/exhaust pulses appear in connected tubes at correct crank angles.

---

## Milestone 5 — First Fire: Single-Cylinder Audio
**Goal:** The engine makes sound. This is the first time you hear EnGen.

- Instantaneous combustion energy release at TDC (internal energy increment)
- Peak temperature cap at ~2800 K
- Exhaust blowdown pulse at EVO — this is the primary audio event
- Jones formula signal at exhaust outlet routed to `engen-audio` → `cpal` → speakers
- Intake audio sampled separately at intake inlet, mixed in
- UI: throttle slider, ignition timing knob, AFR display, audio on/off toggle
- Engine is self-sustaining: combustion torque > friction, RPM stabilises

**Validated:** You can hear a single-cylinder engine. Revving the throttle changes pitch. Retarding ignition timing changes the sound character.

---

## Milestone 6 — Combustion Quality & Species Tracking
**Goal:** Realistic burn shape and mixture-dependent combustion.

- Wiebe burn fraction curve replaces instantaneous energy release
- Species tracking through all cells: O2, fuel vapour, CO2, exhaust products
- Theoretically-perfect fuel injection model (forces target AFR in intake runner)
- AFR-dependent combustion efficiency — combustion only proceeds within valid lambda window
- High residual exhaust fraction causes partial burns / misfires
- Probabilistic misfire model — produces characteristic unsteady idle
- UI: AFR gauge, residual fraction readout, misfire event indicator

**Validated:** Rich/lean mixture changes sound character. Idle is unsteady and irregular. Misfires are audible as skipped beats.

---

## Milestone 7 — Multi-Cylinder: Inline-4
**Goal:** First multi-cylinder engine. Firing order, header scavenging, collector.

- Four cylinders sharing a crankshaft with configurable pin offsets
- Firing order 1-3-4-2 (180° pin spacing)
- Individual exhaust headers per cylinder (zoomies as baseline)
- 4-into-1 collector junction
- Intake plenum shared across all four intake runners
- UI: per-cylinder pressure traces, firing order display, header length slider

**Validated:** Firing order audible in the exhaust note. Changing header length shifts the scavenging resonance. Collector back-pressure behaviour matches expectations.

---

## Milestone 8 — Dissipation & Realism Pass
**Goal:** The engine sounds like a real engine, not a simulation.

- Heat transfer to tube walls (material-specific conductance: cast iron / steel / aluminium)
- Surface finish parameter per tube segment (affects friction coefficient)
- 1D turbulence approximation (fuzzy pulsing characteristic)
- Convolution reverb on audio output (outdoor IR as default, disableable)
- 3D listener position — volume of each source changes with distance
- UI: material selector per tube, reverb IR selector, listener position control

**Validated:** Subjective — does it sound convincingly engine-like? Compare against POWERFIST 212 reference recording.

---

## Milestone 9 — Two-Stroke (Chainsaw)
**Goal:** Two-stroke cycle working, with expansion chamber.

- Crankcase as intake plenum (intake runner connects to crankcase)
- Transfer port as third tube, area controlled by piston position geometrically
- Port timing: intake, transfer, exhaust open/close via piston position
- Expansion chamber exhaust — wave timing critical for power band
- Reference: Stihl MS461 77cc chainsaw

**Validated:** Two-stroke sounds distinctly different from four-stroke. Expansion chamber tuning audibly affects the power band RPM.

---

## Milestone 10 — Performance & SIMD
**Goal:** V12 with long headers runs in real time on target hardware.

- Batch all tube cell updates into a vectorised struct before each sim step
- SIMD vectorisation of the tube update loop (`std::simd` or `wide` crate)
- Real-time factor display in UI (computation time / simulation time)
- Target: V12 with long headers, real-time factor < 100% on i7-8700K class CPU
- CFD spatial resolution slider (quality vs performance tradeoff)

**Validated:** Real-time factor stays below 1.0 at full quality on target hardware. Lowering resolution slider shows clear performance headroom.

---

## Notes

- Each milestone must pass visual/audio validation before the next begins
- No multi-threaded fluid simulation until after M10 (single-thread ceiling first)
- Carburetor simulation deferred indefinitely (simple injection model only)
- 3D editor / Bezier curve UI is out of scope for all milestones above
