# Designing a Full ECU Simulation for EnGen

This is a spec for what's actually needed to make the engine behave like a real one: respond to throttle correctly under all conditions, idle stably, rev-limit, start reliably, and generally stop relying on hand-set constants (`target_afr`, `ignition_timing_deg`, fixed cam timing) that don't change with load or speed the way a real engine's do.

The current model has the *thermodynamics* (combustion, valve flow, crank dynamics) reasonably well covered. What's missing is the *control system* that sits on top of it and makes those constants stop being constants. Below is what a proper ECU needs, roughly in the order you'd want to build it.

---

## 1. Sensor Model (inputs the ECU "reads")

A real ECU doesn't know the true state of the engine — it only knows what its sensors report, often noisy or delayed. To make the rest of this realistic (and to eventually support failure modes / limp mode), it's worth modeling the sensors as a layer separate from the physics ground truth:

- **MAP sensor** (Manifold Absolute Pressure) — derived from intake runner pressure near the throttle body, the primary load signal for speed-density fueling.
- **TPS** (Throttle Position Sensor) — directly reports throttle plate angle/lift; used for transient enrichment and as a cross-check against MAP.
- **RPM / crank position sensor** — derived from crank angle, with realistic update only once per tooth/trigger event rather than continuous, and ideally with synchronization logic (the ECU needs to figure out which stroke it's in from a cam or crank reference, not just read `theta` directly).
- **Coolant temperature sensor (ECT)** — drives cold-start enrichment and idle target adjustments. Doesn't exist in the current model at all; would need a simple thermal mass model for the engine block.
- **Intake air temperature (IAT)** — affects air density calculation for fueling; could initially be hardcoded to ambient and refined later.
- **O2 sensor** — reads actual exhaust AFR (post-combustion), feeds closed-loop fuel trim. This is the sensor that should ultimately replace the current `target_afr` being applied as ground truth.
- **Knock sensor** *(stretch goal)* — only meaningful once a real knock model exists in combustion; flagged here for completeness.

Each sensor reading should go through the modeled value (e.g., true MAP) with optional noise and an update rate, rather than being read directly off the solver state every substep — this is what makes the ECU asynchronous and realistic, the way a real ECU sampling at 1-10kHz against a crank that can spin past sensor resolution actually behaves.

---

## 2. Speed-Density or Alpha-N Fueling (replacing the constant `target_afr`)

This is the most important structural change. Right now fuel mass is computed as:

```rust
let m_fuel = cyl.mass / (self.target_afr + 1.0);
```

This is backwards from how a real ECU works — it's deriving the *correct* fuel mass after the fact from however much air *actually* ended up in the cylinder, guaranteeing perfect stoichiometry regardless of throttle, valve timing, or RPM. A real ECU has no way to know `cyl.mass` directly; it has to *predict* the incoming air charge from sensor readings *before* injecting fuel, and that prediction is necessarily imperfect.

Two standard approaches, either is a legitimate target:

- **Speed-Density**: estimate air mass per cycle from MAP, RPM, intake air temperature, and a **Volumetric Efficiency (VE) table** indexed by RPM and MAP/load. This is the dominant approach on modern OEM ECUs.
- **Alpha-N**: estimate air mass from throttle position (TPS) and RPM instead of MAP — simpler, common on race engines and small/simple ECUs, less accurate during transients but doesn't need a MAP sensor model at all.

Either way, you'd want:

- A **VE table** (RPM x MAP, or RPM x TPS), populated initially with a smooth surface (could even be auto-generated from a few solver runs at steady-state operating points, essentially "self-calibrating" against the CFD model — a nice opportunity to make the ECU's table genuinely derived from your own physics rather than guessed).
- A **fuel injector model**: pulse width = desired fuel mass / (injector flow rate at current fuel rail pressure), with injector dead-time (small constant offset that matters more at idle than WOT) and minimum/maximum pulse width limits.
- This naturally reintroduces realistic throttle response: less throttle → lower MAP → lower predicted VE-based air charge → less fuel commanded → less torque, properly causally linked to the action you actually took, rather than computed backwards from however much air the CFD model happened to let through.

---

## 3. Closed-Loop AFR / Fuel Trim

Once fueling is open-loop (predicted from VE table), it needs the feedback loop a real ECU uses to correct for tuning error:

- Compare the O2 sensor's reported AFR against the desired AFR (itself usually a small target table — slightly rich at idle and WOT, closer to stoichiometric/14.7 in the part-throttle cruise region).
- Apply **short-term fuel trim**: a fast PI(D) correction applied to each injection pulse.
- Apply **long-term fuel trim**: a slow-moving correction that nudges the underlying VE table itself, the way real ECUs "learn" over time and store trims by RPM/load cell.
- This is also the natural place to implement **closed-loop vs open-loop transitions** — real ECUs run open-loop (ignoring O2 feedback) during cold start, WOT, and deceleration fuel cut, only trusting the O2 sensor in steady part-throttle conditions.

---

## 4. Ignition Timing Map (replacing the constant `ignition_timing_deg`)

Same structural issue as fueling — right now spark timing is a single fixed number regardless of load or speed. A real ECU has a full **spark advance table**, typically RPM x MAP (or RPM x load), because:

- More advance is generally tolerable (and wanted, for efficiency) at low load/low RPM.
- Advance must be pulled back at high load to avoid knock, and adjusted with RPM because flame propagation time is roughly constant in real time but the available crank-angle window shrinks as RPM rises.

Minimum viable version: a 2D table (RPM x MAP) of base timing, with:

- **Cold-engine timing retard** for warmup, once a coolant temp model exists.
- **Knock retard** — even a crude model is valuable: if cylinder peak pressure or pressure-rise-rate exceeds a threshold (a usable proxy for knock without modeling actual auto-ignition chemistry), pull timing back a few degrees and slowly let it creep back if conditions clear. This is a real, important ECU behavior worth having even in simplified form.
- **Dwell control** for the coil charge time, if you ever want to model coil-on-plug behavior realistically (probably a stretch goal, low priority relative to the rest).

---

## 5. Idle Air/Speed Control (IAC / electronic throttle idle control)

This is what actually fixes the "throttle does nothing at idle" symptom you're chasing, and it's worth tackling early since it's the most visible symptom right now:

- A real engine idles with the throttle plate *nearly fully closed* — not the engine being plumbed straight from a wide-open runner with a tiny bypass. Idle airflow comes through a dedicated **idle air control path** (an IAC stepper/solenoid bypassing the throttle plate, or on modern drive-by-wire systems, the ECU just commands a small throttle plate angle directly), which is sized specifically to flow just enough air to sustain idle RPM against engine friction/accessory load.
- The ECU runs a **closed-loop idle speed controller**: a PID (or similar) loop targeting a desired idle RPM (itself a small table vs coolant temp, since cold engines idle higher), adjusting IAC opening or idle ignition timing trim to hold that RPM steady against changing load (AC compressor kicking in, etc.).
- This effectively decouples "the throttle plate's main bore" from "how much air gets into the engine at idle" — which is the realistic version of what you were trying to fake with the `state.throttle.max(0.03)` floor. The floor approach can't work the way you want because it ties idle air directly to the same restrictive flow path as part/full throttle; a real idle circuit is architecturally separate.
- For your current solver architecture, the cleanest way to add this without restructuring the whole intake topology might be a small modeled bypass orifice in parallel with the throttle valve boundary condition, with its own independently-controlled lift, sized intentionally small (much smaller than your runner's bulk volume, to avoid the same "plenum reservoir masks everything" effect you just diagnosed).

---

## 6. Rev Limiter / Redline

Genuinely missing right now, as you noted (only a numerical safety clamp on omega exists). A real implementation needs a documented control strategy because "redline" actually has a few standard flavors that produce noticeably different driving feel:

- **Hard cut**: ignition (and optionally fuel) cut entirely the instant RPM crosses the limit, restored once RPM drops below limit minus a hysteresis band. Abrupt, the classic "bouncing off the limiter" stutter.
- **Soft cut**: cut only some cylinders, or cut every Nth ignition event, producing a less violent power reduction. (Less relevant for a single-cylinder engine, but worth designing the multi-cylinder case in mind if this codebase ever grows beyond `new_single_cylinder`.)
- **Fuel cut vs spark cut**: spark cut is simpler but dumps unburned fuel/air into the exhaust (real "fuel cut" rev limiters avoid this and are generally considered the "correct" approach on modern ECUs; spark-cut is the cheaper/older approach and has the side effect of being audibly louder/poppier, which some implementations intentionally keep for that reason).
- Should hook directly into the existing `ignition_on` and (once it exists) fuel-mass-commanded fields, rather than being a separate special case.

---

## 7. Cranking / Start Sequence Logic

The starter torque issue you've been fighting is a symptom of not having real cranking-mode ECU behavior:

- **Cranking enrichment**: during cranking, AFR target should be significantly richer than running AFR (often AFR ~9-12) since cold, slow-moving air doesn't atomize/vaporize fuel well and cylinder filling is erratic at cranking speeds.
- **Cranking timing**: spark timing is usually fixed near TDC or slightly retarded during cranking, independent of the running spark table, since combustion is much less predictable at very low and irregular RPM.
- **Crank-to-run transition**: a defined RPM threshold (and ideally a "has fired N consecutive combustion events" check, not just an RPM threshold) where the ECU hands off from cranking tables to normal running tables. Right now your code disengages the starter at 600 RPM purely based on motor-driven RPM, with no actual concept of "the engine has caught and is running under its own power" versus "the starter just spun it up."
- This is also where you'd implement a **realistic starter motor model** instead of hand-tuning a flat constant torque (45 → 90 N·m and counting) — a real starter has a torque-speed curve (high torque at low/zero speed, falling off as RPM rises), draws from a battery model with its own internal resistance/voltage sag, and disengages via a one-way clutch the moment the engine's own RPM exceeds the starter's drive speed. Getting the starter torque curve right would likely resolve the "needs another torque increase" pattern you've hit twice now without it becoming an arms race.

---

## 8. Transient Fueling (acceleration enrichment / deceleration fuel cut)

Needed for the engine to feel responsive rather than flat:

- **Accel enrichment**: when MAP/TPS rises quickly, briefly add extra fuel beyond the steady-state VE table prediction — physically because a sudden throttle opening causes fuel film on the intake runner walls to lag behind the air response, so without enrichment the mixture briefly leans out badly on tip-in.
- **Decel fuel cut (DFC)**: when throttle closes abruptly at higher RPM (off-throttle, engine still spinning fast), cut fuel entirely until RPM drops near idle — real ECUs do this both for efficiency and emissions, and it changes engine-braking torque feel noticeably.

---

## 9. Suggested Build Order

Given what's already working in your physics layer, I'd sequence it roughly:

1. **Idle air control bypass circuit** — fixes your most visible current symptom and is architecturally necessary before throttle response can mean anything at all speeds.
2. **Speed-density (or Alpha-N) fueling replacing the constant `target_afr`** — this is the change that actually makes the engine's response causally driven by ECU decisions instead of backwards-derived from solver state.
3. **Rev limiter** — comparatively easy once fueling/ignition are properly ECU-driven fields rather than constants, and immediately useful/testable.
4. **Ignition timing table** replacing `ignition_timing_deg`.
5. **Cranking/start sequence and a real starter torque curve** — resolves the starter fights cleanly.
6. **Closed-loop AFR (O2 trim)** and **transient fueling** — polish layer once the open-loop base behavior is solid.
7. Knock model / coolant model / cold-start tables — stretch goals, only worth it once the above is solid and you want more realism.

Items 1-3 alone should fix everything you've been chasing across this whole debugging session: idle won't be a slightly-open-throttle hack anymore, throttle will causally drive fueling instead of mass being derived after the fact from whatever the CFD model happened to let through, and you'll have an actual redline instead of just a numerical clamp.
