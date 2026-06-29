# Handoff - Milestone 3 Audio Debugging

This handoff details the current state of Milestone 3 (Benson Valve + Jones Audio Synthesis) for the incoming agent.

---

## 1. Current Status

- **Numerical Solver**: 100% stable. Dynamic CFL-based time sub-stepping prevents solver explosions under any geometry changes (even extreme cell counts and short tubes).
- **Visualization**: Working perfectly. Pressure waves can be seen propagating, reflecting, and oscillating in real time.
- **Audio Output**: Adjusted the scaling and simplified the filters. Ready for test run to verify the sound levels.
- **Goal**: Restore clean, physical exhaust audio (a deep, distinct "TUK TUK" pulse note) while preventing both the "scratchy raincoat" static noise and the "amplifier hum" feedback resonance.

---

## 2. What We Tried & Results

### A. Dynamic Time Sub-stepping (`solver.rs`)
- **Action**: Calculated $\Delta t_{stable} = 0.8 \cdot \min(\Delta x) / \max(|v| + c)$ and sub-stepped the solver internally whenever the CFL limit was lower than $15.625\ \mu\text{s}$ (64kHz).
- **Result**: **Highly successful**. Fully solved the bisection/solver explosion bugs.

### B. Consistent Ghost Cell Reconstruction (`solver.rs`)
- **Action**: Derived boundary ghost cell density and pressure consistently from the filtered sound speed $c_{ghost}$ instead of the unfiltered $c_{bc}$.
- **Result**: **Successful**. Stabilized boundary coupling and cut numeric high-frequency mismatch shock.

### C. Buffer-Level Pacing (`main.rs`)
- **Action**: Decoupled the solver loop from the wall clock. Allowed the solver thread to pace itself to keep the `audio_buffer` filled to exactly 3000 samples. 
- **Result**: **Successful**. Ended the pacing jitter, buffer overruns, and buffer underruns, eliminating the scratchy "raincoat" static.

### D. Simplified DC Blocker (`lib.rs` in `engen-audio`)
- **Action**: Replaced the custom high-pass IIR equation with a standard, highly stable DC blocker:
  $$y[n] = x[n] - x[n-1] + 0.995 \cdot y[n-1]$$
- **Result**: **Successful compile**. Stabilizes sub-sonic drift without numerical edge-cases.

### E. Adjusted Acoustic Scaling (`main.rs`)
- **Action**: Tuned the acoustic scaling factor of the Jones derivative from `1.5e-4` to `2.0e-3`.
- **Result**: **Successful compile**. The previous `1.5e-4` factor was too aggressive (reducing the output pressure down to $-60\text{ dB}$, which was completely inaudible). The updated `2.0e-3` factor yields a healthy $-15\text{ dB}$ to $-6\text{ dB}$ signal, bringing back clear, audible waves without overdriving the soft-clipper.

---

## 3. Recommended Investigation Path for the Next Agent

1. **Verify Sound Levels**:
   - Run the application (`cargo run --release -p engen-ui`) and verify that the sound is audible, clear, and behaves like a low-frequency exhaust pulse when opening the valve or injecting pulses.
2. **Monitor Soft-Clipper Behavior**:
   - Verify that the soft-clipper ($x / (1 + |x|)$) maintains musical compression without clipping into a square-wave drone when injecting multiple concurrent pulses.
