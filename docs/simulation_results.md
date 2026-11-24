# XEROTON — Physics Simulation Results & Gait Optimization Report

All results in this document were measured by executing the actual Icarus Verilog RTL simulation (`tb/tb_xeroton.v` -> `simulation/joint_targets.csv`) and replaying the exported 20 ms joint frames in PyBullet (`python/main.py --headless`) with `robot/xeroton.urdf` at 250 Hz physics resolution (5 sub-steps = 20.0 ms per Verilog frame).

---

## 1. Robot Physical Model Summary (`robot/xeroton.urdf`)

During inspection of the starter URDF, two physical modeling flaws were identified and corrected before evaluating locomotion:
1. **Body/Leg Self-Collision Fix:** The starter URDF mounted hip joints at `y = ±0.10 m` inside the `0.24 m`-wide body (`y ∈ [-0.12, +0.12] m`). Hips were relocated to `y = ±0.148 m` (`7 mm` clearance from the chassis) with `URDF_USE_SELF_COLLISION` enabled.
2. **Foot Contact Geometry:** Sharp rectangular lower-leg corners were augmented with spherical rubber foot pads (`radius = 0.022 m`, `lateralFriction = 1.4`) to prevent edge-snagging during swing/stance transitions.

| Link Group | Quantity | Mass Each | Total Mass | Dimensions / Notes |
|---|---|---|---|---|
| `base` (chassis + front visor) | 1 | 2.40 kg | 2.40 kg | `0.42 × 0.24 × 0.08 m` box, analytic box inertia |
| `*_upper` (thighs) | 4 | 0.18 kg | 0.72 kg | `0.036 × 0.036 × 0.15 m`, hip Y-axis limits `±0.85 rad` |
| `*_lower` (shins + foot pad) | 4 | 0.14 kg | 0.56 kg | `0.030 × 0.030 × 0.15 m` + `r = 0.022 m` sphere, knee limits `±1.05 rad` |
| **Total Robot Mass** | **9 links** | — | **3.68 kg** | **8 revolute DOF**, neutral standing height `0.322 m`, crouched height `0.311 m` |

---

## 2. Iterative Physics-Driven Gait Optimization (Phase 8)

### Iteration 1 (Baseline v1 — Naive Double-Support Return)
- **What we observed in PyBullet:**
  - In the full 150-frame sequence, the robot stayed upright with 0 body ground hits and net `+0.1769 m` forward progress.
  - **Measured defects:**
    1. Between frames 60 and 80 during `WALK`, `pos_x` slipped backward (`+0.138 m -> +0.045 m`) because Phases 3 and 7 moved the rear-extended diagonal pair from `HIP_BACK` to `HIP_MID` while its feet were still in ground contact (**ground scrubbing**).
    2. High actuator gain (`positionGain = 0.35`, unlimited velocity) caused an unphysical spring-hop on sudden `STAND -> FAULT` transitions (body jumped to `z = 0.377 m`, `0/4` feet on ground, `pitch = +8.0°`).

### Iteration 2 (v2 — Zero-Scrubbing + Realistic Servo Speed Cap)
- **Changes made:** Added realistic hobby-servo speed limits (`maxVelocity = 4.5 rad/s`, `positionGain = 0.22`) in `robot_simulator.py` and removed hip motion during double-support phases.
- **What we observed in PyBullet:**
  - `FAULT` pitch dropped 65% (`8.00° -> 2.80°`) and the unphysical hop vanished (`z = 0.314..0.323 m`).
  - `STOP` pitch dropped 57% (`7.86° -> 3.38°`) and settled smoothly to `4/4` feet grounded (`roll = 0.0°, pitch = 0.0°`).
  - **Remaining defect:** Entering `WALK` Phase 0 from `STAND` jumped Pair B directly from `1350 -> 1280 us` while still planted, kicking the robot `-0.172 m` backward before forward walking began.

### Iteration 3 (Final v3 — Lift-First Zero-Scrubbing 8-Phase Diagonal Trot)
- **Changes made in `rtl/gait_controller.v`:**
  - Re-ordered the 8-phase gait table so Phase 0 lifts Pair A directly from `STAND` (`HIP_MID = 1350 us`) with zero initial hip scrub.
  - Enforced strict **lift-before-swing** (Phases 3 & 7 lift the rear pair vertically to `KNEE_LIFT = 1940 us` before moving the hip forward) and **plant-before-lift** (Phases 2 & 6 lower the front pair into 4-foot double support before lifting the opposite pair).
  - Every stance leg strictly sweeps `1270 -> 1350 -> 1430 us` (monotonic `+X` forward thrust).

---

## 3. Quantitative Before vs. After Comparison

### A. Full 150-Frame Combined Sequence (`IDLE -> STAND -> WALK -> STOP -> WALK -> ESTOP FAULT -> INVALID CMD FAULT -> WATCHDOG FAULT`)

| Metric | Baseline (v1) | Intermediate (v2) | Final Tuned (v3) | Improvement (v1 -> v3) |
|---|---|---|---|---|
| Net Forward Travel (`Δx`, 3.00 s total) | `+0.1769 m` | `+0.0131 m` | **`+0.5318 m`** | **+200.6% (3.0× forward distance)** |
| Lateral Drift (`Δy`) | `-0.0120 m` | `+0.0063 m` | **`+0.0135 m`** | **2.5% heading drift over 53 cm** |
| Backward Slippage During `WALK` | Yes (`+0.138 -> +0.045 m`) | Initial kick (`-0.172 m`) | **None (monotonic `+X`)** | **Eliminated** |
| `STOP` Max Pitch (`|pitch|`) | `7.86°` | `3.38°` | **`2.88°`** | **-63.4% pitch disturbance** |
| `FAULT` Airborne Hop (`max z`) | `0.377 m` (0/4 feet) | `0.323 m` | **`0.330 m`** | **Eliminated high bounce** |
| Body Ground Collisions (all 150 frames) | `0` | `0` | **`0`** | **Zero chassis impacts** |
| Robot Remained Upright (all 5 states) | Yes | Yes | **Yes** | **100% upright** |

### B. Dedicated `WALK` Demonstration (`--demo walk`, 76 Verilog frames = 1.52 s)

| Metric | Baseline (v1) | Intermediate (v2) | Final Tuned (v3) |
|---|---|---|---|
| Net Forward Distance (`Δx`) | `+0.3516 m` | `+0.2036 m` | **`+0.4986 m` (~0.33 m/s)** |
| Lateral Drift (`Δy`) | `-0.0128 m` | `-0.0090 m` | **`+0.0172 m`** |
| Max Absolute Roll | `2.84°` | `2.63°` | `5.29°` |
| Max Absolute Pitch | `8.56°` | `5.12°` | `8.94°` |
| Min Chassis Height (`z`) | `0.3073 m` | `0.3071 m` | **`0.3086 m`** |
| Body Ground Contact Frames | `0` | `0` | **`0`** |
| Upright Throughout Demo | Yes | Yes | **Yes** |

### C. Monotonic Forward Progression Sample (Final v3 Combined Run)

| Verilog Frame | Time (ms) | FSM State | `pos_x` (m) | `pos_y` (m) | `pos_z` (m) | Roll / Pitch (deg) | Chassis Hit |
|---|---|---|---|---|---|---|---|
| `010` | 200 ms | `IDLE` | `+0.000` | `+0.000` | `0.322` | `0.0° / 0.0°` | No |
| `020` | 400 ms | `STAND` | `-0.028` | `0.000` | `0.321` | `0.0° / -3.7°` | No |
| `030` | 600 ms | `WALK` | `-0.004` | `+0.002` | `0.311` | `-0.3° / +0.3°` | No |
| `040` | 800 ms | `WALK` | `+0.063` | `-0.010` | `0.309` | `+2.1° / +2.2°` | No |
| `050` | 1000 ms | `WALK` | `+0.138` | `+0.028` | `0.325` | `-4.9° / -6.1°` | No |
| `060` | 1200 ms | `WALK` | `+0.195` | `+0.019` | `0.318` | `-2.2° / -0.5°` | No |
| `070` | 1400 ms | `WALK` | `+0.298` | `-0.015` | `0.322` | `+4.6° / +1.8°` | No |
| `080` | 1600 ms | `WALK` | `+0.370` | `+0.012` | `0.326` | `-0.8° / -6.0°` | No |
| `090` | 1800 ms | `WALK` | `+0.448` | `+0.025` | `0.314` | `-2.9° / +4.3°` | No |
| `110` | 2200 ms | `STOP` | `+0.486` | `+0.012` | `0.311` | `0.0° / +0.1°` | No |
| `150` | 3000 ms | `FAULT` | `+0.532` | `+0.013` | `0.326` | `0.0° / -2.5°` | No |

---

## 4. Honest Engineering Assessment of Stability & Limitations

- **What is demonstrated:** In flat-ground PyBullet physics simulation, the final v3 Verilog-driven 8-phase lift-first diagonal trot achieves **repeatable, monotonic forward locomotion (+50 cm in 1.52 s)** with **0 chassis collisions**, **< 5.3° roll**, **< 9.0° pitch**, and **clean transitions across IDLE, STAND, WALK, STOP, and FAULT**.
- **What is NOT claimed:** This is an **open-loop, flat-ground** diagonal trot without IMU attitude feedback, terrain adaptation, or active torque/ZMP balancing. On uneven slopes, slippery floors, or external pushes, an open-loop FSM will eventually drift or tip without adding closed-loop IMU pitch/roll stabilization and foot-contact sensing.
