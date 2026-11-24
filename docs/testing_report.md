# XEROTON Studio 3.0 — Final Verification & Testing Report

## 1. Verified Test Environment

| Component | Version / Location | Status |
|---|---|---|
| OS | Windows 11 (x64) | Verified |
| Icarus Verilog (`iverilog`, `vvp`) | `13.0 (stable)` (`C:\msys64\ucrt64\bin`) | **CONNECTED** |
| GTKWave (`gtkwave`) | `3.3.128` (`C:\msys64\ucrt64\bin`) | **INSTALLED** |
| Python Runtime | `3.11.18` (`XEROTON\.env\python.exe`) | **VERIFIED** |
| PyBullet / NumPy / pytest | `3.25` (API `202010061`) / `2.4.6` / `9.2.0` | **VERIFIED** |

---

## 2. Automated Verification Results (All Executed & Observed)

### A. In-App Testing & Diagnostics Centre (`POST /api/diagnostics/run`)
Saved automatically to `simulation/diagnostics/diagnostics_report.json` and `simulation/diagnostics/diagnostics_latest.log`:

| Diagnostic Suite | Status | Verified Behavior |
|---|---|---|
| `verilog_compile` | **PASS** | Compiled `xeroton_sim.vvp` & `xeroton_live.vvp` with `iverilog -g2012 -Wall` (0 errors, 0 warnings) |
| `verilog_testbench` | **PASS** | `33 / 33` assertions PASSED, `0` FAILED across 9 states (`IDLE, STAND, WALK, TROT, STOP, WAVE, DANCE, SIT, FAULT`), `prev_state`, `frame_counter`, E-STOP latch/clear, invalid cmd `4'd9`, watchdog timeout, and limit clamping |
| `pwm_timing` | **PASS** | `219 / 219` consecutive frames verified at exact `20,000.00 us` (`50.00 Hz`) and `438 / 438` CH0/CH1 pulses verified within `±1.0 us` |
| `csv_bridge` | **PASS** | `4 / 4` tests in `tests/test_data_validation.py` PASSED (18-column CSV, 9 states, 8 walk phases, pulse bounds, VCD 50 Hz edge parsing) |
| `pybullet_bridge` | **PASS** | `7 / 7` tests in `tests/test_bridge.py` PASSED (angle symmetry, limit clamping, CSV filtering, live `WAVE`/`SIT`/`TROT`/`ESTOP` runner, `StudioEngine` single-step + manual safety gate + JARVIS safety priority, URDF 8-joint mapping, 219-frame headless PyBullet replay) |

**Total Automated Diagnostics:** **`5 / 5` suites PASSED (`11 / 11` pytest tests + `33 / 33` Verilog assertions), `0` FAILED.**

### B. Live Studio 3.0 HTTP & Control-Room API Verification

| Endpoint | Observed Result |
|---|---|
| `GET /`, `/styles.css`, `/scene3d.js`, `/app.js`, `/lib/three.min.js` | Served cleanly (`36.5 KB`, `14.7 KB`, `26.0 KB`, `30.7 KB`, `603.4 KB`) |
| `GET /api/state` | `RTL: CONNECTED`, `IMU: SIMULATED_IMU_ESTIMATE (No physical MPU6050 connected)` |
| `GET /api/vcd` | Parsed real `simulation/xeroton.vcd`: `20.0 ms` period, `50.0 Hz` |
| `GET /api/pwm_export?format=csv` | Exported live captured 8-channel PWM history (`3,784 bytes`) |
| `POST /api/jarvis {"text":"Stand up"}` | `ok: True`, `intent: RTL_STAND` (and E-STOP priority + FAULT motion lockout verified in `pytest`) |
| `POST /api/manual_joints` | `ok: True`, `control_source: MANUAL_SAFE` (clamped & lockout-enforced) |
| `POST /api/diagnostics/run` | `5 / 5` real suites PASSED |

---

## 3. Issues Discovered & Resolved During Phase 9 Verification

1. **Stale fault snapshot when paused/unit-tested:** `StudioEngine.get_snapshot()` did not immediately reflect `self.fault_latched` prior to the next 20 ms physics tick, causing `jarvis.py` to return `RTL_WALK` (blocked by engine) instead of `BLOCKED_BY_FAULT`. Fixed by syncing `fault_latched`, `fault_code`, `fault_reason`, and `state` directly in `get_snapshot()`.
2. **TROT stride-vs-slew mismatch:** Initial `TROT` (`100 us` half-stride at `3` frames/phase) exceeded the `45 us/frame` slew window (`200 us` peak-to-peak > `135 us`), causing high pitch oscillation (`22.7°`). Reduced default `TROT` half-stride to `65 us` and knee lift to `1915 us`; Gait Lab continues to honestly tag unvalidated/high-oscillation gaits as `EXPERIMENTAL`.
