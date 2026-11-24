# XEROTON Studio 3.0 — Verilog RTL + PyBullet 8-DOF Quadruped Engineering Environment

**XEROTON Studio 3.0** is an integrated robotics development, simulation, and hardware-verification environment for an **8-DOF quadruped robot dog**, built around a strict hardware-first principle:

> **Synthesizable Verilog RTL controls the robot FSM, safety interlocks, gait trajectories, and 8-channel 50 Hz servo PWM outputs. Python + PyBullet simulates real 250 Hz rigid-body physics from those Verilog outputs and streams live telemetry to a 10-section WebGL robotics control room.**

No Verilog execution, PWM waveform, sensor reading, or physics metric is faked. Simulated IMU values are explicitly tagged as **Simulated IMU Estimates (No physical MPU6050 connected)**, and gait stability is evaluated dynamically by PyBullet before moving from **EXPERIMENTAL** to **VALIDATED (FLAT-GROUND SIM)**.

---

## System Architecture

```
User / JARVIS Local Text Parser / Gait Lab / Manual Safety Gate
                            │
                            ▼
Verilog Safety Supervisor (rtl/safety_controller.v)
  [1-clock E-STOP · Invalid Cmd Trap (8..15) · Watchdog · Per-Joint Clamping]
                            │
                            ▼
Verilog 9-State Smooth FSM (rtl/gait_controller.v)
  [IDLE · STAND · WALK · TROT · STOP · WAVE · DANCE · SIT · FAULT]
  [prev_state · 16-bit frame_counter · runtime stride_us & phase_frames]
                            │
                            ▼
8 Parallel 50 Hz Servo PWM Channels (rtl/servo_pwm.v)
  [20.00 ms period · 1000–2000 us range guard · glitch-free frame latch]
                            │
                            ▼
Icarus Verilog (tb_xeroton.v / tb_live_runner.v -> vvp)
   ├──► simulation/xeroton.vcd         (GTKWave + Studio VCD Inspector)
   └──► simulation/joint_targets.csv   (18-column structured CSV bridge)
                            │
                            ▼
Python Bridge & Safety Layer (python/verilog_bridge.py + studio_server.py)
                            │
                            ▼
PyBullet 250 Hz Rigid-Body Physics (python/robot_simulator.py + robot/xeroton.urdf)
                            │  (50 Hz SSE /api/stream + REST APIs)
                            ▼
10-Section WebGL Control Room (web/index.html · styles.css · scene3d.js · app.js)
```

---

## Ten Control-Room Sections

| # | Section | Capabilities |
|---|---|---|
| **01** | **Overview** | Live 3D robot preview, synced OLED face mirror, KPI cards, quick Verilog commands, mini 8-ch PWM scope. |
| **02** | **Robot Simulation** | 5 engineering view modes (**Normal PBR**, **Wireframe**, **Joint Skeleton**, **URDF Collision Geometry**, **Transparent X-Ray**), orbit/pan/zoom, **Front/Rear/Side/Top/3-4/OLED** camera presets, Follow & Reset Camera, and **Start/Pause/Resume/Step +20ms/Stop/Reset/Speed** transport controls. |
| **03** | **Verilog Control** | Live RTL register observability (`state`, `prev_state`, `command[3:0]`, `frame_counter`, `CLK_HZ` sim vs. FPGA, `reset`, `fault_code[3:0]`, `j0..j7` dec/hex), explicit `CONNECTED` / `NOT CONNECTED` status, invalid command injection (`4'd9`), E-STOP, deliberate `fault_clear`, and FSM transition log. |
| **04** | **Joint Control** | 8 channels (`FL_HIP..RR_KNEE`) showing target vs. actual angle, pulse width (`us`), movement state, angle limits, manual sliders, per-joint neutral reset, per-leg lifts, all-neutral, and E-STOP — strictly safety-gated with clear `MANUAL_SAFE` vs `VERILOG_RTL` tagging. |
| **05** | **PWM Analyzer** | Full 8-channel 50 Hz oscilloscope driven from live RTL/control output: per-channel pulse width (`us`), period (`20.00 ms`), frequency (`50.00 Hz`), duty cycle (`%`), pause/resume, timebase (`2.5 / 5 / 20 ms`), zoom, channel toggles, **CSV/JSON export**, and **real VCD file inspector** (`simulation/xeroton.vcd`). |
| **06** | **Sensor Telemetry** | Live **Simulated IMU Estimates** (`Roll/Pitch/Yaw`, position, linear/angular velocity), 8 joint positions & velocities, foot contacts, ground penetration (`mm`), foot slip (`m/s`), physics error count, and 4 real-time rolling graphs. |
| **07** | **Gait Laboratory** | Configurable **Stand, Walk, Trot, Wave, Dance, Sit, Stop** modes with live 8-phase timeline, active grounded legs, runtime **Stride Length (`us`)** & **Cadence (`frames/phase`)** passed directly to compiled Verilog RTL (`+STRIDE=`, `+PHASE_FRAMES=`), and live PyBullet stability scorecard. |
| **08** | **JARVIS Assistant** | Local deterministic **Text Command Mode** parser mapping natural phrases (`"Stand up"`, `"Start walking"`, `"Move front left leg"`, `"Show PWM signals"`, `"Run gait test"`, `"Explain the current fault"`) safely through the controller — E-STOP always overrides, FAULT blocks motion until deliberate reset. |
| **09** | **Testing & Diagnostics** | Executes real `iverilog`, `vvp`, `pytest`, CSV/VCD, URDF, and PyBullet tests on demand; displays honest `PASS` / `FAIL` / `NOT RUN` / `NOT CONNECTED` statuses and saves logs to `simulation/diagnostics/`. |
| **10** | **System Architecture** | Visual pipeline diagram, 8-DOF joint/PWM mapping table, 50 Hz timing math, FPGA vs. ESP32/PCA9685 deployment roadmap, and explicit system limitations. |

---

## Software Requirements & Installation (Windows 11)

- **Windows 11 (x64)**
- **Icarus Verilog 12+/13+ (`iverilog`, `vvp`) & GTKWave** — via MSYS2 (`pacman -S mingw-w64-ucrt-x86_64-iverilog mingw-w64-ucrt-x86_64-gtkwave`) or [bleyer.org/icarus](https://bleyer.org/icarus/).
- **Python 3.10+ with PyBullet & pytest** — automated via `setup.bat` (uses `micromamba` + `conda-forge` prebuilt `pybullet` in `XEROTON\.env` so Visual C++ Build Tools are not required).

### Commands (PowerShell from `XEROTON\`)

```powershell
# 1. Verify / set up toolchain & Python environment
.\setup.bat

# 2. Compile Verilog RTL + verification testbench + live runner
.\build_verilog.bat

# 3. Run Verilog testbench (generates simulation\joint_targets.csv & xeroton.vcd)
.\run_verilog.bat

# 4. Launch XEROTON Studio 3.0 in browser (http://127.0.0.1:8765)
.\run_python_simulation.bat

# 5. Or run the complete 8-step automated check, compile, test & Studio launch
.\run_all.bat

# 6. Headless CI / verification runs
.\run_all.bat --headless --quiet
.\.env\python.exe -m pytest tests -v

# 7. Inspect digital waveforms in GTKWave
C:\msys64\ucrt64\bin\gtkwave.exe .\simulation\xeroton.vcd

# 8. Optional legacy PyBullet OpenGL + Tkinter window
.\run_python_simulation.bat --pybullet-gui
```

---

## Troubleshooting

- **`run_all.bat is not recognized` in PowerShell:** Prefix local scripts with `.\` (e.g. `.\run_all.bat`).
- **`RTL: NOT CONNECTED` in topbar:** Ensure `iverilog.exe` and `vvp.exe` are installed in `C:\msys64\ucrt64\bin`, `C:\iverilog\bin`, or `PATH`, then run `.\build_verilog.bat`.
- **Port 8765 already in use:** Pass a custom port: `.\run_python_simulation.bat --port 8770`.
- **Browser does not open automatically:** Navigate manually to `http://127.0.0.1:8765`.
- **Controller refuses motion after E-STOP or Invalid Command:** Click **DELIBERATE RESET & CLEAR FAULT** in the top red fault banner (or type `Clear fault` in JARVIS).

---

## Hardware Deployment Roadmap & Limitations

- **FPGA Target (Direct RTL):** Synthesize `rtl/*.v` directly to an Artix-7 / Cyclone / iCE40 FPGA, overriding `CLK_HZ = 50_000_000` (`FRAME_TICKS = 1,000,000`, `TICKS_PER_US = 50`). Add external 5–6 V high-current BEC servo power, level shifting, physical horn calibration, hardware E-STOP cutoff switch, and an IMU (e.g., MPU6050 over I2C/SPI) for closed-loop stabilization.
- **ESP32 + PCA9685 Alternative:** ESP32 does **not** directly execute Verilog RTL; port the FSM & safety logic to C/C++ firmware driving a PCA9685 50 Hz I2C PWM driver.
- **Limitations:** Current gaits are open-loop flat-ground sequences using simulated PyBullet IMU estimates; servo pulse-to-angle scaling is uncalibrated for physical hardware; no real motors are actuated.
