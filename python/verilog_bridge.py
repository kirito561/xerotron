"""
Verilog-to-Python Bridge for XEROTON Studio 3.0.

Loads and validates frame-by-frame joint pulse targets exported by the
Icarus Verilog testbench (simulation/joint_targets.csv), converts servo
pulse widths (1000..2000 us) into calibrated joint angles (radians)
respecting joint limits, invokes the real compiled `xeroton_live.vvp`
RTL engine with runtime gait parameters, checks RTL toolchain connectivity,
and parses real VCD waveforms (`simulation/xeroton.vcd`).

IMPORTANT: This module NEVER fabricates Verilog execution or synthetic gaits.
"""

from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

JOINT_NAMES: tuple[str, ...] = (
    "front_left_hip",
    "front_left_knee",
    "front_right_hip",
    "front_right_knee",
    "rear_left_hip",
    "rear_left_knee",
    "rear_right_hip",
    "rear_right_knee",
)

JOINT_SHORT_LABELS: tuple[str, ...] = (
    "FL_HIP",
    "FL_KNEE",
    "FR_HIP",
    "FR_KNEE",
    "RL_HIP",
    "RL_KNEE",
    "RR_HIP",
    "RR_KNEE",
)

# Joint limits matching robot/xeroton.urdf (radians)
JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-0.95, 0.95),   # j0: FL_HIP
    (-1.10, 1.10),   # j1: FL_KNEE
    (-0.95, 0.95),   # j2: FR_HIP
    (-1.10, 1.10),   # j3: FR_KNEE
    (-0.95, 0.95),   # j4: RL_HIP
    (-1.10, 1.10),   # j5: RL_KNEE
    (-0.95, 0.95),   # j6: RR_HIP
    (-1.10, 1.10),   # j7: RR_KNEE
)

# Hardware pulse safety limits matching rtl/safety_controller.v (us)
JOINT_LIMITS_US: tuple[tuple[int, int], ...] = (
    (1050, 1900),    # j0: FL_HIP
    (1050, 1950),    # j1: FL_KNEE
    (1050, 1900),    # j2: FR_HIP
    (1050, 1950),    # j3: FR_KNEE
    (1050, 1900),    # j4: RL_HIP
    (1050, 1950),    # j5: RL_KNEE
    (1050, 1900),    # j6: RR_HIP
    (1050, 1950),    # j7: RR_KNEE
)

STATE_CODE_TO_NAME: dict[int, str] = {
    0: "IDLE",
    1: "STAND",
    2: "WALK_FORWARD",
    3: "STOP",
    4: "FAULT",
    5: "WAVE",
    6: "DANCE",
    7: "SIT",
    8: "TROT",
    9: "SLEEP",
    10: "WALK_BACKWARD",
    11: "STRAFE_LEFT",
    12: "STRAFE_RIGHT",
    13: "TURN_LEFT",
    14: "TURN_RIGHT",
}

COMMAND_NAME_TO_CODE: dict[str, int] = {
    "IDLE": 0,
    "STAND": 1,
    "WALK": 2,
    "WALK_FORWARD": 2,
    "STOP": 3,
    "WAVE": 4,
    "DANCE": 5,
    "SIT": 6,
    "TROT": 7,
    "SLEEP": 8,
    "WALK_BACKWARD": 9,
    "STRAFE_LEFT": 10,
    "STRAFE_RIGHT": 11,
    "TURN_LEFT": 12,
    "TURN_RIGHT": 13,
    "INVALID": 15,
}

MIN_PULSE_US = 1000.0
MAX_PULSE_US = 2000.0
NEUTRAL_PULSE_US = 1500.0
FULL_SCALE_ANGLE_RAD = 0.95

REQUIRED_COLUMNS: tuple[str, ...] = (
    "frame",
    "time_ms",
    "command",
    "state",
    "state_name",
    "gait_phase",
    "fault_active",
    "fault_code",
    "j0_us",
    "j1_us",
    "j2_us",
    "j3_us",
    "j4_us",
    "j5_us",
    "j6_us",
    "j7_us",
)


@dataclass(frozen=True)
class VerilogFrame:
    """Single 20 ms PWM frame exported by the Verilog RTL simulation."""

    frame: int
    time_ms: float
    command: int
    state_code: int
    state_name: str
    gait_phase: int
    fault_active: bool
    fault_code: int
    pulses_us: tuple[float, ...]
    angles_rad: tuple[float, ...]
    prev_state_code: int = 0
    prev_state_name: str = "IDLE"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pulse_us_to_angle_rad(pulse_us: float, joint_index: int) -> float:
    """Convert a servo PWM pulse width (us) into a clamped joint angle (rad)."""
    if not (0 <= joint_index < len(JOINT_LIMITS_RAD)):
        raise ValueError(f"Invalid joint_index={joint_index}; expected 0..7")

    clamped_pulse = clamp(float(pulse_us), MIN_PULSE_US, MAX_PULSE_US)
    raw_angle = ((clamped_pulse - NEUTRAL_PULSE_US) / 500.0) * FULL_SCALE_ANGLE_RAD
    low_rad, high_rad = JOINT_LIMITS_RAD[joint_index]
    return clamp(raw_angle, low_rad, high_rad)


def angle_rad_to_pulse_us(angle_rad: float, joint_index: int) -> float:
    """Convert a joint angle (rad) to a safety-clamped servo pulse width (us)."""
    if not (0 <= joint_index < len(JOINT_LIMITS_US)):
        raise ValueError(f"Invalid joint_index={joint_index}; expected 0..7")
    raw_us = NEUTRAL_PULSE_US + (float(angle_rad) / FULL_SCALE_ANGLE_RAD) * 500.0
    lo, hi = JOINT_LIMITS_US[joint_index]
    return clamp(raw_us, float(lo), float(hi))


def load_verilog_csv(csv_path: str | Path) -> list[VerilogFrame]:
    """Read and validate the Verilog-generated joint targets CSV file."""
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(
            f"Verilog output CSV not found at '{path}'. "
            "Run build_verilog.bat and run_verilog.bat first."
        )

    frames: list[VerilogFrame] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file '{path}' is empty or missing a header row.")

        missing = [col for col in REQUIRED_COLUMNS if col not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV file '{path}' is missing required columns: {missing}")

        for line_no, row in enumerate(reader, start=2):
            try:
                frame_num = int(row["frame"])
                time_ms = float(row["time_ms"])
                command = int(row["command"])
                state_code = int(row["state"])
                state_name = row["state_name"].strip() or STATE_CODE_TO_NAME.get(
                    state_code, "UNKNOWN"
                )
                prev_state_code = int(row["prev_state"]) if row.get("prev_state") else 0
                prev_state_name = (
                    row.get("prev_state_name", "").strip()
                    or STATE_CODE_TO_NAME.get(prev_state_code, "IDLE")
                )
                gait_phase = int(row["gait_phase"])
                fault_active = bool(int(row["fault_active"]))
                fault_code = int(row["fault_code"])
                raw_pulses = tuple(float(row[f"j{i}_us"]) for i in range(8))
            except (KeyError, ValueError, TypeError) as exc:
                raise ValueError(
                    f"Malformed data in '{path}' at line {line_no}: {exc}"
                ) from exc

            for idx, pulse in enumerate(raw_pulses):
                if not (MIN_PULSE_US <= pulse <= MAX_PULSE_US):
                    raise ValueError(
                        f"Frame {frame_num} j{idx}_us={pulse} outside valid "
                        f"[{MIN_PULSE_US}, {MAX_PULSE_US}] us range."
                    )

            angles = tuple(
                pulse_us_to_angle_rad(pulse, idx)
                for idx, pulse in enumerate(raw_pulses)
            )
            frames.append(
                VerilogFrame(
                    frame=frame_num,
                    time_ms=time_ms,
                    command=command,
                    state_code=state_code,
                    state_name=state_name,
                    gait_phase=gait_phase,
                    fault_active=fault_active,
                    fault_code=fault_code,
                    pulses_us=raw_pulses,
                    angles_rad=angles,
                    prev_state_code=prev_state_code,
                    prev_state_name=prev_state_name,
                )
            )

    if not frames:
        raise ValueError(f"CSV file '{path}' contains zero frames.")

    return frames


def filter_frames_by_state(
    frames: Sequence[VerilogFrame], state_filter: str
) -> list[VerilogFrame]:
    """Filter actual Verilog-exported frames for a specific demo state."""
    key = state_filter.strip().upper()
    if key == "ALL":
        return list(frames)
    filtered = [f for f in frames if f.state_name.upper() == key]
    if not filtered:
        available = sorted({f.state_name for f in frames})
        raise ValueError(
            f"No Verilog frames found for state '{key}'. Available states: {available}"
        )
    return filtered


def summarize_verilog_frames(frames: Iterable[VerilogFrame]) -> dict[str, int]:
    """Count exported frames per FSM state."""
    counts: dict[str, int] = {}
    for item in frames:
        counts[item.state_name] = counts.get(item.state_name, 0) + 1
    return counts


def find_tool(exe_name: str) -> tuple[str | None, dict[str, str]]:
    """Locate an Icarus Verilog / GTKWave tool and build an environment with its bin on PATH."""
    env = os.environ.copy()
    candidates = [
        Path(rf"C:\msys64\ucrt64\bin\{exe_name}.exe"),
        Path(rf"C:\iverilog\bin\{exe_name}.exe"),
    ]
    for cand in candidates:
        if cand.exists():
            env["PATH"] = str(cand.parent) + os.pathsep + env.get("PATH", "")
            return str(cand), env

    found = shutil.which(exe_name)
    return found, env


def check_rtl_toolchain(project_root: Path) -> dict[str, Any]:
    """Inspect real availability of Icarus Verilog, vvp, GTKWave, and compiled artifacts."""
    iverilog_exe, _ = find_tool("iverilog")
    vvp_exe, _ = find_tool("vvp")
    gtkwave_exe, _ = find_tool("gtkwave")
    sim_vvp = project_root / "simulation" / "xeroton_sim.vvp"
    live_vvp = project_root / "simulation" / "xeroton_live.vvp"
    csv_file = project_root / "simulation" / "joint_targets.csv"
    vcd_file = project_root / "simulation" / "xeroton.vcd"

    connected = bool(iverilog_exe and vvp_exe and live_vvp.exists())
    return {
        "connected": connected,
        "status_label": "CONNECTED" if connected else "NOT CONNECTED",
        "iverilog_found": bool(iverilog_exe),
        "vvp_found": bool(vvp_exe),
        "gtkwave_found": bool(gtkwave_exe),
        "sim_vvp_exists": sim_vvp.exists(),
        "live_vvp_exists": live_vvp.exists(),
        "csv_exists": csv_file.exists(),
        "vcd_exists": vcd_file.exists(),
        "vcd_size_mb": round(vcd_file.stat().st_size / (1024 * 1024), 2) if vcd_file.exists() else 0.0,
        "sim_clock_hz": 1_000_000,
        "fpga_target_clock_hz": 50_000_000,
        "pwm_hz": 50,
        "pwm_period_ms": 20.0,
    }


def run_live_verilog_command(
    project_root: Path,
    command_code: int,
    estop: bool = False,
    num_frames: int = 64,
    stride_us: int = 0,
    phase_frames: int = 0,
    out_csv_rel: str = "simulation/live_targets.csv",
) -> list[VerilogFrame]:
    """
    Invoke the compiled Verilog RTL live runner (`xeroton_live.vvp`) for a user
    command and gait parameters, returning fresh Verilog-generated frames.
    """
    vvp_path = project_root / "simulation" / "xeroton_live.vvp"
    if not vvp_path.exists():
        raise FileNotFoundError(
            f"Compiled live Verilog runner missing at '{vvp_path}'. Run build_verilog.bat first."
        )

    vvp_exe, env = find_tool("vvp")
    if not vvp_exe:
        raise FileNotFoundError(
            "vvp (Icarus Verilog runtime) not found — RTL simulator NOT CONNECTED."
        )

    cmd = [
        vvp_exe,
        str(vvp_path),
        f"+CMD={int(command_code)}",
        f"+ESTOP={1 if estop else 0}",
        f"+FRAMES={int(num_frames)}",
        f"+STRIDE={int(stride_us)}",
        f"+PHASE_FRAMES={int(phase_frames)}",
        f"+OUT={out_csv_rel}",
    ]
    result = subprocess.run(
        cmd,
        cwd=str(project_root),
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Live Verilog execution failed (exit {result.returncode}): "
            f"{result.stderr or result.stdout}"
        )

    return load_verilog_csv(project_root / out_csv_rel)


def parse_vcd_summary(vcd_path: Path, max_edges: int = 30) -> dict[str, Any]:
    """
    Parse real header & initial PWM edge timing from simulation/xeroton.vcd.
    Stops early after `max_edges` CH0 pulses so UI inspection stays instant (<0.2s)
    even for multi-second 1 MHz VCD dumps.
    """
    if not vcd_path.exists():
        return {"available": False, "error": f"VCD file not found: {vcd_path}"}

    signals: list[str] = []
    timescale = ""
    servo_pwm_id: str | None = None
    rising_ps: list[int] = []
    pulses_us: list[float] = []

    current_ps = 0
    prev_ch0 = "0"
    last_rise_ps: int | None = None
    in_header = True
    var_re = re.compile(r"^\$var\s+\S+\s+(\d+)\s+(\S+)\s+(\S+)")

    with vcd_path.open("r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            if in_header:
                if line.startswith("$timescale"):
                    timescale = line
                elif line.startswith("$var"):
                    m = var_re.match(line)
                    if m:
                        bits, sig_id, sig_name = m.group(1), m.group(2), m.group(3)
                        label = f"{sig_name}[{int(bits)-1}:0]" if int(bits) > 1 else sig_name
                        if label not in signals and len(signals) < 32:
                            signals.append(label)
                        if sig_name == "servo_pwm" and servo_pwm_id is None:
                            servo_pwm_id = sig_id
                elif line.startswith("$enddefinitions"):
                    in_header = False
                continue

            if line.startswith("#"):
                current_ps = int(line[1:])
            elif servo_pwm_id and line.startswith("b") and line.endswith(" " + servo_pwm_id):
                bits = line[1 : -(len(servo_pwm_id) + 1)]
                ch0 = bits[-1] if bits else "0"
                if prev_ch0 == "0" and ch0 == "1":
                    rising_ps.append(current_ps)
                    last_rise_ps = current_ps
                elif prev_ch0 == "1" and ch0 == "0" and last_rise_ps is not None:
                    pulses_us.append(round((current_ps - last_rise_ps) / 1_000_000.0, 2))
                    if len(pulses_us) >= max_edges:
                        break
                prev_ch0 = ch0

    periods_ms = [
        round((rising_ps[i] - rising_ps[i - 1]) / 1_000_000_000.0, 4)
        for i in range(1, len(rising_ps))
    ]
    avg_period = round(sum(periods_ms) / len(periods_ms), 4) if periods_ms else 0.0
    freq_hz = round(1000.0 / avg_period, 3) if avg_period > 0 else 0.0

    return {
        "available": True,
        "path": str(vcd_path),
        "size_mb": round(vcd_path.stat().st_size / (1024 * 1024), 2),
        "timescale": timescale or "1ps",
        "signals": signals,
        "sampled_frames": len(pulses_us),
        "measured_ch0_period_ms": avg_period,
        "measured_ch0_freq_hz": freq_hz,
        "sample_ch0_pulses_us": pulses_us[:16],
    }
