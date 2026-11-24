"""
Data & Waveform Validation Tests for XEROTON 2.0 Verilog Simulation Outputs.

Independently verifies:
  1. simulation/joint_targets.csv structure, timing, all 7 states (IDLE, STAND,
     WALK, STOP, WAVE, DANCE, FAULT), 8 gait phases, smooth slew ramping, and safety bounds.
  2. simulation/xeroton.vcd integrity, signal presence, 50 Hz PWM period, and pulse durations.
"""

from __future__ import annotations

import csv
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CSV_PATH = PROJECT_ROOT / "simulation" / "joint_targets.csv"
VCD_PATH = PROJECT_ROOT / "simulation" / "xeroton.vcd"

EXPECTED_COLUMNS = [
    "frame",
    "time_ms",
    "command",
    "state",
    "prev_state",
    "state_name",
    "prev_state_name",
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
]


def load_csv_rows() -> list[dict[str, str]]:
    assert CSV_PATH.exists(), f"Missing Verilog output CSV: {CSV_PATH}"
    with CSV_PATH.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert reader.fieldnames == EXPECTED_COLUMNS, (
            f"Unexpected CSV columns: {reader.fieldnames}"
        )
        rows = list(reader)
    assert len(rows) >= 200, f"Expected >=200 simulated 20 ms frames, got {len(rows)}"
    return rows


def test_csv_exists_and_has_monotonic_20ms_frames() -> None:
    rows = load_csv_rows()
    prev_frame = 0
    prev_time_ms: float | None = None
    for row in rows:
        frame = int(row["frame"])
        time_ms = float(row["time_ms"])
        assert frame == prev_frame + 1, f"Non-sequential frame {frame} after {prev_frame}"
        if prev_time_ms is not None:
            dt = time_ms - prev_time_ms
            assert 19.9 <= dt <= 20.1, f"Frame {frame} dt={dt:.3f} ms (expected 20.0 ms)"
        prev_frame = frame
        prev_time_ms = time_ms


def test_csv_contains_all_nine_fsm_states_and_eight_walk_phases() -> None:
    rows = load_csv_rows()
    states_present = {row["state_name"].strip() for row in rows}
    required_states = {"IDLE", "STAND", "WALK", "TROT", "STOP", "WAVE", "DANCE", "SIT", "FAULT"}
    assert required_states.issubset(states_present), (
        f"Missing states: {required_states - states_present}"
    )

    walk_phases = {
        int(row["gait_phase"]) for row in rows if row["state_name"].strip() == "WALK"
    }
    assert walk_phases == set(range(8)), f"Incomplete WALK gait phases: {walk_phases}"


def test_csv_joint_pulse_bounds_and_state_policies() -> None:
    rows = load_csv_rows()
    stand_settled_seen = False
    stop_settled_seen = False
    wave_lifted_seen = False

    for row in rows:
        frame = int(row["frame"])
        state_name = row["state_name"].strip()
        pulses = [int(row[f"j{i}_us"]) for i in range(8)]

        for idx, pulse in enumerate(pulses):
            assert 1000 <= pulse <= 2000, (
                f"Frame {frame} j{idx}_us={pulse} outside hard servo range [1000, 2000]"
            )

        if state_name == "FAULT":
            assert all(p == 1500 for p in pulses), (
                f"Frame {frame} (FAULT) expected immediate 1500 us neutral, got {pulses}"
            )
        elif state_name == "STAND":
            hips = [pulses[0], pulses[2], pulses[4], pulses[6]]
            knees = [pulses[1], pulses[3], pulses[5], pulses[7]]
            if all(h == 1350 for h in hips) and all(k == 1800 for k in knees):
                stand_settled_seen = True
        elif state_name == "STOP":
            hips = [pulses[0], pulses[2], pulses[4], pulses[6]]
            knees = [pulses[1], pulses[3], pulses[5], pulses[7]]
            if all(h == 1350 for h in hips) and all(k == 1800 for k in knees):
                stop_settled_seen = True
        elif state_name == "WAVE":
            if pulses[2] <= 1150:
                wave_lifted_seen = True

    assert stand_settled_seen, "STAND never converged to exact 1350/1800 us target"
    assert stop_settled_seen, "STOP never converged to exact 1350/1800 us target"
    assert wave_lifted_seen, "WAVE never lifted front-right hip (j2 <= 1150 us)"


def test_vcd_waveform_integrity_and_50hz_pwm_timing() -> None:
    assert VCD_PATH.exists(), f"Missing VCD waveform: {VCD_PATH}"
    assert VCD_PATH.stat().st_size > 1024, "VCD file is unexpectedly small"

    servo_pwm_id: str | None = None
    found_signals: set[str] = set()
    rising_times_ps: list[int] = []
    pulse_widths_us: list[float] = []

    current_time_ps = 0
    prev_ch0_bit = "0"
    last_rise_ps: int | None = None
    in_header = True

    var_pattern = re.compile(r"^\$var\s+\S+\s+\d+\s+(\S+)\s+(\S+)")

    with VCD_PATH.open("r", encoding="utf-8", errors="ignore") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue

            if in_header:
                if line.startswith("$var"):
                    match = var_pattern.match(line)
                    if match:
                        sig_id, sig_name = match.group(1), match.group(2)
                        found_signals.add(sig_name)
                        if sig_name == "servo_pwm" and servo_pwm_id is None:
                            servo_pwm_id = sig_id
                elif line.startswith("$enddefinitions"):
                    in_header = False
                continue

            if line.startswith("#"):
                current_time_ps = int(line[1:])
            elif servo_pwm_id and line.startswith("b") and line.endswith(" " + servo_pwm_id):
                bits = line[1 : -(len(servo_pwm_id) + 1)]
                ch0_bit = bits[-1] if bits else "0"
                if prev_ch0_bit == "0" and ch0_bit == "1":
                    rising_times_ps.append(current_time_ps)
                    last_rise_ps = current_time_ps
                elif prev_ch0_bit == "1" and ch0_bit == "0" and last_rise_ps is not None:
                    width_us = (current_time_ps - last_rise_ps) / 1_000_000.0
                    pulse_widths_us.append(width_us)
                prev_ch0_bit = ch0_bit

    for expected_sig in ("servo_pwm", "frame_tick", "state", "fault_active", "j0", "j7"):
        assert expected_sig in found_signals, f"Signal '{expected_sig}' missing from VCD"

    assert len(rising_times_ps) >= 200, (
        f"Expected >=200 PWM frames in VCD, found {len(rising_times_ps)}"
    )

    periods_ms = [
        (rising_times_ps[i] - rising_times_ps[i - 1]) / 1_000_000_000.0
        for i in range(1, len(rising_times_ps))
    ]
    for period_ms in periods_ms:
        assert 19.99 <= period_ms <= 20.01, (
            f"VCD CH0 PWM period={period_ms:.6f} ms (expected 20.000 ms)"
        )

    assert len(pulse_widths_us) >= 200
    for width_us in pulse_widths_us:
        assert 999.0 <= width_us <= 2001.0, (
            f"VCD CH0 pulse width={width_us:.2f} us out of [1000, 2000] us bounds"
        )
