"""
Unit & Integration Tests for the XEROTON Verilog-to-Python Bridge and PyBullet Model.
"""

from __future__ import annotations

import sys
from pathlib import Path
import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PYTHON_DIR = PROJECT_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from verilog_bridge import (
    JOINT_LIMITS_RAD,
    JOINT_NAMES,
    filter_frames_by_state,
    load_verilog_csv,
    pulse_us_to_angle_rad,
    run_live_verilog_command,
    summarize_verilog_frames,
)


def test_pulse_to_angle_neutral_and_symmetry() -> None:
    for idx in range(8):
        assert pulse_us_to_angle_rad(1500.0, idx) == pytest.approx(0.0, abs=1e-9)
        pos = pulse_us_to_angle_rad(1750.0, idx)
        neg = pulse_us_to_angle_rad(1250.0, idx)
        assert pos == pytest.approx(-neg, abs=1e-9)
        assert pos > 0.0


def test_pulse_to_angle_respects_joint_limits() -> None:
    for idx, (low, high) in enumerate(JOINT_LIMITS_RAD):
        assert low <= pulse_us_to_angle_rad(500.0, idx) <= high
        assert low <= pulse_us_to_angle_rad(1000.0, idx) <= high
        assert low <= pulse_us_to_angle_rad(2000.0, idx) <= high
        assert low <= pulse_us_to_angle_rad(3000.0, idx) <= high


def test_load_actual_verilog_csv_and_filter_demos() -> None:
    csv_path = PROJECT_ROOT / "simulation" / "joint_targets.csv"
    frames = load_verilog_csv(csv_path)
    assert len(frames) >= 100

    counts = summarize_verilog_frames(frames)
    for state in ("IDLE", "STAND", "WALK", "TROT", "STOP", "WAVE", "DANCE", "SIT", "FAULT"):
        assert counts.get(state, 0) > 0, f"Expected state {state} in Verilog CSV"
        subset = filter_frames_by_state(frames, state)
        assert len(subset) == counts[state]


def test_live_verilog_runner_produces_authentic_wave_sit_trot_and_estop_frames() -> None:
    wave_frames = run_live_verilog_command(
        project_root=PROJECT_ROOT, command_code=4, estop=False, num_frames=20
    )
    assert len(wave_frames) == 20
    assert all(f.state_name == "WAVE" for f in wave_frames)

    sit_frames = run_live_verilog_command(
        project_root=PROJECT_ROOT, command_code=6, estop=False, num_frames=16
    )
    assert len(sit_frames) == 16
    assert all(f.state_name == "SIT" for f in sit_frames)

    trot_frames = run_live_verilog_command(
        project_root=PROJECT_ROOT, command_code=7, estop=False, num_frames=20, stride_us=105, phase_frames=3
    )
    assert len(trot_frames) == 20
    assert all(f.state_name == "TROT" for f in trot_frames)

    fault_frames = run_live_verilog_command(
        project_root=PROJECT_ROOT, command_code=2, estop=True, num_frames=10
    )
    assert len(fault_frames) == 10
    assert all(f.state_name == "FAULT" and f.fault_active for f in fault_frames)
    assert all(all(p == 1500.0 for p in f.pulses_us) for f in fault_frames)


def test_studio_engine_safety_gate_single_step_and_jarvis() -> None:
    pytest.importorskip("pybullet")
    from jarvis import parse_and_execute_jarvis
    from studio_server import StudioEngine
    from verilog_bridge import parse_vcd_summary

    vcd = parse_vcd_summary(PROJECT_ROOT / "simulation" / "xeroton.vcd", max_edges=10)
    assert vcd["available"] is True
    assert 19.9 <= vcd["measured_ch0_period_ms"] <= 20.1

    eng = StudioEngine(
        PROJECT_ROOT,
        PROJECT_ROOT / "robot" / "xeroton.urdf",
        PROJECT_ROOT / "simulation" / "joint_targets.csv",
    )
    try:
        # Single-step execution (+20 ms)
        step_res = eng.handle_command({"action": "STEP"})
        assert step_res["ok"] is True

        # Safe manual joint clamping
        m_ok = eng.handle_manual_joints({"pulses_us": [1400, 1750, 1500, 1500, 1500, 1500, 1500, 1500]})
        assert m_ok["ok"] is True
        assert m_ok["control_source"] == "Manual Control (Safety-Gated)"

        # JARVIS navigation & E-STOP safety priority
        j_nav = parse_and_execute_jarvis("Show PWM signals", eng)
        assert j_nav["ok"] is True and j_nav["navigate"] == "pwm"

        j_estop = parse_and_execute_jarvis("Emergency stop now", eng)
        assert j_estop["ok"] is True and eng.fault_latched is True

        # Motion MUST be blocked while fault is latched
        j_blocked = parse_and_execute_jarvis("Start walking", eng)
        assert j_blocked["ok"] is False and j_blocked["intent"] == "BLOCKED_BY_FAULT"

        # Explain fault works while latched, then deliberate reset unlocks
        j_exp = parse_and_execute_jarvis("Explain the current fault", eng)
        assert j_exp["ok"] is True and "ACTIVE FAULT" in j_exp["reply"]

        j_clr = parse_and_execute_jarvis("Clear fault", eng)
        assert j_clr["ok"] is True and eng.fault_latched is False
    finally:
        eng.stop()


def test_pybullet_urdf_loads_and_maps_all_eight_joints() -> None:
    pybullet = pytest.importorskip("pybullet")
    from robot_simulator import XerotonSimulator

    urdf_path = PROJECT_ROOT / "robot" / "xeroton.urdf"
    sim = XerotonSimulator(urdf_path=urdf_path, gui=False)
    sim.connect()
    try:
        assert sim.robot_id is not None
        assert pybullet.getNumJoints(sim.robot_id, physicsClientId=sim.client_id) == 8
        for name in JOINT_NAMES:
            assert name in sim.joint_indices
        assert len(sim.foot_link_indices) == 4
    finally:
        sim.disconnect()


def test_pybullet_end_to_end_verilog_replay_headless() -> None:
    pytest.importorskip("pybullet")
    from robot_simulator import XerotonSimulator

    frames = load_verilog_csv(PROJECT_ROOT / "simulation" / "joint_targets.csv")
    sim = XerotonSimulator(urdf_path=PROJECT_ROOT / "robot" / "xeroton.urdf", gui=False)
    sim.connect()
    try:
        history = sim.run_sequence(frames, real_time=False)
        assert len(history) == len(frames)
        # Ensure compact Reference-2 robot (~0.175 m tall) never fell through floor or flipped
        for row in history:
            assert row.pos_z > 0.08, f"Robot collapsed on frame {row.frame}: z={row.pos_z}"
            assert abs(row.roll_deg) < 45.0, f"Excessive roll on frame {row.frame}: {row.roll_deg}"
            assert abs(row.pitch_deg) < 45.0, f"Excessive pitch on frame {row.frame}: {row.pitch_deg}"
    finally:
        sim.disconnect()
