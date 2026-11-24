"""
Automated End-to-End Directional Movement & Manual Pilot Control Tests (Tests 1-15).

Verifies real Icarus Verilog RTL -> Python Command Manager -> 250 Hz PyBullet Physics:
  - Stationary STAND startup (autonomous demo disabled by default)
  - Hold-to-move FORWARD (W), BACKWARD (S), STRAFE_LEFT (A), STRAFE_RIGHT (D),
    TURN_LEFT (Q), TURN_RIGHT (E), STOP (SPACE), NEUTRAL STAND (R)
  - Multi-cycle continuous forward hold with ZERO spontaneous reversal
  - Rapid direction switching & monotonic sequence stale-command rejection
  - E-STOP lockout while moving + deliberate reset recovery
  - Dead-man heartbeat timeout on focus loss
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PYTHON_DIR = PROJECT_ROOT / "python"
if str(PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_DIR))

from studio_server import StudioEngine  # noqa: E402


def _step_n(eng: StudioEngine, n: int) -> dict:
    """Deterministically advance n 20ms Verilog/PyBullet frames without wall-clock jitter."""
    with eng._lock:
        for _ in range(n):
            eng._step_one_frame_locked()
        return dict(eng.latest_snapshot)


def _make_engine() -> StudioEngine:
    pytest.importorskip("pybullet")
    eng = StudioEngine(
        project_root=PROJECT_ROOT,
        urdf_path=PROJECT_ROOT / "robot" / "xeroton.urdf",
        demo_csv_path=PROJECT_ROOT / "simulation" / "joint_targets.csv",
    )
    assert eng.toolchain["connected"], "Icarus Verilog RTL toolchain must be connected"
    return eng


def test_startup_stationary_stand_and_consistent_status() -> None:
    eng = _make_engine()
    try:
        s0 = _step_n(eng, 20)
        s1 = _step_n(eng, 40)
        assert s1["state"] == "STAND"
        assert s1["active_direction"] == "STOP"
        assert s1["control_priority"] == "P2_STOP"
        assert s1["rtl_connected"] is True
        assert s1["rtl_status"] == "CONNECTED"
        assert s1["control_source"] == "Icarus Verilog RTL"
        assert abs(s1["base_pos"][0] - s0["base_pos"][0]) < 0.015
        assert abs(s1["base_pos"][1] - s0["base_pos"][1]) < 0.015
        assert s1["motion_settings"]["preset"] == "SLOW"
    finally:
        eng.stop()


def test_forward_backward_release_and_no_self_reversal() -> None:
    eng = _make_engine()
    try:
        # Use NORMAL preset for crisp multi-cycle travel in fast unit test
        eng.handle_settings({"preset": "NORMAL"})
        s_start = _step_n(eng, 10)
        x0 = s_start["base_pos"][0]

        # TEST 1 & TEST 11: Hold W for 5+ full gait cycles (160 frames = 5 cycles * 32 fr)
        seq = 1
        x_samples = [x0]
        for cyc in range(5):
            res = eng.handle_pilot_command({"vx": 1.0, "vy": 0.0, "wz": 0.0, "hold": True, "seq": seq})
            seq += 1
            assert res["ok"] and res["direction"] == "FORWARD"
            # Refresh heartbeat so dead-man timer stays alive during fast deterministic stepping
            snap = _step_n(eng, 32)
            x_samples.append(snap["base_pos"][0])

        # Verify positive forward travel AND monotonic progress (never reverses!)
        assert x_samples[-1] - x0 > 0.20, f"Expected forward +X travel, got {x_samples}"
        for i in range(1, len(x_samples)):
            assert x_samples[i] > x_samples[i - 1] + 0.01, (
                f"Unexpected reversal/stall during continuous W hold: {x_samples}"
            )

        # TEST 2: Release W -> stops cleanly and holds steady
        res_stop = eng.handle_pilot_command({"vx": 0.0, "vy": 0.0, "wz": 0.0, "hold": False, "seq": seq})
        seq += 1
        assert res_stop["ok"] and res_stop["direction"] == "STOP"
        s_after_settle = _step_n(eng, 20)
        s_hold = _step_n(eng, 40)
        assert s_hold["state"] == "STAND"
        assert abs(s_hold["base_pos"][0] - s_after_settle["base_pos"][0]) < 0.02

        # TEST 3 & TEST 4: Hold S -> moves backward (-X), release S -> stops
        x_before_back = s_hold["base_pos"][0]
        for _ in range(3):
            res_b = eng.handle_pilot_command({"vx": -1.0, "vy": 0.0, "wz": 0.0, "hold": True, "seq": seq})
            seq += 1
            assert res_b["ok"] and res_b["direction"] == "BACKWARD"
            s_back = _step_n(eng, 32)
        assert s_back["base_pos"][0] < x_before_back - 0.15, (
            f"Expected backward -X travel: before={x_before_back:.3f}, after={s_back['base_pos'][0]:.3f}"
        )

        eng.handle_pilot_command({"vx": 0.0, "vy": 0.0, "wz": 0.0, "hold": False, "seq": seq})
        s_b1 = _step_n(eng, 20)
        s_b2 = _step_n(eng, 40)
        assert s_b2["state"] == "STAND"
        assert abs(s_b2["base_pos"][0] - s_b1["base_pos"][0]) < 0.02
    finally:
        eng.stop()


def test_strafe_left_right_and_turn_left_right() -> None:
    eng = _make_engine()
    try:
        eng.handle_settings({"preset": "NORMAL"})
        seq = 1

        # TEST 5: Strafe LEFT (+Y)
        eng.handle_command({"action": "RESET"})
        s0 = _step_n(eng, 10)
        for _ in range(4):
            res = eng.handle_pilot_command({"vx": 0.0, "vy": 1.0, "wz": 0.0, "hold": True, "seq": seq})
            seq += 1
            assert res["ok"] and res["direction"] == "STRAFE_LEFT"
            s_left = _step_n(eng, 32)
        dy_l = s_left["base_pos"][1] - s0["base_pos"][1]
        assert dy_l > 0.05, f"Expected STRAFE_LEFT +Y > 0.05m, got dY={dy_l:+.4f}m"
        assert abs(s_left["rpy_deg"][2] - s0["rpy_deg"][2]) < 10.0

        # TEST 6: Strafe RIGHT (-Y)
        eng.handle_command({"action": "RESET"})
        s0 = _step_n(eng, 10)
        for _ in range(4):
            res = eng.handle_pilot_command({"vx": 0.0, "vy": -1.0, "wz": 0.0, "hold": True, "seq": seq})
            seq += 1
            assert res["ok"] and res["direction"] == "STRAFE_RIGHT"
            s_right = _step_n(eng, 32)
        dy_r = s_right["base_pos"][1] - s0["base_pos"][1]
        assert dy_r < -0.05, f"Expected STRAFE_RIGHT -Y < -0.05m, got dY={dy_r:+.4f}m"
        assert abs(s_right["rpy_deg"][2] - s0["rpy_deg"][2]) < 10.0

        # TEST 7: Turn LEFT (+yaw CCW)
        eng.handle_command({"action": "RESET"})
        s0 = _step_n(eng, 10)
        for _ in range(5):
            res = eng.handle_pilot_command({"vx": 0.0, "vy": 0.0, "wz": 1.0, "hold": True, "seq": seq})
            seq += 1
            assert res["ok"] and res["direction"] == "TURN_LEFT"
            s_tl = _step_n(eng, 32)
        dyaw_l = s_tl["rpy_deg"][2] - s0["rpy_deg"][2]
        assert dyaw_l > 8.0, f"Expected TURN_LEFT +yaw > 8 deg, got {dyaw_l:+.2f} deg"

        # TEST 8: Turn RIGHT (-yaw CW)
        eng.handle_command({"action": "RESET"})
        s0 = _step_n(eng, 10)
        for _ in range(5):
            res = eng.handle_pilot_command({"vx": 0.0, "vy": 0.0, "wz": -1.0, "hold": True, "seq": seq})
            seq += 1
            assert res["ok"] and res["direction"] == "TURN_RIGHT"
            s_tr = _step_n(eng, 32)
        dyaw_r = s_tr["rpy_deg"][2] - s0["rpy_deg"][2]
        assert dyaw_r < -8.0, f"Expected TURN_RIGHT -yaw < -8 deg, got {dyaw_r:+.2f} deg"
    finally:
        eng.stop()


def test_stop_rapid_switching_estop_reset_and_deadman_timeout() -> None:
    eng = _make_engine()
    try:
        # TEST 10 & TEST 12: Rapid switching + stale sequence rejection
        eng.handle_pilot_command({"vx": 0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 10})
        _step_n(eng, 6)
        eng.handle_pilot_command({"vx": -0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 11})
        # Out-of-order delayed packet with older seq=10 MUST be ignored!
        stale = eng.handle_pilot_command({"vx": 0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 10})
        assert stale.get("ignored_stale") is True
        snap = _step_n(eng, 4)
        assert snap["active_direction"] == "BACKWARD"
        assert snap["state"] == "WALK_BACKWARD"

        # TEST 9: Immediate SPACE stop clears velocity & direction immediately
        res_sp = eng.handle_pilot_command({"immediate_stop": True, "seq": 12})
        assert res_sp["ok"] and res_sp["direction"] == "STOP"
        snap = _step_n(eng, 4)
        assert snap["active_direction"] == "STOP"
        assert snap["cmd_vel"] == {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        assert snap["state"] == "STAND"

        # TEST 13: Trigger E-STOP while walking -> latches FAULT & blocks pilot commands
        eng.handle_pilot_command({"vx": 0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 13})
        _step_n(eng, 8)
        eng.handle_command({"action": "FAULT", "reason": "Test E-STOP while walking"})
        snap_f = _step_n(eng, 4)
        assert snap_f["fault_active"] is True
        assert snap_f["state"] == "FAULT"
        assert snap_f["control_priority"] == "P1_ESTOP"
        blocked = eng.handle_pilot_command({"vx": 0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 14})
        assert blocked["ok"] is False

        # TEST 14: Clear fault + Reset -> returns cleanly to valid neutral STAND at origin
        eng.handle_command({"action": "CLEAR_FAULT"})
        eng.handle_command({"action": "RESET"})
        snap_r = _step_n(eng, 10)
        assert snap_r["fault_active"] is False
        assert snap_r["state"] == "STAND"
        assert abs(snap_r["base_pos"][0]) < 0.02
        assert abs(snap_r["base_pos"][1]) < 0.02

        # TEST 15: Dead-man timeout (focus loss / stopped heartbeat) automatically stops robot
        eng.handle_pilot_command({"vx": 0.65, "vy": 0.0, "wz": 0.0, "hold": True, "seq": 15})
        _step_n(eng, 4)
        assert eng.active_direction == "FORWARD"
        # Simulate >650ms heartbeat lapse
        with eng._lock:
            eng._pilot_last_heartbeat = time.monotonic() - 1.0
        snap_dm = _step_n(eng, 2)
        assert snap_dm["active_direction"] == "STOP"
        assert snap_dm["state"] == "STAND"
        assert snap_dm["control_priority"] == "P2_STOP"
    finally:
        eng.stop()
