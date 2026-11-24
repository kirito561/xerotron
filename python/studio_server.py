"""
XEROTON Studio 3.0 — Real-Time Verilog RTL + PyBullet Engineering Server.

Unified backend orchestrating:
  - Real Icarus Verilog RTL execution (`xeroton_live.vvp` & `joint_targets.csv`)
  - Single Authoritative Movement Command Manager (`vx, vy, wz` REP-103)
  - Strict 4-Tier Command Priority:
      P1_ESTOP > P2_STOP > P3_MANUAL_PILOT / P3_MANUAL_JOINTS > P4_DEMO
  - Hold-to-Move Dead-Man Control (monotonic sequence guard + 650 ms heartbeat timeout)
  - Real 250 Hz PyBullet rigid-body physics (`robot_simulator.py`)
  - Movement Settings & Presets (SLOW default, NORMAL, FAST)
  - Consistent non-contradictory RTL / Simulator / Bridge / PyBullet status
  - JARVIS Local Safe Text Command Parser (`jarvis.py`) & Automated Diagnostics
"""

from __future__ import annotations

import csv
import io
import json
import mimetypes
import threading
import time
import webbrowser
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional, Sequence
from urllib.parse import parse_qs, urlparse

from diagnostics import DiagnosticsManager
from jarvis import parse_and_execute_jarvis
from robot_simulator import XerotonSimulator
from verilog_bridge import (
    COMMAND_NAME_TO_CODE,
    JOINT_LIMITS_US,
    VerilogFrame,
    check_rtl_toolchain,
    load_verilog_csv,
    parse_vcd_summary,
    pulse_us_to_angle_rad,
    run_live_verilog_command,
)

# Documented REP-103 Robot Body Coordinate Convention:
#   +X = Forward (toward OLED face), +Y = Left, +Z = Up
#   vx > 0 : FORWARD,      vx < 0 : BACKWARD
#   vy > 0 : STRAFE_LEFT,  vy < 0 : STRAFE_RIGHT
#   wz > 0 : TURN_LEFT,    wz < 0 : TURN_RIGHT
COORD_CONVENTION = "REP-103 (+X Forward, +Y Left, +Z Up; +wz CCW Turn Left)"

MOTION_PRESETS: dict[str, dict[str, Any]] = {
    "SLOW": {
        "preset": "SLOW",
        "walk_speed": 0.65,
        "turn_speed": 0.70,
        "stride_us": 60,
        "phase_frames": 6,
        "smoothing": 0.55,
    },
    "NORMAL": {
        "preset": "NORMAL",
        "walk_speed": 1.00,
        "turn_speed": 1.00,
        "stride_us": 76,
        "phase_frames": 4,
        "smoothing": 0.72,
    },
    "FAST": {
        "preset": "FAST",
        "walk_speed": 1.35,
        "turn_speed": 1.35,
        "stride_us": 92,
        "phase_frames": 3,
        "smoothing": 0.90,
    },
}

STAND_PULSES_US: tuple[float, ...] = (1350.0, 1800.0, 1350.0, 1800.0, 1350.0, 1800.0, 1350.0, 1800.0)
PILOT_DEADMAN_TIMEOUT_S = 0.65


class GaitEvalTracker:
    """Tracks live PyBullet stability metrics for the currently executing gait."""

    def __init__(self, gait_name: str = "STAND") -> None:
        self.reset(gait_name)

    def reset(self, gait_name: str, start_x: float = 0.0) -> None:
        self.gait_name = gait_name
        self.frames = 0
        self.start_x = start_x
        self.curr_x = start_x
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0
        self.min_z = 9.9
        self.sum_z = 0.0
        self.sum_feet = 0
        self.max_slip = 0.0
        self.max_pen = 0.0
        self.body_hits = 0
        self.fell = False

    def record(self, snap: dict[str, Any]) -> None:
        self.frames += 1
        x, _y, z = snap["base_pos"]
        if self.frames == 1:
            self.start_x = x
        self.curr_x = x
        r, p = abs(snap["rpy_deg"][0]), abs(snap["rpy_deg"][1])
        self.max_abs_roll = max(self.max_abs_roll, r)
        self.max_abs_pitch = max(self.max_abs_pitch, p)
        self.min_z = min(self.min_z, z)
        self.sum_z += z
        self.sum_feet += sum(1 for f in snap["foot_contacts"] if f)
        self.max_slip = max(self.max_slip, float(snap.get("foot_slip_mps", 0.0)))
        self.max_pen = max(self.max_pen, float(snap.get("max_penetration_m", 0.0)))
        if snap.get("body_hit"):
            self.body_hits += 1
        if r > 48.0 or p > 48.0 or z < 0.075:
            self.fell = True

    def summary(self) -> dict[str, Any]:
        avg_z = (self.sum_z / self.frames) if self.frames else 0.0
        avg_feet = (self.sum_feet / self.frames) if self.frames else 0.0
        if self.fell or self.body_hits > 0:
            label = "UNSTABLE (FALL / BODY CONTACT)"
        elif self.frames < 40:
            label = "EXPERIMENTAL (COLLECTING PHYSICS DATA)"
        elif self.max_abs_roll < 15.0 and self.max_abs_pitch < 18.0 and self.max_pen < 0.015:
            label = "VALIDATED (FLAT-GROUND SIM)"
        else:
            label = "EXPERIMENTAL (HIGH OSCILLATION)"

        return {
            "gait": self.gait_name,
            "frames_evaluated": self.frames,
            "distance_x_m": round(self.curr_x - self.start_x, 4),
            "max_abs_roll_deg": round(self.max_abs_roll, 2),
            "max_abs_pitch_deg": round(self.max_abs_pitch, 2),
            "avg_height_m": round(avg_z, 4),
            "min_height_m": round(self.min_z if self.frames else 0.0, 4),
            "avg_feet_in_contact": round(avg_feet, 2),
            "max_foot_slip_mps": round(self.max_slip, 4),
            "max_penetration_mm": round(self.max_pen * 1000.0, 2),
            "body_hits": self.body_hits,
            "fell": self.fell,
            "stability_label": label,
        }


class StudioEngine:
    """
    Thread-safe real-time Verilog + PyBullet simulation & authoritative
    Movement Command Manager.

    Guarantees:
      - Single physics stepping loop (250 Hz -> 5 substeps / 20 ms Verilog frame)
      - Single authoritative command queue (no duplicate gait loops or overwrites)
      - Stationary STAND on startup (autonomous demo DISABLED by default)
      - Dead-man hold-to-move timeout automatically stops robot on release/disconnect
    """

    def __init__(
        self,
        project_root: Path,
        urdf_path: Path,
        demo_csv_path: Path,
        playback_speed: float = 1.0,
    ) -> None:
        self.project_root = project_root
        self.urdf_path = urdf_path
        self.demo_csv_path = demo_csv_path
        self.speed = max(0.25, min(2.0, playback_speed))

        self._lock = threading.Lock()
        self._vvp_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._warmup_thread: Optional[threading.Thread] = None

        self.toolchain = check_rtl_toolchain(project_root)
        self.diagnostics = DiagnosticsManager(project_root)

        self.sim = XerotonSimulator(urdf_path=urdf_path, gui=False)
        self.sim.connect()

        # Real Icarus Verilog session cache: (cmd_code, stride_us, phase_frames) -> (frames, exec_ms)
        self._rtl_cache: dict[tuple[int, int, int], tuple[list[VerilogFrame], float]] = {}

        # Movement settings (default SLOW preset per spec)
        self.preset_name: str = "SLOW"
        self.walk_speed: float = float(MOTION_PRESETS["SLOW"]["walk_speed"])
        self.turn_speed: float = float(MOTION_PRESETS["SLOW"]["turn_speed"])
        self.stride_us: int = int(MOTION_PRESETS["SLOW"]["stride_us"])
        self.phase_frames: int = int(MOTION_PRESETS["SLOW"]["phase_frames"])
        self.smoothing: float = float(MOTION_PRESETS["SLOW"]["smoothing"])

        # Centralized Movement Command State (P1 > P2 > P3 > P4)
        self.cmd_vel: dict[str, float] = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.active_direction: str = "STOP"
        self.active_command: str = "STAND"
        self.control_priority: str = "P2_STOP"
        self.pilot_hold_active: bool = False
        self._pilot_last_heartbeat: float = 0.0
        self._last_pilot_seq: int = -1

        self.control_source: str = (
            "Icarus Verilog RTL" if self.toolchain["connected"] else "DISCONNECTED"
        )
        self.mode_label: str = "MANUAL PILOT: STAND"
        self.source_badge: str = self.control_source
        self.loop_enabled: bool = True
        self.paused: bool = False
        self.stopped: bool = False
        self.rtl_exec_ms: float = 0.0
        self.total_steps: int = 0
        self.reset_active: bool = False

        # Safety & Fault Monitor state
        self.fault_latched: bool = False
        self.fault_code: int = 0
        self.fault_reason: str = ""
        self.fault_history: deque[dict[str, Any]] = deque(maxlen=25)
        self.fsm_history: deque[dict[str, Any]] = deque(maxlen=30)
        self.pwm_history: deque[dict[str, Any]] = deque(maxlen=250)
        self._last_logged_state: str = ""

        # Authoritative frame queue (starts cleanly in stationary STAND — NO autonomous demo!)
        self.queue: list[VerilogFrame] = []
        self.queue_idx: int = 0
        self.manual_frame: Optional[VerilogFrame] = None
        self.gait_eval = GaitEvalTracker("STAND")

        stand_frame = self._make_stand_frame(prev_state="IDLE")
        if self.toolchain["connected"]:
            try:
                frames, exec_ms = self._fetch_rtl_cycle("STAND", stride_us=0, phase_frames=0)
                if frames:
                    stand_frame = frames[-1]
                    self.queue = [stand_frame]
                    self.rtl_exec_ms = exec_ms
            except Exception:
                self.queue = [stand_frame]
        else:
            self.queue = [stand_frame]

        self.sim.reset_pose(initial_angles=stand_frame.angles_rad)
        for _ in range(12):
            self.sim.step_idle()
        self.latest_snapshot: dict[str, Any] = self._build_snapshot(stand_frame)

    # ------------------------------------------------------------------
    # Real Verilog RTL Execution + Session Verification Cache
    # ------------------------------------------------------------------
    def _fetch_rtl_cycle(
        self,
        action: str,
        stride_us: int = 0,
        phase_frames: int = 0,
        num_frames: int = 0,
        force_refresh: bool = False,
    ) -> tuple[list[VerilogFrame], float]:
        """Execute real `vvp xeroton_live.vvp` for a command & cache verified cycle."""
        cmd_code = COMMAND_NAME_TO_CODE[action]
        eff_stride = int(stride_us) if action in (
            "WALK", "WALK_FORWARD", "WALK_BACKWARD", "TROT", "TURN_LEFT", "TURN_RIGHT"
        ) else 0
        eff_pf = int(phase_frames) if action in (
            "WALK", "WALK_FORWARD", "WALK_BACKWARD", "TROT",
            "STRAFE_LEFT", "STRAFE_RIGHT", "TURN_LEFT", "TURN_RIGHT", "WAVE", "DANCE",
        ) else 0
        key = (cmd_code, eff_stride, eff_pf)

        if not force_refresh and num_frames == 0 and key in self._rtl_cache:
            return self._rtl_cache[key]

        with self._vvp_lock:
            if not force_refresh and num_frames == 0 and key in self._rtl_cache:
                return self._rtl_cache[key]
            t0 = time.perf_counter()
            out_rel = f"simulation/live_cmd_{cmd_code}.csv"
            frames = run_live_verilog_command(
                project_root=self.project_root,
                command_code=cmd_code,
                estop=False,
                num_frames=num_frames,
                stride_us=eff_stride,
                phase_frames=eff_pf,
                out_csv_rel=out_rel,
            )
            exec_ms = (time.perf_counter() - t0) * 1000.0
            if num_frames == 0 and frames:
                self._rtl_cache[key] = (frames, exec_ms)
            return frames, exec_ms

    def _effective_directional_stride(self, action: str) -> int:
        if action in ("TURN_LEFT", "TURN_RIGHT"):
            scaled = int(round(self.stride_us * (0.75 + 0.25 * self.turn_speed)))
        else:
            scaled = int(round(self.stride_us * (0.75 + 0.25 * self.walk_speed)))
        return max(35, min(105, scaled))

    def _warm_pilot_rtl_cache(self) -> None:
        """Background pre-execution of real `vvp` for the 6 pilot directions at current settings."""
        if not self.toolchain["connected"]:
            return
        pilot_actions = (
            "STAND", "STOP", "WALK_FORWARD", "WALK_BACKWARD",
            "STRAFE_LEFT", "STRAFE_RIGHT", "TURN_LEFT", "TURN_RIGHT",
        )
        for act in pilot_actions:
            if self._stop_event.is_set():
                return
            try:
                st = self._effective_directional_stride(act)
                pf = self.phase_frames
                self._fetch_rtl_cycle(act, stride_us=st, phase_frames=pf, num_frames=0)
            except Exception:
                return

    # ------------------------------------------------------------------
    # Frame Builders & Safety Fault Latch
    # ------------------------------------------------------------------
    def _make_safe_frame(
        self,
        state_name: str,
        state_code: int,
        pulses_us: Optional[Sequence[float]] = None,
        fault_active: bool = False,
        fault_code: int = 0,
        command: int = 0,
        prev_state: str = "IDLE",
    ) -> VerilogFrame:
        p_vals = tuple(float(v) for v in (pulses_us if pulses_us is not None else [1500.0] * 8))
        angles = tuple(pulse_us_to_angle_rad(v, i) for i, v in enumerate(p_vals))
        return VerilogFrame(
            frame=self.total_steps + 1,
            time_ms=round(self.sim.sim_time_s * 1000.0, 2),
            command=command,
            state_code=state_code,
            state_name=state_name,
            gait_phase=0,
            fault_active=fault_active,
            fault_code=fault_code,
            pulses_us=p_vals,
            angles_rad=angles,
            prev_state_code=0,
            prev_state_name=prev_state,
        )

    def _make_stand_frame(self, prev_state: str = "IDLE") -> VerilogFrame:
        return self._make_safe_frame(
            "STAND", 1, pulses_us=STAND_PULSES_US, command=1, prev_state=prev_state
        )

    def _latch_fault(self, code: int, reason: str) -> None:
        self.fault_latched = True
        self.fault_code |= int(code)
        self.fault_reason = reason
        self.manual_frame = None
        self.pilot_hold_active = False
        self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.active_direction = "STOP"
        self.active_command = "FAULT"
        self.control_priority = "P1_ESTOP"
        self.fault_history.appendleft({
            "time": datetime.now().strftime("%H:%M:%S"),
            "code": f"0b{self.fault_code:04b}",
            "reason": reason,
        })

    def _clear_motion_to_stand_locked(self, reason_label: str = "MANUAL PILOT: STAND") -> None:
        """Priority 2 STOP: immediately clear active movement and smoothly return to STAND."""
        prev = self.latest_snapshot.get("state", "STAND") if hasattr(self, "latest_snapshot") else "IDLE"
        self.pilot_hold_active = False
        self.manual_frame = None
        self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.active_direction = "STOP"
        self.active_command = "STAND"
        self.control_priority = "P2_STOP"
        self.mode_label = reason_label
        if self.toolchain["connected"]:
            self.control_source = "Icarus Verilog RTL"
            self.source_badge = "Icarus Verilog RTL (STAND)"
        else:
            self.control_source = "DISCONNECTED"
            self.source_badge = "DISCONNECTED"

        # Use cached real STAND frame if available
        cached = self._rtl_cache.get((COMMAND_NAME_TO_CODE["STAND"], 0, 0))
        stand_frame = cached[0][-1] if (cached and cached[0]) else self._make_stand_frame(prev_state=prev)
        self.queue = [stand_frame]
        self.queue_idx = 0

    def _motion_settings_dict(self) -> dict[str, Any]:
        gait_freq_hz = round(50.0 / (8.0 * max(1, self.phase_frames)), 2)
        return {
            "preset": self.preset_name,
            "walk_speed": round(self.walk_speed, 2),
            "turn_speed": round(self.turn_speed, 2),
            "stride_us": int(self.stride_us),
            "phase_frames": int(self.phase_frames),
            "gait_freq_hz": gait_freq_hz,
            "smoothing": round(self.smoothing, 2),
        }

    def _build_snapshot(self, frame: VerilogFrame) -> dict[str, Any]:
        snap = self.sim.get_studio_snapshot(frame)
        if self.fault_latched:
            snap["fault_active"] = True
            snap["fault_code"] = self.fault_code or snap["fault_code"] or 1
            snap["state"] = "FAULT"
            snap["fault_reason"] = self.fault_reason
        elif snap["fault_active"]:
            snap["fault_reason"] = self._decode_rtl_fault_code(snap["fault_code"])
        else:
            snap["fault_reason"] = ""

        # Record FSM state transitions
        if snap["state"] != self._last_logged_state:
            self.fsm_history.appendleft({
                "time": datetime.now().strftime("%H:%M:%S"),
                "from": self._last_logged_state or "RESET",
                "to": snap["state"],
                "cmd": snap["command"],
                "source": self.control_source,
            })
            self._last_logged_state = snap["state"]

        # Record real PWM history for export
        self.pwm_history.append({
            "frame": snap["frame"],
            "sim_time_s": snap["sim_time_s"],
            "state": snap["state"],
            "control_source": self.control_source,
            "period_ms": 20.0,
            "freq_hz": 50.0,
            "pulses_us": snap["pulses_us"],
        })

        rtl_conn = bool(self.toolchain.get("connected", False))
        pyb_conn = self.sim.is_connected()
        if not pyb_conn:
            pyb_status = "DISCONNECTED"
        elif self.stopped:
            pyb_status = "STOPPED"
        elif self.paused:
            pyb_status = "PAUSED"
        else:
            pyb_status = "RUNNING (250 Hz)"

        snap["api_version"] = "3.3"
        snap["rtl_connected"] = rtl_conn
        snap["rtl_status"] = "CONNECTED" if rtl_conn else "DISCONNECTED"
        snap["verilog_sim_status"] = "READY (Icarus Verilog)" if rtl_conn else "DISCONNECTED"
        snap["python_bridge_status"] = "ACTIVE" if pyb_conn else "DISCONNECTED"
        snap["pybullet_status"] = pyb_status
        snap["physics_status"] = (
            "FAULT LOCKOUT" if self.fault_latched else pyb_status
        )

        # Never report contradictory RTL source when disconnected
        if not rtl_conn and self.control_source == "Icarus Verilog RTL":
            snap["control_source"] = "DISCONNECTED"
        else:
            snap["control_source"] = self.control_source or "DISCONNECTED"

        snap["active_command"] = self.active_command
        snap["active_direction"] = self.active_direction
        snap["active_gait"] = snap["state"]
        snap["cmd_vel"] = dict(self.cmd_vel)
        snap["control_priority"] = self.control_priority
        snap["pilot_hold_active"] = self.pilot_hold_active
        snap["motion_settings"] = self._motion_settings_dict()
        snap["coord_convention"] = COORD_CONVENTION

        snap["mode_label"] = self.mode_label
        snap["source_badge"] = self.source_badge
        snap["loop_enabled"] = self.loop_enabled
        snap["paused"] = self.paused
        snap["stopped"] = self.stopped
        snap["reset_active"] = self.reset_active
        snap["speed"] = self.speed
        snap["rtl_exec_ms"] = round(self.rtl_exec_ms, 1)
        snap["tick"] = self.total_steps
        snap["stride_us"] = self.stride_us
        snap["phase_frames"] = self.phase_frames
        snap["toolchain"] = self.toolchain
        snap["fault_history"] = list(self.fault_history)[:10]
        snap["fsm_history"] = list(self.fsm_history)[:12]
        snap["gait_eval"] = self.gait_eval.summary()
        return snap

    @staticmethod
    def _decode_rtl_fault_code(code: int) -> str:
        parts = []
        if code & 1:
            parts.append("E-STOP")
        if code & 2:
            parts.append("Invalid Verilog Command")
        if code & 4:
            parts.append("Watchdog Timeout")
        if code & 8:
            parts.append("Joint Limit Clamp")
        return " + ".join(parts) if parts else "RTL Fault"

    # ------------------------------------------------------------------
    # Authoritative Single Physics Loop (50 Hz Verilog -> 250 Hz PyBullet)
    # ------------------------------------------------------------------
    def _step_one_frame_locked(self) -> None:
        """Advance exactly 1 Verilog/Control frame (20 ms = 5 PyBullet substeps)."""
        # Dead-Man Control Principle: if a held manual pilot input stops sending
        # heartbeats (e.g. window lost focus / network drop), release & stop safely!
        if (
            self.pilot_hold_active
            and not self.fault_latched
            and (time.monotonic() - self._pilot_last_heartbeat) > PILOT_DEADMAN_TIMEOUT_S
        ):
            self._clear_motion_to_stand_locked("DEAD-MAN STOP -> STAND")

        if self.fault_latched:
            frame = self._make_safe_frame(
                "FAULT", 4, fault_active=True, fault_code=self.fault_code,
                prev_state=self.latest_snapshot.get("state", "IDLE"),
            )
        elif self.control_source == "Manual Control (Safety-Gated)" and self.manual_frame is not None:
            frame = self.manual_frame
        elif self.queue:
            if self.queue_idx >= len(self.queue):
                last_state = self.queue[-1].state_name
                is_static_pose = (
                    self.mode_label != "VERILOG FULL DEMO"
                    and last_state in ("IDLE", "STAND", "STOP", "SIT", "SLEEP", "FAULT")
                )
                if self.loop_enabled and not is_static_pose:
                    self.queue_idx = 0
                    if self.mode_label == "VERILOG FULL DEMO":
                        self.sim.reset_pose(initial_angles=self.queue[0].angles_rad)
                        self.gait_eval.reset("DEMO")
                else:
                    # Hold settled static pose cleanly with zero ramp jump
                    self.queue_idx = len(self.queue) - 1
            frame = self.queue[self.queue_idx]
            self.queue_idx += 1
        else:
            frame = self._make_stand_frame()

        # Contact-coupled lateral abduction speed (strictly 0 unless actively in STRAFE_LEFT/RIGHT)
        strafe_vy = 0.0
        if not self.fault_latched:
            if frame.state_name == "STRAFE_LEFT":
                strafe_vy = +0.16 * self.walk_speed
            elif frame.state_name == "STRAFE_RIGHT":
                strafe_vy = -0.16 * self.walk_speed

        self.sim.apply_verilog_frame(
            frame,
            real_time=False,
            smoothing_alpha=self.smoothing,
            strafe_target_vy=strafe_vy,
        )
        self.total_steps += 1
        snap = self._build_snapshot(frame)
        self.gait_eval.record(snap)

        # Runtime Fall & Physics Error Safety Detection
        if not self.fault_latched:
            roll, pitch = abs(snap["rpy_deg"][0]), abs(snap["rpy_deg"][1])
            z = snap["base_pos"][2]
            if snap["physics_error_count"] > 0:
                self._latch_fault(0b1000, f"Physics error: {snap['last_physics_error']}")
            elif roll > 50.0 or pitch > 50.0 or z < 0.075:
                self._latch_fault(
                    0b1000,
                    f"Robot fall detected (roll={roll:.1f} deg, pitch={pitch:.1f} deg, z={z:.2f} m)",
                )

        snap["gait_eval"] = self.gait_eval.summary()
        self.latest_snapshot = snap

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_loop, name="xeroton-physics", daemon=True)
        self._thread.start()
        self._warmup_thread = threading.Thread(
            target=self._warm_pilot_rtl_cache, name="xeroton-rtl-warm", daemon=True
        )
        self._warmup_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.sim.disconnect()

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            t0 = time.perf_counter()
            with self._lock:
                if not self.paused and not self.stopped:
                    self._step_one_frame_locked()
                else:
                    self.latest_snapshot["paused"] = self.paused
                    self.latest_snapshot["stopped"] = self.stopped
                    self.latest_snapshot["loop_enabled"] = self.loop_enabled
                    self.latest_snapshot["speed"] = self.speed
                    self.latest_snapshot["pybullet_status"] = (
                        "STOPPED" if self.stopped else "PAUSED"
                    )
                    self.latest_snapshot["physics_status"] = (
                        "FAULT LOCKOUT" if self.fault_latched else self.latest_snapshot["pybullet_status"]
                    )

            target_dt = 0.020 / max(self.speed, 0.25)
            sleep_for = target_dt - (time.perf_counter() - t0)
            if sleep_for > 0.001:
                time.sleep(sleep_for)

    def get_snapshot(self) -> dict[str, Any]:
        with self._lock:
            snap = dict(self.latest_snapshot)
            if self.fault_latched:
                snap["fault_active"] = True
                snap["fault_code"] = self.fault_code or snap.get("fault_code", 1)
                snap["fault_reason"] = self.fault_reason
                snap["state"] = "FAULT"
                snap["control_priority"] = "P1_ESTOP"
            snap["fault_history"] = list(self.fault_history)[:10]
            return snap

    # ------------------------------------------------------------------
    # Priority 2 & 3: Centralized Manual Pilot Movement Command Interface
    # ------------------------------------------------------------------
    @staticmethod
    def _resolve_pilot_direction(vx: float, vy: float, wz: float) -> tuple[str, str]:
        """Map canonical (vx, vy, wz) command to (direction_label, rtl_action)."""
        deadzone = 0.05
        ax, ay, az = abs(vx), abs(vy), abs(wz)
        if ax < deadzone and ay < deadzone and az < deadzone:
            return "STOP", "STAND"
        # Pick dominant requested axis cleanly (prevents diagonal/conflicting overwrites)
        if ax >= ay and ax >= az:
            return ("FORWARD", "WALK_FORWARD") if vx > 0 else ("BACKWARD", "WALK_BACKWARD")
        if ay >= ax and ay >= az:
            return ("STRAFE_LEFT", "STRAFE_LEFT") if vy > 0 else ("STRAFE_RIGHT", "STRAFE_RIGHT")
        return ("TURN_LEFT", "TURN_LEFT") if wz > 0 else ("TURN_RIGHT", "TURN_RIGHT")

    def handle_pilot_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        """
        Single authoritative Manual Pilot endpoint (`POST /api/pilot`).
        Payload: { vx, vy, wz, hold, seq, immediate_stop, neutral_stand }
        """
        immediate_stop = bool(payload.get("immediate_stop", False))
        neutral_stand = bool(payload.get("neutral_stand", False))
        seq = int(payload.get("seq", -1))

        # Priority 1: Fault lockout blocks motion (STOP is still allowed to zero velocity)
        with self._lock:
            if self.fault_latched:
                self.pilot_hold_active = False
                self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                self.active_direction = "STOP"
                return {
                    "ok": False,
                    "error": f"Blocked by FAULT lockout ({self.fault_reason}). Clear fault first.",
                }

            # Reject stale out-of-order async requests (never reject STOP/STAND!)
            if not immediate_stop and not neutral_stand and seq >= 0 and seq < self._last_pilot_seq:
                return {"ok": True, "ignored_stale": True, "seq": seq}
            if seq >= 0:
                self._last_pilot_seq = seq

            # Priority 2: Immediate STOP (SPACE) or Neutral STAND (R)
            if immediate_stop or neutral_stand:
                self.paused = False
                self.stopped = False
                label = "STOP (SPACE) -> STAND" if immediate_stop else "NEUTRAL STAND (R)"
                self._clear_motion_to_stand_locked(label)
                return {
                    "ok": True,
                    "direction": "STOP",
                    "command": "STAND",
                    "cmd_vel": dict(self.cmd_vel),
                    "priority": self.control_priority,
                }

        vx = max(-1.5, min(1.5, float(payload.get("vx", 0.0))))
        vy = max(-1.5, min(1.5, float(payload.get("vy", 0.0))))
        wz = max(-1.5, min(1.5, float(payload.get("wz", 0.0))))
        hold = bool(payload.get("hold", True))

        direction, rtl_action = self._resolve_pilot_direction(vx, vy, wz)

        if direction == "STOP":
            with self._lock:
                self._clear_motion_to_stand_locked("MANUAL PILOT: STAND")
                return {
                    "ok": True,
                    "direction": "STOP",
                    "command": "STAND",
                    "cmd_vel": dict(self.cmd_vel),
                    "priority": self.control_priority,
                }

        # Check real RTL toolchain availability
        if not self.toolchain["connected"]:
            self.toolchain = check_rtl_toolchain(self.project_root)
            if not self.toolchain["connected"]:
                with self._lock:
                    self._clear_motion_to_stand_locked("RTL DISCONNECTED")
                return {"ok": False, "error": "Verilog RTL simulator DISCONNECTED."}

        # Fast-path heartbeat: if ALREADY executing this exact direction, keep phase seamless!
        with self._lock:
            if (
                self.control_priority == "P3_MANUAL_PILOT"
                and self.active_direction == direction
                and self.active_command == rtl_action
                and not self.paused
                and not self.stopped
            ):
                self.cmd_vel = {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}
                self.pilot_hold_active = hold
                self._pilot_last_heartbeat = time.monotonic()
                return {
                    "ok": True,
                    "direction": direction,
                    "command": rtl_action,
                    "cmd_vel": dict(self.cmd_vel),
                    "priority": self.control_priority,
                    "heartbeat": True,
                }
            eff_stride = self._effective_directional_stride(rtl_action)
            eff_pf = self.phase_frames

        # Fetch genuine Icarus Verilog RTL cycle (outside engine lock so physics stays smooth!)
        try:
            frames, exec_ms = self._fetch_rtl_cycle(
                rtl_action, stride_us=eff_stride, phase_frames=eff_pf, num_frames=0
            )
        except Exception as exc:
            return {"ok": False, "error": f"Verilog RTL execution error: {exc}"}

        # Commit atomically under lock — re-check seq so a newer STOP/direction wasn't missed!
        with self._lock:
            if self.fault_latched:
                return {"ok": False, "error": "Blocked by FAULT lockout."}
            if seq >= 0 and seq < self._last_pilot_seq:
                return {"ok": True, "ignored_stale": True, "seq": seq}

            cmd_code = COMMAND_NAME_TO_CODE[rtl_action]
            self.control_source = "Icarus Verilog RTL"
            self.control_priority = "P3_MANUAL_PILOT"
            self.manual_frame = None
            self.queue = frames
            self.queue_idx = 0
            self.active_direction = direction
            self.active_command = rtl_action
            self.cmd_vel = {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}
            self.pilot_hold_active = hold
            self._pilot_last_heartbeat = time.monotonic()
            self.mode_label = f"PILOT: {direction}"
            self.source_badge = f"Icarus Verilog RTL (CMD {cmd_code})"
            self.rtl_exec_ms = exec_ms
            self.paused = False
            self.stopped = False
            self.gait_eval.reset(rtl_action, start_x=self.latest_snapshot["base_pos"][0])

        return {
            "ok": True,
            "direction": direction,
            "command": rtl_action,
            "command_code": COMMAND_NAME_TO_CODE[rtl_action],
            "cmd_vel": {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)},
            "priority": "P3_MANUAL_PILOT",
            "frames": len(frames),
            "rtl_exec_ms": round(exec_ms, 1),
        }

    # ------------------------------------------------------------------
    # Movement Settings & Presets (SLOW / NORMAL / FAST)
    # ------------------------------------------------------------------
    def handle_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        preset = str(payload.get("preset", "")).strip().upper()
        with self._lock:
            if preset in MOTION_PRESETS:
                cfg = MOTION_PRESETS[preset]
                self.preset_name = preset
                self.walk_speed = float(cfg["walk_speed"])
                self.turn_speed = float(cfg["turn_speed"])
                self.stride_us = int(cfg["stride_us"])
                self.phase_frames = int(cfg["phase_frames"])
                self.smoothing = float(cfg["smoothing"])
            else:
                self.preset_name = "CUSTOM"
                if "walk_speed" in payload:
                    self.walk_speed = max(0.30, min(1.50, float(payload["walk_speed"])))
                if "turn_speed" in payload:
                    self.turn_speed = max(0.30, min(1.50, float(payload["turn_speed"])))
                if "stride_us" in payload:
                    self.stride_us = max(35, min(105, int(payload["stride_us"])))
                if "phase_frames" in payload:
                    self.phase_frames = max(2, min(8, int(payload["phase_frames"])))
                if "smoothing" in payload:
                    self.smoothing = max(0.20, min(1.00, float(payload["smoothing"])))
            settings = self._motion_settings_dict()

        # Refresh real RTL cache for new settings asynchronously so next press is instant
        threading.Thread(
            target=self._warm_pilot_rtl_cache, name="xeroton-rtl-refresh", daemon=True
        ).start()
        return {"ok": True, "motion_settings": settings}

    # ------------------------------------------------------------------
    # Manual Individual Joint Control (Safety-Gated)
    # ------------------------------------------------------------------
    def handle_manual_joints(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Apply manual joint targets strictly through the safety limiter."""
        with self._lock:
            if self.fault_latched:
                return {
                    "ok": False,
                    "error": f"Blocked by FAULT lockout ({self.fault_reason}). Clear fault deliberately first.",
                }

            raw_pulses = payload.get("pulses_us")
            if not isinstance(raw_pulses, list) or len(raw_pulses) != 8:
                self._latch_fault(0b0010, "Invalid manual joint command payload (expected 8 values)")
                return {"ok": False, "error": "Invalid joint payload — FAULT latched."}

            clamped_pulses: list[float] = []
            clamped_flags: list[bool] = []
            for i, val in enumerate(raw_pulses):
                try:
                    fv = float(val)
                except (TypeError, ValueError):
                    self._latch_fault(0b0010, f"Non-numeric pulse on joint {i}")
                    return {"ok": False, "error": f"Non-numeric pulse on j{i} — FAULT latched."}
                lo, hi = JOINT_LIMITS_US[i]
                if fv < 900.0 or fv > 2100.0:
                    self._latch_fault(0b1000, f"Joint j{i} limit violation: {fv:.0f} us outside hard servo bounds")
                    return {"ok": False, "error": f"Hard limit violation on j{i} ({fv:.0f} us) — FAULT latched."}
                c_val = max(float(lo), min(float(hi), fv))
                clamped_flags.append(abs(c_val - fv) > 0.5)
                clamped_pulses.append(c_val)

            prev = self.latest_snapshot.get("state", "IDLE")
            self.pilot_hold_active = False
            self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
            self.active_direction = "STOP"
            self.active_command = "MANUAL_JOINTS"
            self.control_priority = "P3_MANUAL_JOINTS"
            self.control_source = "Manual Control (Safety-Gated)"
            self.mode_label = payload.get("reason", "MANUAL JOINT CONTROL")
            self.source_badge = "Manual Control (Safety-Gated)"
            self.paused = False
            self.stopped = False
            self.manual_frame = self._make_safe_frame(
                "MANUAL", 9, pulses_us=clamped_pulses, prev_state=prev
            )
            return {
                "ok": True,
                "control_source": self.control_source,
                "pulses_us": [int(round(v)) for v in clamped_pulses],
                "clamped": clamped_flags,
            }

    # ------------------------------------------------------------------
    # General Command / Transport / Safety Handler
    # ------------------------------------------------------------------
    def handle_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action", "")).strip().upper()

        # Priority 1: Always allow E-STOP immediately
        if action in ("FAULT", "ESTOP"):
            reason = str(payload.get("reason") or "Operator Emergency Stop")
            with self._lock:
                self._latch_fault(0b0001, reason)
                self.control_source = (
                    "Icarus Verilog RTL" if self.toolchain["connected"] else "DISCONNECTED"
                )
                self.mode_label = "EMERGENCY STOP"
                self.source_badge = "Icarus Verilog RTL (E-STOP)"
                self.paused = False
                self.stopped = False
            if self.toolchain["connected"]:
                try:
                    t0 = time.perf_counter()
                    with self._vvp_lock:
                        frames = run_live_verilog_command(
                            self.project_root, command_code=2, estop=True, num_frames=16,
                            out_csv_rel="simulation/live_estop.csv",
                        )
                    with self._lock:
                        self.queue = frames
                        self.queue_idx = 0
                        self.rtl_exec_ms = (time.perf_counter() - t0) * 1000.0
                except Exception:
                    pass
            return {"ok": True, "action": "FAULT", "fault_code": self.fault_code, "reason": reason}

        # Deliberate Fault Clear -> returns safely to stationary STAND
        if action == "CLEAR_FAULT":
            with self._lock:
                self.fault_latched = False
                self.fault_code = 0
                self.fault_reason = ""
                self.manual_frame = None
                self.pilot_hold_active = False
                self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                self.active_direction = "STOP"
                self.active_command = "IDLE"
                self.control_priority = "P2_STOP"
                self.control_source = (
                    "Icarus Verilog RTL" if self.toolchain["connected"] else "DISCONNECTED"
                )
                self.mode_label = "LIVE: IDLE"
                self.source_badge = "Icarus Verilog RTL (CMD 0)"
                idle_frame = self._make_safe_frame("IDLE", 0, prev_state="FAULT")
                self.queue = [idle_frame]
                self.queue_idx = 0
                self.sim.reset_pose(initial_angles=idle_frame.angles_rad)
                self.sim.sim_time_s = 0.0
                self.gait_eval.reset("IDLE")
                self.paused = False
                self.stopped = False
                self.latest_snapshot = self._build_snapshot(idle_frame)
            return {"ok": True, "cleared": True}

        # Simulation Transport Controls
        if action == "START":
            with self._lock:
                self.paused = False
                self.stopped = False
            return {"ok": True, "running": True}

        if action == "PAUSE":
            with self._lock:
                self.paused = True
            return {"ok": True, "paused": True}

        if action == "RESUME":
            with self._lock:
                self.paused = False
                self.stopped = False
            return {"ok": True, "paused": False}

        if action == "TOGGLE_PAUSE":
            with self._lock:
                self.paused = not self.paused
                val = self.paused
            return {"ok": True, "paused": val}

        if action == "SIM_STOP":
            with self._lock:
                self.pilot_hold_active = False
                self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                self.active_direction = "STOP"
                self.stopped = True
                self.paused = True
            return {"ok": True, "stopped": True}

        if action == "STEP":
            with self._lock:
                self.paused = True
                self.stopped = False
                self._step_one_frame_locked()
                snap = dict(self.latest_snapshot)
            return {"ok": True, "stepped_frame": snap["frame"], "state": snap["state"]}

        if action == "LOOP":
            with self._lock:
                self.loop_enabled = bool(payload.get("value", True))
                val = self.loop_enabled
            return {"ok": True, "loop_enabled": val}

        if action == "SPEED":
            with self._lock:
                self.speed = max(0.25, min(2.0, float(payload.get("value", 1.0))))
                val = self.speed
            return {"ok": True, "speed": val}

        if action == "RESET":
            with self._lock:
                self.reset_active = True
                self.paused = False
                self.stopped = False
                self._clear_motion_to_stand_locked("RESET -> STAND")
                stand_frame = self.queue[0]
                self.sim.reset_pose(initial_angles=stand_frame.angles_rad)
                for _ in range(10):
                    self.sim.step_idle()
                self.sim.sim_time_s = 0.0
                self.gait_eval.reset("STAND")
                self.latest_snapshot = self._build_snapshot(stand_frame)
                self.reset_active = False
            return {"ok": True, "reset": True}

        # Block motion commands while fault is latched!
        if self.fault_latched:
            return {
                "ok": False,
                "error": f"Controller locked in FAULT ({self.fault_reason}). Clear fault deliberately first.",
            }

        # Priority 4: Optional Autonomous Demo (explicit user request only)
        if action == "DEMO":
            try:
                frames = load_verilog_csv(self.demo_csv_path)
            except Exception as exc:
                with self._lock:
                    self._latch_fault(0b0100, f"RTL CSV load error: {exc}")
                return {"ok": False, "error": str(exc)}
            with self._lock:
                self.pilot_hold_active = False
                self.cmd_vel = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                self.active_direction = "DEMO"
                self.active_command = "DEMO"
                self.control_priority = "P4_DEMO"
                self.control_source = "Verilog Testbench CSV"
                self.manual_frame = None
                self.queue = frames
                self.queue_idx = 0
                self.mode_label = "VERILOG FULL DEMO"
                self.source_badge = "Verilog Testbench CSV"
                self.paused = False
                self.stopped = False
                self.rtl_exec_ms = 0.0
                self.sim.reset_pose(initial_angles=frames[0].angles_rad)
                self.gait_eval.reset("DEMO")
            return {"ok": True, "frames": len(frames), "mode": self.mode_label}

        if action in COMMAND_NAME_TO_CODE:
            self.toolchain = check_rtl_toolchain(self.project_root)
            if not self.toolchain["connected"]:
                with self._lock:
                    self._latch_fault(0b0100, "Verilog RTL simulator DISCONNECTED (iverilog/vvp missing)")
                return {"ok": False, "error": "Verilog simulator DISCONNECTED."}

            cmd_code = COMMAND_NAME_TO_CODE[action]
            stride = int(payload.get("stride_us", self.stride_us))
            pframes = int(payload.get("phase_frames", self.phase_frames))
            n_frames = int(payload.get("num_frames", 0))

            frames, exec_ms = self._fetch_rtl_cycle(
                action=action,
                stride_us=stride,
                phase_frames=pframes,
                num_frames=n_frames,
                force_refresh=(action == "INVALID" or "stride_us" in payload or "phase_frames" in payload),
            )

            dir_map = {
                "WALK": "FORWARD",
                "WALK_FORWARD": "FORWARD",
                "WALK_BACKWARD": "BACKWARD",
                "STRAFE_LEFT": "STRAFE_LEFT",
                "STRAFE_RIGHT": "STRAFE_RIGHT",
                "TURN_LEFT": "TURN_LEFT",
                "TURN_RIGHT": "TURN_RIGHT",
                "TROT": "FORWARD",
            }
            vel_map = {
                "FORWARD": {"vx": self.walk_speed, "vy": 0.0, "wz": 0.0},
                "BACKWARD": {"vx": -self.walk_speed, "vy": 0.0, "wz": 0.0},
                "STRAFE_LEFT": {"vx": 0.0, "vy": self.walk_speed, "wz": 0.0},
                "STRAFE_RIGHT": {"vx": 0.0, "vy": -self.walk_speed, "wz": 0.0},
                "TURN_LEFT": {"vx": 0.0, "vy": 0.0, "wz": self.turn_speed},
                "TURN_RIGHT": {"vx": 0.0, "vy": 0.0, "wz": -self.turn_speed},
            }
            active_dir = dir_map.get(action, "STOP")

            with self._lock:
                self.stride_us = max(35, min(105, stride))
                self.phase_frames = max(2, min(8, pframes))
                self.pilot_hold_active = False
                self.control_source = "Icarus Verilog RTL"
                self.control_priority = "P2_STOP" if action in ("STOP", "IDLE", "STAND") else "P3_MANUAL_PILOT"
                self.active_direction = active_dir
                self.active_command = action
                self.cmd_vel = dict(vel_map.get(active_dir, {"vx": 0.0, "vy": 0.0, "wz": 0.0}))
                self.manual_frame = None
                self.queue = frames
                self.queue_idx = 0
                self.mode_label = f"LIVE: {action}"
                self.source_badge = f"Icarus Verilog RTL (CMD {cmd_code})"
                self.rtl_exec_ms = exec_ms
                self.paused = False
                self.stopped = False
                self.gait_eval.reset(action, start_x=self.latest_snapshot["base_pos"][0])
                if action == "INVALID" or (frames and frames[0].fault_active):
                    self._latch_fault(0b0010, f"Verilog safety_controller trapped invalid command {cmd_code}")

            return {
                "ok": True,
                "action": action,
                "command_code": cmd_code,
                "frames": len(frames),
                "stride_us": self.stride_us,
                "phase_frames": self.phase_frames,
                "rtl_exec_ms": round(exec_ms, 1),
            }

        return {"ok": False, "error": f"Unknown action '{action}'"}

    def export_pwm_data(self, fmt: str = "csv") -> tuple[str, bytes]:
        with self._lock:
            rows = list(self.pwm_history)
        if fmt == "json":
            return "application/json", json.dumps(rows, indent=2).encode("utf-8")

        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow([
            "frame", "sim_time_s", "state", "control_source", "period_ms", "freq_hz",
            "FL_HIP_us", "FL_KNEE_us", "FR_HIP_us", "FR_KNEE_us",
            "RL_HIP_us", "RL_KNEE_us", "RR_HIP_us", "RR_KNEE_us",
        ])
        for r in rows:
            w.writerow([
                r["frame"], r["sim_time_s"], r["state"], r["control_source"],
                r["period_ms"], r["freq_hz"], *r["pulses_us"],
            ])
        return "text/csv", buf.getvalue().encode("utf-8")


def make_handler(engine: StudioEngine, web_dir: Path) -> type[BaseHTTPRequestHandler]:
    class StudioRequestHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            pass

        def _send_json(self, data: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(data).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            qs = parse_qs(parsed.query)

            if path == "/api/state":
                self._send_json(engine.get_snapshot())
                return

            if path == "/api/vcd":
                summary = parse_vcd_summary(engine.project_root / "simulation" / "xeroton.vcd")
                self._send_json(summary)
                return

            if path == "/api/diagnostics":
                self._send_json(engine.diagnostics.get_report())
                return

            if path == "/api/pwm_export":
                fmt = (qs.get("format", ["csv"])[0]).lower()
                mime, data = engine.export_pwm_data(fmt)
                ext = "json" if fmt == "json" else "csv"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Disposition", f'attachment; filename="xeroton_pwm_capture.{ext}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            if path == "/api/stream":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while True:
                        snap = engine.get_snapshot()
                        msg = f"data: {json.dumps(snap)}\n\n".encode("utf-8")
                        self.wfile.write(msg)
                        self.wfile.flush()
                        time.sleep(0.025)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return

            rel = "index.html" if path in ("/", "") else path.lstrip("/")
            file_path = (web_dir / rel).resolve()
            if not str(file_path).startswith(str(web_dir.resolve())) or not file_path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND, "Not found")
                return

            mime, _ = mimetypes.guess_type(str(file_path))
            data = file_path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mime or "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8")) if raw else {}
            except Exception:
                payload = {}

            try:
                if path == "/api/pilot":
                    res = engine.handle_pilot_command(payload)
                    self._send_json(res, status=200 if res.get("ok") else 400)
                elif path == "/api/settings":
                    res = engine.handle_settings(payload)
                    self._send_json(res, status=200 if res.get("ok") else 400)
                elif path == "/api/command":
                    res = engine.handle_command(payload)
                    self._send_json(res, status=200 if res.get("ok") else 400)
                elif path == "/api/manual_joints":
                    res = engine.handle_manual_joints(payload)
                    self._send_json(res, status=200 if res.get("ok") else 400)
                elif path == "/api/jarvis":
                    res = parse_and_execute_jarvis(str(payload.get("text", "")), engine)
                    self._send_json(res, status=200)
                elif path == "/api/diagnostics/run":
                    res = engine.diagnostics.run_all()
                    self._send_json(res, status=200)
                else:
                    self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, status=500)

    return StudioRequestHandler


def run_studio_server(
    project_root: Path,
    urdf_path: Path,
    demo_csv_path: Path,
    host: str = "127.0.0.1",
    port: int = 8765,
    open_browser: bool = True,
    playback_speed: float = 1.0,
) -> None:
    web_dir = project_root / "web"
    if not (web_dir / "index.html").exists():
        raise FileNotFoundError(f"Missing Studio UI at {web_dir / 'index.html'}")

    engine = StudioEngine(
        project_root=project_root,
        urdf_path=urdf_path,
        demo_csv_path=demo_csv_path,
        playback_speed=playback_speed,
    )
    engine.start()

    class StrictStudioServer(ThreadingHTTPServer):
        allow_reuse_address = False

    handler_cls = make_handler(engine, web_dir)
    try:
        server = StrictStudioServer((host, port), handler_cls)
    except OSError as exc:
        engine.stop()
        raise RuntimeError(
            f"Port {port} is already in use (likely an older XEROTON server window is still open). "
            f"Close the previous PowerShell window or launch with --port {port + 1}."
        ) from exc
    server.daemon_threads = True
    url = f"http://{host}:{port}"

    print("======================================================================")
    print(" XEROTON STUDIO 3.0 — Robotics Engineering Control Environment")
    print("======================================================================")
    print(f" Studio URL  : {url}")
    print(f" Verilog RTL : {engine.toolchain['status_label']} (iverilog + vvp)")
    print(" Control Mode: Manual Pilot (Hold-to-Move WASD + QE + SPACE + R)")
    print(" Physics     : PyBullet 250 Hz Rigid-Body Engine")
    print(" Press Ctrl+C in this terminal to stop the server.")
    print("======================================================================")

    if open_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down XEROTON Studio 3.0...")
    finally:
        server.server_close()
        engine.stop()
