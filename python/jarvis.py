"""
JARVIS Local Safe Text Command Parser for XEROTON Studio 3.0.

Explicitly operates in TEXT COMMAND MODE using a deterministic local rule parser.
Every recognized phrase maps strictly to an explicit supported controller, manual
safety-gated joint, UI navigation, gait lab, or diagnostics command.

SAFETY INVARIANTS:
  1. Emergency stop ALWAYS takes priority over every other command.
  2. JARVIS NEVER bypasses the safety controller or fault latch.
  3. While a fault is latched, motion commands are refused until deliberate reset.
"""

from __future__ import annotations

import re
from typing import Any


def parse_and_execute_jarvis(text: str, engine: Any) -> dict[str, Any]:
    """Parse a natural-language text command and route it safely through StudioEngine."""
    raw = (text or "").strip()
    if not raw:
        return {
            "ok": False,
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "EMPTY",
            "reply": "Enter a command such as 'Stand up', 'Start walking', 'Show PWM signals', or 'Explain the current fault'.",
        }

    q = raw.lower()
    snap = engine.get_snapshot()

    # 1. HIGHEST PRIORITY: Emergency Stop (always overrides everything!)
    if any(k in q for k in ("emergency", "e-stop", "estop", "kill", "abort")):
        res = engine.handle_command({"action": "FAULT", "reason": "JARVIS Emergency Stop command"})
        return {
            "ok": True,
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "ESTOP_OVERRIDE",
            "reply": "EMERGENCY STOP executed immediately via Verilog safety override. Controller locked in FAULT state until deliberate reset.",
            "result": res,
        }

    # 2. Deliberate Fault Reset / Clear
    if any(k in q for k in ("clear fault", "reset fault", "recover", "unlock fault")):
        res = engine.handle_command({"action": "CLEAR_FAULT"})
        return {
            "ok": bool(res.get("ok")),
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "CLEAR_FAULT",
            "reply": "Fault latch deliberately cleared. Controller returned safely to IDLE.",
            "result": res,
        }

    # 3. Diagnostic / Status / Fault Explanation (allowed even while faulted!)
    if any(k in q for k in ("explain", "fault", "why", "error", "status")):
        if snap.get("fault_active"):
            code = int(snap.get("fault_code", 0))
            reasons = []
            if code & 1:
                reasons.append("Bit 0: Emergency Stop asserted")
            if code & 2:
                reasons.append("Bit 1: Invalid Verilog command code (>13)")
            if code & 4:
                reasons.append("Bit 2: Watchdog communication timeout")
            if code & 8:
                reasons.append("Bit 3: Joint limit clamp triggered")
            rt_reason = snap.get("fault_reason") or "Hardware/Safety supervisor fault"
            detail = "; ".join(reasons) if reasons else rt_reason
            return {
                "ok": True,
                "mode": "TEXT_COMMAND_MODE",
                "input": raw,
                "intent": "EXPLAIN_FAULT",
                "reply": f"ACTIVE FAULT (code 0b{code:04b}): {rt_reason}. Register bits: {detail}. Use 'Clear fault' or the Fault Reset button to resume safely.",
            }
        return {
            "ok": True,
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "STATUS_REPORT",
            "reply": (
                f"No active fault. Current FSM state: {snap['state']} (prev: {snap.get('prev_state','IDLE')}), "
                f"control source: {snap.get('control_source','VERILOG_RTL')}, "
                f"pose X={snap['base_pos'][0]:+.2f}m Z={snap['base_pos'][2]:.2f}m, "
                f"roll={snap['rpy_deg'][0]:+.1f} deg, pitch={snap['rpy_deg'][1]:+.1f} deg."
            ),
        }

    # 4. UI Navigation Commands (safe in any state)
    nav_rules = [
        (("pwm", "waveform", "oscilloscope", "duty"), "pwm", "Opened the 8-Channel Real PWM Analyzer."),
        (("joint", "angle", "slider"), "joints", "Opened the 8-Joint Control Panel."),
        (("telemetry", "sensor", "imu", "graph"), "telemetry", "Opened Sensor Telemetry (Simulated IMU estimates & live graphs)."),
        (("verilog", "rtl", "fsm", "register"), "verilog", "Opened the Verilog RTL Control Centre."),
        (("architecture", "diagram", "roadmap"), "architecture", "Opened System Architecture."),
        (("diagnostic", "test suite", "run tests"), "testing", "Opened Testing & Diagnostics. Triggering automated verification..."),
        (("overview", "home", "dashboard"), "overview", "Switched to Overview."),
        (("simulation", "3d view", "camera"), "simulation", "Switched to 3D Robot Simulation."),
    ]
    if any(k in q for k in ("show", "display", "open", "view", "go to", "switch")) or "diagnostic" in q or "run tests" in q:
        for keywords, section, msg in nav_rules:
            if any(k in q for k in keywords):
                extra = {"navigate": section}
                if section == "testing" and ("run" in q or "test" in q):
                    extra["trigger_diagnostics"] = True
                return {
                    "ok": True,
                    "mode": "TEXT_COMMAND_MODE",
                    "input": raw,
                    "intent": f"UI_NAV_{section.upper()}",
                    "reply": msg,
                    **extra,
                }

    # 5. Gait Test Shortcut
    if "gait test" in q or "gait lab" in q:
        if snap.get("fault_active"):
            return {
                "ok": False,
                "mode": "TEXT_COMMAND_MODE",
                "input": raw,
                "intent": "BLOCKED_BY_FAULT",
                "reply": "Refused: Safety controller is in FAULT lockout. Clear the fault deliberately before running a gait test.",
            }
        res = engine.handle_command({"action": "WALK"})
        return {
            "ok": True,
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "RUN_GAIT_TEST",
            "navigate": "gait",
            "reply": "Opened Gait Laboratory and started real Verilog WALK execution with live physics stability scoring.",
            "result": res,
        }

    # 6. Fault Guard Before Any Motion Command
    if snap.get("fault_active"):
        return {
            "ok": False,
            "mode": "TEXT_COMMAND_MODE",
            "input": raw,
            "intent": "BLOCKED_BY_FAULT",
            "reply": (
                f"Safety interlock blocked '{raw}': controller is latched in FAULT "
                f"({snap.get('fault_reason', 'active fault')}). Clear fault deliberately first."
            ),
        }

    # 7. Individual Leg Manual Safe Commands (e.g., "Move front left leg")
    leg_map = {
        "front left": ("FL", 0, 1),
        "fl": ("FL", 0, 1),
        "front right": ("FR", 2, 3),
        "fr": ("FR", 2, 3),
        "rear left": ("RL", 4, 5),
        "rl": ("RL", 4, 5),
        "rear right": ("RR", 6, 7),
        "rr": ("RR", 6, 7),
    }
    if any(k in q for k in ("move", "lift", "raise", "test")) and any(l in q for l in leg_map):
        for phrase, (leg_id, hip_i, knee_i) in leg_map.items():
            if phrase in q:
                # Route strictly through the manual safety gate!
                pulses = list(snap["pulses_us"])
                pulses[hip_i] = 1160
                pulses[knee_i] = 1900
                res = engine.handle_manual_joints({"pulses_us": pulses, "reason": f"JARVIS move {leg_id} leg"})
                return {
                    "ok": bool(res.get("ok")),
                    "mode": "TEXT_COMMAND_MODE",
                    "input": raw,
                    "intent": f"MANUAL_MOVE_{leg_id}",
                    "navigate": "joints",
                    "reply": f"Moved {leg_id} leg (hip=1160 us, knee=1900 us) via Manual Safety Gate. Active source switched to MANUAL_SAFE.",
                    "result": res,
                }

    # 8. Explicit Verilog RTL Motion Commands
    rtl_rules = [
        (("turn left", "rotate left", "spin left"), "TURN_LEFT", "Sent CMD 12 (TURN_LEFT) to Verilog RTL. Differential CCW yaw turn."),
        (("turn right", "rotate right", "spin right"), "TURN_RIGHT", "Sent CMD 13 (TURN_RIGHT) to Verilog RTL. Differential CW yaw turn."),
        (("strafe left", "sideways left", "step left"), "STRAFE_LEFT", "Sent CMD 10 (STRAFE_LEFT) to Verilog RTL. Lateral left weight-shift gait."),
        (("strafe right", "sideways right", "step right"), "STRAFE_RIGHT", "Sent CMD 11 (STRAFE_RIGHT) to Verilog RTL. Lateral right weight-shift gait."),
        (("backward", "reverse", "walk back"), "WALK_BACKWARD", "Sent CMD 9 (WALK_BACKWARD) to Verilog RTL. Reverse 8-phase walking gait."),
        (("stand", "get up", "ready"), "STAND", "Sent CMD 1 (STAND) to Verilog RTL. Robot moving to crouched standing stance."),
        (("trot", "run", "fast walk"), "TROT", "Sent CMD 7 (TROT) to Verilog RTL. Running brisk diagonal trot."),
        (("walk", "march", "forward"), "WALK_FORWARD", "Sent CMD 2 (WALK_FORWARD) to Verilog RTL. Executing 8-phase lift-first walking gait."),
        (("wave", "say hi", "hello", "greet"), "WAVE", "Sent CMD 4 (WAVE) to Verilog RTL. Balancing on 3 legs and waving front-right paw."),
        (("dance", "groove", "boogie"), "DANCE", "Sent CMD 5 (DANCE) to Verilog RTL. Executing bow & wiggle dance sequence."),
        (("sit", "sit down"), "SIT", "Sent CMD 6 (SIT) to Verilog RTL. Transitioning to rear-crouched sitting pose."),
        (("sleep", "nap", "goodnight"), "SLEEP", "Sent CMD 8 (SLEEP) to Verilog RTL. Curling into low resting sleep pose with closed OLED eyes."),
        (("stop", "halt", "freeze"), "STOP", "Sent CMD 3 (STOP) to Verilog RTL. Halting gait to safe standing pose."),
        (("idle", "rest", "neutral"), "IDLE", "Sent CMD 0 (IDLE) to Verilog RTL. Returning all 8 joints to 1500 us neutral."),
        (("demo", "replay"), "DEMO", "Loaded full verified Verilog testbench demonstration CSV."),
        (("reset",), "RESET", "Reset PyBullet robot base position cleanly."),
    ]
    for keywords, action, reply in rtl_rules:
        if any(k in q for k in keywords):
            res = engine.handle_command({"action": action})
            return {
                "ok": bool(res.get("ok")),
                "mode": "TEXT_COMMAND_MODE",
                "input": raw,
                "intent": f"RTL_{action}",
                "reply": reply if res.get("ok") else f"Command failed: {res.get('error')}",
                "result": res,
            }

    return {
        "ok": False,
        "mode": "TEXT_COMMAND_MODE",
        "input": raw,
        "intent": "UNRECOGNIZED",
        "reply": (
            f"Unrecognized command '{raw}'. Supported commands: Stand up, Start walking, Trot, Wave, Dance, Sit, "
            "Stop, Emergency stop, Clear fault, Move front left leg, Show PWM signals, Display joint angles, "
            "Run gait test, Explain the current fault."
        ),
    }
