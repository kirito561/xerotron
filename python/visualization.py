"""
3D HUD & Camera Visualization Helpers for XEROTON 2.0 in PyBullet GUI.

Displays a clean, unobtrusive 3D header above the robot (state, frame, phase)
while the companion OLED Control Deck displays the enlarged animated pixel eyes,
tactile command pad, and live 8-channel 50 Hz Verilog PWM pulse bars.
"""

from __future__ import annotations

import math

import pybullet as p

from robot_simulator import FrameTelemetry
from verilog_bridge import VerilogFrame

STATE_COLORS: dict[str, tuple[float, float, float]] = {
    "IDLE":  (0.45, 0.78, 0.98),
    "STAND": (0.20, 0.92, 0.52),
    "WALK":  (0.15, 0.85, 1.00),
    "STOP":  (1.00, 0.78, 0.22),
    "WAVE":  (0.22, 0.98, 0.75),
    "DANCE": (0.98, 0.42, 0.86),
    "FAULT": (1.00, 0.22, 0.22),
}


class SimulationHUD:
    """Renders compact live Verilog telemetry above the robot in the 3D viewport."""

    def __init__(self, client_id: int) -> None:
        self.client_id = client_id
        self._text_ids: dict[str, int] = {}

    def _upsert_text(
        self,
        key: str,
        text: str,
        position: list[float],
        color: tuple[float, float, float],
        size: float = 1.25,
    ) -> None:
        prev_id = self._text_ids.get(key)
        kwargs = {
            "text": text,
            "textPosition": position,
            "textColorRGB": list(color),
            "textSize": size,
            "physicsClientId": self.client_id,
        }
        if prev_id is not None:
            kwargs["replaceItemUniqueId"] = prev_id
        self._text_ids[key] = p.addUserDebugText(**kwargs)

    def update(
        self,
        frame: VerilogFrame,
        telemetry: FrameTelemetry,
        follow_camera: bool = True,
    ) -> None:
        """Refresh the 3D overlay text and smoothly track the robot chassis."""
        if not p.isConnected(self.client_id):
            return

        color = STATE_COLORS.get(frame.state_name, (1.0, 1.0, 1.0))
        bx, by, bz = telemetry.pos_x, telemetry.pos_y, telemetry.pos_z

        header = f"XEROTON | {frame.state_name} | Frame {frame.frame} | Phase {frame.gait_phase}"
        if frame.fault_active:
            header += f" | FAULT 0b{frame.fault_code:04b}"

        pulses = frame.pulses_us
        joints_str = (
            f"FL[{pulses[0]:.0f},{pulses[1]:.0f}] "
            f"FR[{pulses[2]:.0f},{pulses[3]:.0f}] "
            f"RL[{pulses[4]:.0f},{pulses[5]:.0f}] "
            f"RR[{pulses[6]:.0f},{pulses[7]:.0f}] us"
        )

        self._upsert_text("header", header, [bx - 0.12, by, bz + 0.30], color, size=1.35)
        self._upsert_text("joints", joints_str, [bx - 0.16, by, bz + 0.24], (0.18, 0.18, 0.20), size=1.0)

        if follow_camera:
            p.resetDebugVisualizerCamera(
                cameraDistance=0.82,
                cameraYaw=132.0,
                cameraPitch=-20.0,
                cameraTargetPosition=[bx + 0.04, by, 0.21],
                physicsClientId=self.client_id,
            )


def format_console_line(frame: VerilogFrame, telemetry: FrameTelemetry) -> str:
    """Format a compact console status row for terminal monitoring."""
    p_str = " ".join(f"j{i}={int(frame.pulses_us[i])}" for i in range(8))
    return (
        f"[Frame {frame.frame:03d}] {frame.state_name:5s} ph={frame.gait_phase} "
        f"fault={int(frame.fault_active)} | {p_str} | "
        f"xyz=({telemetry.pos_x:+.3f},{telemetry.pos_y:+.3f},{telemetry.pos_z:.3f}) "
        f"rp=({telemetry.roll_deg:+.1f},{telemetry.pitch_deg:+.1f})deg "
        f"feet={telemetry.feet_in_contact}/4 body_hit={int(telemetry.body_ground_contact)}"
    )
