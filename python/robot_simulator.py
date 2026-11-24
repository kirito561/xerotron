"""
PyBullet 3D Physics Simulator for XEROTON 2.0 (Expressive 8-DOF Robot Dog).

Loads robot/xeroton.urdf, builds a warm studio workbench environment, renders
animated glowing OLED pixel eyes directly onto the robot's front screen,
maps the 8 revolute joints to Verilog channels j0..j7, and records physics
telemetry (position, orientation, foot ground contacts, chassis collisions,
and joint tracking error).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import pybullet as p
import pybullet_data

from verilog_bridge import JOINT_NAMES, VerilogFrame

# Exact 1:1 timing synchronization with 50 Hz (20 ms) Verilog PWM frames:
#   5 substeps * (1 / 250 s) = 5 * 4 ms = 20.0 ms per Verilog frame.
PHYSICS_HZ = 250
PHYSICS_DT = 1.0 / PHYSICS_HZ
SUBSTEPS_PER_FRAME = 5

FOOT_LINK_NAMES: tuple[str, ...] = ("fl_lower", "fr_lower", "rl_lower", "rr_lower")


@dataclass
class FrameTelemetry:
    """Physics measurements captured at the end of a single 20 ms Verilog frame."""

    frame: int
    state_name: str
    gait_phase: int
    fault_active: bool
    pos_x: float
    pos_y: float
    pos_z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    feet_in_contact: int
    body_ground_contact: bool
    max_joint_error_rad: float


class RobotOLED3DFace:
    """Draws live glowing pixel-eye expressions directly on the robot's 3D OLED screen."""

    def __init__(self, client_id: int, robot_id: int) -> None:
        self.client_id = client_id
        self.robot_id = robot_id
        self._line_ids: list[int] = []
        self._last_signature: tuple[str, int] | None = None

    def clear(self) -> None:
        if not p.isConnected(self.client_id):
            return
        for item_id in self._line_ids:
            p.removeUserDebugItem(item_id, physicsClientId=self.client_id)
        self._line_ids.clear()

    def _add_face_line(
        self,
        y1: float,
        z1: float,
        y2: float,
        z2: float,
        color: tuple[float, float, float],
        width: float = 5.0,
    ) -> None:
        # OLED glass surface sits at x = +0.214 in the robot's `base` local frame
        x = 0.216
        item_id = p.addUserDebugLine(
            [x, y1, z1],
            [x, y2, z2],
            lineColorRGB=list(color),
            lineWidth=width,
            parentObjectUniqueId=self.robot_id,
            parentLinkIndex=-1,
            physicsClientId=self.client_id,
        )
        self._line_ids.append(item_id)

    def _draw_rect_eye(
        self,
        cy: float,
        cz: float,
        half_w: float,
        half_h: float,
        color: tuple[float, float, float],
        width: float = 6.0,
    ) -> None:
        # Fill cute glowing rounded-block OLED eye with horizontal scanlines
        steps = 4
        for i in range(steps + 1):
            z = cz - half_h + (2.0 * half_h * i / steps)
            self._add_face_line(cy - half_w, z, cy + half_w, z, color, width=width)

    def update(self, state_name: str, tick: int) -> None:
        """Redraw 3D OLED eyes when expression or blink frame changes."""
        if not p.isConnected(self.client_id):
            return

        # Natural blink every ~35 frames during STAND/WALK
        blink_phase = 1 if (tick % 36 in (34, 35)) else 0
        dance_beat = (tick // 4) % 2
        sig = (state_name, blink_phase if state_name in ("STAND", "WALK") else dance_beat)
        if sig == self._last_signature:
            return
        self._last_signature = sig
        self.clear()

        z_mid = 0.010
        y_left = 0.036
        y_right = -0.036

        if state_name == "FAULT":
            # Bright red X-X alert eyes
            col = (1.0, 0.18, 0.20) if (dance_beat == 0) else (1.0, 0.55, 0.15)
            for cy in (y_left, y_right):
                self._add_face_line(cy - 0.018, z_mid - 0.016, cy + 0.018, z_mid + 0.016, col, 6.0)
                self._add_face_line(cy - 0.018, z_mid + 0.016, cy + 0.018, z_mid - 0.016, col, 6.0)
        elif state_name == "IDLE":
            # Sleepy horizontal eyes
            col = (0.35, 0.72, 0.95)
            for cy in (y_left, y_right):
                self._add_face_line(cy - 0.020, z_mid, cy + 0.020, z_mid, col, 6.0)
        elif state_name == "WAVE":
            # Joyful ^ ^ happy smiling eyes!
            col = (0.20, 0.98, 0.72)
            for cy in (y_left, y_right):
                self._add_face_line(cy - 0.020, z_mid - 0.008, cy, z_mid + 0.015, col, 6.5)
                self._add_face_line(cy, z_mid + 0.015, cy + 0.020, z_mid - 0.008, col, 6.5)
        elif state_name == "DANCE":
            # Sparkling neon eyes pulsing with the beat
            col = (0.98, 0.35, 0.85) if dance_beat == 0 else (0.15, 0.95, 1.0)
            for cy in (y_left, y_right):
                if dance_beat == 0:
                    self._add_face_line(cy - 0.018, z_mid - 0.008, cy, z_mid + 0.016, col, 6.5)
                    self._add_face_line(cy, z_mid + 0.016, cy + 0.018, z_mid - 0.008, col, 6.5)
                else:
                    self._draw_rect_eye(cy, z_mid + 0.004, 0.017, 0.017, col, 6.0)
        elif state_name == "STOP":
            # Calm amber-gold steady eyes
            col = (1.0, 0.78, 0.20)
            for cy in (y_left, y_right):
                self._draw_rect_eye(cy, z_mid, 0.017, 0.010, col, 6.0)
        elif blink_phase == 1:
            # Quick cute blink line
            col = (0.18, 0.92, 1.0)
            for cy in (y_left, y_right):
                self._add_face_line(cy - 0.019, z_mid, cy + 0.019, z_mid, col, 5.5)
        else:
            # Bright glowing cyan eyes (STAND / WALK)
            col = (0.15, 0.92, 1.0) if state_name == "STAND" else (0.25, 0.80, 1.0)
            h = 0.017 if state_name == "STAND" else 0.013
            for cy in (y_left, y_right):
                self._draw_rect_eye(cy, z_mid, 0.017, h, col, 6.0)


class XerotonSimulator:
    """Manages the PyBullet physics world and XEROTON URDF model."""

    def __init__(
        self,
        urdf_path: str | Path,
        gui: bool = True,
        spawn_height: float = 0.192,
        motor_force: float = 12.0,
        position_gain: float = 0.24,
        velocity_gain: float = 1.0,
        max_velocity: float = 5.2,
    ) -> None:
        self.urdf_path = Path(urdf_path)
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {self.urdf_path}")

        self.gui = gui
        self.spawn_height = spawn_height
        self.motor_force = motor_force
        self.position_gain = position_gain
        self.velocity_gain = velocity_gain
        self.max_velocity = max_velocity

        self.client_id: Optional[int] = None
        self.plane_id: Optional[int] = None
        self.robot_id: Optional[int] = None
        self.face_3d: Optional[RobotOLED3DFace] = None
        self.joint_indices: dict[str, int] = {}
        self.foot_link_indices: dict[str, int] = {}
        self._tick_counter: int = 0
        self.sim_time_s: float = 0.0
        self.physics_error_count: int = 0
        self.last_physics_error: str = ""
        self._smoothed_angles: Optional[list[float]] = None

    def connect(self) -> None:
        """Start PyBullet, configure gravity & timestep, and load studio scene + URDF."""
        mode = p.GUI if self.gui else p.DIRECT
        self.client_id = p.connect(mode)
        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet physics server.")

        p.resetSimulation(physicsClientId=self.client_id)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client_id)
        p.setTimeStep(PHYSICS_DT, physicsClientId=self.client_id)

        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        p.changeDynamics(
            self.plane_id,
            -1,
            lateralFriction=0.90,
            spinningFriction=0.02,
            rollingFriction=0.015,
            restitution=0.0,
            physicsClientId=self.client_id,
        )

        if self.gui:
            self._build_workbench_props()

        flags = p.URDF_USE_SELF_COLLISION
        self.robot_id = p.loadURDF(
            str(self.urdf_path),
            basePosition=[0.0, 0.0, self.spawn_height],
            baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            flags=flags,
            physicsClientId=self.client_id,
        )

        self._map_joints_and_configure_dynamics()

        if self.gui and self.robot_id is not None:
            self.face_3d = RobotOLED3DFace(self.client_id, self.robot_id)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            # Warm front-3/4 camera angle so OLED face, ears, and waving paw are front & center
            p.resetDebugVisualizerCamera(
                cameraDistance=0.82,
                cameraYaw=132.0,
                cameraPitch=-20.0,
                cameraTargetPosition=[0.04, 0.0, 0.21],
                physicsClientId=self.client_id,
            )

    def _build_workbench_props(self) -> None:
        """Add subtle warm desk coloration and a decorative 3D-printer filament spool prop."""
        assert self.client_id is not None and self.plane_id is not None
        # Warm studio oak-desk tint on the ground plane
        p.changeVisualShape(
            self.plane_id, -1, rgbaColor=[0.84, 0.75, 0.64, 1.0],
            physicsClientId=self.client_id
        )
        # Subtle runway distance markers (every 0.25 m along +X)
        for step_i in range(0, 12):
            x = step_i * 0.25
            col = [0.68, 0.56, 0.44] if step_i % 2 == 0 else [0.76, 0.66, 0.55]
            p.addUserDebugLine(
                [x, -0.55, 0.002], [x, 0.55, 0.002], col, lineWidth=1.5,
                physicsClientId=self.client_id
            )

        # Decorative orange 3D-printer filament spool sitting off to the side on the desk
        spool_x, spool_y = 0.55, -0.52
        flange_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.11, length=0.012, rgbaColor=[0.14, 0.14, 0.16, 1.0],
            physicsClientId=self.client_id
        )
        fil_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.095, length=0.075, rgbaColor=[0.96, 0.48, 0.12, 1.0],
            physicsClientId=self.client_id
        )
        p.createMultiBody(0, -1, flange_vis, [spool_x, spool_y, 0.006], physicsClientId=self.client_id)
        p.createMultiBody(0, -1, fil_vis, [spool_x, spool_y, 0.050], physicsClientId=self.client_id)
        p.createMultiBody(0, -1, flange_vis, [spool_x, spool_y, 0.094], physicsClientId=self.client_id)

    def _map_joints_and_configure_dynamics(self) -> None:
        assert self.client_id is not None and self.robot_id is not None
        self.joint_indices.clear()
        self.foot_link_indices.clear()

        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for idx in range(num_joints):
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joint_indices[joint_name] = idx
            if link_name in FOOT_LINK_NAMES:
                self.foot_link_indices[link_name] = idx
                p.changeDynamics(
                    self.robot_id,
                    idx,
                    lateralFriction=1.05,
                    spinningFriction=0.02,
                    rollingFriction=0.015,
                    restitution=0.0,
                    physicsClientId=self.client_id,
                )

        missing = [name for name in JOINT_NAMES if name not in self.joint_indices]
        if missing:
            raise RuntimeError(
                f"URDF '{self.urdf_path}' is missing expected joints: {missing}"
            )

    def reset_pose(self, initial_angles: Optional[Sequence[float]] = None) -> None:
        """Reset robot base and all 8 joints cleanly before running a demo."""
        assert self.client_id is not None and self.robot_id is not None
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [0.0, 0.0, self.spawn_height],
            p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            physicsClientId=self.client_id,
        )
        p.resetBaseVelocity(
            self.robot_id,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            physicsClientId=self.client_id,
        )
        self._smoothed_angles = [
            float(initial_angles[i]) if initial_angles is not None else 0.0
            for i in range(len(JOINT_NAMES))
        ]
        for i, name in enumerate(JOINT_NAMES):
            angle = self._smoothed_angles[i]
            idx = self.joint_indices[name]
            p.resetJointState(
                self.robot_id, idx, targetValue=angle, targetVelocity=0.0,
                physicsClientId=self.client_id
            )
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.motor_force,
                positionGain=self.position_gain,
                velocityGain=self.velocity_gain,
                maxVelocity=self.max_velocity,
                physicsClientId=self.client_id,
            )

    def apply_verilog_frame(
        self,
        frame: VerilogFrame,
        real_time: bool = False,
        playback_speed: float = 1.0,
        smoothing_alpha: float = 1.0,
        strafe_target_vy: float = 0.0,
    ) -> FrameTelemetry:
        """Apply all 8 Verilog joint targets synchronously and step 20 ms of physics."""
        assert self.client_id is not None and self.robot_id is not None

        alpha = max(0.15, min(1.0, float(smoothing_alpha)))
        if frame.fault_active or self._smoothed_angles is None or alpha >= 0.999:
            self._smoothed_angles = [float(a) for a in frame.angles_rad]
        else:
            self._smoothed_angles = [
                prev + alpha * (float(tgt) - prev)
                for prev, tgt in zip(self._smoothed_angles, frame.angles_rad)
            ]

        ordered_indices = [self.joint_indices[name] for name in JOINT_NAMES]
        for idx, angle in zip(ordered_indices, self._smoothed_angles):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.motor_force,
                positionGain=self.position_gain,
                velocityGain=self.velocity_gain,
                maxVelocity=self.max_velocity,
                physicsClientId=self.client_id,
            )

        is_strafe_push = (
            not frame.fault_active
            and abs(strafe_target_vy) > 1e-4
            and frame.state_name in ("STRAFE_LEFT", "STRAFE_RIGHT")
            and frame.gait_phase in (0, 1, 3, 4, 6)
        )

        sleep_dt = (PHYSICS_DT / max(playback_speed, 0.1)) if (real_time and self.gui) else 0.0
        for _ in range(SUBSTEPS_PER_FRAME):
            if is_strafe_push and self.plane_id is not None:
                # 8-DOF pitch-only URDF lacks hip abduction (X-axis) servos; apply
                # contact-gated stance-foot lateral reaction strictly during active
                # Verilog STRAFE stepping phases when >=2 feet are physically grounded.
                contacts = p.getContactPoints(
                    bodyA=self.robot_id, bodyB=self.plane_id, physicsClientId=self.client_id
                )
                grounded = {c[3] for c in contacts}
                stance = [l_idx for l_idx in self.foot_link_indices.values() if l_idx in grounded]
                if len(stance) >= 2:
                    pos, quat = p.getBasePositionAndOrientation(
                        self.robot_id, physicsClientId=self.client_id
                    )
                    lin_vel, _ = p.getBaseVelocity(
                        self.robot_id, physicsClientId=self.client_id
                    )
                    rot = p.getMatrixFromQuaternion(quat)
                    by_x, by_y = rot[1], rot[4]
                    cur_vy = lin_vel[0] * by_x + lin_vel[1] * by_y
                    f_lat = max(-30.0, min(30.0, (strafe_target_vy - cur_vy) * 180.0))
                    p.applyExternalForce(
                        self.robot_id,
                        -1,
                        [by_x * f_lat, by_y * f_lat, 0.0],
                        [pos[0], pos[1], 0.04],
                        p.WORLD_FRAME,
                        physicsClientId=self.client_id,
                    )
            p.stepSimulation(physicsClientId=self.client_id)
            self.sim_time_s += PHYSICS_DT
            if sleep_dt > 0.0:
                time.sleep(sleep_dt)

        self._tick_counter += 1
        if self.face_3d is not None:
            self.face_3d.update(frame.state_name, self._tick_counter)

        return self.measure_telemetry(frame)

    def measure_telemetry(self, frame: VerilogFrame) -> FrameTelemetry:
        """Read robot pose, contacts, and joint tracking error from PyBullet."""
        assert self.client_id is not None and self.robot_id is not None
        assert self.plane_id is not None

        pos, quat = p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.client_id
        )
        roll, pitch, yaw = p.getEulerFromQuaternion(quat)

        contacts = p.getContactPoints(
            bodyA=self.robot_id, bodyB=self.plane_id, physicsClientId=self.client_id
        )
        contact_links = {c[3] for c in contacts}
        feet_in_contact = sum(
            1 for idx in self.foot_link_indices.values() if idx in contact_links
        )
        body_ground_contact = (-1 in contact_links)

        joint_states = p.getJointStates(
            self.robot_id,
            [self.joint_indices[name] for name in JOINT_NAMES],
            physicsClientId=self.client_id,
        )
        max_err = max(
            abs(joint_states[i][0] - frame.angles_rad[i]) for i in range(8)
        )

        return FrameTelemetry(
            frame=frame.frame,
            state_name=frame.state_name,
            gait_phase=frame.gait_phase,
            fault_active=frame.fault_active,
            pos_x=pos[0],
            pos_y=pos[1],
            pos_z=pos[2],
            roll_deg=math.degrees(roll),
            pitch_deg=math.degrees(pitch),
            yaw_deg=math.degrees(yaw),
            feet_in_contact=feet_in_contact,
            body_ground_contact=body_ground_contact,
            max_joint_error_rad=max_err,
        )

    def get_studio_snapshot(self, frame: VerilogFrame) -> dict:
        """Return full 6-DOF pose, simulated IMU velocities, 8 joint states, foot contacts & penetration."""
        assert self.client_id is not None and self.robot_id is not None
        assert self.plane_id is not None

        pos, quat = p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.client_id
        )
        lin_vel, ang_vel = p.getBaseVelocity(
            self.robot_id, physicsClientId=self.client_id
        )
        roll, pitch, yaw = p.getEulerFromQuaternion(quat)

        contacts = p.getContactPoints(
            bodyA=self.robot_id, bodyB=self.plane_id, physicsClientId=self.client_id
        )
        contact_links = {c[3] for c in contacts}
        foot_contacts = [
            self.foot_link_indices.get(name, -99) in contact_links
            for name in FOOT_LINK_NAMES
        ]
        body_hit = (-1 in contact_links)

        # Ground penetration depth (positive meters when contact distance < 0)
        max_pen = 0.0
        for c in contacts:
            dist = float(c[8])
            if dist < 0.0 and (-dist) > max_pen:
                max_pen = -dist

        # Horizontal foot slip velocity (m/s) averaged across grounded feet
        slip_speeds = []
        for idx_i, link_name in enumerate(FOOT_LINK_NAMES):
            if foot_contacts[idx_i]:
                l_idx = self.foot_link_indices[link_name]
                ls = p.getLinkState(
                    self.robot_id, l_idx, computeLinkVelocity=1, physicsClientId=self.client_id
                )
                lv = ls[6]
                slip_speeds.append(math.hypot(float(lv[0]), float(lv[1])))
        foot_slip = (sum(slip_speeds) / len(slip_speeds)) if slip_speeds else 0.0

        ordered_indices = [self.joint_indices[name] for name in JOINT_NAMES]
        joint_states = p.getJointStates(
            self.robot_id, ordered_indices, physicsClientId=self.client_id
        )
        actual_angles = [round(float(js[0]), 5) for js in joint_states]
        joint_vels = [round(float(js[1]), 4) for js in joint_states]

        # Detect NaN/Inf physics anomalies honestly
        if any(math.isnan(v) or math.isinf(v) for v in (*pos, *lin_vel, *actual_angles)):
            self.physics_error_count += 1
            self.last_physics_error = "Non-finite value detected in PyBullet solver state"

        return {
            "frame": frame.frame,
            "time_ms": round(frame.time_ms, 2),
            "sim_time_s": round(self.sim_time_s, 3),
            "command": frame.command,
            "state": frame.state_name,
            "prev_state": frame.prev_state_name,
            "gait_phase": frame.gait_phase,
            "fault_active": frame.fault_active,
            "fault_code": frame.fault_code,
            "pulses_us": [int(round(v)) for v in frame.pulses_us],
            "target_angles_rad": [round(float(a), 5) for a in frame.angles_rad],
            "actual_angles_rad": actual_angles,
            "joint_velocities_rad_s": joint_vels,
            "base_pos": [round(float(pos[0]), 5), round(float(pos[1]), 5), round(float(pos[2]), 5)],
            "base_quat": [
                round(float(quat[0]), 5),
                round(float(quat[1]), 5),
                round(float(quat[2]), 5),
                round(float(quat[3]), 5),
            ],
            "base_lin_vel": [round(float(v), 4) for v in lin_vel],
            "base_ang_vel": [round(float(v), 4) for v in ang_vel],
            "rpy_deg": [
                round(math.degrees(roll), 2),
                round(math.degrees(pitch), 2),
                round(math.degrees(yaw), 2),
            ],
            "foot_contacts": foot_contacts,
            "body_hit": body_hit,
            "max_penetration_m": round(max_pen, 5),
            "foot_slip_mps": round(foot_slip, 4),
            "physics_error_count": self.physics_error_count,
            "last_physics_error": self.last_physics_error,
            "imu_source": "SIMULATED_IMU_ESTIMATE (No physical MPU6050 connected)",
        }

    def run_sequence(
        self,
        frames: Sequence[VerilogFrame],
        real_time: bool = False,
        playback_speed: float = 1.0,
        on_frame: Optional[Callable[[VerilogFrame, FrameTelemetry], None]] = None,
    ) -> list[FrameTelemetry]:
        """Replay a sequence of Verilog frames and collect telemetry for every frame."""
        if not frames:
            return []
        self.reset_pose(initial_angles=frames[0].angles_rad)
        assert self.client_id is not None
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.client_id)

        history: list[FrameTelemetry] = []
        for frame in frames:
            if self.gui and not p.isConnected(self.client_id):
                break
            telemetry = self.apply_verilog_frame(
                frame, real_time=real_time, playback_speed=playback_speed
            )
            history.append(telemetry)
            if on_frame is not None:
                on_frame(frame, telemetry)
        return history

    def is_connected(self) -> bool:
        return self.client_id is not None and bool(p.isConnected(self.client_id))

    def step_idle(self) -> None:
        if self.is_connected():
            assert self.client_id is not None
            p.stepSimulation(physicsClientId=self.client_id)

    def disconnect(self) -> None:
        if self.client_id is not None and p.isConnected(self.client_id):
            p.disconnect(physicsClientId=self.client_id)
        self.client_id = None
