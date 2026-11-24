"""
Main Entry Point for the XEROTON 2.0 Python + PyBullet Simulation.

Reads actual Verilog-generated joint target data from simulation/joint_targets.csv
(and in interactive GUI mode invokes the real compiled `xeroton_live.vvp` Verilog
RTL runner on every button click) to drive the 8-DOF PyBullet robot dog model.

Usage examples:
  python python/main.py                        # 3D PyBullet + Interactive OLED Control Deck
  python python/main.py --demo wave --hold     # Replay only WAVE frames from Verilog CSV
  python python/main.py --demo dance --hold    # Replay only DANCE frames from Verilog CSV
  python python/main.py --headless             # Headless physics verification & metrics export
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from robot_simulator import FrameTelemetry, XerotonSimulator
from verilog_bridge import (
    VerilogFrame,
    filter_frames_by_state,
    load_verilog_csv,
    summarize_verilog_frames,
)
from visualization import SimulationHUD, format_console_line


def compute_physics_summary(history: list[FrameTelemetry]) -> dict:
    """Compute objective per-state and overall physics stability metrics."""
    if not history:
        return {"total_frames": 0}

    by_state: dict[str, list[FrameTelemetry]] = {}
    for row in history:
        by_state.setdefault(row.state_name, []).append(row)

    state_stats: dict[str, dict] = {}
    for st, rows in by_state.items():
        dx = 0.0
        dy = 0.0
        for prev_r, curr_r in zip(rows, rows[1:]):
            if curr_r.frame == prev_r.frame + 1:
                dx += curr_r.pos_x - prev_r.pos_x
                dy += curr_r.pos_y - prev_r.pos_y
        avg_z = sum(r.pos_z for r in rows) / len(rows)
        min_z = min(r.pos_z for r in rows)
        max_roll = max(abs(r.roll_deg) for r in rows)
        max_pitch = max(abs(r.pitch_deg) for r in rows)
        avg_feet = sum(r.feet_in_contact for r in rows) / len(rows)
        body_hits = sum(1 for r in rows if r.body_ground_contact)
        max_jerr = max(r.max_joint_error_rad for r in rows)
        state_stats[st] = {
            "frames": len(rows),
            "delta_x_m": round(dx, 4),
            "delta_y_m": round(dy, 4),
            "avg_height_m": round(avg_z, 4),
            "min_height_m": round(min_z, 4),
            "max_abs_roll_deg": round(max_roll, 2),
            "max_abs_pitch_deg": round(max_pitch, 2),
            "avg_feet_in_contact": round(avg_feet, 2),
            "body_ground_contact_frames": body_hits,
            "max_joint_tracking_err_rad": round(max_jerr, 4),
            "upright": (max_roll < 35.0 and max_pitch < 35.0 and body_hits == 0),
        }

    return {
        "total_frames": len(history),
        "start_xyz": [
            round(history[0].pos_x, 4),
            round(history[0].pos_y, 4),
            round(history[0].pos_z, 4),
        ],
        "end_xyz": [
            round(history[-1].pos_x, 4),
            round(history[-1].pos_y, 4),
            round(history[-1].pos_z, 4),
        ],
        "net_forward_x_m": round(history[-1].pos_x - history[0].pos_x, 4),
        "overall_max_abs_roll_deg": round(max(abs(r.roll_deg) for r in history), 2),
        "overall_max_abs_pitch_deg": round(max(abs(r.pitch_deg) for r in history), 2),
        "total_body_ground_contacts": sum(1 for r in history if r.body_ground_contact),
        "per_state": state_stats,
    }


def save_telemetry_csv(history: list[FrameTelemetry], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "frame", "state_name", "gait_phase", "fault_active",
            "pos_x", "pos_y", "pos_z", "roll_deg", "pitch_deg", "yaw_deg",
            "feet_in_contact", "body_ground_contact", "max_joint_error_rad",
        ])
        for r in history:
            writer.writerow([
                r.frame, r.state_name, r.gait_phase, int(r.fault_active),
                f"{r.pos_x:.5f}", f"{r.pos_y:.5f}", f"{r.pos_z:.5f}",
                f"{r.roll_deg:.2f}", f"{r.pitch_deg:.2f}", f"{r.yaw_deg:.2f}",
                r.feet_in_contact, int(r.body_ground_contact), f"{r.max_joint_error_rad:.5f}",
            ])


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="XEROTON 2.0 8-DOF Quadruped Simulation (driven by Verilog RTL)."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=PROJECT_ROOT / "simulation" / "joint_targets.csv",
        help="Path to Verilog-generated joint_targets.csv",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=PROJECT_ROOT / "robot" / "xeroton.urdf",
        help="Path to XEROTON URDF file",
    )
    parser.add_argument(
        "--demo",
        choices=["all", "idle", "stand", "walk", "trot", "stop", "wave", "dance", "sit", "fault"],
        default="all",
        help="Replay all Verilog frames or filter to a specific FSM state",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run PyBullet in DIRECT mode (no GUI) for automated verification",
    )
    parser.add_argument(
        "--pybullet-gui",
        action="store_true",
        help="Use legacy PyBullet OpenGL GUI + Tkinter Control Deck instead of WebGL Studio 3.0",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="HTTP port for XEROTON Studio 3.0 WebGL interface (default: 8765)",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not auto-open the web browser when starting Studio 3.0",
    )
    parser.add_argument(
        "--no-deck",
        action="store_true",
        help="Disable the Tkinter Control Deck when using --pybullet-gui",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="GUI playback speed multiplier (default: 1.0 = real time)",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Keep GUI window open after playback completes until manually closed",
    )
    parser.add_argument(
        "--metrics-out",
        type=Path,
        default=PROJECT_ROOT / "simulation" / "physics_metrics.json",
        help="Where to write JSON physics metrics summary",
    )
    parser.add_argument(
        "--telemetry-out",
        type=Path,
        default=PROJECT_ROOT / "simulation" / "physics_telemetry.csv",
        help="Where to write frame-by-frame PyBullet telemetry CSV",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-frame console output",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    print("======================================================================")
    print(" XEROTON 2.0 Python + PyBullet Simulation (Verilog-Driven)")
    print("======================================================================")
    print(f" Verilog CSV : {args.csv}")
    print(f" Robot URDF  : {args.urdf}")
    print(f" Demo Mode   : {args.demo.upper()} | Graphics: {'DIRECT' if args.headless else 'GUI'}")

    all_frames = load_verilog_csv(args.csv)
    counts = summarize_verilog_frames(all_frames)
    print(f" Loaded {len(all_frames)} Verilog frames: {counts}")

    # Default interactive experience: XEROTON Studio 3.0 (PBR Three.js 3D WebGL + Real Verilog RTL + PyBullet)
    if not args.headless and not args.pybullet_gui and args.demo == "all":
        from studio_server import run_studio_server

        run_studio_server(
            project_root=PROJECT_ROOT,
            urdf_path=args.urdf,
            demo_csv_path=args.csv,
            port=args.port,
            open_browser=not args.no_browser,
            playback_speed=args.speed,
        )
        return 0

    sim = XerotonSimulator(urdf_path=args.urdf, gui=not args.headless)
    sim.connect()
    hud = SimulationHUD(sim.client_id) if (not args.headless and sim.client_id is not None) else None

    try:
        # Legacy PyBullet GUI + Tkinter Control Deck mode (--pybullet-gui)
        if not args.headless and args.pybullet_gui and not args.no_deck and args.demo == "all":
            from control_deck import XerotonControlDeck

            print(" Launching Interactive OLED Control Deck + 3D PyBullet Robot Dog...")
            print(" Click any command button (IDLE/STAND/WALK/WAVE/DANCE/STOP/E-STOP)")
            print(" to run REAL Verilog RTL and watch XEROTON respond live!")
            deck = XerotonControlDeck(
                project_root=PROJECT_ROOT,
                sim=sim,
                hud=hud,
                full_demo_csv=args.csv,
                playback_speed=args.speed,
            )
            deck.run()
            return 0

        frames = filter_frames_by_state(all_frames, args.demo)
        print(f" Replaying {len(frames)} frames ({len(frames) * 0.02:.2f} s simulated)...")

        def handle_frame(frame: VerilogFrame, telemetry: FrameTelemetry) -> None:
            if hud is not None:
                hud.update(frame, telemetry, follow_camera=True)
            if not args.quiet and (
                frame.frame == 1
                or frame.frame % 15 == 0
                or frame.fault_active
            ):
                print(format_console_line(frame, telemetry))

        history = sim.run_sequence(
            frames,
            real_time=not args.headless,
            playback_speed=args.speed,
            on_frame=handle_frame,
        )

        summary = compute_physics_summary(history)
        args.metrics_out.parent.mkdir(parents=True, exist_ok=True)
        args.metrics_out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        save_telemetry_csv(history, args.telemetry_out)

        print("======================================================================")
        print(" PHYSICS EVALUATION SUMMARY")
        print(json.dumps(summary, indent=2))
        print(f" Saved telemetry : {args.telemetry_out}")
        print(f" Saved metrics   : {args.metrics_out}")
        print("======================================================================")

        if not args.headless and args.hold:
            print("Playback complete. Close the PyBullet window to exit.")
            while sim.is_connected():
                sim.step_idle()
                time.sleep(1.0 / 250.0)
    finally:
        sim.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
