"""
Interactive Sesame-Style OLED Control Deck for XEROTON 2.0.

Provides a sleek dark-mode companion dashboard next to the PyBullet 3D window:
  - Enlarged live OLED Pixel Face Display (animated blinking/happy/waving/dancing/alert eyes)
  - Phone-style tactile buttons that invoke REAL compiled Verilog RTL (`xeroton_live.vvp`)
    for IDLE, STAND, WALK, WAVE HI, DANCE, STOP, E-STOP, or Full Demo Replay
  - Live 8-Channel 50 Hz Verilog PWM Pulse Width Scope (1000..2000 us bars + degrees)
  - Live FSM State, Frame #, Gait Phase, Fault Code, and Physics Telemetry readout
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from pathlib import Path
from typing import Optional

from robot_simulator import FrameTelemetry, XerotonSimulator
from verilog_bridge import (
    COMMAND_NAME_TO_CODE,
    JOINT_NAMES,
    VerilogFrame,
    load_verilog_csv,
    run_live_verilog_command,
)
from visualization import SimulationHUD

BG_DARK = "#12151C"
CARD_BG = "#1B202C"
OLED_BG = "#04080E"
TEXT_MAIN = "#EEF2F7"
TEXT_DIM = "#8E99AD"

SHORT_JOINT_LABELS = (
    "j0 FL Hip",
    "j1 FL Knee",
    "j2 FR Hip",
    "j3 FR Knee",
    "j4 RL Hip",
    "j5 RL Knee",
    "j6 RR Hip",
    "j7 RR Knee",
)

STATE_HEX_COLOR = {
    "IDLE":  "#5EB6FF",
    "STAND": "#2CE582",
    "WALK":  "#1ED6FF",
    "STOP":  "#FFBE3B",
    "WAVE":  "#35F2B8",
    "DANCE": "#F96CE0",
    "FAULT": "#FF3B47",
}

STATE_EMOTION_LABEL = {
    "IDLE":  "SLEEPY / RESTING",
    "STAND": "ALERT & READY",
    "WALK":  "TROTTING FORWARD",
    "STOP":  "SAFE HOLD",
    "WAVE":  "SAYING HI!",
    "DANCE": "HAPPY DANCE GROOVE",
    "FAULT": "EMERGENCY LOCKOUT",
}


class XerotonControlDeck:
    """Tkinter interactive controller streaming live Verilog RTL outputs to PyBullet."""

    def __init__(
        self,
        project_root: Path,
        sim: XerotonSimulator,
        hud: Optional[SimulationHUD],
        full_demo_csv: Path,
        playback_speed: float = 1.0,
    ) -> None:
        self.project_root = project_root
        self.sim = sim
        self.hud = hud
        self.full_demo_csv = full_demo_csv
        self.playback_speed = max(0.25, playback_speed)

        self.queue: list[VerilogFrame] = []
        self.queue_index: int = 0
        self.active_mode: str = "DEMO"
        self.active_cmd_code: int = 1
        self.active_estop: bool = False
        self.loop_enabled: bool = True
        self.tick_count: int = 0
        self.closed: bool = False

        self.root = tk.Tk()
        self.root.title("XEROTON 2.0 — Verilog RTL OLED Control Deck")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("460x760+30+40")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

        # Start immediately with the verified full Verilog demonstration
        self._load_full_demo()

    def _build_ui(self) -> None:
        # Header Bar
        hdr = tk.Frame(self.root, bg=BG_DARK, pady=8, padx=14)
        hdr.pack(fill="x")
        tk.Label(
            hdr,
            text="XEROTON 8-DOF",
            font=("Segoe UI", 15, "bold"),
            fg=TEXT_MAIN,
            bg=BG_DARK,
        ).pack(side="left")
        self.source_badge = tk.Label(
            hdr,
            text="VERILOG RTL LIVE",
            font=("Consolas", 9, "bold"),
            fg="#091318",
            bg="#2CE582",
            padx=8,
            pady=3,
        )
        self.source_badge.pack(side="right")

        # OLED Pixel Face Canvas
        oled_frame = tk.Frame(self.root, bg="#2A3142", padx=3, pady=3)
        oled_frame.pack(padx=14, pady=(2, 8), fill="x")
        self.oled_canvas = tk.Canvas(
            oled_frame,
            width=424,
            height=148,
            bg=OLED_BG,
            highlightthickness=0,
        )
        self.oled_canvas.pack(fill="x")

        # Status & Emotion Banner
        status_card = tk.Frame(self.root, bg=CARD_BG, padx=12, pady=8)
        status_card.pack(padx=14, pady=(0, 8), fill="x")
        self.state_label = tk.Label(
            status_card,
            text="STATE: IDLE   |   SLEEPY / RESTING",
            font=("Segoe UI", 11, "bold"),
            fg="#5EB6FF",
            bg=CARD_BG,
            anchor="w",
        )
        self.state_label.pack(fill="x")
        self.meta_label = tk.Label(
            status_card,
            text="Frame: 000 | Phase: 0 | Fault: 0b0000 | Pos: (0.00, 0.00, 0.32)m",
            font=("Consolas", 9),
            fg=TEXT_DIM,
            bg=CARD_BG,
            anchor="w",
        )
        self.meta_label.pack(fill="x", pady=(3, 0))

        # Phone-Style Tactile Command Pad
        pad_card = tk.Frame(self.root, bg=CARD_BG, padx=10, pady=10)
        pad_card.pack(padx=14, pady=(0, 8), fill="x")

        tk.Label(
            pad_card,
            text="VERILOG RTL COMMAND PAD (Runs real vvp RTL on click)",
            font=("Segoe UI", 8, "bold"),
            fg=TEXT_DIM,
            bg=CARD_BG,
            anchor="w",
        ).pack(fill="x", pady=(0, 6))

        grid = tk.Frame(pad_card, bg=CARD_BG)
        grid.pack(fill="x")
        for col in range(3):
            grid.columnconfigure(col, weight=1, uniform="cmd")

        buttons = [
            ("IDLE", "IDLE", 0, False, "#2A3950", 0, 0),
            ("STAND", "STAND", 1, False, "#1D5E43", 0, 1),
            ("WALK", "WALK", 2, False, "#145D78", 0, 2),
            ("WAVE HI", "WAVE", 4, False, "#19695B", 1, 0),
            ("DANCE", "DANCE", 5, False, "#6E2863", 1, 1),
            ("STOP", "STOP", 3, False, "#7A5A1A", 1, 2),
        ]
        for label, name, code, estop, color, r, c in buttons:
            btn = tk.Button(
                grid,
                text=f"{label}\n(CMD {code})",
                font=("Segoe UI", 9, "bold"),
                fg="#FFFFFF",
                bg=color,
                activebackground="#3A4760",
                activeforeground="#FFFFFF",
                relief="flat",
                bd=0,
                pady=8,
                cursor="hand2",
                command=lambda n=name, cd=code, es=estop: self._trigger_live_verilog(n, cd, es),
            )
            btn.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")

        # Bottom Action Row: E-STOP, Full Demo Replay, Reset Pose
        row2 = tk.Frame(pad_card, bg=CARD_BG)
        row2.pack(fill="x", pady=(4, 0))
        tk.Button(
            row2,
            text="E-STOP / FAULT",
            font=("Segoe UI", 9, "bold"),
            fg="#FFFFFF",
            bg="#B5222C",
            activebackground="#D9323D",
            relief="flat",
            pady=7,
            cursor="hand2",
            command=lambda: self._trigger_live_verilog("FAULT", 2, True),
        ).pack(side="left", expand=True, fill="x", padx=(4, 4))

        tk.Button(
            row2,
            text="REPLAY FULL DEMO",
            font=("Segoe UI", 9, "bold"),
            fg="#FFFFFF",
            bg="#314266",
            activebackground="#445A8A",
            relief="flat",
            pady=7,
            cursor="hand2",
            command=self._load_full_demo,
        ).pack(side="left", expand=True, fill="x", padx=(4, 4))

        tk.Button(
            row2,
            text="RESET POS",
            font=("Segoe UI", 9, "bold"),
            fg="#FFFFFF",
            bg="#2D3342",
            activebackground="#444C63",
            relief="flat",
            pady=7,
            cursor="hand2",
            command=self._reset_robot_position,
        ).pack(side="left", expand=True, fill="x", padx=(4, 4))

        # Loop toggle + speed note
        opts = tk.Frame(pad_card, bg=CARD_BG)
        opts.pack(fill="x", pady=(6, 0))
        self.loop_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            opts,
            text="Loop active Verilog behavior continuously",
            variable=self.loop_var,
            fg=TEXT_DIM,
            bg=CARD_BG,
            selectcolor=BG_DARK,
            activebackground=CARD_BG,
            activeforeground=TEXT_MAIN,
            font=("Segoe UI", 8),
        ).pack(side="left")

        # Live 8-Channel 50 Hz PWM Scope Card
        pwm_card = tk.Frame(self.root, bg=CARD_BG, padx=12, pady=10)
        pwm_card.pack(padx=14, pady=(0, 10), fill="both", expand=True)
        tk.Label(
            pwm_card,
            text="LIVE 8-CHANNEL 50 Hz VERILOG PWM OUTPUTS (1000 - 2000 us)",
            font=("Segoe UI", 8, "bold"),
            fg=TEXT_DIM,
            bg=CARD_BG,
            anchor="w",
        ).pack(fill="x", pady=(0, 6))

        self.pwm_canvas = tk.Canvas(
            pwm_card,
            width=408,
            height=224,
            bg="#0F131C",
            highlightthickness=0,
        )
        self.pwm_canvas.pack(fill="both", expand=True)

        # Keyboard shortcuts matching the buttons
        self.root.bind("0", lambda _e: self._trigger_live_verilog("IDLE", 0, False))
        self.root.bind("1", lambda _e: self._trigger_live_verilog("STAND", 1, False))
        self.root.bind("2", lambda _e: self._trigger_live_verilog("WALK", 2, False))
        self.root.bind("3", lambda _e: self._trigger_live_verilog("STOP", 3, False))
        self.root.bind("4", lambda _e: self._trigger_live_verilog("WAVE", 4, False))
        self.root.bind("5", lambda _e: self._trigger_live_verilog("DANCE", 5, False))
        self.root.bind("e", lambda _e: self._trigger_live_verilog("FAULT", 2, True))
        self.root.bind("r", lambda _e: self._reset_robot_position())

    def _draw_oled_face(self, state_name: str) -> None:
        c = self.oled_canvas
        c.delete("all")
        w, h = 424, 148
        cy = h // 2 + 2
        lx, rx = 148, 276
        color = STATE_HEX_COLOR.get(state_name, "#1ED6FF")

        # Subtle scanline grid on OLED glass
        for y in range(6, h, 8):
            c.create_line(0, y, w, y, fill="#08121E")

        blink = (self.tick_count % 36) in (34, 35)
        beat = (self.tick_count // 4) % 2

        if state_name == "FAULT":
            col = "#FF3B47" if beat == 0 else "#FF8B2E"
            for cx in (lx, rx):
                c.create_line(cx - 28, cy - 26, cx + 28, cy + 26, fill=col, width=10, capstyle="round")
                c.create_line(cx - 28, cy + 26, cx + 28, cy - 26, fill=col, width=10, capstyle="round")
            c.create_text(w // 2, h - 16, text="FAULT OVERRIDE ACTIVE", fill=col, font=("Consolas", 10, "bold"))
        elif state_name == "IDLE":
            for cx in (lx, rx):
                c.create_line(cx - 32, cy, cx + 32, cy, fill=color, width=10, capstyle="round")
            # Cute floating Zzz
            z_off = (self.tick_count // 6) % 3
            c.create_text(345, 36 - z_off * 4, text="z", fill="#5EB6FF", font=("Consolas", 12, "bold"))
            c.create_text(365, 24 - z_off * 4, text="Z", fill="#5EB6FF", font=("Consolas", 15, "bold"))
        elif state_name == "WAVE":
            # Joyful ^ ^ curved smiling eyes + little mouth
            for cx in (lx, rx):
                c.create_arc(
                    cx - 32, cy - 18, cx + 32, cy + 36,
                    start=15, extent=150, style="arc", outline=color, width=10
                )
            c.create_arc(
                w // 2 - 20, cy + 16, w // 2 + 20, cy + 44,
                start=200, extent=140, style="arc", outline=color, width=5
            )
        elif state_name == "DANCE":
            col = "#F96CE0" if beat == 0 else "#1ED6FF"
            bounce = -6 if beat == 0 else 6
            for cx in (lx, rx):
                if beat == 0:
                    c.create_arc(
                        cx - 30, cy - 20 + bounce, cx + 30, cy + 32 + bounce,
                        start=15, extent=150, style="arc", outline=col, width=10
                    )
                else:
                    c.create_oval(cx - 26, cy - 26 + bounce, cx + 26, cy + 26 + bounce, fill=col, outline="")
                    c.create_oval(cx - 8, cy - 12 + bounce, cx + 4, cy + bounce, fill="#FFFFFF", outline="")
            c.create_arc(
                w // 2 - 22, cy + 14 + bounce, w // 2 + 22, cy + 42 + bounce,
                start=195, extent=150, style="arc", outline=col, width=5
            )
        elif state_name == "STOP":
            for cx in (lx, rx):
                c.create_rectangle(cx - 28, cy - 14, cx + 28, cy + 14, fill=color, outline="")
        elif blink:
            for cx in (lx, rx):
                c.create_line(cx - 30, cy, cx + 30, cy, fill=color, width=8, capstyle="round")
        else:
            # Cute glossy rounded eyes (STAND / WALK)
            eh = 28 if state_name == "STAND" else 22
            look_x = int(6 * math.sin(self.tick_count * 0.12)) if state_name == "STAND" else 8
            for cx in (lx, rx):
                c.create_oval(cx - 26, cy - eh, cx + 26, cy + eh, fill=color, outline="")
                c.create_oval(cx - 12 + look_x, cy - 12, cx + 2 + look_x, cy + 2, fill="#FFFFFF", outline="")

        # Small corner badge
        c.create_text(14, 14, text="OLED 128x64", fill="#294860", font=("Consolas", 8), anchor="nw")

    def _draw_pwm_scope(self, frame: VerilogFrame) -> None:
        c = self.pwm_canvas
        c.delete("all")
        row_h = 27
        bar_left = 96
        bar_right = 312
        bar_w = bar_right - bar_left

        # Center neutral line (1500 us)
        mid_x = bar_left + bar_w // 2
        c.create_line(mid_x, 4, mid_x, 220, fill="#2A3347", dash=(3, 3))

        accent = STATE_HEX_COLOR.get(frame.state_name, "#1ED6FF")
        for i in range(8):
            y0 = 6 + i * row_h
            ym = y0 + 10
            pulse = frame.pulses_us[i]
            deg = math.degrees(frame.angles_rad[i])
            frac = max(0.0, min(1.0, (pulse - 1000.0) / 1000.0))
            fill_x = bar_left + int(frac * bar_w)

            c.create_text(8, ym, text=SHORT_JOINT_LABELS[i], fill=TEXT_DIM, font=("Consolas", 9), anchor="w")
            c.create_rectangle(bar_left, y0 + 2, bar_right, y0 + 18, fill="#1B2230", outline="#283145")
            c.create_rectangle(bar_left, y0 + 2, fill_x, y0 + 18, fill=accent, outline="")
            c.create_text(
                bar_right + 8, ym,
                text=f"{int(pulse):4d}us {deg:+5.1f}d",
                fill=TEXT_MAIN,
                font=("Consolas", 8),
                anchor="w",
            )

    def _load_full_demo(self) -> None:
        try:
            self.queue = load_verilog_csv(self.full_demo_csv)
            self.queue_index = 0
            self.active_mode = "DEMO"
            self.sim.reset_pose(initial_angles=self.queue[0].angles_rad)
            self.source_badge.config(text="VERILOG CSV DEMO", bg="#5EB6FF")
        except Exception as exc:
            self.state_label.config(text=f"LOAD ERROR: {exc}", fg="#FF3B47")

    def _trigger_live_verilog(self, name: str, cmd_code: int, estop: bool) -> None:
        """Execute real Icarus Verilog RTL (`xeroton_live.vvp`) for the clicked command."""
        self.source_badge.config(text="RUNNING VERILOG...", bg="#FFBE3B")
        self.root.update_idletasks()
        try:
            n_frames = 64 if name in ("WALK", "WAVE", "DANCE") else 32
            frames = run_live_verilog_command(
                project_root=self.project_root,
                command_code=cmd_code,
                estop=estop,
                num_frames=n_frames,
            )
            self.queue = frames
            self.queue_index = 0
            self.active_mode = name
            self.active_cmd_code = cmd_code
            self.active_estop = estop
            self.source_badge.config(
                text=f"VERILOG LIVE (CMD {cmd_code})" if not estop else "VERILOG E-STOP!",
                bg="#FF3B47" if estop else "#2CE582",
            )
        except Exception as exc:
            self.source_badge.config(text="VERILOG ERROR", bg="#FF3B47")
            self.state_label.config(text=f"RTL ERROR: {exc}", fg="#FF3B47")

    def _reset_robot_position(self) -> None:
        if self.queue:
            idx = min(self.queue_index, len(self.queue) - 1)
            self.sim.reset_pose(initial_angles=self.queue[idx].angles_rad)
        else:
            self.sim.reset_pose()

    def _on_close(self) -> None:
        self.closed = True
        self.root.destroy()

    def _step_tick(self) -> None:
        if self.closed or not self.sim.is_connected():
            if not self.closed:
                self._on_close()
            return

        if self.queue:
            if self.queue_index >= len(self.queue):
                if self.loop_var.get():
                    if self.active_mode == "DEMO":
                        self.queue_index = 0
                        self.sim.reset_pose(initial_angles=self.queue[0].angles_rad)
                    else:
                        # Loop seamlessly by repeating the Verilog cycle frames
                        self.queue_index = 0
                else:
                    self.queue_index = len(self.queue) - 1

            frame = self.queue[self.queue_index]
            self.queue_index += 1
            telemetry = self.sim.apply_verilog_frame(frame, real_time=False)
            if self.hud is not None:
                self.hud.update(frame, telemetry, follow_camera=True)

            self.tick_count += 1
            self._draw_oled_face(frame.state_name)
            self._draw_pwm_scope(frame)

            col = STATE_HEX_COLOR.get(frame.state_name, TEXT_MAIN)
            emo = STATE_EMOTION_LABEL.get(frame.state_name, frame.state_name)
            self.state_label.config(text=f"STATE: {frame.state_name}   |   {emo}", fg=col)
            self.meta_label.config(
                text=(
                    f"Frame {frame.frame:03d} | Ph {frame.gait_phase} | "
                    f"Fault 0b{frame.fault_code:04b} | "
                    f"X={telemetry.pos_x:+.2f}m Z={telemetry.pos_z:.2f}m "
                    f"R/P={telemetry.roll_deg:+.0f}/{telemetry.pitch_deg:+.0f}d"
                )
            )
        else:
            self.sim.step_idle()

        delay_ms = max(8, int(20.0 / self.playback_speed))
        self.root.after(delay_ms, self._step_tick)

    def run(self) -> None:
        self.root.after(20, self._step_tick)
        self.root.mainloop()
