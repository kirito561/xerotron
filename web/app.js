/* ==========================================================================
   XEROTON STUDIO 3.0 — Main Control-Room Application Logic
   Connects live 50 Hz SSE telemetry + REST APIs to all 10 Dashboard Sections.
   ========================================================================== */
(() => {
  const JOINT_LABELS = [
    "FL_HIP", "FL_KNEE", "FR_HIP", "FR_KNEE",
    "RL_HIP", "RL_KNEE", "RR_HIP", "RR_KNEE",
  ];
  const JOINT_LIMITS_US = [
    [1050, 1900], [1050, 1950], [1050, 1900], [1050, 1950],
    [1050, 1900], [1050, 1950], [1050, 1900], [1050, 1950],
  ];
  const JOINT_LIMITS_RAD = [
    [-0.95, 0.95], [-1.10, 1.10], [-0.95, 0.95], [-1.10, 1.10],
    [-0.95, 0.95], [-1.10, 1.10], [-0.95, 0.95], [-1.10, 1.10],
  ];
  const STATE_COLORS = window.XerotonScene3D.STATE_COLORS;

  // 1. Initialize 3D PBR Scene
  const scene3d = window.XerotonScene3D.createScene(
    document.getElementById("canvas3d"),
    document.getElementById("oledMirror")
  );

  // 2. Section Navigation & Responsive Stage Layout
  // Sections with 3D viewport visible vs full-width data panels
  const STAGE_MODES = {
    overview:     "default",        // 3D + 420px Mission Overview
    pilot:        "default",        // 3D + 420px Manual Pilot Control
    simulation:   "sim-wide",       // Extra-wide 3D + 370px Sim Inspector
    joints:       "split-balanced", // 3D + Spacious 8-Joint Panel
    gait:         "split-balanced", // 3D + Gait Lab
    jarvis:       "split-balanced", // 3D + JARVIS Terminal
    verilog:      "full-panel",
    pwm:          "full-panel",
    telemetry:    "full-panel",
    testing:      "full-panel",
    architecture: "full-panel",
  };

  let activeSection = "overview";
  let lastSnapshot = null;
  const stageGrid = document.getElementById("stage-grid");

  function switchSection(sec) {
    if (!STAGE_MODES[sec]) return;
    activeSection = sec;
    document.querySelectorAll(".nav-item").forEach((b) => b.classList.toggle("active", b.dataset.sec === sec));
    document.querySelectorAll(".section-view").forEach((v) => v.classList.toggle("active", v.id === "sec-" + sec));

    stageGrid.classList.remove("full-panel", "sim-wide", "split-balanced");
    const mode = STAGE_MODES[sec];
    if (mode !== "default") stageGrid.classList.add(mode);

    requestAnimationFrame(() => {
      scene3d.resize();
      if (lastSnapshot) renderActiveSection(lastSnapshot);
    });
    if (sec === "testing") loadDiagnostics();
    if (sec === "pwm") loadVcdSummary();
  }

  document.querySelectorAll(".nav-item").forEach((b) => b.addEventListener("click", () => switchSection(b.dataset.sec)));
  document.querySelectorAll("[data-goto]").forEach((b) => b.addEventListener("click", () => switchSection(b.dataset.goto)));

  // 3. View Modes & Camera Controls
  let followOn = true;
  function setViewModeUI(mode) {
    scene3d.setViewMode(mode);
    document.querySelectorAll("[data-vmode]").forEach((b) => b.classList.toggle("active", b.dataset.vmode === mode));
  }
  document.querySelectorAll("[data-vmode]").forEach((b) => b.addEventListener("click", () => setViewModeUI(b.dataset.vmode)));

  document.querySelectorAll("[data-cam]").forEach((b) => {
    b.addEventListener("click", () => {
      scene3d.setCameraPreset(b.dataset.cam);
      document.querySelectorAll("#camPresetBar [data-cam]").forEach((x) => x.classList.toggle("active", x.dataset.cam === b.dataset.cam));
    });
  });

  function updateFollowUI() {
    scene3d.setFollow(followOn);
    document.getElementById("followToggleBtn").classList.toggle("active", followOn);
    document.getElementById("simFollowBtn").classList.toggle("active", followOn);
    document.getElementById("simFollowBtn").textContent = followOn ? "Follow Robot: ON" : "Follow Robot: OFF";
  }
  document.getElementById("followToggleBtn").addEventListener("click", () => { followOn = !followOn; updateFollowUI(); });
  document.getElementById("simFollowBtn").addEventListener("click", () => { followOn = !followOn; updateFollowUI(); });
  const doResetCam = () => { scene3d.resetCamera(); followOn = true; updateFollowUI(); };
  document.getElementById("resetCamBtn").addEventListener("click", doResetCam);
  document.getElementById("simResetCamBtn").addEventListener("click", doResetCam);

  // 4. REST API Helpers
  async function postJSON(url, body) {
    const resp = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return resp.json();
  }

  async function sendCmd(action, extra = {}) {
    return postJSON("/api/command", { action, ...extra });
  }

  document.querySelectorAll("[data-rtl]").forEach((b) => {
    b.addEventListener("click", () => sendCmd(b.dataset.rtl));
  });
  document.querySelectorAll("[data-sim]").forEach((b) => {
    b.addEventListener("click", () => sendCmd(b.dataset.sim));
  });
  document.getElementById("globalEstopBtn").addEventListener("click", () => {
    pilot.releaseAll(false);
    sendCmd("FAULT", { reason: "Topbar Emergency Stop button" });
  });
  document.getElementById("clearFaultBtn").addEventListener("click", () => sendCmd("CLEAR_FAULT"));
  document.getElementById("speedSelect").addEventListener("change", (e) => sendCmd("SPEED", { value: parseFloat(e.target.value) }));
  document.getElementById("loopToggle").addEventListener("change", (e) => sendCmd("LOOP", { value: e.target.checked }));

  // 4B. Single Authoritative Hold-to-Move Manual Pilot Controller (WASD + QE + SPACE + R)
  const pilot = (() => {
    const VALID_KEYS = new Set(["KeyW", "KeyS", "KeyA", "KeyD", "KeyQ", "KeyE"]);
    const activeKeys = new Set();
    let pointerKey = null;
    let seq = 0;
    let heartbeatTimer = null;
    let lastSentVec = { vx: 0, vy: 0, wz: 0 };
    let motionCfg = { walk_speed: 0.65, turn_speed: 0.70, stride_us: 60, phase_frames: 6, smoothing: 0.55, preset: "SLOW" };

    function isTypingTarget(el) {
      if (!el) return false;
      const tag = (el.tagName || "").toUpperCase();
      return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || el.isContentEditable;
    }

    function computeVector() {
      const held = new Set(activeKeys);
      if (pointerKey) held.add(pointerKey);

      // REP-103: +X Forward (W), -X Backward (S), +Y Strafe Left (A), -Y Strafe Right (D), +wz Turn Left (Q), -wz Turn Right (E)
      // Simultaneous opposite keys cancel cleanly to 0!
      const fwd = (held.has("KeyW") ? 1 : 0) - (held.has("KeyS") ? 1 : 0);
      const lat = (held.has("KeyA") ? 1 : 0) - (held.has("KeyD") ? 1 : 0);
      const yaw = (held.has("KeyQ") ? 1 : 0) - (held.has("KeyE") ? 1 : 0);

      return {
        vx: +(fwd * motionCfg.walk_speed).toFixed(3),
        vy: +(lat * motionCfg.walk_speed).toFixed(3),
        wz: +(yaw * motionCfg.turn_speed).toFixed(3),
        held,
      };
    }

    function syncButtonHighlights(held) {
      document.querySelectorAll("[data-pilot-key]").forEach((btn) => {
        btn.classList.toggle("held", held.has(btn.dataset.pilotKey));
      });
    }

    function stopHeartbeat() {
      if (heartbeatTimer !== null) {
        clearInterval(heartbeatTimer);
        heartbeatTimer = null;
      }
    }

    function dispatchPilot(payload) {
      seq += 1;
      return postJSON("/api/pilot", { ...payload, seq }).catch(() => {});
    }

    function updateMotion(forceSend = false) {
      const { vx, vy, wz, held } = computeVector();
      syncButtonHighlights(held);
      const moving = (vx !== 0 || vy !== 0 || wz !== 0);
      const changed = (vx !== lastSentVec.vx || vy !== lastSentVec.vy || wz !== lastSentVec.wz);

      if (changed || forceSend) {
        lastSentVec = { vx, vy, wz };
        dispatchPilot({ vx, vy, wz, hold: moving });
      }

      if (moving) {
        if (heartbeatTimer === null) {
          heartbeatTimer = setInterval(() => {
            const cur = computeVector();
            if (cur.vx !== 0 || cur.vy !== 0 || cur.wz !== 0) {
              dispatchPilot({ vx: cur.vx, vy: cur.vy, wz: cur.wz, hold: true });
            } else {
              stopHeartbeat();
            }
          }, 200);
        }
      } else {
        stopHeartbeat();
      }
    }

    function releaseAll(sendStop = true) {
      activeKeys.clear();
      pointerKey = null;
      stopHeartbeat();
      syncButtonHighlights(new Set());
      lastSentVec = { vx: 0, vy: 0, wz: 0 };
      if (sendStop) {
        dispatchPilot({ vx: 0, vy: 0, wz: 0, hold: false, immediate_stop: true });
      }
    }

    function neutralStand() {
      activeKeys.clear();
      pointerKey = null;
      stopHeartbeat();
      syncButtonHighlights(new Set());
      lastSentVec = { vx: 0, vy: 0, wz: 0 };
      dispatchPilot({ vx: 0, vy: 0, wz: 0, hold: false, neutral_stand: true });
    }

    // Keyboard listeners (hold-to-move + SPACE stop + R neutral stand)
    window.addEventListener("keydown", (e) => {
      if (isTypingTarget(e.target) || e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.code === "Space") {
        e.preventDefault();
        releaseAll(true);
        return;
      }
      if (e.code === "KeyR") {
        e.preventDefault();
        neutralStand();
        return;
      }
      if (!VALID_KEYS.has(e.code)) return;
      e.preventDefault();
      if (e.repeat && activeKeys.has(e.code)) return;
      activeKeys.add(e.code);
      updateMotion(false);
    });

    window.addEventListener("keyup", (e) => {
      if (!VALID_KEYS.has(e.code)) return;
      if (activeKeys.delete(e.code)) {
        updateMotion(false);
      }
    });

    // Dead-man focus & visibility safety release
    window.addEventListener("blur", () => releaseAll(true));
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) releaseAll(true);
    });

    // Mouse / Touch hold-to-move on D-pad buttons (release anywhere stops safely!)
    document.querySelectorAll("[data-pilot-key]").forEach((btn) => {
      btn.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        pointerKey = btn.dataset.pilotKey;
        updateMotion(true);
      });
    });
    const endPointerHold = () => {
      if (pointerKey !== null) {
        pointerKey = null;
        updateMotion(false);
      }
    };
    window.addEventListener("pointerup", endPointerHold);
    window.addEventListener("pointercancel", endPointerHold);

    document.querySelectorAll("[data-pilot-stop]").forEach((btn) => {
      btn.addEventListener("click", () => releaseAll(true));
    });
    document.querySelectorAll("[data-pilot-stand]").forEach((btn) => {
      btn.addEventListener("click", () => neutralStand());
    });

    // Movement Settings Presets & Sliders (SLOW default, NORMAL, FAST)
    const setWalkEl = document.getElementById("setWalkSpeed");
    const setTurnEl = document.getElementById("setTurnSpeed");
    const setStrideEl = document.getElementById("setStrideUs");
    const setPfEl = document.getElementById("setPhaseFrames");
    const setSmoothEl = document.getElementById("setSmoothing");

    function renderSettingsUI(cfg) {
      motionCfg = { ...motionCfg, ...cfg };
      document.getElementById("presetLabel").textContent = motionCfg.preset || "CUSTOM";
      document.querySelectorAll("[data-preset]").forEach((b) => {
        b.classList.toggle("active", b.dataset.preset === motionCfg.preset);
      });
      setWalkEl.value = motionCfg.walk_speed;
      setTurnEl.value = motionCfg.turn_speed;
      setStrideEl.value = motionCfg.stride_us;
      setPfEl.value = motionCfg.phase_frames;
      setSmoothEl.value = motionCfg.smoothing;
      const hz = (50.0 / (8 * motionCfg.phase_frames)).toFixed(2);
      document.getElementById("valWalkSpeed").textContent = `${Number(motionCfg.walk_speed).toFixed(2)}×`;
      document.getElementById("valTurnSpeed").textContent = `${Number(motionCfg.turn_speed).toFixed(2)}×`;
      document.getElementById("valStrideUs").textContent = `${motionCfg.stride_us} us`;
      document.getElementById("valGaitFreq").textContent = `${motionCfg.phase_frames} fr (${hz} Hz)`;
      document.getElementById("valSmoothing").textContent = `${Number(motionCfg.smoothing).toFixed(2)}`;
    }

    document.querySelectorAll("[data-preset]").forEach((btn) => {
      btn.addEventListener("click", async () => {
        const res = await postJSON("/api/settings", { preset: btn.dataset.preset }).catch(() => null);
        if (res && res.motion_settings) renderSettingsUI(res.motion_settings);
      });
    });

    const pushCustomSettings = async () => {
      const body = {
        walk_speed: parseFloat(setWalkEl.value),
        turn_speed: parseFloat(setTurnEl.value),
        stride_us: parseInt(setStrideEl.value, 10),
        phase_frames: parseInt(setPfEl.value, 10),
        smoothing: parseFloat(setSmoothEl.value),
      };
      renderSettingsUI({ ...body, preset: "CUSTOM" });
      await postJSON("/api/settings", body).catch(() => {});
    };
    [setWalkEl, setTurnEl, setStrideEl, setPfEl, setSmoothEl].forEach((el) => {
      el.addEventListener("change", pushCustomSettings);
      el.addEventListener("input", () => {
        const pf = parseInt(setPfEl.value, 10);
        const hz = (50.0 / (8 * pf)).toFixed(2);
        document.getElementById("valWalkSpeed").textContent = `${parseFloat(setWalkEl.value).toFixed(2)}×`;
        document.getElementById("valTurnSpeed").textContent = `${parseFloat(setTurnEl.value).toFixed(2)}×`;
        document.getElementById("valStrideUs").textContent = `${setStrideEl.value} us`;
        document.getElementById("valGaitFreq").textContent = `${pf} fr (${hz} Hz)`;
        document.getElementById("valSmoothing").textContent = `${parseFloat(setSmoothEl.value).toFixed(2)}`;
      });
    });

    return { releaseAll, neutralStand, renderSettingsUI };
  })();

  // 5. Build 8-Joint Control Sliders
  const sliderHost = document.getElementById("jointSlidersHost");
  const jointSliderEls = [];
  const jointStatEls = [];
  let manualPulses = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];
  let userDraggingJoint = -1;

  for (let i = 0; i < 8; i++) {
    const [loUs, hiUs] = JOINT_LIMITS_US[i];
    const [loRad, hiRad] = JOINT_LIMITS_RAD[i];
    const row = document.createElement("div");
    row.className = "joint-row";
    row.innerHTML = `
      <div class="joint-top">
        <span class="joint-name">CH${i} · ${JOINT_LABELS[i]}</span>
        <div class="joint-stats" id="jstat-${i}">Tgt: 0.00 | Act: 0.00 rad | 1500 us | IDLE</div>
      </div>
      <div class="joint-slider-row">
        <span style="font-family:Consolas,monospace;font-size:10px;color:#5d6e8c;">${loRad.toFixed(2)}</span>
        <input type="range" id="jslider-${i}" min="${loUs}" max="${hiUs}" step="5" value="1500" />
        <span style="font-family:Consolas,monospace;font-size:10px;color:#5d6e8c;">+${hiRad.toFixed(2)}</span>
        <button class="tbtn" id="jrst-${i}" style="padding:3px 7px;font-size:10px;">Neutral</button>
      </div>
    `;
    sliderHost.appendChild(row);
    const sl = row.querySelector(`#jslider-${i}`);
    jointSliderEls.push(sl);
    jointStatEls.push(row.querySelector(`#jstat-${i}`));

    sl.addEventListener("mousedown", () => { userDraggingJoint = i; });
    sl.addEventListener("mouseup", () => { userDraggingJoint = -1; });
    sl.addEventListener("input", () => {
      manualPulses[i] = parseInt(sl.value, 10);
      postJSON("/api/manual_joints", { pulses_us: manualPulses, reason: `Manual ${JOINT_LABELS[i]}` });
    });
    row.querySelector(`#jrst-${i}`).addEventListener("click", () => {
      manualPulses[i] = 1500;
      sl.value = 1500;
      postJSON("/api/manual_joints", { pulses_us: manualPulses, reason: `Reset ${JOINT_LABELS[i]} to neutral` });
    });
  }

  document.getElementById("jcAllNeutralBtn").addEventListener("click", () => {
    manualPulses = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];
    jointSliderEls.forEach((s) => (s.value = 1500));
    postJSON("/api/manual_joints", { pulses_us: manualPulses, reason: "All-joint neutral (1500 us)" });
  });

  const LEG_MAP = { FL: [0, 1], FR: [2, 3], RL: [4, 5], RR: [6, 7] };
  document.querySelectorAll("[data-leg]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const [h, k] = LEG_MAP[btn.dataset.leg];
      manualPulses = [1350, 1800, 1350, 1800, 1350, 1800, 1350, 1800];
      manualPulses[h] = 1140;
      manualPulses[k] = 1920;
      jointSliderEls.forEach((s, idx) => (s.value = manualPulses[idx]));
      postJSON("/api/manual_joints", { pulses_us: manualPulses, reason: `Manual lift ${btn.dataset.leg} leg` });
    });
  });

  // 6. Real PWM Analyzer Setup
  const pwmEnabled = [true, true, true, true, true, true, true, true];
  let pwmPaused = false;
  const chBoxHost = document.getElementById("pwmChannelChecks");
  for (let i = 0; i < 8; i++) {
    const lbl = document.createElement("label");
    lbl.style.cssText = "display:inline-flex;align-items:center;gap:4px;cursor:pointer;";
    lbl.innerHTML = `<input type="checkbox" checked data-ch="${i}" /> CH${i} (${JOINT_LABELS[i]})`;
    lbl.querySelector("input").addEventListener("change", (e) => { pwmEnabled[i] = e.target.checked; });
    chBoxHost.appendChild(lbl);
  }
  const pwmPauseBtn = document.getElementById("pwmPauseBtn");
  pwmPauseBtn.addEventListener("click", () => {
    pwmPaused = !pwmPaused;
    pwmPauseBtn.classList.toggle("active", pwmPaused);
    pwmPauseBtn.textContent = pwmPaused ? "Resume Scope" : "Pause Scope";
  });

  function drawPwmCanvas(canvas, pulses, stateName, fullMode) {
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;
    ctx.fillStyle = "#060a12";
    ctx.fillRect(0, 0, W, H);

    const activeIdx = fullMode ? pwmEnabled.map((v, i) => (v ? i : -1)).filter((i) => i >= 0) : [0,1,2,3,4,5,6,7];
    if (!activeIdx.length) return;

    const timebaseUs = fullMode ? parseFloat(document.getElementById("pwmTimebase").value) : 2500;
    const zoom = fullMode ? parseFloat(document.getElementById("pwmZoom").value) : 1.0;
    const visibleUs = timebaseUs / zoom;

    const col = STATE_COLORS[stateName] || "#22d3ee";
    const x0 = fullMode ? 112 : 66;
    const x1 = fullMode ? W - 115 : W - 56;
    const waveW = x1 - x0;
    const rowH = H / activeIdx.length;

    // Vertical reference grid (1000, 1500, 2000 us)
    ctx.strokeStyle = "rgba(96, 165, 250, 0.14)";
    ctx.lineWidth = 1;
    for (const refUs of [1000, 1500, 2000]) {
      if (refUs <= visibleUs) {
        const gx = x0 + (refUs / visibleUs) * waveW;
        ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, H); ctx.stroke();
        if (fullMode) {
          ctx.fillStyle = "#5d6e8c"; ctx.font = "9px Consolas, monospace";
          ctx.fillText(`${refUs}us`, gx + 2, 10);
        }
      }
    }

    activeIdx.forEach((ch, row) => {
      const yBase = (row + 1) * rowH - (fullMode ? 7 : 3);
      const yHigh = row * rowH + (fullMode ? 10 : 4);
      const us = pulses[ch] ?? 1500;
      const duty = (us / 20000.0) * 100.0;
      const xStart = x0 + 6;
      const xEnd = xStart + Math.min(1, us / visibleUs) * (waveW - 6);

      ctx.fillStyle = "#8a9ab8";
      ctx.font = fullMode ? "11px Consolas, monospace" : "9px Consolas, monospace";
      ctx.textAlign = "left";
      ctx.fillText(`CH${ch} ${JOINT_LABELS[ch]}`, 6, yBase - 2);

      ctx.fillStyle = col + "22";
      ctx.fillRect(xStart, yHigh, Math.max(2, xEnd - xStart), yBase - yHigh);

      ctx.strokeStyle = col;
      ctx.lineWidth = fullMode ? 2.0 : 1.5;
      ctx.beginPath();
      ctx.moveTo(x0, yBase);
      ctx.lineTo(xStart, yBase);
      ctx.lineTo(xStart, yHigh);
      ctx.lineTo(xEnd, yHigh);
      ctx.lineTo(xEnd, yBase);
      ctx.lineTo(x1, yBase);
      ctx.stroke();

      ctx.fillStyle = "#edf2fb";
      ctx.textAlign = "right";
      ctx.fillText(fullMode ? `${us}us (${duty.toFixed(1)}%)` : `${us}us`, W - 6, yBase - 2);
    });
  }

  async function loadVcdSummary() {
    const box = document.getElementById("vcdSummaryBox");
    box.textContent = "Parsing simulation/xeroton.vcd ...";
    try {
      const d = await (await fetch("/api/vcd")).json();
      if (!d.available) { box.textContent = d.error || "VCD file not available."; return; }
      box.textContent = [
        `File        : ${d.path} (${d.size_mb} MB)`,
        `Timescale   : ${d.timescale}`,
        `CH0 Period  : ${d.measured_ch0_period_ms} ms (${d.measured_ch0_freq_hz} Hz)`,
        `Sampled     : ${d.sampled_frames} PWM frames`,
        `CH0 Pulses  : [${(d.sample_ch0_pulses_us || []).join(", ")}] us`,
        `Signals     : ${(d.signals || []).join(", ")}`,
      ].join("\n");
    } catch (err) {
      box.textContent = "VCD parse failed: " + err;
    }
  }
  document.getElementById("refreshVcdBtn").addEventListener("click", loadVcdSummary);

  // 7. Rolling Telemetry Graphs (Ring Buffer of 160 Real Samples)
  const MAX_HIST = 160;
  const hist = { roll: [], pitch: [], z: [], joints: [[],[],[],[],[],[],[],[]], feet: [[],[],[],[]] };

  function pushTelemetryHistory(s) {
    const pushCap = (arr, v) => { arr.push(v); if (arr.length > MAX_HIST) arr.shift(); };
    pushCap(hist.roll, s.rpy_deg[0]);
    pushCap(hist.pitch, s.rpy_deg[1]);
    pushCap(hist.z, s.base_pos[2]);
    for (let i = 0; i < 8; i++) pushCap(hist.joints[i], s.actual_angles_rad[i]);
    for (let i = 0; i < 4; i++) pushCap(hist.feet[i], s.foot_contacts[i] ? 1 : 0);
  }

  function drawLineGraph(canvasId, seriesList, yMin, yMax, unit) {
    const c = document.getElementById(canvasId);
    const ctx = c.getContext("2d");
    const W = c.width, H = c.height;
    ctx.fillStyle = "#060a12"; ctx.fillRect(0, 0, W, H);

    // Horizontal grid lines
    ctx.strokeStyle = "rgba(96,165,250,0.12)"; ctx.lineWidth = 1;
    for (let k = 0; k <= 4; k++) {
      const y = 10 + (k / 4) * (H - 20);
      ctx.beginPath(); ctx.moveTo(36, y); ctx.lineTo(W - 8, y); ctx.stroke();
      const val = yMax - (k / 4) * (yMax - yMin);
      ctx.fillStyle = "#5d6e8c"; ctx.font = "9px Consolas, monospace"; ctx.textAlign = "right";
      ctx.fillText(val.toFixed(1), 33, y + 3);
    }

    seriesList.forEach(({ data, color, label }, si) => {
      if (data.length < 2) return;
      ctx.strokeStyle = color; ctx.lineWidth = 1.6;
      ctx.beginPath();
      data.forEach((v, idx) => {
        const x = 38 + (idx / (MAX_HIST - 1)) * (W - 46);
        const norm = Math.max(0, Math.min(1, (v - yMin) / (yMax - yMin)));
        const y = (H - 10) - norm * (H - 20);
        if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      if (label) {
        ctx.fillStyle = color; ctx.font = "9px Consolas, monospace"; ctx.textAlign = "left";
        ctx.fillText(`${label}:${data[data.length - 1].toFixed(2)}${unit}`, 42 + si * 82, 12);
      }
    });
  }

  function drawContactsGraph() {
    const c = document.getElementById("gContacts");
    const ctx = c.getContext("2d");
    const W = c.width, H = c.height;
    ctx.fillStyle = "#060a12"; ctx.fillRect(0, 0, W, H);
    const names = ["FL", "FR", "RL", "RR"];
    const cols = ["#22d3ee", "#34d399", "#60a5fa", "#f472b6"];
    const rowH = H / 4;
    for (let f = 0; f < 4; f++) {
      const data = hist.feet[f];
      const y0 = f * rowH + 6, h = rowH - 12;
      ctx.fillStyle = "#8a9ab8"; ctx.font = "10px Consolas, monospace"; ctx.textAlign = "left";
      ctx.fillText(names[f], 8, y0 + h / 2 + 3);
      for (let i = 0; i < data.length; i++) {
        if (data[i]) {
          const x = 36 + (i / MAX_HIST) * (W - 44);
          ctx.fillStyle = cols[f];
          ctx.fillRect(x, y0, Math.max(2, (W - 44) / MAX_HIST + 1), h);
        }
      }
    }
  }

  // 8. Gait Laboratory Controls
  const phaseStrip = document.getElementById("gaitPhaseStrip");
  for (let p = 0; p < 8; p++) {
    const d = document.createElement("div");
    d.className = "phase-cell";
    d.id = `phase-cell-${p}`;
    d.innerHTML = `PH ${p}<br/><span style="font-size:8px;opacity:.75;">${p < 4 ? "A-SWG" : "B-SWG"}</span>`;
    phaseStrip.appendChild(d);
  }

  let selectedGait = "WALK";
  const strideInput = document.getElementById("gaitStride");
  const cadenceInput = document.getElementById("gaitPhaseFrames");
  function refreshGaitParamLabels() {
    const st = parseInt(strideInput.value, 10);
    const pf = parseInt(cadenceInput.value, 10);
    const hz = (50.0 / (8 * pf)).toFixed(2);
    document.getElementById("lblStride").textContent = `${st} us`;
    document.getElementById("lblFreq").textContent = `${pf} frames/phase (${hz} Hz gait cycle)`;
  }
  strideInput.addEventListener("input", refreshGaitParamLabels);
  cadenceInput.addEventListener("input", refreshGaitParamLabels);
  refreshGaitParamLabels();

  document.querySelectorAll("[data-gait]").forEach((btn) => {
    btn.addEventListener("click", () => {
      selectedGait = btn.dataset.gait;
      sendCmd(selectedGait, {
        stride_us: parseInt(strideInput.value, 10),
        phase_frames: parseInt(cadenceInput.value, 10),
      });
    });
  });
  document.getElementById("applyGaitParamsBtn").addEventListener("click", () => {
    sendCmd(selectedGait, {
      stride_us: parseInt(strideInput.value, 10),
      phase_frames: parseInt(cadenceInput.value, 10),
    });
  });

  // 9. JARVIS Local Safe Text Assistant
  const jarvisLog = document.getElementById("jarvis-log");
  const jarvisInput = document.getElementById("jarvisInput");

  function appendJarvis(who, text, cls = "") {
    const div = document.createElement("div");
    div.className = `j-entry ${cls}`;
    div.innerHTML = `<strong>[${who}]</strong> ${text}`;
    jarvisLog.appendChild(div);
    jarvisLog.scrollTop = jarvisLog.scrollHeight;
  }

  async function runJarvis(text) {
    if (!text.trim()) return;
    appendJarvis("OPERATOR", text, "user");
    jarvisInput.value = "";
    try {
      const res = await postJSON("/api/jarvis", { text });
      appendJarvis(`JARVIS · ${res.intent || "OK"}`, res.reply || JSON.stringify(res), res.ok ? "" : "err");
      if (res.navigate) switchSection(res.navigate);
      if (res.trigger_diagnostics) runAllDiagnostics();
    } catch (err) {
      appendJarvis("JARVIS ERROR", String(err), "err");
    }
  }
  document.getElementById("jarvisSendBtn").addEventListener("click", () => runJarvis(jarvisInput.value));
  jarvisInput.addEventListener("keydown", (e) => { if (e.key === "Enter") runJarvis(jarvisInput.value); });
  document.querySelectorAll("[data-jcmd]").forEach((b) => b.addEventListener("click", () => runJarvis(b.dataset.jcmd)));

  // 10. Testing & Diagnostics Centre
  function renderDiagnostics(rep) {
    const body = document.getElementById("diagTableBody");
    body.innerHTML = "";
    (rep.tests || []).forEach((t) => {
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${t.name}</td>
        <td><span class="status-badge st-${t.status}">${t.status.replace("_", " ")}</span></td>
        <td>${t.duration_ms ? t.duration_ms + " ms" : "—"}</td>
        <td>${t.detail}</td>
      `;
      body.appendChild(tr);
    });
    const sum = rep.summary;
    document.getElementById("diagMeta").textContent = sum
      ? `Last Run: ${rep.timestamp} · ${sum.passed}/${sum.total} PASSED (${sum.failed} FAILED)`
      : "Status: Not run in this session";
    if (rep.log_file) document.getElementById("diagLogPath").textContent = rep.log_file;
    document.getElementById("diagLogBox").textContent = rep.log_tail || "";
  }

  async function loadDiagnostics() {
    try { renderDiagnostics(await (await fetch("/api/diagnostics")).json()); } catch (_) {}
  }

  async function runAllDiagnostics() {
    const btn = document.getElementById("runDiagBtn");
    btn.disabled = true; btn.textContent = "Running Real Tests...";
    document.getElementById("diagMeta").textContent = "Executing iverilog, vvp, pytest, PyBullet...";
    try {
      const rep = await postJSON("/api/diagnostics/run", {});
      renderDiagnostics(rep);
    } catch (err) {
      document.getElementById("diagLogBox").textContent = "Diagnostics execution failed: " + err;
    } finally {
      btn.disabled = false; btn.textContent = "Run All Diagnostics";
    }
  }
  document.getElementById("runDiagBtn").addEventListener("click", runAllDiagnostics);

  // 11. Live 50 Hz SSE Telemetry Updater
  let uiFrameCount = 0;
  let settingsSyncedOnce = false;

  function applySnapshot(s) {
    lastSnapshot = s;
    if (!settingsSyncedOnce && s.motion_settings) {
      settingsSyncedOnce = true;
      pilot.renderSettingsUI(s.motion_settings);
    }
    scene3d.updateFromSnapshot(s);
    pushTelemetryHistory(s);
    uiFrameCount++;

    // Detect stale/mismatched backend cleanly and NEVER show contradictory status
    const isManual = String(s.control_source || "").toLowerCase().includes("manual");
    const rtlConn = (typeof s.rtl_connected === "boolean") ? s.rtl_connected : !!(s.toolchain && s.toolchain.connected);
    const rtlLabel = s.rtl_status || (rtlConn ? "CONNECTED" : "DISCONNECTED");
    const srcLabel = (!rtlConn && String(s.control_source || "").includes("Verilog"))
      ? "DISCONNECTED"
      : (s.control_source || s.source_badge || "DISCONNECTED");
    const simTime = typeof s.sim_time_s === "number" ? s.sim_time_s : ((s.time_ms || 0) / 1000.0);

    // Topbar Status Pills (100% consistent)
    const rtlPill = document.getElementById("rtlConnPill");
    rtlPill.className = "pill " + (rtlConn ? "ok" : "err");
    document.getElementById("rtlConnText").textContent = `RTL: ${rtlLabel}`;

    const srcPill = document.getElementById("srcPill");
    srcPill.textContent = `SRC: ${srcLabel}`;
    srcPill.className = "pill " + (isManual ? "warn" : (srcLabel === "DISCONNECTED" ? "err" : "cyan"));

    const stPill = document.getElementById("statePill");
    stPill.textContent = `STATE: ${s.state || "UNKNOWN"}`;
    stPill.className = "pill " + (s.fault_active ? "err" : "cyan");

    document.getElementById("topPauseBtn").classList.toggle("active", !!s.paused);

    // Persistent Fault Banner
    const fb = document.getElementById("fault-banner");
    fb.classList.toggle("visible", !!s.fault_active);
    if (s.fault_active) {
      document.getElementById("bannerFaultCode").textContent = "0b" + Number(s.fault_code || 0).toString(2).padStart(4, "0");
      document.getElementById("bannerFaultReason").textContent = s.fault_reason || "Hardware fault active";
    }

    // 3D Viewport Overlays (show X, Y, Z, Yaw for clear directional verification)
    document.getElementById("vpTimeBadge").textContent = `t=${simTime.toFixed(2)}s · F${String(s.frame || 0).padStart(3, "0")} · ${s.state || "STAND"}`;
    document.getElementById("vpPoseTxt").textContent =
      `X:${s.base_pos[0] >= 0 ? "+" : ""}${s.base_pos[0].toFixed(2)} Y:${s.base_pos[1] >= 0 ? "+" : ""}${s.base_pos[1].toFixed(2)} Z:${s.base_pos[2].toFixed(2)}m Yaw:${s.rpy_deg[2].toFixed(1)}°`;
    ["led-fl", "led-fr", "led-rl", "led-rr"].forEach((id, i) => {
      document.getElementById(id).classList.toggle("on", !!s.foot_contacts[i]);
    });

    // Sync active command button highlights
    document.querySelectorAll(".cmd-btn[data-rtl], .cmd-btn[data-gait]").forEach((b) => {
      const k = b.dataset.rtl || b.dataset.gait;
      b.classList.toggle("active", k === s.state);
    });

    // Section-specific updates (~15 Hz for buttery 60 FPS WebGL)
    if (uiFrameCount % 2 !== 0) return;
    renderActiveSection(s);
  }

  function renderActiveSection(s) {
    const isManual = String(s.control_source || "").toLowerCase().includes("manual");
    const srcLabel = s.control_source || s.source_badge || "Disconnected";
    const tc = s.toolchain || { connected: false, status_label: "NOT CONNECTED" };
    const simTime = typeof s.sim_time_s === "number" ? s.sim_time_s : ((s.time_ms || 0) / 1000.0);

    // 01. Overview
    const cv = s.cmd_vel || { vx: 0, vy: 0, wz: 0 };
    if (activeSection === "overview") {
      document.getElementById("ovEmotion").textContent = `${s.state} · Ph ${s.gait_phase}`;
      document.getElementById("ovState").textContent = s.state;
      document.getElementById("ovPrev").textContent = `Dir: ${s.active_direction || "STOP"} · Frame ${s.frame}`;
      document.getElementById("ovSrc").textContent = srcLabel;
      document.getElementById("ovRtlMs").textContent = s.rtl_exec_ms > 0 ? `vvp: ${s.rtl_exec_ms} ms` : "50.00 Hz (20 ms)";
      document.getElementById("ovPos").textContent = `${s.base_pos[0] >= 0 ? "+" : ""}${s.base_pos[0].toFixed(2)} / ${s.base_pos[1] >= 0 ? "+" : ""}${s.base_pos[1].toFixed(2)}m`;
      document.getElementById("ovRpy").textContent = `Z:${s.base_pos[2].toFixed(2)}m Yaw:${s.rpy_deg[2].toFixed(1)}°`;
      const fCount = s.foot_contacts.filter(Boolean).length;
      document.getElementById("ovFault").textContent = s.fault_active ? `FAULT 0b${Number(s.fault_code).toString(2).padStart(4,"0")}` : "NOMINAL";
      document.getElementById("ovFault").style.color = s.fault_active ? "#ff6b81" : "#10b981";
      document.getElementById("ovFeet").textContent = `Foot Contacts: ${fCount}/4`;
      document.getElementById("ovVx").textContent = (cv.vx >= 0 ? "+" : "") + Number(cv.vx || 0).toFixed(2);
      document.getElementById("ovVy").textContent = (cv.vy >= 0 ? "+" : "") + Number(cv.vy || 0).toFixed(2);
      document.getElementById("ovWz").textContent = (cv.wz >= 0 ? "+" : "") + Number(cv.wz || 0).toFixed(2);
      drawPwmCanvas(document.getElementById("miniPwmCanvas"), s.pulses_us, s.state, false);
    }

    // 02. Manual Pilot Control
    if (activeSection === "pilot") {
      const dir = s.active_direction || "STOP";
      const prio = s.control_priority || "P2_STOP";
      document.getElementById("pilotDirBadge").textContent = dir;
      const pb = document.getElementById("pilotPriorityBadge");
      pb.textContent = prio;
      pb.className = "pill " + (prio === "P1_ESTOP" ? "err" : (prio === "P3_MANUAL_PILOT" ? "ok" : "cyan"));
      document.getElementById("plVx").textContent = (cv.vx >= 0 ? "+" : "") + Number(cv.vx || 0).toFixed(2);
      document.getElementById("plVy").textContent = (cv.vy >= 0 ? "+" : "") + Number(cv.vy || 0).toFixed(2);
      document.getElementById("plWz").textContent = (cv.wz >= 0 ? "+" : "") + Number(cv.wz || 0).toFixed(2);

      const rtlOk = (typeof s.rtl_connected === "boolean") ? s.rtl_connected : !!tc.connected;
      const stRtl = document.getElementById("stRtl");
      stRtl.textContent = s.rtl_status || (rtlOk ? "CONNECTED" : "DISCONNECTED");
      stRtl.className = "sv " + (rtlOk ? "ok" : "err");

      const stSim = document.getElementById("stSim");
      stSim.textContent = s.verilog_sim_status || (rtlOk ? "READY" : "DISCONNECTED");
      stSim.className = "sv " + (rtlOk ? "ok" : "err");

      const stBr = document.getElementById("stBridge");
      stBr.textContent = s.python_bridge_status || "ACTIVE";
      stBr.className = "sv " + (stBr.textContent === "ACTIVE" ? "ok" : "err");

      const stPyb = document.getElementById("stPyb");
      stPyb.textContent = s.pybullet_status || "RUNNING (250 Hz)";
      stPyb.className = "sv " + (stPyb.textContent.startsWith("RUNNING") ? "ok" : "warn");

      document.getElementById("stCmd").textContent = s.active_command || s.state;
      document.getElementById("stFsm").textContent = `${s.state} (Ph ${s.gait_phase})`;
      const stDir = document.getElementById("stDir");
      stDir.textContent = dir;
      stDir.className = "sv " + (dir === "STOP" ? "" : "cyan");

      const stPhys = document.getElementById("stPhys");
      stPhys.textContent = s.physics_status || "RUNNING";
      stPhys.className = "sv " + (s.fault_active ? "err" : (stPhys.textContent.startsWith("RUNNING") ? "ok" : "warn"));
    }

    // 02. Robot Simulation
    if (activeSection === "simulation") {
      document.getElementById("simTimeVal").textContent = `${simTime.toFixed(3)} s`;
      document.getElementById("simZVal").textContent = `${s.base_pos[2].toFixed(4)} m`;
      document.getElementById("simPenVal").textContent = `${((s.max_penetration_m || 0) * 1000).toFixed(2)} mm`;
      document.getElementById("simErrVal").textContent = String(s.physics_error_count || 0);
    }

    // 03. Verilog Control
    if (activeSection === "verilog") {
      const badge = document.getElementById("vcConnBadge");
      badge.className = "pill " + (tc.connected ? "ok" : "err");
      badge.textContent = tc.connected ? "CONNECTED (Icarus Verilog 13.0)" : "NOT CONNECTED";
      document.getElementById("vcState").textContent = s.state;
      document.getElementById("vcPrevState").textContent = s.prev_state || "IDLE";
      document.getElementById("vcCmd").textContent = `4'd${s.command}`;
      document.getElementById("vcFrame").textContent = String(s.frame);
      document.getElementById("vcReset").textContent = s.reset_active ? "ASSERTED (1)" : "DEASSERTED (0)";
      document.getElementById("vcFaultReg").textContent = `4'b${Number(s.fault_code || 0).toString(2).padStart(4, "0")}`;
      document.getElementById("vcFaultTxt").textContent = s.fault_reason || "No active fault";
      document.getElementById("vcExecMs").textContent = `${s.rtl_exec_ms || 0} ms`;
      document.getElementById("vcExecSrc").textContent = srcLabel;

      document.getElementById("vcRegTable").innerHTML = s.pulses_us.map((us, i) => `
        <tr><td>j${i}</td><td>${JOINT_LABELS[i]}</td><td>${us}</td><td>12'h${us.toString(16).toUpperCase().padStart(3,"0")}</td><td>${(us / 200).toFixed(2)}%</td></tr>
      `).join("");

      document.getElementById("vcFsmLog").innerHTML = (s.fsm_history || []).map((e) => `
        <tr><td>${e.time}</td><td>${e.from}</td><td style="color:#22d3ee;">${e.to}</td><td>${e.cmd}</td><td>${e.source}</td></tr>
      `).join("");

      const fh = s.fault_history || [];
      document.getElementById("vcFaultHistory").textContent = fh.length
        ? fh.map((f) => `[${f.time}] ${f.code} — ${f.reason}`).join("\n")
        : "No faults recorded in this session.";
    }

    // 04. Joint Control
    if (activeSection === "joints") {
      const jb = document.getElementById("jcSourceBadge");
      jb.textContent = isManual ? "ACTIVE: Manual Control (Safety-Gated)" : `ACTIVE: ${srcLabel}`;
      jb.className = "pill " + (isManual ? "warn" : "cyan");

      for (let i = 0; i < 8; i++) {
        const tgt = s.target_angles_rad[i], act = s.actual_angles_rad[i], vel = (s.joint_velocities_rad_s || [])[i] || 0;
        const us = s.pulses_us[i];
        const status = s.fault_active ? "LOCKED" : (Math.abs(vel) > 0.08 ? "MOVING" : "HOLD");
        jointStatEls[i].textContent = `Tgt:${tgt >= 0 ? "+" : ""}${tgt.toFixed(2)} | Act:${act >= 0 ? "+" : ""}${act.toFixed(2)} rad | ${us} us | ${status}`;
        if (userDraggingJoint !== i && !isManual) {
          jointSliderEls[i].value = us;
          manualPulses[i] = us;
        }
      }
    }

    // 05. Real PWM Analyzer
    if (activeSection === "pwm") {
      if (!pwmPaused) {
        drawPwmCanvas(document.getElementById("fullPwmCanvas"), s.pulses_us, s.state, true);
      }
      document.getElementById("pwmMetricsTable").innerHTML = s.pulses_us.map((us, i) => `
        <tr><td>CH${i}</td><td>${JOINT_LABELS[i]}</td><td>${us} us</td><td>20.00 ms</td><td>50.00 Hz</td><td>${(us / 200).toFixed(2)}%</td></tr>
      `).join("");
    }

    // 06. Sensor Telemetry
    if (activeSection === "telemetry") {
      document.getElementById("telRpy").textContent = `${s.rpy_deg[0].toFixed(1)}° / ${s.rpy_deg[1].toFixed(1)}° / ${s.rpy_deg[2].toFixed(1)}°`;
      document.getElementById("telPos").textContent = `${s.base_pos[0].toFixed(2)}, ${s.base_pos[1].toFixed(2)}, ${s.base_pos[2].toFixed(2)}`;
      const lv = s.base_lin_vel || [0,0,0];
      document.getElementById("telVel").textContent = `${lv[0].toFixed(2)}, ${lv[1].toFixed(2)}, ${lv[2].toFixed(2)}`;
      document.getElementById("telTime").textContent = `${simTime.toFixed(2)}s / ${s.physics_error_count || 0}`;
      document.getElementById("telSlip").textContent = `Slip: ${(s.foot_slip_mps || 0).toFixed(3)} m/s · Pen: ${((s.max_penetration_m || 0) * 1000).toFixed(2)} mm`;

      drawLineGraph("gRollPitch", [
        { data: hist.roll, color: "#22d3ee", label: "Roll" },
        { data: hist.pitch, color: "#f472b6", label: "Pitch" },
      ], -20, 20, "°");
      drawLineGraph("gHeight", [{ data: hist.z, color: "#10b981", label: "Z" }], 0.10, 0.25, "m");
      const jColors = ["#22d3ee","#38bdf8","#34d399","#10b981","#fbbf24","#f59e0b","#f472b6","#a78bfa"];
      drawLineGraph("gJoints", hist.joints.map((d, i) => ({ data: d, color: jColors[i], label: i < 4 ? JOINT_LABELS[i] : "" })), -1.1, 1.1, "");
      drawContactsGraph();

      document.getElementById("telJointTable").innerHTML = JOINT_LABELS.map((nm, i) => `
        <tr><td>${nm}</td><td>${s.target_angles_rad[i].toFixed(3)}</td><td>${s.actual_angles_rad[i].toFixed(3)}</td><td>${((s.joint_velocities_rad_s || [])[i] || 0).toFixed(3)}</td><td>${s.pulses_us[i]}</td></tr>
      `).join("");
    }

    // 07. Gait Laboratory
    if (activeSection === "gait") {
      const cyclicStates = new Set([
        "WALK", "WALK_FORWARD", "WALK_BACKWARD", "STRAFE_LEFT", "STRAFE_RIGHT",
        "TURN_LEFT", "TURN_RIGHT", "TROT", "WAVE", "DANCE",
      ]);
      for (let p = 0; p < 8; p++) {
        document.getElementById(`phase-cell-${p}`).classList.toggle("active", p === s.gait_phase && cyclicStates.has(s.state));
      }
      const activeTxt = s.foot_contacts.map((on, i) => (on ? ["FL","FR","RL","RR"][i] : null)).filter(Boolean).join(" + ") || "AIRBORNE";
      document.getElementById("gaitActiveLegs").textContent = `GROUNDED: ${activeTxt}`;

      const ge = s.gait_eval || {};
      const sb = document.getElementById("gaitStabBadge");
      sb.textContent = ge.stability_label || "EXPERIMENTAL";
      sb.className = "pill " + (String(ge.stability_label).startsWith("VALIDATED") ? "ok" : (String(ge.stability_label).startsWith("UNSTABLE") ? "err" : "warn"));
      document.getElementById("gaitFramesEval").textContent = `${ge.frames_evaluated || 0} frames (${((ge.frames_evaluated || 0) * 0.02).toFixed(2)}s)`;
      document.getElementById("geDist").textContent = `${(ge.distance_x_m || 0) >= 0 ? "+" : ""}${(ge.distance_x_m || 0).toFixed(3)} m`;
      document.getElementById("geRP").textContent = `${ge.max_abs_roll_deg || 0}° / ${ge.max_abs_pitch_deg || 0}°`;
      document.getElementById("geH").textContent = `${(ge.avg_height_m || 0).toFixed(3)} / ${(ge.min_height_m || 0).toFixed(3)} m`;
      document.getElementById("geFeet").textContent = `${ge.avg_feet_in_contact || 0} / 4`;
      document.getElementById("geSlip").textContent = `${(ge.max_foot_slip_mps || 0).toFixed(3)} m/s`;
      document.getElementById("gePen").textContent = `${ge.max_penetration_mm || 0} mm / ${ge.fell ? "FALL!" : "UPRIGHT"}`;
    }
  }

  function connectSSE() {
    const es = new EventSource("/api/stream");
    es.onmessage = (ev) => {
      try { applySnapshot(JSON.parse(ev.data)); } catch (_) {}
    };
    es.onerror = () => {
      document.getElementById("rtlConnPill").className = "pill err";
      document.getElementById("rtlConnText").textContent = "RTL: DISCONNECTED";
      document.getElementById("srcPill").textContent = "SRC: Disconnected";
      document.getElementById("srcPill").className = "pill err";
      es.close();
      setTimeout(connectSSE, 1200);
    };
  }
  connectSSE();
})();
