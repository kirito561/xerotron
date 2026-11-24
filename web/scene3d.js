/* ==========================================================================
   XEROTON STUDIO — Compact DIY 8-DOF Servo Quadruped 3D Engine (Reference 2)
   1:1 Geometric Match with robot/xeroton.urdf:
     - Body         : 0.220 x 0.132 x 0.068 m (dark brown / bronze 3D-printed)
     - Front OLED   : 0.96"-style recessed OLED module with non-mirrored eyes
     - Pink Ears    : Cute decorative pink ear accents above the OLED face
     - Hip Origins  : (+/-0.080, +/-0.092, -0.004) with black micro-servo housings
     - Upper Legs   : L1 = 0.080 m mechanical linkages + knee micro-servos
     - Lower Legs   : L2 = 0.082 m linkages + small rounded rubber feet (r=0.016)
     - Wiring       : Exposed orange & yellow 3-wire servo cables
     - View Modes   : Normal (default), Wireframe, Skeleton, Collision, Transparent
   ========================================================================== */
window.XerotonScene3D = (() => {
  const STATE_COLORS = {
    IDLE:          "#38bdf8",
    STAND:         "#22d3ee",
    WALK:          "#06b6d4",
    WALK_FORWARD:  "#06b6d4",
    WALK_BACKWARD: "#f59e0b",
    STRAFE_LEFT:   "#34d399",
    STRAFE_RIGHT:  "#34d399",
    TURN_LEFT:     "#38bdf8",
    TURN_RIGHT:    "#38bdf8",
    TROT:          "#3b82f6",
    STOP:          "#fbbf24",
    WAVE:          "#34d399",
    DANCE:         "#f472b6",
    SIT:           "#a78bfa",
    SLEEP:         "#818cf8",
    FAULT:         "#f43f5e",
    MANUAL:        "#fbbf24",
  };

  function drawRoundedRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
    ctx.fill();
  }

  function createScene(canvas3d, mirrorCanvas) {
    // -----------------------------------------------------------------------
    // 1. Live OLED 256x128 Pixel Face Renderer (All Expressions)
    // -----------------------------------------------------------------------
    const oledTexCanvas = document.createElement("canvas");
    oledTexCanvas.width = 256;
    oledTexCanvas.height = 128;
    const oledCtx = oledTexCanvas.getContext("2d");
    const mirrorCtx = mirrorCanvas ? mirrorCanvas.getContext("2d") : null;

    function renderOLEDFace(stateName, timeSec) {
      const ctx = oledCtx;
      const w = 256, h = 128;
      ctx.fillStyle = "#03070e";
      ctx.fillRect(0, 0, w, h);

      // Authentic OLED scanline grid
      ctx.fillStyle = "rgba(34, 211, 238, 0.045)";
      for (let y = 0; y < h; y += 4) ctx.fillRect(0, y, w, 1);

      const st = (stateName || "IDLE").toUpperCase();
      const col = STATE_COLORS[st] || "#22d3ee";
      const blinkPhase = timeSec % 3.8;
      const isBlinking = (blinkPhase > 3.60 && blinkPhase < 3.74);
      const beat = Math.floor(timeSec * 5.0) % 2;
      const lx = 78, rx = 178, cy = 64;

      ctx.save();
      ctx.shadowColor = col;
      ctx.shadowBlur = 16;

      if (st === "FAULT") {
        // WARNING expression: flashing red X-X + label
        const fcol = beat === 0 ? "#ff2e4c" : "#ff8822";
        ctx.strokeStyle = fcol; ctx.shadowColor = fcol;
        ctx.lineWidth = 10; ctx.lineCap = "round";
        for (const cx of [lx, rx]) {
          ctx.beginPath();
          ctx.moveTo(cx - 22, cy - 22); ctx.lineTo(cx + 22, cy + 22);
          ctx.moveTo(cx + 22, cy - 22); ctx.lineTo(cx - 22, cy + 22);
          ctx.stroke();
        }
        ctx.shadowBlur = 4; ctx.fillStyle = fcol;
        ctx.font = "bold 14px Consolas, monospace"; ctx.textAlign = "center";
        ctx.fillText("FAULT WARNING", w / 2, 114);
      } else if (st === "SLEEP") {
        // SLEEP expression: peaceful closed curved U-U eyes + floating Zzz
        ctx.strokeStyle = "#818cf8"; ctx.shadowColor = "#818cf8";
        ctx.lineWidth = 9; ctx.lineCap = "round";
        for (const cx of [lx, rx]) {
          ctx.beginPath();
          ctx.arc(cx, cy - 6, 22, 0.15 * Math.PI, 0.85 * Math.PI);
          ctx.stroke();
        }
        const drift = Math.sin(timeSec * 2.2) * 5;
        ctx.fillStyle = "#a5b4fc";
        ctx.font = "bold 15px Consolas, monospace"; ctx.fillText("z", 210, 42 + drift);
        ctx.font = "bold 20px Consolas, monospace"; ctx.fillText("Z", 224, 28 - drift * 0.5);
      } else if (st === "IDLE") {
        // IDLE expression: relaxed calm half-lidded eyes
        ctx.fillStyle = col;
        for (const cx of [lx, rx]) {
          drawRoundedRect(ctx, cx - 22, cy - 10, 44, 24, 9);
          ctx.shadowBlur = 0; ctx.fillStyle = "#e0f2fe";
          ctx.beginPath(); ctx.arc(cx - 6, cy - 1, 4, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = col; ctx.shadowBlur = 14;
        }
      } else if (st === "WAVE" || st === "SIT") {
        // Happy ^ ^ smiling eyes + rosy blush
        ctx.strokeStyle = col; ctx.lineWidth = 10; ctx.lineCap = "round";
        for (const cx of [lx, rx]) {
          ctx.beginPath();
          ctx.arc(cx, cy + 12, 24, Math.PI * 1.15, Math.PI * 1.85);
          ctx.stroke();
        }
        ctx.fillStyle = "rgba(244, 114, 182, 0.55)"; ctx.shadowColor = "#f472b6";
        ctx.beginPath(); ctx.ellipse(42, 84, 11, 6, 0, 0, Math.PI * 2); ctx.fill();
        ctx.beginPath(); ctx.ellipse(214, 84, 11, 6, 0, 0, Math.PI * 2); ctx.fill();
      } else if (st === "DANCE") {
        // DANCE: playful sparkly bouncing neon eyes
        const dcol = beat === 0 ? "#f472b6" : "#22d3ee";
        const bounce = Math.sin(timeSec * 12) * 5;
        ctx.fillStyle = dcol; ctx.strokeStyle = dcol; ctx.shadowColor = dcol;
        if (beat === 0) {
          ctx.lineWidth = 10; ctx.lineCap = "round";
          for (const cx of [lx, rx]) {
            ctx.beginPath();
            ctx.arc(cx, cy + 10 + bounce, 22, Math.PI * 1.15, Math.PI * 1.85);
            ctx.stroke();
          }
        } else {
          for (const cx of [lx, rx]) {
            drawRoundedRect(ctx, cx - 22, cy - 22 + bounce, 44, 44, 14);
            ctx.shadowBlur = 0; ctx.fillStyle = "#ffffff";
            ctx.beginPath(); ctx.arc(cx - 7, cy - 8 + bounce, 6, 0, Math.PI * 2); ctx.fill();
            ctx.fillStyle = dcol; ctx.shadowBlur = 16;
          }
        }
        ctx.lineWidth = 5;
        ctx.beginPath(); ctx.arc(w / 2, 76 + bounce, 16, 0.15 * Math.PI, 0.85 * Math.PI); ctx.stroke();
      } else if (
        st === "WALK" || st === "WALK_FORWARD" || st === "WALK_BACKWARD" ||
        st === "STRAFE_LEFT" || st === "STRAFE_RIGHT" ||
        st === "TURN_LEFT" || st === "TURN_RIGHT" || st === "TROT"
      ) {
        // Directional locomotion alert eyes with live gaze direction
        const glanceX =
          (st === "STRAFE_LEFT" || st === "TURN_LEFT") ? -9 :
          (st === "STRAFE_RIGHT" || st === "TURN_RIGHT") ? 9 : 0;
        const glanceY = st === "WALK_BACKWARD" ? 4 : -2;
        ctx.fillStyle = col;
        for (const cx of [lx, rx]) {
          drawRoundedRect(ctx, cx - 22 + glanceX * 0.4, cy - 18, 44, 36, 12);
          ctx.shadowBlur = 0; ctx.fillStyle = "#ffffff";
          ctx.beginPath(); ctx.arc(cx + glanceX, cy - 4 + glanceY, 6, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = col; ctx.shadowBlur = 16;
        }
        ctx.shadowBlur = 4; ctx.fillStyle = col;
        ctx.font = "bold 11px Consolas, monospace"; ctx.textAlign = "center";
        ctx.fillText(st.replace("_", " "), w / 2, 118);
      } else if (st === "STOP") {
        // Calm steady eyes
        ctx.fillStyle = col;
        for (const cx of [lx, rx]) drawRoundedRect(ctx, cx - 22, cy - 12, 44, 24, 8);
      } else if (isBlinking) {
        ctx.strokeStyle = col; ctx.lineWidth = 8; ctx.lineCap = "round";
        for (const cx of [lx, rx]) {
          ctx.beginPath(); ctx.moveTo(cx - 24, cy); ctx.lineTo(cx + 24, cy); ctx.stroke();
        }
      } else {
        // STAND / MANUAL: normal bright glossy blinking digital eyes
        const lookX = Math.sin(timeSec * 1.2) * 6;
        ctx.fillStyle = col;
        for (const cx of [lx, rx]) {
          drawRoundedRect(ctx, cx - 22, cy - 24, 44, 48, 14);
          ctx.shadowBlur = 0; ctx.fillStyle = "#ffffff";
          ctx.beginPath(); ctx.arc(cx - 6 + lookX, cy - 10, 7, 0, Math.PI * 2); ctx.fill();
          ctx.beginPath(); ctx.arc(cx + 8 + lookX, cy + 7, 3, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = col; ctx.shadowBlur = 16;
        }
      }
      ctx.restore();

      if (mirrorCtx) mirrorCtx.drawImage(oledTexCanvas, 0, 0, mirrorCanvas.width, mirrorCanvas.height);
    }

    // -----------------------------------------------------------------------
    // 2. Three.js Renderer, Studio Lighting & Warm Workbench Scene
    // -----------------------------------------------------------------------
    const renderer = new THREE.WebGLRenderer({ canvas: canvas3d, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x131926);
    scene.fog = new THREE.FogExp2(0x131926, 0.22);

    const camera = new THREE.PerspectiveCamera(38, 1, 0.02, 25);

    // Soft studio 3-point lighting tuned for rich dark-bronze 3D-printed PLA contrast
    scene.add(new THREE.HemisphereLight(0xf6efe8, 0x1e2533, 0.62));

    const keyLight = new THREE.DirectionalLight(0xfff3e2, 1.15);
    keyLight.position.set(1.2, 2.0, 1.5);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.set(2048, 2048);
    keyLight.shadow.camera.near = 0.1; keyLight.shadow.camera.far = 8;
    keyLight.shadow.camera.left = -1.2; keyLight.shadow.camera.right = 1.2;
    keyLight.shadow.camera.top = 1.2; keyLight.shadow.camera.bottom = -1.2;
    keyLight.shadow.bias = -0.0004;
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0xdbeafe, 0.35);
    fillLight.position.set(-1.5, 1.0, 1.2);
    scene.add(fillLight);

    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.42);
    rimLight.position.set(-1.2, 1.2, -1.6);
    scene.add(rimLight);

    // Z-up Root Group -> exact 1:1 match with PyBullet (X forward, Y left, Z up)
    const worldZUp = new THREE.Group();
    worldZUp.rotation.x = -Math.PI / 2;
    scene.add(worldZUp);

    // Warm Natural Oak DIY Workbench Surface Texture
    function makeWorkbenchTexture() {
      const c = document.createElement("canvas");
      c.width = 512; c.height = 512;
      const ctx = c.getContext("2d");
      ctx.fillStyle = "#c8a47e";
      ctx.fillRect(0, 0, 512, 512);
      for (let i = 0; i < 380; i++) {
        const y = Math.random() * 512;
        const a = 0.025 + Math.random() * 0.05;
        ctx.fillStyle = i % 2 === 0 ? `rgba(92,56,28,${a})` : `rgba(242,216,184,${a})`;
        ctx.fillRect(0, y, 512, 1 + Math.random() * 3.5);
      }
      // Subtle engineering calibration lines (every 0.25 m)
      ctx.strokeStyle = "rgba(65, 42, 22, 0.13)";
      ctx.lineWidth = 2;
      ctx.strokeRect(0, 0, 512, 512);
      ctx.strokeStyle = "rgba(65, 42, 22, 0.06)";
      ctx.lineWidth = 1;
      for (let g = 128; g < 512; g += 128) {
        ctx.beginPath(); ctx.moveTo(g, 0); ctx.lineTo(g, 512); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, g); ctx.lineTo(512, g); ctx.stroke();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(20, 20);
      tex.encoding = THREE.sRGBEncoding;
      return tex;
    }

    const floorMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(10, 10),
      new THREE.MeshStandardMaterial({ map: makeWorkbenchTexture(), roughness: 0.74, metalness: 0.04 })
    );
    floorMesh.receiveShadow = true;
    worldZUp.add(floorMesh);

    // Subtle procedural 3D-printed PLA layer-line bump texture
    function make3DPrintTexture() {
      const c = document.createElement("canvas");
      c.width = 64; c.height = 64;
      const ctx = c.getContext("2d");
      ctx.fillStyle = "#808080";
      ctx.fillRect(0, 0, 64, 64);
      for (let y = 0; y < 64; y += 3) {
        ctx.fillStyle = "#989898"; ctx.fillRect(0, y, 64, 1);
        ctx.fillStyle = "#686868"; ctx.fillRect(0, y + 1, 64, 1);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(6, 6);
      return tex;
    }
    const plaBump = make3DPrintTexture();

    // -----------------------------------------------------------------------
    // 3. Reference 2 Materials & Geometry Tracking (for 5 Engineering Modes)
    // -----------------------------------------------------------------------
    const oledTexture = new THREE.CanvasTexture(oledTexCanvas);
    oledTexture.encoding = THREE.sRGBEncoding;

    const visualMaterials = [];
    function regMat(m) { visualMaterials.push(m); return m; }

    const matBronze   = regMat(new THREE.MeshStandardMaterial({ color: 0x2c1910, roughness: 0.54, metalness: 0.18, bumpMap: plaBump, bumpScale: 0.0009 }));
    const matBronzeDk = regMat(new THREE.MeshStandardMaterial({ color: 0x1b0f0a, roughness: 0.60, metalness: 0.14, bumpMap: plaBump, bumpScale: 0.0006 }));
    const matLinkTan  = regMat(new THREE.MeshStandardMaterial({ color: 0x422619, roughness: 0.50, metalness: 0.14, bumpMap: plaBump, bumpScale: 0.0006 }));
    const matServo    = regMat(new THREE.MeshStandardMaterial({ color: 0x111317, roughness: 0.36, metalness: 0.28 }));
    const matHorn     = regMat(new THREE.MeshStandardMaterial({ color: 0xf1ece6, roughness: 0.32, metalness: 0.08 }));
    const matScrew    = regMat(new THREE.MeshStandardMaterial({ color: 0xd1d5db, roughness: 0.24, metalness: 0.85 }));
    const matPinkEar  = regMat(new THREE.MeshStandardMaterial({ color: 0xf472b6, roughness: 0.38, metalness: 0.05 }));
    const matOledPcb  = regMat(new THREE.MeshStandardMaterial({ color: 0x090d16, roughness: 0.34, metalness: 0.25 }));
    const matBrass    = regMat(new THREE.MeshStandardMaterial({ color: 0xd4a64a, roughness: 0.30, metalness: 0.80 }));
    const matPaw      = regMat(new THREE.MeshStandardMaterial({ color: 0x16161a, roughness: 0.88, metalness: 0.02 }));
    const matWireOrg  = regMat(new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.45 }));
    const matWireYel  = regMat(new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.45 }));
    const matWireBrn  = regMat(new THREE.MeshStandardMaterial({ color: 0x7c2d12, roughness: 0.45 }));

    const matCollision = new THREE.MeshBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.25, depthWrite: false });
    const matCollisionEdge = new THREE.MeshBasicMaterial({ color: 0x38bdf8, wireframe: true });
    const matSkeletonBone = new THREE.MeshBasicMaterial({ color: 0xfbbf24 });
    const matSkeletonJoint = new THREE.MeshBasicMaterial({ color: 0x22d3ee });

    const visualMeshes = [];
    const collisionObjects = [];
    const skeletonObjects = [];
    const trackVis = (m) => { visualMeshes.push(m); return m; };
    const trackCol = (o) => { o.visible = false; collisionObjects.push(o); return o; };
    const trackSkel = (o) => { o.visible = false; skeletonObjects.push(o); return o; };

    function makeRoundedBox(sx, sy, sz, radius, material) {
      const shape = new THREE.Shape();
      const hw = sx / 2 - radius, hh = sy / 2 - radius;
      shape.moveTo(-hw, -sy / 2);
      shape.lineTo(hw, -sy / 2);
      shape.quadraticCurveTo(sx / 2, -sy / 2, sx / 2, -hh);
      shape.lineTo(sx / 2, hh);
      shape.quadraticCurveTo(sx / 2, sy / 2, hw, sy / 2);
      shape.lineTo(-hw, sy / 2);
      shape.quadraticCurveTo(-sx / 2, sy / 2, -sx / 2, hh);
      shape.lineTo(-sx / 2, -hh);
      shape.quadraticCurveTo(-sx / 2, -sy / 2, -hw, -sy / 2);
      const geo = new THREE.ExtrudeGeometry(shape, {
        depth: Math.max(0.002, sz - radius * 2),
        bevelEnabled: true, bevelThickness: radius, bevelSize: radius, bevelSegments: 3,
      });
      geo.center();
      const mesh = new THREE.Mesh(geo, material);
      mesh.castShadow = true; mesh.receiveShadow = true;
      return trackVis(mesh);
    }

    function addCollisionBox(parent, sx, sy, sz, px, py, pz) {
      const grp = new THREE.Group();
      grp.position.set(px, py, pz);
      grp.add(new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), matCollision));
      grp.add(new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), matCollisionEdge));
      parent.add(trackCol(grp));
    }

    function addCollisionSphere(parent, r, px, py, pz) {
      const grp = new THREE.Group();
      grp.position.set(px, py, pz);
      grp.add(new THREE.Mesh(new THREE.SphereGeometry(r, 16, 16), matCollision));
      grp.add(new THREE.Mesh(new THREE.SphereGeometry(r, 12, 12), matCollisionEdge));
      parent.add(trackCol(grp));
    }

    // Build an authentic MG90S/SG90 micro-servo motor unit
    function buildMicroServo(sideSign) {
      const g = new THREE.Group();
      // Main matte-black rectangular servo body
      const body = trackVis(new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.020, 0.034), matServo));
      body.castShadow = true; body.receiveShadow = true;
      g.add(body);
      // Mounting flange tabs (front/rear)
      const flange = trackVis(new THREE.Mesh(new THREE.BoxGeometry(0.044, 0.020, 0.004), matServo));
      flange.position.set(0, 0, 0.008);
      g.add(flange);
      // Tiny silver mounting screws
      for (const sx of [-0.019, 0.019]) {
        const scr = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0022, 0.022, 10), matScrew));
        scr.position.set(sx, 0, 0.008);
        g.add(scr);
      }
      // Output spline boss pointing outward along sideSign * Y
      const boss = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0085, 0.0085, 0.006, 16), matServo));
      boss.position.set(0, sideSign * 0.012, 0);
      g.add(boss);
      return g;
    }

    // -----------------------------------------------------------------------
    // 4. Compact Reference 2 Robot Dog Body & 8-DOF Articulated Hierarchy
    // -----------------------------------------------------------------------
    const robotBase = new THREE.Group();
    worldZUp.add(robotBase);

    // Compact Rectangular Dark-Brown / Bronze 3D-Printed Body Enclosure (0.220 x 0.132 x 0.068 m)
    const bodyMain = makeRoundedBox(0.218, 0.130, 0.064, 0.008, matBronze);
    bodyMain.position.set(0, 0, 0.0);
    robotBase.add(bodyMain);

    // Darker recessed top lid & seam trim + 4 corner M2.5 screws
    const topLid = makeRoundedBox(0.194, 0.112, 0.008, 0.003, matBronzeDk);
    topLid.position.set(-0.004, 0, 0.034);
    robotBase.add(topLid);
    for (const sx of [-0.084, 0.076]) {
      for (const sy of [-0.044, 0.044]) {
        const bolt = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.004, 12), matScrew));
        bolt.rotation.x = Math.PI / 2;
        bolt.position.set(sx, sy, 0.038);
        robotBase.add(bolt);
      }
    }

    // Lower dark chassis plate
    const belly = makeRoundedBox(0.196, 0.114, 0.010, 0.003, matServo);
    belly.position.set(0, 0, -0.032);
    robotBase.add(belly);

    // Front 0.96" OLED Display Module PCB + Recessed Glass (+X face)
    const oledPcb = makeRoundedBox(0.012, 0.102, 0.056, 0.003, matOledPcb);
    oledPcb.position.set(0.112, 0, 0.002);
    robotBase.add(oledPcb);

    // 4 tiny brass OLED mounting standoffs
    for (const sy of [-0.044, 0.044]) {
      for (const sz of [-0.020, 0.024]) {
        const pin = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0028, 0.0028, 0.006, 10), matBrass));
        pin.rotation.z = Math.PI / 2;
        pin.position.set(0.116, sy, sz);
        robotBase.add(pin);
      }
    }

    // Non-mirrored Glowing OLED Screen Plane
    // Right-handed basis (det = +1): +u=(0,+1,0) [viewer right], +v=(0,0,+1) [up], +n=(+1,0,0) [front +X]
    const oledPlane = trackVis(new THREE.Mesh(
      new THREE.PlaneGeometry(0.088, 0.044),
      new THREE.MeshStandardMaterial({
        map: oledTexture,
        emissiveMap: oledTexture,
        emissive: new THREE.Color(0xffffff),
        emissiveIntensity: 1.50,
        roughness: 0.14,
        metalness: 0.05,
      })
    ));
    const oledBasis = new THREE.Matrix4().makeBasis(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 1),
      new THREE.Vector3(1, 0, 0)
    );
    oledPlane.quaternion.setFromRotationMatrix(oledBasis);
    // oledPcb front beveled surface ends at x = 0.112 + 0.006 + 0.003 = 0.121 -> place OLED proud at 0.1235!
    oledPlane.position.set(0.1235, 0, 0.002);
    robotBase.add(oledPlane);

    // Cute Pink Decorative Ear-Like Accents Near the Front OLED Face
    const earGroupL = new THREE.Group(), earGroupR = new THREE.Group();
    function buildPinkEar(sideSign) {
      const g = new THREE.Group();
      g.position.set(0.094, sideSign * 0.044, 0.036);
      g.rotation.set(sideSign * 0.22, -0.10, sideSign * 0.10);

      // Small bronze outer backing ear
      const os = new THREE.Shape();
      os.moveTo(-0.015, 0); os.lineTo(0.015, 0); os.lineTo(0, 0.032); os.closePath();
      const og = new THREE.ExtrudeGeometry(os, { depth: 0.008, bevelEnabled: true, bevelSize: 0.0025, bevelThickness: 0.0025, bevelSegments: 2 });
      og.center();
      const om = trackVis(new THREE.Mesh(og, matBronzeDk));
      om.quaternion.setFromRotationMatrix(oledBasis);
      om.position.z = 0.014; om.castShadow = true;
      g.add(om);

      // Bright pink decorative front ear face
      const is = new THREE.Shape();
      is.moveTo(-0.011, 0); is.lineTo(0.011, 0); is.lineTo(0, 0.023); is.closePath();
      const ig = new THREE.ExtrudeGeometry(is, { depth: 0.003, bevelEnabled: true, bevelSize: 0.001, bevelThickness: 0.001, bevelSegments: 2 });
      ig.center();
      const im = trackVis(new THREE.Mesh(ig, matPinkEar));
      im.quaternion.setFromRotationMatrix(oledBasis);
      im.position.set(0.006, 0, 0.013);
      g.add(im);
      return g;
    }
    earGroupL.add(buildPinkEar(1));
    earGroupR.add(buildPinkEar(-1));
    robotBase.add(earGroupL);
    robotBase.add(earGroupR);

    // Small rear power switch / wire exit grommet detail
    const rearPort = makeRoundedBox(0.008, 0.042, 0.018, 0.002, matServo);
    rearPort.position.set(-0.110, 0, 0.008);
    robotBase.add(rearPort);

    // Base Collision Box (exact URDF: 0.220 x 0.132 x 0.068) & Skeleton Spine
    addCollisionBox(robotBase, 0.220, 0.132, 0.068, 0, 0, 0);
    robotBase.add(trackSkel(new THREE.Mesh(new THREE.BoxGeometry(0.160, 0.010, 0.010), matSkeletonBone)));
    robotBase.add(trackSkel(new THREE.AxesHelper(0.10)));

    // -----------------------------------------------------------------------
    // 4 Articulated Legs (Exact URDF Origins: x=+/-0.080, y=+/-0.092, z=-0.004)
    // Upper Leg L1 = 0.080 m | Lower Leg L2 = 0.082 m | Foot Sphere r = 0.016 m
    // -----------------------------------------------------------------------
    const legSpecs = [
      { name: "FL", x:  0.080, y:  0.092, side:  1, hipIdx: 0, kneeIdx: 1 },
      { name: "FR", x:  0.080, y: -0.092, side: -1, hipIdx: 2, kneeIdx: 3 },
      { name: "RL", x: -0.080, y:  0.092, side:  1, hipIdx: 4, kneeIdx: 5 },
      { name: "RR", x: -0.080, y: -0.092, side: -1, hipIdx: 6, kneeIdx: 7 },
    ];
    const jointPivots = new Array(8);

    for (const leg of legSpecs) {
      // Visible Hip Micro-Servo mounted right against the brown body side
      const hipServo = buildMicroServo(leg.side);
      hipServo.position.set(leg.x, leg.side * 0.074, -0.004);
      robotBase.add(hipServo);

      // Exposed Bright Orange / Yellow / Brown 3-Wire Servo Harness from top body slot to hip servo!
      [matWireOrg, matWireYel, matWireBrn].forEach((wm, wi) => {
        const off = (wi - 1) * 0.0028;
        const curve = new THREE.CatmullRomCurve3([
          new THREE.Vector3(leg.x * 0.65, leg.side * 0.048, 0.032 + off),
          new THREE.Vector3(leg.x * 0.85, leg.side * 0.076, 0.038 + off),
          new THREE.Vector3(leg.x * 0.96, leg.side * 0.084, 0.016 + off),
          new THREE.Vector3(leg.x,        leg.side * 0.078, 0.004 + off),
        ]);
        const wire = trackVis(new THREE.Mesh(new THREE.TubeGeometry(curve, 14, 0.0016, 6, false), wm));
        wire.castShadow = true;
        robotBase.add(wire);
      });

      // Skeleton shoulder strut
      const shBone = trackSkel(new THREE.Mesh(new THREE.BoxGeometry(0.008, Math.abs(leg.y), 0.008), matSkeletonBone));
      shBone.position.set(leg.x, leg.y / 2, -0.004);
      robotBase.add(shBone);

      // HIP PIVOT (exact URDF joint origin & Y-axis pitch)
      const hipPivot = new THREE.Group();
      hipPivot.position.set(leg.x, leg.y, -0.004);
      robotBase.add(hipPivot);
      jointPivots[leg.hipIdx] = hipPivot;

      // White DIY servo horn + center steel screw at hip pivot
      const hipHorn = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.010, 20), matHorn));
      hipHorn.position.set(0, -leg.side * 0.003, 0);
      hipPivot.add(hipHorn);
      const hipScrew = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.022, 12), matScrew));
      hipPivot.add(hipScrew);

      // Sculpted Short 3D-Printed Upper Leg Linkage (L1 = 0.080 m)
      const upperArm = makeRoundedBox(0.024, 0.014, 0.086, 0.004, matLinkTan);
      upperArm.position.set(0, 0, -0.040);
      hipPivot.add(upperArm);

      // Mechanical side reinforcement bar (gives visible linkage look!)
      const linkRod = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.072, 10), matBronzeDk));
      linkRod.rotation.x = Math.PI / 2;
      linkRod.position.set(0.009, leg.side * 0.009, -0.040);
      hipPivot.add(linkRod);

      // Visible Knee Micro-Servo Motor mounted flush on inner side of knee pivot (z = -0.078)
      const kneeServo = buildMicroServo(leg.side);
      kneeServo.position.set(0, -leg.side * 0.011, -0.078);
      hipPivot.add(kneeServo);

      // Exposed Orange & Yellow Servo Wire running neatly down the upper leg to the knee servo!
      [matWireOrg, matWireYel].forEach((wm, wi) => {
        const off = (wi - 0.5) * 0.003;
        const kCurve = new THREE.CatmullRomCurve3([
          new THREE.Vector3(-0.008 + off, -leg.side * 0.008, -0.006),
          new THREE.Vector3(-0.017 + off, -leg.side * 0.012, -0.042),
          new THREE.Vector3(-0.010 + off, -leg.side * 0.010, -0.074),
        ]);
        hipPivot.add(trackVis(new THREE.Mesh(new THREE.TubeGeometry(kCurve, 12, 0.0015, 6, false), wm)));
      });

      // Upper Collision Box & Skeleton
      addCollisionBox(hipPivot, 0.024, 0.018, 0.074, 0, 0, -0.040);
      hipPivot.add(trackSkel(new THREE.Mesh(new THREE.SphereGeometry(0.011, 14, 14), matSkeletonJoint)));
      const thighBone = trackSkel(new THREE.Mesh(new THREE.CylinderGeometry(0.0045, 0.0045, 0.080, 10), matSkeletonBone));
      thighBone.rotation.x = Math.PI / 2; thighBone.position.set(0, 0, -0.040);
      hipPivot.add(thighBone);

      // KNEE PIVOT (exact URDF origin z = -0.080, Y-axis pitch)
      const kneePivot = new THREE.Group();
      kneePivot.position.set(0, 0, -0.080);
      hipPivot.add(kneePivot);
      jointPivots[leg.kneeIdx] = kneePivot;

      // White knee servo horn + pivot screw
      const kneeHorn = trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.009, 18), matHorn));
      kneeHorn.position.set(0, leg.side * 0.007, 0);
      kneePivot.add(kneeHorn);
      kneePivot.add(trackVis(new THREE.Mesh(new THREE.CylinderGeometry(0.0032, 0.0032, 0.028, 12), matScrew)));

      // Short Tapered 3D-Printed Lower Leg / Shin (L2 = 0.082 m)
      const lowerArm = makeRoundedBox(0.019, 0.013, 0.078, 0.004, matBronze);
      lowerArm.position.set(0, leg.side * 0.008, -0.040);
      kneePivot.add(lowerArm);

      // Small Rounded Dark Rubber Foot Pad (r = 0.016 m matching URDF!)
      const foot = trackVis(new THREE.Mesh(new THREE.SphereGeometry(0.016, 20, 20), matPaw));
      foot.position.set(0, leg.side * 0.006, -0.082);
      foot.castShadow = true; foot.receiveShadow = true;
      kneePivot.add(foot);

      // Lower Collision & Skeleton
      addCollisionBox(kneePivot, 0.018, 0.016, 0.068, 0, 0, -0.038);
      addCollisionSphere(kneePivot, 0.016, 0, 0, -0.082);
      kneePivot.add(trackSkel(new THREE.Mesh(new THREE.SphereGeometry(0.010, 14, 14), matSkeletonJoint)));
      const shinBone = trackSkel(new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.082, 10), matSkeletonBone));
      shinBone.rotation.x = Math.PI / 2; shinBone.position.set(0, 0, -0.041);
      kneePivot.add(shinBone);
      const footNode = trackSkel(new THREE.Mesh(new THREE.SphereGeometry(0.010, 12, 12), new THREE.MeshBasicMaterial({ color: 0x10b981 })));
      footNode.position.set(0, 0, -0.082);
      kneePivot.add(footNode);
    }

    // -----------------------------------------------------------------------
    // 5. Engineering View Modes (Normal is strictly DEFAULT)
    // -----------------------------------------------------------------------
    let currentViewMode = "normal";
    function setViewMode(mode) {
      currentViewMode = mode || "normal";
      const showVis  = (currentViewMode !== "skeleton");
      const isWire   = (currentViewMode === "wireframe");
      const isTrans  = (currentViewMode === "transparent");
      const showSkel = (currentViewMode === "skeleton" || currentViewMode === "transparent");
      const showCol  = (currentViewMode === "collision" || currentViewMode === "transparent");

      visualMeshes.forEach((m) => { m.visible = showVis; });
      visualMaterials.forEach((mat) => {
        mat.wireframe = isWire;
        mat.transparent = isTrans;
        mat.opacity = isTrans ? 0.30 : 1.0;
        mat.needsUpdate = true;
      });
      skeletonObjects.forEach((o) => { o.visible = showSkel; });
      collisionObjects.forEach((o) => { o.visible = showCol; });
    }
    setViewMode("normal");

    // -----------------------------------------------------------------------
    // 6. Camera Orbit, Pan, Zoom & Full-Body Studio Presets for Compact Robot
    // -----------------------------------------------------------------------
    const CAM_PRESETS = {
      hero:  { az: 0.65, el: 0.28, dist: 0.72 },
      front: { az: 0.01, el: 0.14, dist: 0.60 },
      rear:  { az: 3.13, el: 0.20, dist: 0.64 },
      side:  { az: 1.57, el: 0.14, dist: 0.76 },
      top:   { az: 0.01, el: 1.44, dist: 0.80 },
      face:  { az: 0.12, el: 0.12, dist: 0.38 },
    };
    let camAz = CAM_PRESETS.hero.az, camEl = CAM_PRESETS.hero.el, camDist = CAM_PRESETS.hero.dist;
    let followRobot = true;
    const panOffset = new THREE.Vector3(0, 0, 0);
    const camTarget = new THREE.Vector3(0.01, 0.11, 0.0);

    let dragMode = null, lastMX = 0, lastMY = 0;
    canvas3d.addEventListener("contextmenu", (e) => e.preventDefault());
    canvas3d.addEventListener("mousedown", (e) => {
      dragMode = (e.button === 2 || e.shiftKey) ? "pan" : "orbit";
      lastMX = e.clientX; lastMY = e.clientY;
    });
    window.addEventListener("mouseup", () => { dragMode = null; });
    window.addEventListener("mousemove", (e) => {
      if (!dragMode) return;
      const dx = e.clientX - lastMX, dy = e.clientY - lastMY;
      lastMX = e.clientX; lastMY = e.clientY;
      if (dragMode === "orbit") {
        camAz -= dx * 0.008;
        camEl = Math.max(0.04, Math.min(1.48, camEl + dy * 0.008));
      } else {
        const right = new THREE.Vector3(-Math.sin(camAz), 0, Math.cos(camAz));
        panOffset.addScaledVector(right, -dx * 0.0016 * camDist);
        panOffset.y += dy * 0.0016 * camDist;
      }
    });
    canvas3d.addEventListener("wheel", (e) => {
      e.preventDefault();
      camDist = Math.max(0.22, Math.min(1.8, camDist + e.deltaY * 0.0005));
    }, { passive: false });

    function setCameraPreset(name) {
      const p = CAM_PRESETS[name] || CAM_PRESETS.hero;
      camAz = p.az; camEl = p.el; camDist = p.dist;
      panOffset.set(0, 0, 0);
    }
    function setFollow(enabled) { followRobot = !!enabled; }
    function resetCamera() { setCameraPreset("hero"); followRobot = true; }

    function resize() {
      const parent = canvas3d.parentElement;
      if (!parent || parent.clientWidth <= 2 || parent.clientHeight <= 2) return;
      renderer.setSize(parent.clientWidth, parent.clientHeight, false);
      camera.aspect = parent.clientWidth / parent.clientHeight;
      camera.updateProjectionMatrix();
    }
    window.addEventListener("resize", resize);
    resize();

    // -----------------------------------------------------------------------
    // 7. 60 FPS Render Loop Synced to Exact PyBullet 6-DOF Pose & 8 Joints
    // -----------------------------------------------------------------------
    let latestState = {
      state: "IDLE",
      base_pos: [0, 0, 0.178],
      base_quat: [0, 0, 0, 1],
      actual_angles_rad: [0, 0, 0, 0, 0, 0, 0, 0],
    };
    const clock = new THREE.Clock();
    const targetPos = new THREE.Vector3();
    const targetQuat = new THREE.Quaternion();

    function animate() {
      requestAnimationFrame(animate);
      const t = clock.getElapsedTime();
      renderOLEDFace(latestState.state || "IDLE", t);
      oledTexture.needsUpdate = true;

      const bp = latestState.base_pos || [0, 0, 0.178];
      const bq = latestState.base_quat || [0, 0, 0, 1];
      targetPos.set(bp[0], bp[1], bp[2]);
      targetQuat.set(bq[0], bq[1], bq[2], bq[3]);
      robotBase.position.lerp(targetPos, 0.50);
      robotBase.quaternion.slerp(targetQuat, 0.50);

      const angs = latestState.actual_angles_rad || [0,0,0,0,0,0,0,0];
      for (let i = 0; i < 8; i++) {
        jointPivots[i].rotation.y += ((angs[i] || 0) - jointPivots[i].rotation.y) * 0.60;
      }

      if (latestState.state === "WAVE" || latestState.state === "DANCE") {
        earGroupL.rotation.x = Math.sin(t * 9) * 0.08;
        earGroupR.rotation.x = -Math.sin(t * 9) * 0.08;
      } else {
        earGroupL.rotation.x *= 0.9;
        earGroupR.rotation.x *= 0.9;
      }

      if (followRobot) {
        const goal = new THREE.Vector3(robotBase.position.x, 0.105, -robotBase.position.y).add(panOffset);
        camTarget.lerp(goal, 0.28);
      } else {
        camTarget.lerp(new THREE.Vector3(0.0, 0.105, 0).add(panOffset), 0.28);
      }

      const cx = camTarget.x + camDist * Math.cos(camEl) * Math.cos(camAz);
      const cy = camTarget.y + camDist * Math.sin(camEl);
      const cz = camTarget.z + camDist * Math.cos(camEl) * Math.sin(camAz);
      camera.position.set(cx, cy, cz);
      camera.lookAt(camTarget);

      keyLight.position.set(camTarget.x + 1.0, 2.0, camTarget.z + 1.3);
      keyLight.target.position.copy(camTarget);
      keyLight.target.updateMatrixWorld();

      renderer.render(scene, camera);
    }
    animate();

    return {
      updateFromSnapshot(snap) { latestState = snap; },
      setViewMode,
      setCameraPreset,
      setFollow,
      resetCamera,
      resize,
      getViewMode: () => currentViewMode,
    };
  }

  return { createScene, STATE_COLORS };
})();
