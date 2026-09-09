/* handfly: the hero instrument. The reader flies the 1-D hover rocket (model A) by hand with a
   throttle slider and tries to hold 10 m. A PD autopilot can take over for contrast. */
(function () {
  'use strict';
  const G = window.GTPL;

  /* ------------------------------------------------------------------ */
  /* pure math: no DOM here. Registered on GTPL.math.handfly for tests.  */
  /* ------------------------------------------------------------------ */
  const params = {
    m: 1.0,            // kg
    g: 9.81,           // m/s^2
    Tmin: 1.0,         // N   flameout floor
    Tmax: 20.0,        // N
    tau: 0.20,         // s   first-order thrust lag
    dt: 1 / 240,       // s   physics step; the controller runs every step
    target: 10.0,      // m
    band: 0.5,         // m   "on target" means |z - target| <= band
    noiseZ: 0.02,      // m   1-sigma noise on the displayed altitude only
    gust: { N: -0.6, T: 2.0 },              // downward force for 2 s (simspec -30 N on 50 kg, rescaled per kg)
    pd: { Kp: 2.0, Kd: 3.6, tauD: 0.05 },   // simspec S1 autopilot 100 N/m, 180 N s/m on 50 kg, rescaled per kg
    duration: 30.0,    // s   reference run length for the scoring check
  };
  const clamp = G.clamp;

  function hoverThrust(p) { return p.m * p.g; }
  function hoverThrottle(p) { return (hoverThrust(p) - p.Tmin) / (p.Tmax - p.Tmin); }
  function throttleToThrust(u, p) { return p.Tmin + clamp(u, 0, 1) * (p.Tmax - p.Tmin); }
  function thrustToThrottle(T, p) { return clamp((T - p.Tmin) / (p.Tmax - p.Tmin), 0, 1); }

  /* rocket on the pad, engine at the floor: nothing happens until the throttle moves */
  function makeState(p) { return { z: 0, v: 0, T: p.Tmin, t: 0 }; }

  /* semi-implicit Euler, exact first-order lag for a held command. dist: extra vertical force [N]. */
  function step(s, Tcmd, dt, p, dist) {
    const Tc = clamp(Tcmd, p.Tmin, p.Tmax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    const a = (s.T - p.m * p.g + (dist || 0)) / p.m;
    s.v += a * dt;
    s.z += s.v * dt;
    if (s.z <= 0) { s.z = 0; if (s.v < 0) s.v = 0; }
    s.t += dt;
    return s;
  }
  function finite(s) { return isFinite(s.z) && isFinite(s.v) && isFinite(s.T) && isFinite(s.t); }
  function inBand(z, p) { return Math.abs(z - p.target) <= p.band; }

  /* PD autopilot with gravity feedforward. Derivative of the measurement, low-passed (tauD).
     Returns the unclipped thrust command in N; the caller clips to [Tmin, Tmax]. */
  function makeAutopilot(p) {
    let prevZ = null, vf = 0;
    return {
      reset() { prevZ = null; vf = 0; },
      update(meas, dt) {
        const v = prevZ === null ? 0 : (meas - prevZ) / dt;
        prevZ = meas;
        vf += (v - vf) * dt / (p.pd.tauD + dt);
        return p.m * p.g + p.pd.Kp * (p.target - meas) - p.pd.Kd * vf;
      },
      get vFilt() { return vf; },
    };
  }

  /* --- scenario runners used by the node test (and nothing else) --- */
  /* autopilot from the pad for `duration` s: peak, time it enters the band for good, fraction in band, error after 10 s */
  function runAutopilot(p, opts) {
    opts = opts || {};
    const dt = p.dt, tEnd = opts.tEnd || p.duration;
    const s = makeState(p), ap = makeAutopilot(p);
    let peak = 0, tIn = 0, lastOut = 0, maxErrLate = 0, n = 0, Tcmd = p.Tmin, minT = Infinity, maxT = -Infinity;
    const steps = Math.round(tEnd / dt);
    for (let i = 0; i < steps; i++) {
      Tcmd = clamp(ap.update(s.z, dt), p.Tmin, p.Tmax);
      if (Tcmd < minT) minT = Tcmd; if (Tcmd > maxT) maxT = Tcmd;
      step(s, Tcmd, dt, p, 0);
      n++;
      if (s.z > peak) peak = s.z;
      if (inBand(s.z, p)) tIn += dt; else lastOut = s.t;
      if (s.t > 10 && Math.abs(s.z - p.target) > maxErrLate) maxErrLate = Math.abs(s.z - p.target);
    }
    return { peak, tEnterForGood: lastOut, fracInBand: tIn / (n * dt), maxErrAfter10: maxErrLate, zEnd: s.z, vEnd: s.v, minT, maxT };
  }
  /* the naive hand strategy: full throttle until 10 m, then floor throttle until the climb stops, then hover thrust */
  function runBangBang(p, opts) {
    opts = opts || {};
    const dt = p.dt, tEnd = opts.tEnd || p.duration;
    const s = makeState(p);
    let phase = 0, peak = 0, tPeak = 0, vAtTarget = 0;
    const steps = Math.round(tEnd / dt);
    for (let i = 0; i < steps; i++) {
      if (phase === 0 && s.z >= p.target) { phase = 1; vAtTarget = s.v; }
      if (phase === 1 && s.v <= 0) phase = 2;
      const Tcmd = phase === 0 ? p.Tmax : phase === 1 ? p.Tmin : hoverThrust(p);
      step(s, Tcmd, dt, p, 0);
      if (s.z > peak) { peak = s.z; tPeak = s.t; }
    }
    return { peak, overshoot: peak - p.target, tPeak, vAtTarget };
  }
  /* autopilot holding a hover, gust applied from t = t0 for gust.T seconds */
  function runGust(p, opts) {
    opts = opts || {};
    const dt = p.dt, tEnd = opts.tEnd || 12, t0 = opts.t0 === undefined ? 2 : opts.t0;
    const s = makeState(p); s.z = p.target; s.T = hoverThrust(p);
    const ap = makeAutopilot(p);
    let minZ = Infinity, tBack = null, worst = 0;
    const steps = Math.round(tEnd / dt);
    for (let i = 0; i < steps; i++) {
      const Tcmd = clamp(ap.update(s.z, dt), p.Tmin, p.Tmax);
      const gusting = s.t >= t0 && s.t < t0 + p.gust.T;
      step(s, Tcmd, dt, p, gusting ? p.gust.N : 0);
      if (s.z < minZ) minZ = s.z;
      const e = Math.abs(s.z - p.target);
      if (s.t > t0 && e > worst) worst = e;
      if (s.t >= t0 + p.gust.T) { if (e > 0.05) tBack = null; else if (tBack === null) tBack = s.t; }
    }
    return { minZ, sag: p.target - minZ, worst, tBack, zEnd: s.z };
  }

  const math = { params, hoverThrust, hoverThrottle, throttleToThrust, thrustToThrottle, makeState, step, finite, inBand,
    makeAutopilot, runAutopilot, runBangBang, runGust };
  G.math = G.math || {};
  G.math.handfly = math;

  /* ------------------------------------------------------------------ */
  /* mount: all DOM and canvas work                                      */
  /* ------------------------------------------------------------------ */
  G.sims.handfly = function mount(root) {
    const el = G.el, p = params;
    const ui = G.scaffold(root);
    const stage = G.canvas(ui.stage, { aspect: 2.1, minHeight: 300 });
    ui.stage.appendChild(G.hidden('A rocket on a launch pad with a dashed target line at 10 m and, beside it, a chart of altitude against the setpoint over the last 12 s.'));

    /* hint overlay, shown until the first input */
    const hint = el('span', {
      style: 'font-family: var(--font-ui); font-size: .8125rem; color: var(--ink-2); background: var(--bg-2); border: 1px solid var(--line); border-radius: 4px; padding: .35rem .75rem;',
      text: 'Drag the throttle',
    });
    const overlay = el('div', { class: 'stage-overlay' }, [hint]);
    ui.stage.appendChild(overlay);

    const chart = new G.StripChart({
      duration: 12, autoscale: true, floor: 0, ymin: 0, ymax: 12, padding: 0.12,
      yLabel: 'altitude (m)', xLabel: 't (s)',
      series: [
        { key: 'ref', color: '--sig-ref', dash: [6, 5], label: 'setpoint', endDot: false },
        { key: 'z', color: '--sig-act', label: 'altitude' },
      ],
      thresholds: [],
    });

    /* HUD */
    const roAlt = G.readout({ label: 'Altitude', unit: 'm', digits: 2 });
    const roV = G.readout({ label: 'Vertical speed', unit: 'm/s', digits: 2 });
    const roT = G.readout({ label: 'Thrust', unit: '%', digits: 0 });
    const roScore = G.readout({ label: 'Time on target', unit: 's', digits: 1 });
    const roTime = G.readout({ label: 'Time', unit: 's', digits: 1 });
    ui.hud.append(roAlt.root, roV.root, roT.root, roScore.root, roTime.root);

    /* state */
    let s = makeState(p);
    let throttle = 0;                 // 0..1, what the slider says
    let Tcmd = p.Tmin;                // N, clipped command sent to the engine
    let auto = false;
    const ap = makeAutopilot(p);
    let gustLeft = 0;
    let score = 0;
    let k = 0;
    let lastStacked = null;           // layout mode of the last frame (wide: stage left, chart right)
    const rand = G.rng(11);           // flame flicker
    const noiseRand = G.rng(23);      // displayed-altitude noise
    const pushObj = { ref: p.target, z: 0 };

    /* controls */
    const hoverPct = Math.round(hoverThrottle(p) * 100);
    const sl = G.slider({ label: 'Throttle', unit: '%', min: 0, max: 100, step: 1, value: 0, digits: 0, vertical: true,
      onInput: (v) => { throttle = v / 100; touched(); } });
    sl.input.style.alignSelf = 'center';
    sl.input.setAttribute('aria-label', 'Throttle, percent');
    /* the vertical range's percentage height has nothing to resolve against; size it from the stage */
    const fitSlider = (w, h) => { sl.input.style.height = Math.round(clamp(h - 120, 160, 230)) + 'px'; };
    stage.onResize(fitSlider); fitSlider(stage.width, stage.height);
    const hoverNote = el('div', { class: 'legend', text: 'hover thrust ' + G.fmt(hoverThrust(p), 2) + ' N = ' + hoverPct + ' %' });
    const tog = G.toggle({ label: 'Let the computer fly', value: false, onChange: (v) => { setAuto(v); touched(); } });
    const bGust = G.button({ label: 'Gust', kind: 'primary', onClick: () => { gustLeft = p.gust.T; touched(); } });
    const bReset = G.button({ label: 'Reset', onClick: reset });
    const bPlay = G.button({ label: 'Pause', kind: 'ghost', onClick: () => loop.toggle() });
    ui.controls.append(
      G.group('Throttle', [sl.root, hoverNote]),
      G.group('Autopilot', [tog.root]),
      el('div', { class: 'btn-row' }, [bGust.root, bReset.root, bPlay.root]),
    );
    ui.foot.append(el('p', { class: 'caption', text: 'Toy rocket: 1 kg, thrust 1 to 20 N, 0.2 s thrust lag. Target 10 m, counted as on target within 0.5 m.' }));

    function touched() { overlay.hidden = true; }
    function setAuto(v) {
      auto = !!v;
      if (auto) { ap.reset(); sl.input.disabled = true; }
      else { sl.input.disabled = false; }
      tog.set(auto, true);
    }

    /* physics */
    function stepSim(dt) {
      if (auto) {
        Tcmd = clamp(ap.update(s.z, dt), p.Tmin, p.Tmax);
        throttle = thrustToThrottle(Tcmd, p);
      } else {
        Tcmd = throttleToThrust(throttle, p);
      }
      let dist = 0;
      if (gustLeft > 0) { dist = p.gust.N; gustLeft -= dt; }
      step(s, Tcmd, dt, p, dist);
      if (!finite(s)) { reset(); return; }
      if (inBand(s.z, p)) score += dt;
      if ((++k & 3) === 0) { pushObj.z = s.z; chart.push(s.t, pushObj); }
    }

    /* drawing */
    function render() {
      const ctx = stage.ctx, W = stage.width, H = stage.height, g = G.theme.get;
      stage.clear();
      const stacked = W < 520;
      if (stacked !== lastStacked) { lastStacked = stacked; overlay.style.right = stacked ? '' : '62%'; }
      let sr, cr;
      if (stacked) {
        sr = { x: 0, y: 0, w: W, h: Math.round(H * 0.56) };
        cr = { x: 4, y: sr.h + 4, w: W - 8, h: H - sr.h - 8 };
      } else {
        sr = { x: 0, y: 0, w: Math.round(W * 0.38), h: H };
        cr = { x: sr.w + 6, y: 4, w: W - sr.w - 10, h: H - 8 };
      }
      drawStage(ctx, sr);
      /* divider */
      ctx.save(); ctx.strokeStyle = g('--line'); ctx.lineWidth = 1; ctx.beginPath();
      if (stacked) { ctx.moveTo(0, sr.h + 0.5); ctx.lineTo(W, sr.h + 0.5); } else { ctx.moveTo(sr.w + 0.5, 0); ctx.lineTo(sr.w + 0.5, H); }
      ctx.stroke(); ctx.restore();
      chart.draw(ctx, cr);

      if (auto) sl.set(Math.round(throttle * 100), true);
      const on = inBand(s.z, p);
      roAlt.set(Math.max(0, s.z + p.noiseZ * G.gauss(noiseRand)), on ? 'good' : '');
      roV.set(s.v);
      roT.set(thrustToThrottle(s.T, p) * 100);
      roScore.set(score, on ? 'good' : '');
      roTime.set(s.t);
    }

    function drawStage(ctx, r) {
      const g = G.theme.get;
      const top = r.y + 18, y0 = r.y + r.h - 26;
      const zTop = 22;                        // m visible above the pad
      const mpp = zTop / (y0 - top);          // meters per px
      const zy = (z) => y0 - z / mpp;
      const scale = clamp(r.h * 0.17, 40, 64);
      const rx = r.x + r.w * 0.58;
      /* target band */
      ctx.save();
      ctx.fillStyle = g('--gold-soft');
      ctx.fillRect(r.x, zy(p.target + p.band), r.w, zy(p.target - p.band) - zy(p.target + p.band));
      ctx.restore();
      G.drawGround(ctx, r, { y0, mpp, ticks: [0, 5, 10, 15, 20] });
      G.drawSetpoint(ctx, r, { y: zy(p.target), label: 'target ' + G.fmt(p.target, 0) + ' m' });
      /* rocket, or an off-frame marker when it has left the stage */
      const y = zy(s.z) - 0.16 * scale;
      if (y > top + 0.4 * scale) {
        G.drawRocket(ctx, { x: rx, y, scale, thrust01: s.T / p.Tmax, tilt: 0, gimbal: 0, rand });
      } else {
        ctx.save();
        ctx.fillStyle = g('--sig-act'); ctx.strokeStyle = g('--sig-act'); ctx.lineWidth = 2; ctx.lineJoin = 'round';
        ctx.beginPath(); ctx.moveTo(rx - 9, top + 14); ctx.lineTo(rx, top + 2); ctx.lineTo(rx + 9, top + 14); ctx.stroke();
        ctx.font = '11px ' + g('--font-mono'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ctx.fillText(G.fmt(s.z, 1) + ' m', rx, top + 20);
        ctx.restore();
      }
      /* gust arrows */
      if (gustLeft > 0) {
        ctx.save();
        ctx.strokeStyle = g('--sig-fb'); ctx.fillStyle = g('--sig-fb'); ctx.lineWidth = 1.5; ctx.lineCap = 'round';
        const ay = clamp(zy(s.z) - 0.9 * scale, top + 4, y0 - 40), ax = rx - 0.55 * scale;
        for (let i = 0; i < 3; i++) {
          const x = ax - i * 10, yb = ay + 22 + (i % 2) * 6;
          ctx.beginPath(); ctx.moveTo(x, ay + (i % 2) * 6); ctx.lineTo(x, yb); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(x - 3, yb - 4); ctx.lineTo(x, yb); ctx.lineTo(x + 3, yb - 4); ctx.stroke();
        }
        ctx.font = '11px ' + g('--font-mono'); ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        ctx.fillText('gust ' + G.fmt(p.gust.N, 1) + ' N', ax + 6, ay - 8);
        ctx.restore();
      }
      /* thrust readout next to the pad: command vs actual */
      ctx.save();
      ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
      ctx.fillText('thrust ' + G.fmt(s.T, 1) + ' N', r.x + r.w - 6, y0 + 20);
      ctx.restore();
    }

    function reset() {
      s = makeState(p);
      throttle = 0; Tcmd = p.Tmin; gustLeft = 0; score = 0; k = 0;
      ap.reset();
      setAuto(false);
      sl.set(0, true);
      chart.clear();
      overlay.hidden = false;
      loop.renderOnce();
    }

    const loop = G.loop({ step: stepSim, render, dt: p.dt, root,
      onState: (running, wanted) => { bPlay.setLabel(wanted ? 'Pause' : 'Play'); } });
    loop.renderOnce();

    return {
      reset,
      renderOnce: () => loop.renderOnce(),
      destroy() { loop.destroy(); stage.destroy(); root.innerHTML = ''; },
    };
  };
})();
