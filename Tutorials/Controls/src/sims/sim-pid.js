/* GTPL controls guide: PID sandbox on model A (1-D hover rocket, toy numbers).
   Pure math lives on GTPL.math.pid (testable in node); all DOM/canvas work is inside mount. */
(function () {
  'use strict';
  const G = window.GTPL;

  /* ------------------------------------------------------------------ */
  /* pure math                                                           */
  /* ------------------------------------------------------------------ */
  const params = {
    m: 1.0,            // kg
    g: 9.81,           // m/s^2
    Tmin: 1.0,         // N, flameout floor
    Tmax: 20.0,        // N
    tau: 0.2,          // s, first-order thrust lag
    dt: 1 / 240,       // s, physics step; the controller runs every step
    tauD: 0.05,        // s, low-pass on the derivative signal
    iMax: 19.0,        // N, clamp on the I contribution (= Tmax - Tmin)
    gustN: -2.0,       // N, downward gust force
    gustT: 2.0,        // s, gust duration
    settleBand: 0.25,  // m
    settleHold: 2.0,   // s
    chartWindow: 12,   // s
    zMaxView: 60,      // m, safety clamp for the stage camera
  };
  const hover = params.m * params.g; // 9.81 N

  const presets = {
    ponly: { label: 'P only', Kp: 2, Ki: 0, Kd: 0, ff: false },
    pd: { label: 'PD', Kp: 2, Ki: 0, Kd: 3, ff: false },
    pid: { label: 'PID', Kp: 2, Ki: 0.5, Kd: 3, ff: false },
    pidff: { label: 'PID + FF', Kp: 2, Ki: 0.5, Kd: 3, ff: true },
  };
  const defaults = { Kp: 2, Ki: 0.5, Kd: 3, ff: false, noise: 0, dmode: 'error', ref: 10, preset: 'pid' };
  const ranges = { Kp: [0, 20], Ki: [0, 5], Kd: [0, 10], noise: [0, 0.3] };
  const setpoints = [5, 10, 15, 40];

  function makeState(z0) { return { z: z0 || 0, v: 0, T: params.Tmin, t: 0 }; }

  /* semi-implicit Euler, exact first-order lag for a held command, ground clamp */
  function stepModel(s, Tcmd, dt, p, dist) {
    p = p || params;
    const Tc = G.clamp(Tcmd, p.Tmin, p.Tmax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    const a = (s.T - p.m * p.g + (dist || 0)) / p.m;
    s.v += a * dt;
    s.z += s.v * dt;
    if (s.z <= 0) { s.z = 0; if (s.v < 0) s.v = 0; }
    s.t += dt;
    return s;
  }

  function makeController() { return { I: 0, prevE: null, prevMeas: null, dFilt: 0 }; }
  function resetDerivative(c) { c.prevE = null; c.prevMeas = null; c.dFilt = 0; }

  /* gains: {Kp, Ki, Kd, ff (bool), dmode: 'error' | 'measurement'}
     Returns the decomposition. I is kept in newtons so a Ki change never bumps the output,
     and it is clamped to +/- iMax (the team's PID_INTEGRAL_MIN/MAX style anti-windup). */
  function controller(c, gains, ref, meas, dt, p) {
    p = p || params;
    const e = ref - meas;
    let dRaw;
    if (gains.dmode === 'measurement') {
      dRaw = c.prevMeas === null ? 0 : -(meas - c.prevMeas) / dt;   // d(e)/dt with the setpoint frozen
      c.prevMeas = meas;
      c.prevE = e;
    } else {
      dRaw = c.prevE === null ? 0 : (e - c.prevE) / dt;
      c.prevE = e;
      c.prevMeas = meas;
    }
    c.dFilt += (dRaw - c.dFilt) * (dt / (p.tauD + dt));
    const FF = gains.ff ? p.m * p.g : 0;
    const P = gains.Kp * e;
    const D = gains.Kd * c.dFilt;
    if (gains.Ki > 0) c.I = G.clamp(c.I + gains.Ki * e * dt, -p.iMax, p.iMax);
    else c.I = 0;
    const I = c.I;
    const uRaw = FF + P + I + D;
    const u = G.clamp(uRaw, p.Tmin, p.Tmax);
    return { u, uRaw, P, I, D, FF, e, sat: uRaw > p.Tmax ? 1 : uRaw < p.Tmin ? -1 : 0 };
  }

  /* overshoot % of the last step, settle time to +/- band held for settleHold */
  function makeMetrics() { return { tChange: 0, z0: 0, ref: 0, step: 0, peak: 0, inBandSince: null, settle: null }; }
  function metricsMark(m, t, z, ref, isStep) {
    m.tChange = t; m.z0 = z; m.ref = ref;
    m.step = isStep ? ref - z : 0; m.peak = 0; m.inBandSince = null; m.settle = null;
  }
  function metricsStep(m, t, z, ref, p) {
    p = p || params;
    if (m.step !== 0) {
      const beyond = m.step > 0 ? z - ref : ref - z;
      if (beyond > m.peak) m.peak = beyond;
    }
    const e = ref - z;
    if (Math.abs(e) <= p.settleBand) {
      if (m.inBandSince === null) m.inBandSince = t;
      if (m.settle === null && t - m.inBandSince >= p.settleHold) m.settle = m.inBandSince - m.tChange;
    } else if (m.settle === null) m.inBandSince = null;
  }
  function overshootPct(m) { return m.step === 0 ? NaN : 100 * m.peak / Math.abs(m.step); }

  /* Offline scenario runner for tests. o: {gains, ref | refAt(t), z0, tEnd, noise, seed, dist(t), logEvery} */
  function scenario(o) {
    const p = params, dt = p.dt;
    const s = makeState(o.z0 || 0);
    if (o.hoverStart) s.T = hover;
    const c = makeController();
    const met = makeMetrics();
    const rand = G.rng(o.seed || 1);
    const gains = Object.assign({ Kp: 2, Ki: 0, Kd: 0, ff: false, dmode: 'error' }, o.gains);
    const refAt = o.refAt || (() => (o.ref === undefined ? 10 : o.ref));
    let ref = refAt(0);
    metricsMark(met, 0, s.z, ref, true);
    const log = [];
    const n = Math.round((o.tEnd || 20) / dt);
    let satCount = 0, out = null, iPeak = 0;
    for (let i = 0; i <= n; i++) {
      const r = refAt(s.t);
      if (r !== ref) { ref = r; metricsMark(met, s.t, s.z, ref, true); }
      const meas = s.z + (o.noise ? o.noise * G.gauss(rand) : 0);
      out = controller(c, gains, ref, meas, dt, p);
      if (out.sat) satCount++;
      if (Math.abs(out.I) > iPeak) iPeak = Math.abs(out.I);
      if (!o.logEvery || i % o.logEvery === 0) log.push({ t: s.t, z: s.z, v: s.v, T: s.T, ref, u: out.u, P: out.P, I: out.I, D: out.D, FF: out.FF });
      const dist = o.dist ? o.dist(s.t) : 0;
      stepModel(s, out.u, dt, p, dist);
      metricsStep(met, s.t, s.z, ref, p);
    }
    return { state: s, ctl: c, metrics: met, log, satFrac: satCount / (n + 1), iPeak, last: out };
  }

  const math = { params, hover, presets, defaults, ranges, setpoints, makeState, stepModel, makeController, resetDerivative, controller,
    makeMetrics, metricsMark, metricsStep, overshootPct, scenario };
  G.math = G.math || {};
  G.math.pid = math;

  /* ------------------------------------------------------------------ */
  /* mount                                                               */
  /* ------------------------------------------------------------------ */
  G.sims.pid = function mount(root) {
    const el = G.el;
    const ui = G.scaffold(root);
    /* two stages side by side (stack under 560px): rocket + bar | charts */
    ui.stage.className = 'stage-row';
    const stageL = el('div', { class: 'stage' });
    const stageR = el('div', { class: 'stage' });
    ui.stage.append(stageL, stageR);
    const cvL = G.canvas(stageL, { aspect: 1.15, minHeight: 280, maxHeight: 420 });
    const cvR = G.canvas(stageR, { aspect: 1.15, minHeight: 280, maxHeight: 420 });
    root.appendChild(G.hidden('Stage shows a rocket over a ground line with a dashed setpoint line, a stacked thrust bar split into feedforward, P, I and D slices with the engine limits marked, and two charts: altitude against setpoint and thrust command against its limits.'));

    /* ---- state ---- */
    const gains = { Kp: defaults.Kp, Ki: defaults.Ki, Kd: defaults.Kd, ff: defaults.ff, dmode: defaults.dmode };
    let ref = defaults.ref, noise = defaults.noise;
    let s = makeState(0), c = makeController(), met = makeMetrics();
    let rand = G.rng(11), flameRand = G.rng(3);
    let gustLeft = 0, out = controller(c, gains, ref, 0, params.dt), pushCount = 0;
    const view = { range: 20, tickStep: 5, ticks: [0, 5, 10, 15, 20] };

    /* ---- charts ---- */
    const chAlt = new G.StripChart({ duration: params.chartWindow, autoscale: true, floor: 0, padding: 0.12, yLabel: 'altitude (m)', xLabel: 't (s)',
      series: [{ key: 'ref', color: '--sig-ref', dash: [6, 5], label: 'setpoint', endDot: false }, { key: 'z', color: '--sig-act', label: 'altitude' }] });
    const chThr = new G.StripChart({ duration: params.chartWindow, ymin: -2, ymax: 24, yLabel: 'thrust (N)', xLabel: 't (s)',
      series: [{ key: 'T', color: '--ink-3', width: 1.25, label: 'actual' }, { key: 'u', color: '--sig-act', label: 'command' }],
      thresholds: [{ y: params.Tmax, color: '--bad', label: 'max 20 N' }, { y: params.Tmin, color: '--bad', label: 'min 1 N' }] });

    /* ---- HUD ---- */
    const roAlt = G.readout({ label: 'Altitude', unit: 'm' });
    const roErr = G.readout({ label: 'Error', unit: 'm' });
    const roThr = G.readout({ label: 'Thrust', unit: 'N', digits: 1 });
    const roI = G.readout({ label: 'I term', unit: 'N', digits: 2 });
    const roOs = G.readout({ label: 'Overshoot', unit: '%', digits: 0 });
    const roSet = G.readout({ label: 'Settle time', unit: 's', digits: 1 });
    ui.hud.append(roAlt.root, roErr.root, roThr.root, roI.root, roOs.root, roSet.root);

    /* ---- controls ---- */
    const sKp = G.slider({ label: 'Kp', unit: 'N/m', min: ranges.Kp[0], max: ranges.Kp[1], step: 0.1, value: gains.Kp, digits: 1, onInput: v => { gains.Kp = v; syncPreset(); } });
    const sKi = G.slider({ label: 'Ki', unit: 'N/(m·s)', min: ranges.Ki[0], max: ranges.Ki[1], step: 0.05, value: gains.Ki, digits: 2, onInput: v => { gains.Ki = v; syncPreset(); } });
    const sKd = G.slider({ label: 'Kd', unit: 'N·s/m', min: ranges.Kd[0], max: ranges.Kd[1], step: 0.1, value: gains.Kd, digits: 1, onInput: v => { gains.Kd = v; syncPreset(); } });
    const tFF = G.toggle({ label: 'Feedforward (gravity)', value: gains.ff, onChange: v => { gains.ff = v; syncPreset(); } });
    const sNoise = G.slider({ label: 'Sensor noise', unit: 'm', min: ranges.noise[0], max: ranges.noise[1], step: 0.01, value: noise, digits: 2, onInput: v => { noise = v; } });
    const segD = G.segmented({ label: 'Derivative on', value: gains.dmode, options: [{ label: 'error', value: 'error' }, { label: 'measurement', value: 'measurement' }],
      onChange: v => { gains.dmode = v; resetDerivative(c); } });
    const segRef = G.segmented({ label: 'Setpoint', value: ref, options: setpoints.map(z => ({ label: z + ' m', value: z })),
      onChange: v => { if (v === ref) return; ref = v; metricsMark(met, s.t, s.z, ref, true); } });
    const segPreset = G.segmented({ label: 'Presets', value: defaults.preset, options: Object.keys(presets).map(k => ({ label: presets[k].label, value: k })),
      onChange: applyPreset });
    const bGust = G.button({ label: 'Gust', kind: 'primary', onClick: () => { gustLeft = params.gustT; metricsMark(met, s.t, s.z, ref, false); } });
    const bReset = G.button({ label: 'Reset', onClick: reset });
    const bPlay = G.button({ label: 'Pause', kind: 'ghost', onClick: () => loop.toggle() });
    ui.controls.append(
      G.group('Gains', [sKp.root, sKi.root, sKd.root]),
      G.group('Presets', [segPreset.root]),
      G.group('Setpoint', [segRef.root]),
      G.group('Options', [tFF.root, sNoise.root]),
      G.group('Derivative on', [segD.root]),
      el('div', { class: 'btn-row' }, [bGust.root, bReset.root, bPlay.root]),
    );
    ui.foot.append(
      el('p', { class: 'caption', text: 'Toy rocket: 1 kg, thrust limited to 1 to 20 N with a 0.2 s lag. Hovering takes 9.81 N.' }),
      G.legend([{ label: 'FF', color: '--sig-ff' }, { label: 'P', color: '--sig-p' }, { label: 'I', color: '--sig-i' }, { label: 'D', color: '--sig-d' }]),
    );

    function matchPreset() {
      for (const k in presets) {
        const p = presets[k];
        if (Math.abs(p.Kp - gains.Kp) < 1e-9 && Math.abs(p.Ki - gains.Ki) < 1e-9 && Math.abs(p.Kd - gains.Kd) < 1e-9 && p.ff === gains.ff) return k;
      }
      return null;
    }
    function syncPreset() { segPreset.set(matchPreset(), true); }
    function applyPreset(k) {
      const p = presets[k];
      if (!p) return;
      gains.Kp = p.Kp; gains.Ki = p.Ki; gains.Kd = p.Kd; gains.ff = p.ff;
      sKp.set(p.Kp, true); sKi.set(p.Ki, true); sKd.set(p.Kd, true); tFF.set(p.ff, true);
      segPreset.set(k, true);
      restart();
    }
    /* restart the run from the pad, keep the current settings */
    function restart() {
      s = makeState(0); c = makeController(); met = makeMetrics();
      rand = G.rng(11); gustLeft = 0; pushCount = 0;
      out = controller(c, gains, ref, 0, params.dt);
      metricsMark(met, 0, 0, ref, true);
      chAlt.clear(); chThr.clear();
      view.range = 20;
    }
    /* restore every default, then restart */
    function reset() {
      gains.Kp = defaults.Kp; gains.Ki = defaults.Ki; gains.Kd = defaults.Kd; gains.ff = defaults.ff; gains.dmode = defaults.dmode;
      ref = defaults.ref; noise = defaults.noise;
      sKp.set(gains.Kp, true); sKi.set(gains.Ki, true); sKd.set(gains.Kd, true); tFF.set(gains.ff, true);
      sNoise.set(noise, true); segD.set(gains.dmode, true); segRef.set(ref, true); segPreset.set(defaults.preset, true);
      restart();
    }

    /* ---- physics step ---- */
    function step(dt) {
      const meas = s.z + (noise > 0 ? noise * G.gauss(rand) : 0);
      out = controller(c, gains, ref, meas, dt, params);
      let dist = 0;
      if (gustLeft > 0) { dist = params.gustN; gustLeft -= dt; }
      stepModel(s, out.u, dt, params, dist);
      metricsStep(met, s.t, s.z, ref, params);
      if (!isFinite(s.z) || !isFinite(s.v) || !isFinite(s.T) || !isFinite(out.u)) { restart(); return; }
      if ((pushCount++ & 3) === 0) { chAlt.push(s.t, { ref, z: s.z }); chThr.push(s.t, { T: s.T, u: out.u }); }
    }

    /* ---- render ---- */
    function renderLeft() {
      const ctx = cvL.ctx, W = cvL.width, H = cvL.height;
      cvL.clear();
      const barW = W < 300 ? 28 : 36, barPad = 42;
      const barX = W - barPad - barW;
      const stage = { x: 0, y: 0, w: barX - 10, h: H };
      /* camera: fixed ground, range follows setpoint and altitude */
      const want = Math.min(params.zMaxView, Math.max(20, ref * 1.3, s.z * 1.15 + 2));
      view.range += (want - view.range) * 0.08;
      const y0 = H - 26;
      const mpp = view.range / (y0 - 44);
      const tstep = G.niceStep(view.range, 4);
      if (tstep !== view.tickStep || view.ticks[view.ticks.length - 1] < view.range - tstep) {
        view.tickStep = tstep; view.ticks.length = 0;
        for (let m = 0; m <= view.range + tstep; m += tstep) view.ticks.push(m);
      }
      G.drawGround(ctx, stage, { y0, mpp, ticks: view.ticks });
      /* start the dashed line past the tick labels so they stay readable */
      G.drawSetpoint(ctx, { x: stage.x + 44, y: stage.y, w: stage.w - 44, h: stage.h }, { y: y0 - ref / mpp, label: 'setpoint ' + ref + ' m' });
      const rocketScale = Math.min(64, H * 0.2);
      const zPx = G.clamp(y0 - s.z / mpp, 20, y0);
      G.drawRocket(ctx, { x: stage.x + stage.w * 0.5, y: zPx, scale: rocketScale, tilt: 0, gimbal: 0,
        thrust01: (s.T - params.Tmin) / (params.Tmax - params.Tmin), rand: flameRand });
      if (gustLeft > 0) {
        ctx.save();
        ctx.font = '11px ' + G.theme.get('--font-mono'); ctx.fillStyle = G.theme.get('--sig-fb'); ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
        ctx.fillText('gust 2 N down', stage.x + stage.w * 0.5, 16);
        ctx.restore();
      }
      G.drawBarStack(ctx, { x: barX, y: 6, w: barW, h: H - 12 }, {
        segments: [{ value: out.FF, color: '--sig-ff' }, { value: out.P, color: '--sig-p' }, { value: out.I, color: '--sig-i' }, { value: out.D, color: '--sig-d' }],
        min: -10, max: 30, limitLo: params.Tmin, limitHi: params.Tmax, title: 'thrust', unit: 'N', labelSide: 'right' });
    }
    function renderRight() {
      const ctx = cvR.ctx, W = cvR.width, H = cvR.height;
      cvR.clear();
      const half = Math.floor(H / 2);
      chAlt.draw(ctx, { x: 0, y: 2, w: W - 2, h: half - 4 });
      chThr.draw(ctx, { x: 0, y: half, w: W - 2, h: H - half - 2 });
    }
    function render() {
      renderLeft();
      renderRight();
      const e = ref - s.z;
      roAlt.set(s.z);
      roErr.set(e, Math.abs(e) <= params.settleBand ? 'good' : Math.abs(e) < 1 ? 'warn' : 'bad');
      roThr.set(s.T, out.sat ? 'bad' : '');
      roI.set(out.I, Math.abs(out.I) >= params.iMax - 1e-9 ? 'bad' : '');
      const os = overshootPct(met);
      roOs.set(isFinite(os) ? os : '—', os > 20 ? 'bad' : os > 5 ? 'warn' : '');
      roSet.set(met.settle === null ? '—' : met.settle, met.settle === null ? '' : 'good');
    }

    const loop = G.loop({ step, render, dt: params.dt, root, onState: (running, wanted) => { bPlay.setLabel(wanted ? 'Pause' : 'Play'); } });
    bPlay.setLabel(loop.wanted ? 'Pause' : 'Play');
    /* a resize clears the canvas bitmap; repaint so a static frame survives when the loop is paused */
    cvL.onResize(render); cvR.onResize(render);
    render();

    return {
      reset,
      loop,
      setPreset: applyPreset,
      destroy: () => { loop.destroy(); cvL.destroy(); cvR.destroy(); root.textContent = ''; },
    };
  };
})();
