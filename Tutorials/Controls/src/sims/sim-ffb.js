/* GTPL controls guide: sim "ffb" — anticipation vs correction.
   Model A hover rocket following a smooth climb reference with feedback only, feedforward only, or both.
   Pure math lives in GTPL.math.ffb (no DOM) so node tests can drive it; all DOM/canvas work is inside mount. */
(function () {
  'use strict';
  const G = window.GTPL;

  /* ------------------------------------------------------------------ */
  /* pure math                                                           */
  /* ------------------------------------------------------------------ */
  const params = {
    g: 9.81,
    mModel: 1.0,        // kg, the mass the controller believes
    Tmin: 1.0,          // N, flame-out floor
    Tmax: 20.0,         // N
    tau: 0.20,          // s, first-order thrust lag
    dt: 1 / 240,        // s, physics step; the controller runs every step
    tauD: 0.05,         // s, derivative low-pass (simspec S3)
    zTarget: 10.0,      // m, climb end
    tClimb: 6.0,        // s, climb duration
    tRun: 12.0,         // s, window over which RMS and peak error are scored
    gustN: -2.0,        // N, downward gust force
    gustT: 2.0,         // s, gust duration
    zStart: 0.0,
  };
  const defaults = { Kp: 2.0, Ki: 0.5, Kd: 3.0, massErr: 0, mode: 'both' };
  const ranges = { Kp: [0, 20], Ki: [0, 5], Kd: [0, 10], massErr: [-10, 10] };
  const modes = ['fb', 'ff', 'both'];

  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  // Minimum-jerk rest-to-rest move z0 -> z1 over T seconds (simspec section 8). Writes into `out` if given.
  function minJerk(z0, z1, T) {
    const d = z1 - z0;
    return function (t, out) {
      out = out || { z: 0, v: 0, a: 0 };
      if (t <= 0) { out.z = z0; out.v = 0; out.a = 0; return out; }
      if (t >= T) { out.z = z1; out.v = 0; out.a = 0; return out; }
      const s = t / T, s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
      out.z = z0 + d * (10 * s3 - 15 * s4 + 6 * s5);
      out.v = d * (30 * s2 - 60 * s3 + 30 * s4) / T;
      out.a = d * (60 * s - 180 * s2 + 120 * s3) / (T * T);
      return out;
    };
  }
  const reference = minJerk(params.zStart, params.zTarget, params.tClimb);

  // Feedforward: the model's guess of the thrust the plan needs.
  function feedforward(r) { return params.mModel * (params.g + r.a); }

  // Plant state. massErr in percent: the real rocket is heavier (+) or lighter (-) than the model.
  function makeState(massErr) {
    const m = params.mModel * (1 + (massErr || 0) / 100);
    return { z: params.zStart, v: 0, T: params.mModel * params.g, m: m, t: 0 };
  }
  function setMass(state, massErr) { state.m = params.mModel * (1 + (massErr || 0) / 100); }

  // simspec stepA: clamp command, exact first-order lag, symplectic Euler, ground clamp.
  function stepModel(s, Tcmd, dt, dist) {
    const Tc = clamp(Tcmd, params.Tmin, params.Tmax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / params.tau));
    const a = (s.T - s.m * params.g + (dist || 0)) / s.m;
    s.v += a * dt;
    s.z += s.v * dt;
    if (s.z <= 0) { s.z = 0; if (s.v < 0) s.v = 0; }
    s.t += dt;
    return s;
  }

  // simspec PID: derivative on the (filtered) measurement, conditional integration + I clamp.
  function PID(o) {
    this.Kp = o.Kp; this.Ki = o.Ki; this.Kd = o.Kd;
    this.tauD = o.tauD === undefined ? params.tauD : o.tauD;
    this.uMin = o.uMin; this.uMax = o.uMax;
    this.iMax = o.iMax === undefined ? (o.uMax - o.uMin) : o.iMax;
    this.out = { u: 0, P: 0, I: 0, D: 0, FF: 0, e: 0, sat: false };
    this.reset();
  }
  PID.prototype.reset = function () { this.integ = 0; this.prevMeas = null; this.vFilt = 0; };
  PID.prototype.update = function (ref, meas, dt, ff, vRef) {
    ff = ff || 0; vRef = vRef || 0;
    const e = ref - meas;
    const vRaw = this.prevMeas === null ? 0 : (meas - this.prevMeas) / dt;
    this.prevMeas = meas;
    this.vFilt += (vRaw - this.vFilt) * (dt / (this.tauD + dt));
    const P = this.Kp * e;
    const D = this.Kd * (vRef - this.vFilt);
    let I = this.Ki * this.integ;
    const uUnsat = ff + P + I + D;
    const sat = uUnsat > this.uMax ? 1 : uUnsat < this.uMin ? -1 : 0;
    if (!(sat === 1 && e > 0) && !(sat === -1 && e < 0)) this.integ += e * dt;
    if (this.Ki > 0) this.integ = clamp(this.integ, -this.iMax / this.Ki, this.iMax / this.Ki);
    I = this.Ki * this.integ;
    const raw = ff + P + I + D;
    const o = this.out;
    o.u = clamp(raw, this.uMin, this.uMax); o.P = P; o.I = I; o.D = D; o.FF = ff; o.e = e; o.sat = o.u !== raw;
    return o;
  };

  // One controller tick. mode: 'fb' | 'ff' | 'both'. Writes {u, ff, fb, P, I, D, e, sat} into `out`.
  function controller(mode, pid, r, meas, dt, out) {
    out = out || { u: 0, ff: 0, fb: 0, P: 0, I: 0, D: 0, e: 0, sat: false };
    if (mode === 'ff') {
      const ff = feedforward(r);
      out.u = clamp(ff, params.Tmin, params.Tmax);
      out.ff = ff; out.fb = 0; out.P = 0; out.I = 0; out.D = 0; out.e = r.z - meas; out.sat = out.u !== ff;
      return out;
    }
    const o = mode === 'fb' ? pid.update(r.z, meas, dt, 0, 0)
                            : pid.update(r.z, meas, dt, feedforward(r), r.v);
    out.u = o.u; out.ff = o.FF; out.fb = o.P + o.I + o.D; out.P = o.P; out.I = o.I; out.D = o.D; out.e = o.e; out.sat = o.sat;
    return out;
  }

  // A run: plant + controller + score. Same object drives the page and the tests.
  function createRun(opts) {
    opts = opts || {};
    const gains = Object.assign({ Kp: defaults.Kp, Ki: defaults.Ki, Kd: defaults.Kd }, opts.gains || {});
    const run = {
      mode: opts.mode || defaults.mode,
      gains: gains,
      massErr: opts.massErr === undefined ? defaults.massErr : opts.massErr,
      state: null, pid: new PID({ Kp: gains.Kp, Ki: gains.Ki, Kd: gains.Kd, uMin: params.Tmin, uMax: params.Tmax }),
      t: 0, ref: { z: 0, v: 0, a: 0 }, ctl: { u: 0, ff: 0, fb: 0, P: 0, I: 0, D: 0, e: 0, sat: false },
      gust: 0, sumSq: 0, tScored: 0, rms: 0, peak: 0,
    };
    restartRun(run);
    return run;
  }
  function restartRun(run) {
    run.state = makeState(run.massErr);
    run.pid.reset();
    run.t = 0; run.gust = 0; run.sumSq = 0; run.tScored = 0; run.rms = 0; run.peak = 0;
    reference(0, run.ref);
    // evaluate the controller once at t = 0 so the first frame shows the real thrust split (e = 0, so the PID state stays clean)
    controller(run.mode, run.pid, run.ref, run.state.z, params.dt, run.ctl);
    run.pid.reset();
    return run;
  }
  function setGains(run, gains) {
    if (gains.Kp !== undefined) run.pid.Kp = run.gains.Kp = gains.Kp;
    if (gains.Ki !== undefined) run.pid.Ki = run.gains.Ki = gains.Ki;
    if (gains.Kd !== undefined) run.pid.Kd = run.gains.Kd = gains.Kd;
  }
  function setMassErr(run, massErr) { run.massErr = massErr; setMass(run.state, massErr); }
  function triggerGust(run) { run.gust = params.gustT; }
  function isFiniteRun(run) {
    const s = run.state;
    return isFinite(s.z) && isFinite(s.v) && isFinite(s.T) && isFinite(run.pid.integ) && isFinite(run.pid.vFilt);
  }
  function stepRun(run, dt) {
    const s = run.state;
    reference(run.t, run.ref);
    controller(run.mode, run.pid, run.ref, s.z, dt, run.ctl);
    const dist = run.gust > 0 ? params.gustN : 0;
    if (run.gust > 0) run.gust -= dt;
    stepModel(s, run.ctl.u, dt, dist);
    run.t += dt;
    // score: tracking error over the first tRun seconds of the run
    const e = run.ref.z - s.z;
    if (run.tScored < params.tRun) {
      run.sumSq += e * e * dt; run.tScored += dt;
      run.rms = Math.sqrt(run.sumSq / run.tScored);
      const ae = Math.abs(e);
      if (ae > run.peak) run.peak = ae;
    }
    return run;
  }
  // Batch runner for tests: returns stats. opts: {mode, gains, massErr, tEnd, gustAt, massErrAt: [t, pct]}
  function simulate(opts) {
    const run = createRun(opts);
    const dt = params.dt, tEnd = opts.tEnd === undefined ? params.tRun : opts.tEnd;
    const n = Math.round(tEnd / dt);
    let gusted = false, massChanged = false, tAtPeak = 0, peakSigned = 0;
    let fbAbsSum = 0, fbCount = 0, satCount = 0, minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < n; i++) {
      if (opts.gustAt !== undefined && !gusted && run.t >= opts.gustAt) { triggerGust(run); gusted = true; }
      if (opts.massErrAt && !massChanged && run.t >= opts.massErrAt[0]) { setMassErr(run, opts.massErrAt[1]); massChanged = true; }
      stepRun(run, dt);
      const e = run.ref.z - run.state.z;
      if (Math.abs(e) > Math.abs(peakSigned)) { peakSigned = e; tAtPeak = run.t; }
      if (run.t <= params.tClimb) { fbAbsSum += Math.abs(run.ctl.fb); fbCount++; }
      if (run.ctl.sat) satCount++;
      if (run.state.z < minZ) minZ = run.state.z;
      if (run.state.z > maxZ) maxZ = run.state.z;
    }
    return {
      rms: run.rms, peak: run.peak, peakSigned, tAtPeak, finalErr: run.ref.z - run.state.z,
      finalZ: run.state.z, finalI: run.ctl.I, finalFF: run.ctl.ff, finalFB: run.ctl.fb, finalT: run.state.T,
      meanAbsFbClimb: fbCount ? fbAbsSum / fbCount : 0, satFrac: satCount / n, minZ, maxZ, run,
    };
  }

  const math = { params, defaults, ranges, modes, minJerk, reference, feedforward, makeState, setMass, stepModel, PID,
    controller, createRun, restartRun, stepRun, setGains, setMassErr, triggerGust, isFiniteRun, simulate };
  G.math = G.math || {}; G.math.ffb = math;

  /* ------------------------------------------------------------------ */
  /* mount                                                               */
  /* ------------------------------------------------------------------ */
  G.sims = G.sims || {};
  G.sims.ffb = function mount(root, opts) {
    const el = G.el;
    const ui = G.scaffold(root);
    root.appendChild(G.hidden('Stage: a hover rocket climbs to follow a dashed reference line that rises from 0 to 10 m over 6 s, beside a thrust bar split into feedforward and feedback slices and charts of altitude and thrust over time.'));
    const stage = G.canvas(ui.stage, { aspect: 1.75, minHeight: 320, maxHeight: 440 });
    const rand = G.rng(11);
    const P = params;

    const run = createRun({});
    let pushAcc = 0;

    const altChart = new G.StripChart({
      duration: P.tRun, autoscale: true, floor: 0, padding: 0.12, yLabel: 'altitude (m)', xLabel: 't (s)',
      series: [
        { key: 'ref', color: '--sig-ref', dash: [6, 5], label: 'reference' },
        { key: 'z', color: '--sig-act', label: 'altitude' },
      ],
    });
    const thrChart = new G.StripChart({
      duration: P.tRun, autoscale: true, padding: 0.12, yLabel: 'thrust (N)', xLabel: 't (s)',
      series: [
        { key: 'ff', color: '--sig-ff', label: 'feedforward' },
        { key: 'fb', color: '--sig-fb', label: 'feedback' },
        { key: 'T', color: '--sig-act', width: 1, label: 'thrust' },
      ],
      thresholds: [{ y: P.Tmin, color: '--bad', label: 'min ' + P.Tmin + ' N' }, { y: P.Tmax, color: '--bad', label: 'max ' + P.Tmax + ' N' }],
    });

    /* HUD */
    const roAlt = G.readout({ label: 'Altitude', unit: 'm' });
    const roErr = G.readout({ label: 'Error', unit: 'm' });
    const roRms = G.readout({ label: 'RMS error', unit: 'm' });
    const roPeak = G.readout({ label: 'Peak error', unit: 'm' });
    const roI = G.readout({ label: 'I term', unit: 'N' });
    const roT = G.readout({ label: 'Thrust', unit: 'N', digits: 1 });
    const roTime = G.readout({ label: 'Run time', unit: 's', digits: 1 });
    ui.hud.append(roAlt.root, roErr.root, roRms.root, roPeak.root, roI.root, roT.root, roTime.root);

    /* controls */
    const seg = G.segmented({
      label: 'Mode',
      options: [{ label: 'Feedback only', value: 'fb' }, { label: 'Feedforward only', value: 'ff' }, { label: 'Both', value: 'both' }],
      value: run.mode,
      onChange: (v) => { run.mode = v; restart(); },
    });
    const sKp = G.slider({ label: 'Kp', unit: 'N/m', min: ranges.Kp[0], max: ranges.Kp[1], step: 0.1, value: run.gains.Kp, digits: 1, onInput: v => setGains(run, { Kp: v }) });
    const sKi = G.slider({ label: 'Ki', unit: 'N/(m·s)', min: ranges.Ki[0], max: ranges.Ki[1], step: 0.05, value: run.gains.Ki, digits: 2, onInput: v => setGains(run, { Ki: v }) });
    const sKd = G.slider({ label: 'Kd', unit: 'N·s/m', min: ranges.Kd[0], max: ranges.Kd[1], step: 0.1, value: run.gains.Kd, digits: 1, onInput: v => setGains(run, { Kd: v }) });
    const sMass = G.slider({ label: 'Mass error', unit: '%', min: ranges.massErr[0], max: ranges.massErr[1], step: 1, value: run.massErr, digits: 0, onInput: v => setMassErr(run, v) });
    const bRestart = G.button({ label: 'Restart run', kind: 'primary', onClick: () => restart() });
    const bGust = G.button({ label: 'Gust', onClick: () => triggerGust(run) });
    const bReset = G.button({ label: 'Reset', onClick: () => reset() });
    const bPlay = G.button({ label: 'Pause', kind: 'ghost', onClick: () => { loop.toggle(); } });
    ui.controls.append(
      G.group('Mode', [seg.root]),
      G.group('Gains', [sKp.root, sKi.root, sKd.root]),
      G.group('Real rocket', [sMass.root]),
      el('div', { class: 'btn-row' }, [bRestart.root, bGust.root, bReset.root, bPlay.root]),
    );
    ui.foot.append(
      el('p', { class: 'caption', text: 'Same PID gains in all three modes. Toy rocket: 1 kg, thrust 1 to 20 N with a 0.2 s lag. The reference climbs from 0 to 10 m in 6 s, then holds. RMS and peak error cover the first 12 s of a run.' }),
      G.legend([{ label: 'reference', color: '--sig-ref', dash: true }, { label: 'altitude', color: '--sig-act' }, { label: 'feedforward', color: '--sig-ff' }, { label: 'feedback', color: '--sig-fb' }]),
    );

    function restart() {
      restartRun(run);
      altChart.clear(); thrChart.clear();
      pushAcc = 0;
    }
    function reset() {
      run.mode = defaults.mode; seg.set(run.mode, true);
      setGains(run, { Kp: defaults.Kp, Ki: defaults.Ki, Kd: defaults.Kd });
      sKp.set(defaults.Kp, true); sKi.set(defaults.Ki, true); sKd.set(defaults.Kd, true);
      run.massErr = defaults.massErr; sMass.set(defaults.massErr, true);
      restart();
    }

    function step(dt) {
      stepRun(run, dt);
      if (!isFiniteRun(run)) { restart(); return; }
      pushAcc += dt;
      if (pushAcc >= 1 / 60 - 1e-9) {
        pushAcc = 0;
        altChart.push(run.t, { ref: run.ref.z, z: run.state.z });
        thrChart.push(run.t, { ff: run.ctl.ff, fb: run.ctl.fb, T: run.state.T });
      }
    }

    const ticks = [0, 5, 10];
    const barSegs = [{ value: 0, color: '--sig-ff' }, { value: 0, color: '--sig-fb' }];
    const barOpts = { segments: barSegs, min: -6, max: 26, limitLo: P.Tmin, limitHi: P.Tmax, title: 'thrust', unit: 'N', labelSide: 'right' };
    const stageRect = { x: 0, y: 0, w: 0, h: 0 };
    const barRect = { x: 0, y: 8, w: 30, h: 0 };
    const chartTop = { x: 0, y: 4, w: 0, h: 0 };
    const chartBot = { x: 0, y: 0, w: 0, h: 0 };
    const rocketOpts = { x: 0, y: 0, scale: 70, tilt: 0, gimbal: 0, thrust01: 0, rand };

    function render() {
      const ctx = stage.ctx, W = stage.width, H = stage.height, g = G.theme.get;
      stage.clear();
      const s = run.state;
      /* rocket stage: left column */
      stageRect.w = Math.round(W * (W < 600 ? 0.26 : 0.30)); stageRect.h = H;
      const y0 = H - 26;
      const mpp = 13 / Math.max(60, H - 70);           // 13 m of altitude visible above the ground
      G.drawGround(ctx, stageRect, { y0, mpp, ticks });
      const yRef = clamp(y0 - run.ref.z / mpp, 12, y0);
      G.drawSetpoint(ctx, stageRect, { y: yRef, label: 'ref' });
      const scale = clamp(stageRect.w * 0.5, 44, 76);
      const yRocket = y0 - s.z / mpp - 0.16 * scale;      // feet sit on the ground line at z = 0
      const offTop = yRocket < 1.05 * scale + 16;
      rocketOpts.x = stageRect.w * 0.5;
      rocketOpts.y = offTop ? 1.05 * scale + 16 : yRocket;
      rocketOpts.scale = scale;
      rocketOpts.thrust01 = (s.T - P.Tmin) / (P.Tmax - P.Tmin);
      G.drawRocket(ctx, rocketOpts);
      if (offTop) {
        ctx.save(); ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ctx.fillText('off scale ' + G.fmt(s.z, 0) + ' m', stageRect.w * 0.5, 4);
        ctx.restore();
      }
      /* thrust bar */
      barRect.x = stageRect.w + 10; barRect.h = H - 16;
      barSegs[0].value = run.ctl.ff; barSegs[1].value = run.ctl.fb;
      G.drawBarStack(ctx, barRect, barOpts);
      /* charts: right column, stacked */
      const cx = barRect.x + barRect.w + 46;
      chartTop.x = cx; chartTop.w = W - cx - 4; chartTop.h = Math.floor(H / 2) - 4;
      chartBot.x = cx; chartBot.w = chartTop.w; chartBot.y = Math.floor(H / 2); chartBot.h = H - chartBot.y - 4;
      altChart.draw(ctx, chartTop);
      thrChart.draw(ctx, chartBot);
      /* HUD */
      const e = run.ref.z - s.z, ae = Math.abs(e);
      roAlt.set(s.z);
      roErr.set(e, ae < 0.25 ? 'good' : ae < 1 ? 'warn' : 'bad');
      roRms.set(run.rms);
      roPeak.set(run.peak);
      roI.set(run.ctl.I);
      roT.set(s.T, run.ctl.sat ? 'bad' : '');
      roTime.set(run.t);
    }

    // a resize resets the canvas bitmap; redraw so a paused sim never shows a blank stage
    stage.onResize(() => render());
    const loop = G.loop({ step, render, dt: P.dt, root, onState: (running, wanted) => { bPlay.setLabel(wanted ? 'Pause' : 'Run'); } });
    bPlay.setLabel(loop.wanted ? 'Pause' : 'Run');
    loop.renderOnce();

    return {
      reset,
      restart,
      loop,
      destroy: () => { loop.destroy(); stage.destroy(); root.textContent = ''; },
    };
  };
})();
