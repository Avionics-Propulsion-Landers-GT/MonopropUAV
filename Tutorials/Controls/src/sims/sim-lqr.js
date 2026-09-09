/* GTPL controls guide: LQR on the planar rocket (model B).
   Pure math lives in GTPL.math.lqr (no DOM); all DOM/canvas work happens in GTPL.sims.lqr(mount). */
(function () {
  'use strict';
  const G = window.GTPL;
  const DEG = Math.PI / 180;

  /* ------------------------------------------------------------------ */
  /* constants (repo toy numbers, see build brief)                       */
  /* ------------------------------------------------------------------ */
  const params = {
    m: 1.0,               // kg
    g: 9.81,              // m/s^2
    L: 1.0,               // m, CoM to nozzle pivot
    I: 0.2,               // kg m^2 about pitch
    Tmin: 1.0,            // N, flameout floor
    Tmax: 20.0,           // N
    deltaMax: 10 * DEG,   // rad, gimbal limit
    tau: 0.20,            // s, thrust lag (truth model only)
    tauServo: 0.05,       // s, gimbal servo lag (truth model only)
    dt: 1 / 240,          // s, physics step
    ctrlDiv: 5,           // controller every 5 physics steps = 48 Hz
  };
  params.dtCtrl = params.dt * params.ctrlDiv;
  params.hover = params.m * params.g;

  /* Bryson weights: 1/max^2 with 1 m, 1 m, 0.2 rad, 1 m/s, 1 m/s, 0.5 rad/s; 4 N of thrust deviation (40 % of hover, same
     acceleration budget as simspec's 200 N on 50 kg); gimbal weight 1000 as in simspec. */
  const Qbase = [1, 1, 25, 1, 1, 4];
  const Rbase = [1 / 16, 1000];
  const sliderRanges = {           // log-scale multipliers on the base weights
    posW: [0.1, 10], angleW: [0.1, 10], gimbalR: [0.03, 30], thrustR: [0.1, 100],
  };
  const defaults = { posW: 1, angleW: 1, gimbalR: 1, thrustR: 1, offset: 5, showRaw: false };
  const offsets = [5, 15, 30];
  const target = [0, 10, 0, 0, 0, 0];
  const STATE_NAMES = ['x', 'z', 'θ', 'vx', 'vz', 'ω'];
  const DISPLAY_NAMES = ['x', 'z', 'tilt', 'vx', 'vz', 'tilt rate'];   // what the K grid headers say; the charts use the same words
  const STATE_UNITS = ['m', 'm', 'deg', 'm/s', 'm/s', 'deg/s'];
  const INPUT_NAMES = ['thrust', 'gimbal'];
  const INPUT_UNITS = ['N', 'deg'];
  const SETTLE_X = 0.25, SETTLE_V = 0.25;   // m, m/s
  const NUDGE_DV = 2.0;                     // m/s lateral impulse (2 N s on 1 kg)

  /* ------------------------------------------------------------------ */
  /* small dense-matrix helpers (arrays of arrays; used outside the loop) */
  /* ------------------------------------------------------------------ */
  const zeros = (r, c) => Array.from({ length: r }, () => new Array(c).fill(0));
  const eye = (n) => { const I = zeros(n, n); for (let i = 0; i < n; i++) I[i][i] = 1; return I; };
  function mul(A, B) {
    const n = A.length, k = B.length, p = B[0].length, C = zeros(n, p);
    for (let i = 0; i < n; i++) for (let t = 0; t < k; t++) { const a = A[i][t]; if (a === 0) continue; for (let j = 0; j < p; j++) C[i][j] += a * B[t][j]; }
    return C;
  }
  const add = (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j]));
  const scale = (A, s) => A.map(r => r.map(v => v * s));
  const normInf = (A) => { let m = 0; for (const r of A) for (const v of r) { const a = Math.abs(v); if (a > m) m = a; } return m; };

  /* continuous linearization about hover: state [x z th vx vz om], input [dT (N), delta (rad)] */
  function linearize(p) {
    const A = zeros(6, 6), B = zeros(6, 2);
    A[0][3] = 1; A[1][4] = 1; A[2][5] = 1;
    A[3][2] = p.g;                     // vx' = g th (+ g delta)
    B[3][1] = p.g;
    B[4][0] = 1 / p.m;                 // vz' = dT/m
    B[5][1] = -p.L * p.m * p.g / p.I;  // om' = -(L m g / I) delta
    return { A, B };
  }
  /* exact discretization via the series for exp([[A,B],[0,0]] dt); A is nilpotent so the series terminates */
  function c2d(A, B, dt, terms) {
    terms = terms || 20;
    const n = A.length, m = B[0].length, N = n + m;
    const Mbig = zeros(N, N);
    for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) Mbig[i][j] = A[i][j] * dt; for (let j = 0; j < m; j++) Mbig[i][n + j] = B[i][j] * dt; }
    let E = eye(N), term = eye(N);
    for (let k = 1; k <= terms; k++) { term = scale(mul(term, Mbig), 1 / k); E = add(E, term); if (normInf(term) < 1e-16) break; }
    return { Ad: E.slice(0, n).map(r => r.slice(0, n)), Bd: E.slice(0, n).map(r => r.slice(n)) };
  }
  /* diagonal Q, R from the four slider multipliers */
  function weights(sl) {
    const Q = Qbase.slice(), R = Rbase.slice();
    Q[0] *= sl.posW; Q[1] *= sl.posW; Q[2] *= sl.angleW; R[0] *= sl.thrustR; R[1] *= sl.gimbalR;
    return { Q, R };
  }

  /* discrete Riccati iteration, allocation-free inner loop (Float64Array, row-major).
     Q, R may be diagonal arrays or full matrices. Returns K (m x n, arrays) with u = -K x. */
  function dlqr(Ad, Bd, Q, R, tol, maxIter) {
    tol = tol || 1e-9; maxIter = maxIter || 5000;
    const n = Ad.length, m = Bd[0].length;
    const A = new Float64Array(n * n), B = new Float64Array(n * m), Qf = new Float64Array(n * n), Rf = new Float64Array(m * m);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) A[i * n + j] = Ad[i][j];
    for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) B[i * m + j] = Bd[i][j];
    if (Array.isArray(Q[0])) { for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) Qf[i * n + j] = Q[i][j]; } else for (let i = 0; i < n; i++) Qf[i * n + i] = Q[i];
    if (Array.isArray(R[0])) { for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) Rf[i * m + j] = R[i][j]; } else for (let i = 0; i < m; i++) Rf[i * m + i] = R[i];
    let P = Qf.slice(), Pn = new Float64Array(n * n);
    const PA = new Float64Array(n * n), PB = new Float64Array(n * m), BtPA = new Float64Array(m * n), S = new Float64Array(m * m);
    const Sinv = new Float64Array(m * m), K = new Float64Array(m * n), PBK = new Float64Array(n * n);
    let it = 0;
    for (; it < maxIter; it++) {
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) { let s = 0; for (let k = 0; k < n; k++) s += P[i * n + k] * A[k * n + j]; PA[i * n + j] = s; }
      for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) { let s = 0; for (let k = 0; k < n; k++) s += P[i * n + k] * B[k * m + j]; PB[i * m + j] = s; }
      for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) { let s = Rf[i * m + j]; for (let k = 0; k < n; k++) s += B[k * m + i] * PB[k * m + j]; S[i * m + j] = s; }
      invSmall(S, m, Sinv);
      for (let i = 0; i < m; i++) for (let j = 0; j < n; j++) { let s = 0; for (let k = 0; k < n; k++) s += B[k * m + i] * PA[k * n + j]; BtPA[i * n + j] = s; }
      for (let i = 0; i < m; i++) for (let j = 0; j < n; j++) { let s = 0; for (let k = 0; k < m; k++) s += Sinv[i * m + k] * BtPA[k * n + j]; K[i * n + j] = s; }
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) { let s = 0; for (let k = 0; k < m; k++) s += PB[i * m + k] * K[k * n + j]; PBK[i * n + j] = s; }
      let d = 0, pmax = 0;
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
        let s = Qf[i * n + j];
        for (let k = 0; k < n; k++) s += A[k * n + i] * (PA[k * n + j] - PBK[k * n + j]);
        Pn[i * n + j] = s;
        const dd = Math.abs(s - P[i * n + j]); if (dd > d) d = dd;
        const aa = Math.abs(s); if (aa > pmax) pmax = aa;
      }
      const tmp = P; P = Pn; Pn = tmp;
      if (d < tol * Math.max(1, pmax)) break;
    }
    const Kout = [];
    for (let i = 0; i < m; i++) { const row = new Array(n); for (let j = 0; j < n; j++) row[j] = K[i * n + j]; Kout.push(row); }
    return { K: Kout, iters: it + 1 };
  }
  /* Gauss-Jordan inverse of a small m x m Float64Array into out */
  function invSmall(S, m, out) {
    const X = new Float64Array(m * 2 * m), w = 2 * m;
    for (let i = 0; i < m; i++) { for (let j = 0; j < m; j++) X[i * w + j] = S[i * m + j]; X[i * w + m + i] = 1; }
    for (let c = 0; c < m; c++) {
      let p = c; for (let r = c + 1; r < m; r++) if (Math.abs(X[r * w + c]) > Math.abs(X[p * w + c])) p = r;
      if (p !== c) for (let j = 0; j < w; j++) { const t = X[c * w + j]; X[c * w + j] = X[p * w + j]; X[p * w + j] = t; }
      const piv = X[c * w + c]; if (Math.abs(piv) < 1e-300) throw new Error('singular');
      for (let j = 0; j < w; j++) X[c * w + j] /= piv;
      for (let r = 0; r < m; r++) if (r !== c) { const f = X[r * w + c]; if (f !== 0) for (let j = 0; j < w; j++) X[r * w + j] -= f * X[c * w + j]; }
    }
    for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) out[i * m + j] = X[i * w + m + j];
  }

  /* one design pass: sliders -> A, B, Ad, Bd, Q, R, K */
  function design(sl, p) {
    p = p || params;
    const { A, B } = linearize(p);
    const { Ad, Bd } = c2d(A, B, p.dtCtrl);
    const { Q, R } = weights(sl);
    const { K, iters } = dlqr(Ad, Bd, Q, R);
    return { A, B, Ad, Bd, Q, R, K, iters };
  }
  /* linear closed-loop check: propagate x_{k+1} = (Ad - Bd K) x_k from every unit vector; returns the largest final norm */
  function closedLoopDecay(Ad, Bd, K, steps) {
    const n = Ad.length, M = zeros(n, n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) { let s = Ad[i][j]; for (let k = 0; k < K.length; k++) s -= Bd[i][k] * K[k][j]; M[i][j] = s; }
    let worst = 0;
    for (let e = 0; e < n; e++) {
      let x = new Array(n).fill(0); x[e] = 1;
      for (let s = 0; s < steps; s++) { const y = new Array(n).fill(0); for (let i = 0; i < n; i++) { let v = 0; for (let j = 0; j < n; j++) v += M[i][j] * x[j]; y[i] = v; } x = y; }
      const nrm = Math.sqrt(x.reduce((a, v) => a + v * v, 0));
      if (nrm > worst) worst = nrm;
    }
    return worst;
  }

  /* ------------------------------------------------------------------ */
  /* nonlinear truth model B with thrust and servo lags                   */
  /* ------------------------------------------------------------------ */
  function makeState(x0, p) {
    p = p || params;
    return { x: x0.slice(), T: p.hover, dl: 0, t: 0 };
  }
  /* u: {T (N, absolute), delta (rad)}; semi-implicit Euler; ground clamp */
  function stepModel(s, u, p, dt, windN) {
    const Tc = G.clamp(u.T, p.Tmin, p.Tmax), dc = G.clamp(u.delta, -p.deltaMax, p.deltaMax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    s.dl += (dc - s.dl) * (1 - Math.exp(-dt / p.tauServo));
    const X = s.x, ang = X[2] + s.dl;
    const ax = (s.T * Math.sin(ang) + (windN || 0)) / p.m;
    const az = s.T * Math.cos(ang) / p.m - p.g;
    const aom = -p.L * s.T * Math.sin(s.dl) / p.I;
    X[3] += ax * dt; X[4] += az * dt; X[5] += aom * dt;
    X[0] += X[3] * dt; X[1] += X[4] * dt; X[2] += X[5] * dt;
    if (X[1] <= 0) { X[1] = 0; if (X[4] < 0) X[4] = 0; }
    s.t += dt;
    return s;
  }
  /* LQR law u = -K (x - target), added to hover thrust; fills `out` (no allocation) */
  function control(K, x, tgt, p, out) {
    let dT = 0, dl = 0;
    for (let j = 0; j < 6; j++) { const e = x[j] - tgt[j]; dT -= K[0][j] * e; dl -= K[1][j] * e; }
    out.dT = dT; out.rawT = p.hover + dT; out.rawDelta = dl;
    out.T = G.clamp(out.rawT, p.Tmin, p.Tmax);
    out.delta = G.clamp(dl, -p.deltaMax, p.deltaMax);
    out.clippedT = out.T !== out.rawT; out.clippedDelta = out.delta !== dl;
    return out;
  }

  /* closed-loop simulation object shared by the page and the node test */
  function Sim(p) {
    this.p = p || params;
    this.K = null;
    this.target = new Array(6).fill(0);
    this.u = { T: this.p.hover, delta: 0, rawT: this.p.hover, rawDelta: 0, dT: 0, clippedT: false, clippedDelta: false };
    this.metrics = {};
    this.reset(defaults.offset);
  }
  Sim.prototype.reset = function (offset) {
    this.offset = offset;
    this.s = makeState([-offset, 10, 0, 0, 0, 0], this.p);
    for (let j = 0; j < 6; j++) this.target[j] = this.s.x[j];   // hold the start point until go()
    this.k = 0; this.started = false; this.tGo = 0;
    this.u.T = this.p.hover; this.u.delta = 0; this.u.rawT = this.p.hover; this.u.rawDelta = 0; this.u.dT = 0; this.u.clippedT = false; this.u.clippedDelta = false;
    this.resetMetrics();
  };
  Sim.prototype.resetMetrics = function () {
    const m = this.metrics;
    m.peakTilt = 0; m.peakGimbalCmd = 0; m.satTime = 0; m.thrustClipTime = 0; m.settleTime = null; m.minZ = this.s.x[1]; m.maxZ = this.s.x[1];
    m.firstGimbalSign = 0; m.firstTiltSign = 0; m.overshoot = 0;
  };
  Sim.prototype.go = function () {
    for (let j = 0; j < 6; j++) this.target[j] = target[j];
    this.started = true; this.tGo = this.s.t;
    this.resetMetrics();
  };
  Sim.prototype.nudge = function (dv) {
    this.s.x[3] += (dv === undefined ? NUDGE_DV : dv);
    this.metrics.settleTime = null; this.tGo = this.s.t; this.started = true;
  };
  Sim.prototype.setK = function (K) { this.K = K; };
  /* one physics step; returns false when the state went non-finite */
  Sim.prototype.tick = function (windN) {
    const p = this.p, s = this.s, m = this.metrics;
    if (this.K && this.k % p.ctrlDiv === 0) control(this.K, s.x, this.target, p, this.u);
    stepModel(s, this.u, p, p.dt, windN);
    this.k++;
    const x = s.x;
    if (!isFinite(x[0]) || !isFinite(x[1]) || !isFinite(x[2]) || !isFinite(x[3]) || !isFinite(x[4]) || !isFinite(x[5])) return false;
    const tilt = Math.abs(x[2]) / DEG, gc = Math.abs(this.u.rawDelta) / DEG;
    if (tilt > m.peakTilt) m.peakTilt = tilt;
    if (gc > m.peakGimbalCmd) m.peakGimbalCmd = gc;
    if (this.u.clippedDelta) m.satTime += p.dt;
    if (this.u.clippedT) m.thrustClipTime += p.dt;
    if (x[1] < m.minZ) m.minZ = x[1];
    if (x[1] > m.maxZ) m.maxZ = x[1];
    if (this.started) {
      if (m.firstGimbalSign === 0 && Math.abs(this.u.rawDelta) > 1e-4) m.firstGimbalSign = Math.sign(this.u.rawDelta);
      if (m.firstTiltSign === 0 && Math.abs(x[2]) > 1e-4) m.firstTiltSign = Math.sign(x[2]);
      const ex = x[0] - this.target[0];
      if (m.settleTime === null && Math.abs(ex) < SETTLE_X && Math.abs(x[3]) < SETTLE_V) m.settleTime = s.t - this.tGo;
      if (this.offset > 0 && ex > m.overshoot) m.overshoot = ex;     // moved from -offset toward 0: overshoot is x > 0
    }
    return true;
  };

  /* headless scenario for tests: {offset, sliders, tEnd, nudgeAt} -> metrics + final state */
  function runScenario(o) {
    const sim = new Sim(params);
    const d = design(Object.assign({}, defaults, o.sliders || {}));
    sim.setK(d.K);
    sim.reset(o.offset === undefined ? defaults.offset : o.offset);
    sim.go();
    const steps = Math.round((o.tEnd || 15) / params.dt);
    let ok = true, nudged = false;
    for (let i = 0; i < steps && ok; i++) {
      if (o.nudgeAt !== undefined && !nudged && sim.s.t >= o.nudgeAt) { sim.nudge(o.nudgeDv); nudged = true; }
      ok = sim.tick(o.wind ? o.wind(sim.s.t) : 0);
    }
    return { ok, metrics: sim.metrics, x: sim.s.x.slice(), t: sim.s.t, K: d.K, iters: d.iters, sim };
  }

  /* two significant digits, minus sign U+2212, tiny values shown as 0 */
  function sig2(v) {
    if (!isFinite(v)) return '—';
    if (Math.abs(v) < 1e-9) return '0';
    const s = Number(Math.abs(v).toPrecision(2)).toString();
    return (v < 0 ? '−' : '') + s;
  }
  /* K in display units: thrust row N per (m, m, deg, m/s, m/s, deg/s); gimbal row deg per the same */
  function displayK(K) {
    const colScale = [1, 1, DEG, 1, 1, DEG];          // per-unit of the column (deg columns: gain per deg = gain per rad * DEG)
    const rowScale = [1, 1 / DEG];                     // gimbal row in deg
    return K.map((row, i) => row.map((k, j) => k * colScale[j] * rowScale[i]));
  }

  const math = {
    params, Qbase, Rbase, sliderRanges, defaults, offsets, target, STATE_NAMES, STATE_UNITS, INPUT_NAMES, INPUT_UNITS,
    SETTLE_X, SETTLE_V, NUDGE_DV,
    linearize, c2d, weights, dlqr, design, closedLoopDecay, makeState, stepModel, control, Sim, runScenario, sig2, displayK,
  };
  G.math = G.math || {}; G.math.lqr = math;

  /* ------------------------------------------------------------------ */
  /* mount                                                               */
  /* ------------------------------------------------------------------ */
  G.sims.lqr = function mount(root, opts) {
    const el = G.el;
    const ui = G.scaffold(root);
    const p = params;

    /* ---- state ---- */
    const sl = { posW: defaults.posW, angleW: defaults.angleW, gimbalR: defaults.gimbalR, thrustR: defaults.thrustR };
    let offset = defaults.offset, showRaw = defaults.showRaw;
    const sim = new Sim(p);
    let des = design(sl, p), solveMs = 0;
    sim.setK(des.K);
    const rand = G.rng(11);

    /* ---- stage canvas (side view + thrust bar) ---- */
    const stage = G.canvas(ui.stage, { aspect: 2.3, minHeight: 260, maxHeight: 360 });
    root.appendChild(G.hidden('Side view of a planar rocket that starts to the left of a target marker at 10 m altitude, tilts toward the target, translates and straightens under LQR control, with a thrust bar beside it.'));

    /* ---- charts canvas ---- */
    const chartsBox = el('div', { class: 'stage' });
    const charts = G.canvas(chartsBox, { aspect: 3.0, minHeight: 300, maxHeight: 340 });
    const chartStates = new G.StripChart({
      duration: 12, autoscale: true, padding: 0.12, yLabel: 'x, z (m) · tilt (deg)', xLabel: 't (s)',
      series: [
        { key: 'x', color: '--sig-act', label: 'x (m)' },
        { key: 'z', color: '--sig-d', label: 'z (m)' },
        { key: 'th', color: '--sig-i', label: 'tilt (deg)' },
      ],
      thresholds: [{ y: 0, color: '--sig-ref', label: 'x target 0 m' }, { y: 10, color: '--sig-ref', label: 'z target 10 m' }],
    });
    const gimbalSeries = { key: 'g', color: '--sig-act', label: 'gimbal (deg)' };
    const rawSeries = { key: 'raw', color: '--sig-fb', dash: [5, 4], label: 'asked (deg)' };
    const chartGimbal = new G.StripChart({
      duration: 12, autoscale: true, padding: 0.12, yLabel: 'gimbal (deg)', xLabel: 't (s)',
      series: [gimbalSeries, rawSeries],
      thresholds: [{ y: 10, color: '--bad', label: '+10 deg limit' }, { y: -10, color: '--bad', label: '−10 deg limit' }],
    });
    const seriesWithRaw = [gimbalSeries, rawSeries], seriesActOnly = [gimbalSeries];
    chartGimbal.o.series = showRaw ? seriesWithRaw : seriesActOnly;

    /* ---- CoM trail (ring buffer) ---- */
    const TRAIL = 480; const trail = new Float32Array(TRAIL * 2); let trailN = 0, trailHead = 0;
    function trailPush(x, z) { trail[trailHead * 2] = x; trail[trailHead * 2 + 1] = z; trailHead = (trailHead + 1) % TRAIL; if (trailN < TRAIL) trailN++; }
    function trailClear() { trailN = 0; trailHead = 0; }

    /* ---- K heatmap (DOM) ---- */
    const cellBase = 'font-family: var(--font-mono); font-size: .78rem; font-variant-numeric: tabular-nums; text-align: center; padding: .4rem .2rem; border: 1px solid var(--line); color: var(--ink); min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;';
    const headBase = 'font-family: var(--font-ui); font-size: .68rem; letter-spacing: .04em; color: var(--ink-3); text-align: center; padding: .25rem .2rem; white-space: nowrap;';
    const rowHeadBase = 'font-family: var(--font-ui); font-size: .72rem; color: var(--ink-2); text-align: right; padding: .4rem .5rem .4rem 0; white-space: nowrap;';
    function matrixGrid(rowLabels, colLabels, title) {
      const cols = colLabels.length;
      const grid = el('div', { style: 'display: grid; grid-template-columns: auto repeat(' + cols + ', minmax(0, 1fr)); gap: 2px; align-items: stretch;' });
      grid.appendChild(el('div', { style: headBase, text: title || '' }));
      colLabels.forEach(c => grid.appendChild(el('div', { style: headBase, text: c })));
      const cells = [];
      rowLabels.forEach(r => {
        grid.appendChild(el('div', { style: rowHeadBase, text: r }));
        const row = [];
        for (let j = 0; j < cols; j++) { const c = el('div', { style: cellBase, text: '0' }); grid.appendChild(c); row.push(c); }
        cells.push(row);
      });
      return { grid, cells };
    }
    const colLabels = DISPLAY_NAMES.map((n, j) => n + ' (' + STATE_UNITS[j] + ')');
    const kGrid = matrixGrid(INPUT_NAMES.map((n, i) => n + ' (' + INPUT_UNITS[i] + ')'), colLabels, 'K');
    const kNote = el('p', { class: 'caption', style: 'margin: .35rem 0 0; font-family: var(--font-mono); font-size: .72rem; color: var(--ink-3);' });
    const kCaption = el('p', { class: 'caption', style: 'margin: .25rem 0 0; font-family: var(--font-ui); font-size: .8125rem; color: var(--ink-3);', text: 'K for the planar sandbox, 2 rows by 6 columns. Each cell is a gain. The nonzero cells in the gimbal row under x and vx mean sideways error and sideways speed are fixed by tilting; the thrust row only looks at z and vz.' });
    const kBox = el('div', { style: 'display: flex; flex-direction: column; gap: .15rem; min-width: 0; overflow-x: auto;' }, [kGrid.grid, kNote, kCaption]);
    function paintCells(cells, Mdisp) {
      let kmax = 0;
      for (const row of Mdisp) for (const v of row) { const a = Math.abs(v); if (a > kmax) kmax = a; }
      for (let i = 0; i < cells.length; i++) for (let j = 0; j < cells[i].length; j++) {
        const v = Mdisp[i][j], a = kmax > 0 ? Math.abs(v) / kmax : 0;
        const c = cells[i][j];
        c.textContent = sig2(v);
        c.style.background = a > 0 ? G.theme.alpha('--gold', a) : 'transparent';
        c.style.color = Math.abs(v) < 1e-9 ? 'var(--ink-3)' : 'var(--ink)';
      }
    }
    function paintK() {
      paintCells(kGrid.cells, displayK(des.K));
      kNote.textContent = 'solver: ' + des.iters + ' iterations, ' + G.fmt(solveMs, 1) + ' ms';
    }

    /* ---- A and B (collapsed) ---- */
    const derivLabels = STATE_NAMES.map(n => 'd' + n + '/dt');
    const aGrid = matrixGrid(derivLabels, STATE_NAMES.slice(), 'A');
    const bGrid = matrixGrid(derivLabels, ['thrust (N)', 'gimbal (rad)'], 'B');
    const abDetails = el('details', { style: 'font-family: var(--font-ui); font-size: .8125rem; color: var(--ink-2);' }, [
      el('summary', { style: 'cursor: pointer;', text: 'Show A and B' }),
      el('p', { class: 'caption', style: 'margin: .5rem 0; font-size: .8125rem; color: var(--ink-3);', text: 'Continuous linear model about hover, per second. A: how the state changes on its own. B: how each input pushes it. States in m, rad, m/s, rad/s.' }),
      el('div', { style: 'display: flex; flex-wrap: wrap; gap: 1rem; min-width: 0; overflow-x: auto;' }, [aGrid.grid, bGrid.grid]),
    ]);
    aGrid.grid.style.flex = '3 1 300px'; bGrid.grid.style.flex = '1.4 1 170px';
    function paintAB() { paintCells(aGrid.cells, des.A); paintCells(bGrid.cells, des.B); }

    /* ---- HUD ---- */
    const roX = G.readout({ label: 'Offset x', unit: 'm', digits: 2 });
    const roTilt = G.readout({ label: 'Peak tilt', unit: 'deg', digits: 1 });
    const roSettle = G.readout({ label: 'Settle time', unit: 's', digits: 2 });
    const roSat = G.readout({ label: 'Gimbal at limit', unit: 's', digits: 2 });
    ui.hud.append(roX.root, roTilt.root, roSettle.root, roSat.root);

    /* order inside inst-main: stage, legend, charts, K, A/B, hud */
    const stageLegend = G.legend([{ label: 'hover thrust', color: '--sig-ff' }, { label: 'K × error', color: '--sig-fb' }, { label: 'path', color: '--sig-act' }, { label: 'target', color: '--sig-ref', dash: true }]);
    const main = ui.stage.parentNode;
    main.insertBefore(stageLegend, ui.hud);
    main.insertBefore(chartsBox, ui.hud);
    main.insertBefore(kBox, ui.hud);
    main.insertBefore(abDetails, ui.hud);

    /* ---- controls ---- */
    const fmtMult = (v) => { const m = Math.pow(10, v); return G.fmt(m, m < 1 ? 2 : m < 10 ? 1 : 0); };
    function weightSlider(label, key) {
      const r = sliderRanges[key];
      return G.slider({
        label, unit: '× default', min: Math.round(Math.log10(r[0]) * 100) / 100, max: Math.round(Math.log10(r[1]) * 100) / 100, step: 0.01, value: Math.log10(sl[key]), format: fmtMult,
        onInput: (v) => { sl[key] = Math.pow(10, v); scheduleDesign(); },
      });
    }
    const sPos = weightSlider('Care about position (Q)', 'posW');
    const sAng = weightSlider('Care about tilt (Q)', 'angleW');
    const sGim = weightSlider('Gimbal effort (R)', 'gimbalR');
    const sThr = weightSlider('Thrust effort (R)', 'thrustR');
    const segMove = G.segmented({
      label: 'Sideways move', options: offsets.map(o => ({ label: o + ' m', value: o })), value: offset,
      onChange: (v) => { offset = v; placeAtOffset(); },
    });
    const togRaw = G.toggle({ label: 'Show unclipped command', value: showRaw, onChange: (v) => { showRaw = v; chartGimbal.o.series = v ? seriesWithRaw : seriesActOnly; renderIfPaused(); } });
    const bGo = G.button({ label: 'Go', kind: 'primary', onClick: go });
    const bNudge = G.button({ label: 'Nudge', onClick: () => { sim.nudge(); renderIfPaused(); } });
    const bReset = G.button({ label: 'Reset', onClick: reset });
    const btnRow = el('div', { class: 'btn-row' }, [bGo.root, bNudge.root, bReset.root]);
    let bPlay = null;
    if (G.reducedMotion) {
      bPlay = G.button({ label: 'Play', onClick: () => { loop.toggle(); } });
      btnRow.appendChild(bPlay.root);
    }
    ui.controls.append(
      G.group('Weights', [sPos.root, sAng.root, sGim.root, sThr.root]),
      G.group('Move', [segMove.root, togRaw.root]),
      btnRow,
    );
    ui.foot.append(el('p', { class: 'caption', text: 'LQR on the planar rocket. Q prices each state error and R prices each input; the heatmap is K, the gain matrix that maps every state to every input.' }));

    /* ---- design scheduling (at most once per frame) ---- */
    let designPending = false;
    function scheduleDesign() {
      if (designPending) return;
      designPending = true;
      requestAnimationFrame(runDesign);
    }
    function runDesign() {
      designPending = false;
      const t0 = performance.now();
      des = design(sl, p);
      solveMs = performance.now() - t0;
      sim.setK(des.K);
      paintK(); paintAB();
      renderIfPaused();
    }

    /* ---- actions ---- */
    function placeAtOffset() {
      sim.reset(offset);
      chartStates.clear(); chartGimbal.clear(); trailClear();
      renderIfPaused();
    }
    function go() {
      sim.reset(offset);
      chartStates.clear(); chartGimbal.clear(); trailClear();
      sim.go();
      if (G.reducedMotion && !loop.running) loop.start();
      renderIfPaused();
    }
    function reset() {
      sl.posW = defaults.posW; sl.angleW = defaults.angleW; sl.gimbalR = defaults.gimbalR; sl.thrustR = defaults.thrustR;
      sPos.set(0, true); sAng.set(0, true); sGim.set(0, true); sThr.set(0, true);
      offset = defaults.offset; segMove.set(offset, true);
      showRaw = defaults.showRaw; togRaw.set(showRaw, true); chartGimbal.o.series = seriesActOnly;
      runDesign();
      placeAtOffset();
    }

    /* ---- physics step ---- */
    let sampleCount = 0;
    function step(dt) {
      const ok = sim.tick(0);
      if (!ok) { placeAtOffset(); return; }
      if (sim.k % p.ctrlDiv === 0) {
        const x = sim.s.x, t = sim.s.t;
        chartStates.push(t, { x: x[0], z: x[1], th: x[2] / DEG });
        chartGimbal.push(t, { g: sim.s.dl / DEG, raw: sim.u.rawDelta / DEG });
        if (sampleCount++ % 2 === 0) trailPush(x[0], x[1]);
      }
    }

    /* ---- render ---- */
    function renderStage() {
      const ctx = stage.ctx, W = stage.width, H = stage.height, g = G.theme.get;
      stage.clear();
      const barW = 30, barX = W - barW - 38;
      const view = { x: 0, y: 0, w: barX - 10, h: H };
      const y0 = H - 22;
      const widthM = offset + 8, heightM = 14;
      const mpp = Math.max(widthM / view.w, heightM / (y0 - 16));
      const xCenter = -offset / 2;
      const xLeft = xCenter - view.w * mpp / 2;
      const sx = (x) => view.x + (x - xLeft) / mpp;
      const sy = (z) => y0 - z / mpp;
      G.drawGround(ctx, view, { y0, mpp, ticks: [0, 5, 10, 15, 20] });
      G.drawSetpoint(ctx, view, { y: sy(10), label: 'target 10 m' });
      /* target marker at (0, 10) */
      ctx.save();
      ctx.strokeStyle = g('--sig-ref'); ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
      const tx = sx(0), ty = sy(10);
      ctx.beginPath(); ctx.arc(tx, ty, 9, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(tx - 14, ty); ctx.lineTo(tx + 14, ty); ctx.moveTo(tx, ty - 14); ctx.lineTo(tx, ty + 14); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(tx + 0.5, ty + 14); ctx.lineTo(tx + 0.5, y0); ctx.setLineDash([2, 5]); ctx.strokeStyle = G.theme.alpha('--sig-ref', 0.5); ctx.stroke(); ctx.setLineDash([]);
      ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText('x = 0 m', tx, y0 + 4);
      /* start marker */
      const sxs = sx(-offset);
      ctx.strokeStyle = g('--line-2'); ctx.beginPath(); ctx.moveTo(sxs + 0.5, y0 - 6); ctx.lineTo(sxs + 0.5, y0); ctx.stroke();
      ctx.fillText('x = −' + offset + ' m', sxs, y0 + 4);
      /* trail */
      if (trailN > 1) {
        ctx.strokeStyle = G.theme.alpha('--sig-act', 0.55); ctx.lineWidth = 1.5; ctx.beginPath();
        const start = (trailHead - trailN + TRAIL) % TRAIL;
        for (let i = 0; i < trailN; i++) { const idx = ((start + i) % TRAIL) * 2; const px = sx(trail[idx]), py = sy(trail[idx + 1]); if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py); }
        ctx.stroke();
      }
      ctx.restore();
      /* rocket, clamped into the view */
      const X = sim.s.x;
      const scaleRocket = G.clamp(2.4 / mpp, 30, 60);
      const cx = G.clamp(sx(X[0]), view.x + 24, view.x + view.w - 24), cy = G.clamp(sy(X[1]), 30, y0);
      G.drawRocket(ctx, { x: cx, y: cy + 0.45 * scaleRocket, scale: scaleRocket, tilt: X[2], gimbal: sim.s.dl, thrust01: (sim.s.T - p.Tmin) / (p.Tmax - p.Tmin), rand });
      /* thrust bar: hover (feedforward) + K x error (feedback) */
      G.drawBarStack(ctx, { x: barX, y: 6, w: barW, h: H - 12 }, {
        segments: [{ value: p.hover, color: '--sig-ff' }, { value: sim.u.dT, color: '--sig-fb' }],
        min: -4, max: 24, limitLo: p.Tmin, limitHi: p.Tmax, title: 'thrust', unit: 'N',
      });
    }
    function renderCharts() {
      const ctx = charts.ctx, W = charts.width, H = charts.height;
      charts.clear();
      if (W < 560) {
        chartStates.draw(ctx, { x: 0, y: 2, w: W, h: H / 2 - 2 });
        chartGimbal.draw(ctx, { x: 0, y: H / 2, w: W, h: H / 2 - 2 });
      } else {
        chartStates.draw(ctx, { x: 0, y: 2, w: W / 2 - 4, h: H - 4 });
        chartGimbal.draw(ctx, { x: W / 2 + 4, y: 2, w: W / 2 - 4, h: H - 4 });
      }
    }
    function renderHud() {
      const m = sim.metrics, x = sim.s.x;
      roX.set(x[0], Math.abs(x[0]) < SETTLE_X ? 'good' : '');
      roTilt.set(m.peakTilt, m.peakTilt > 20 ? 'bad' : m.peakTilt > 12 ? 'warn' : '');
      roSettle.set(m.settleTime === null ? '—' : m.settleTime, m.settleTime === null ? '' : 'good');
      roSat.set(m.satTime, m.satTime > 0 ? 'warn' : '');
    }
    function render() { renderStage(); renderCharts(); renderHud(); }
    function renderIfPaused() { if (!loop.running) loop.renderOnce(); }

    const loop = G.loop({ step, render, dt: p.dt, maxSubsteps: 12, root, onState: (running) => { if (bPlay) bPlay.setLabel(running ? 'Pause' : 'Play'); } });
    const offTheme = G.theme.onChange(() => { paintK(); paintAB(); renderIfPaused(); });
    stage.onResize(renderIfPaused); charts.onResize(renderIfPaused);
    paintK(); paintAB();

    return {
      reset,
      destroy() { loop.stop(); loop.destroy(); offTheme(); stage.destroy(); charts.destroy(); },
      loop, sim,
    };
  };
})();
