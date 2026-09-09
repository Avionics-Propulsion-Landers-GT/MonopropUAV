/* =====================================================================
 *  GTPL Controls Guide — reference simulation code (no dependencies)
 *  Works in the browser (window.SimSpec) and in node (module.exports).
 *
 *  Contents
 *    0. constants           DEFAULTS (all numbers used by the six sims)
 *    1. RNG                 seeded mulberry32 + gaussian
 *    2. matrix helpers      tiny dense-matrix toolkit (arrays of arrays)
 *    3. model A             1-D "hover rocket": stepA, makeStateA
 *    4. model B             2-D "planar rocket": stepB, makeStateB, linearizeB, c2d
 *    5. PID                 anti-windup, derivative on (filtered) measurement, FF input
 *    6. LQR                 dlqr Riccati iteration, eigenvalues (stability check)
 *    7. MPC                 condensed linear MPC (Sx, Su, H) + projected gradient descent
 *    8. reference profiles  min-jerk climb (S2, S4), guidance descent (S6)
 *    9. scenarios           S1..S6 definitions used by the tests and by the page
 * ===================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SimSpec = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  /* ---------------------------------------------------------------- */
  /* 0. DEFAULT CONSTANTS                                              */
  /* ---------------------------------------------------------------- */
  const DEFAULTS = {
    g: 9.81,
    sim: { dt: 1 / 200, ctrlDivA: 2, ctrlDivB: 4, mpcDiv: 20, realtime: 1.0 },
    //   sim 200 Hz; model-A controllers at 100 Hz; model-B LQR at 50 Hz (dt_ctrl 0.02);
    //   MPC re-solved every 20 sim steps = 10 Hz with an internal model step of 0.1 s.
    A: {                            // 1-D hover rocket
      m: 50.0,                      // kg (small lander; hover thrust = 490.5 N)
      Tmin: 350.0,                  // N  (35 % of Tmax — hybrid flame-out floor)
      Tmax: 1000.0,                 // N  (Monarch-class 1 kN engine)
      tau: 0.20,                    // s  thrust lag (chamber-pressure time constant)
      drag: 0.0,                    // N·s/m linear drag (0 = clean double integrator)
      burn: false, Isp: 200.0, mDry: 30.0,  // optional propellant mass loss
      noiseZ: 0.02,                 // m   altitude sensor noise std (1-sigma)
      zMax: 40.0,                   // m   sandbox ceiling for plotting
    },
    B: {                            // 2-D planar rocket
      m: 50.0, L: 0.8, I: 25.0,     // kg, m (CoM->gimbal pivot), kg·m²
      Tmin: 350.0, Tmax: 1000.0,
      deltaMax: 10 * Math.PI / 180, // rad (±10°)
      tau: 0.20, tauServo: 0.05,    // s thrust lag, servo lag
      dtCtrl: 0.02,                 // s LQR/discretization step
    },
    S1: { target: 10.0, band: 0.5, duration: 30.0, pd: { Kp: 100, Kd: 180 } },   // auto-pilot sanity check (FF = m g)
    S2: { zTarget: 10.0, tClimb: 5.0, tEnd: 10.0, massErrors: [-0.05, 0.05], gustN: -30.0, gustT: 5.0 },
    S3: {
      sliders: { Kp: [0, 1000], Ki: [0, 300], Kd: [0, 600] },   // N/m, N/(m·s), N·s/m
      tauD: 0.05,                                               // s, derivative low-pass
      presets: {
        sluggish:      { Kp: 40,  Ki: 0,  Kd: 150, ff: true },
        tuned:         { Kp: 150, Ki: 5,  Kd: 250, ff: true },
        tooAggressive: { Kp: 500, Ki: 0,  Kd: 150, ff: true },
      },
      defaults: { Kp: 150, Ki: 5, Kd: 250, ff: true, noise: false },
      z0: 10.0,                                   // sandbox starts hovering at 10 m ("launch from pad" toggle -> z0 = 0)
      setpoints: [[0, 10], [1, 15], [16, 8]],     // [time, z]: 10 -> 15 at t = 1 s, -> 8 at t = 16 s
    },
    S4: { zTarget: 15.0, tClimb: 8.0, tEnd: 14.0, gains: { Kp: 150, Ki: 5, Kd: 250 }, massError: 0.05 },
    S5: {
      Q: [1, 1, 25, 1, 1, 4],       // x z θ vx vz ω    (Bryson: 1/max² with 1 m, 1 m, 0.2 rad, 1 m/s, 1 m/s, 0.5 rad/s)
      R: [2.5e-5, 1000],            // ΔT [N], δ [rad]  (1/200² for thrust; gimbal weight raised until 5 m offset stays under 10°)
      sliders: {                    // log-scale multipliers applied to the base weights above
        posW:     { base: 1,      range: [0.1, 10] },   // scales Q[x], Q[z]
        angleW:   { base: 25,     range: [0.1, 10] },   // scales Q[θ]
        gimbalR:  { base: 1000,   range: [0.03, 30] },  // scales R[δ]
        thrustR:  { base: 2.5e-5, range: [0.1, 100] },  // scales R[ΔT]
      },
      x0: [5, 10, 0, 0, 0, 0], target: [0, 10, 0, 0, 0, 0], tEnd: 15,
    },
    S6: {
      N: 20, dtMpc: 0.1, uScale: [100, 0.1], maxIter: 40, rho: 2000,
      cone: { k: 0.5, margin: 0.5 },       // |x| <= k z + margin  (26.6° half-angle)
      // A (input limits): same weights for both controllers; LQR + clipping flips, MPC does not
      lateral: { x0: [12, 15, 0, 0, 0, 0], target: [0, 15, 0, 0, 0, 0], tEnd: 15, weights: { posW: 10, angleW: 1, gimbalR: 1, thrustR: 1 } },
      // B (state constraint): already descending at 2 m/s, 7 m off the pad; land at (0,0) inside the cone
      descent: { x0: [7, 15, 0, 0, -2, 0], target: [0, 0, 0, 0, 0, 0], tEnd: 20, weights: { posW: 1, angleW: 1, gimbalR: 1, thrustR: 1 } },
      // B2 (lookahead): track a guidance line (7,15)->(0,0) in 6 s (min-jerk in x and z), then hold at the pad
      guidance: { x0: [7, 15, 0, 0, 0, 0], duration: 6, tEnd: 10, weights: { posW: 1, angleW: 1, gimbalR: 1, thrustR: 1 } },
    },
  };

  /* ---------------------------------------------------------------- */
  /* 1. RNG                                                            */
  /* ---------------------------------------------------------------- */
  function makeRng(seed) {
    let a = (seed >>> 0) || 0x9e3779b9;
    const rng = {
      next() {            // mulberry32, uniform [0,1)
        a |= 0; a = (a + 0x6D2B79F5) | 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a);
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      },
      gauss() {           // Box–Muller, std 1
        let u = 0, v = 0;
        while (u === 0) u = rng.next();
        v = rng.next();
        return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
      },
    };
    return rng;
  }

  /* ---------------------------------------------------------------- */
  /* 2. MATRIX HELPERS (row-major arrays of arrays)                    */
  /* ---------------------------------------------------------------- */
  const M = {
    zeros: (r, c) => Array.from({ length: r }, () => new Array(c).fill(0)),
    eye: (n) => { const I = M.zeros(n, n); for (let i = 0; i < n; i++) I[i][i] = 1; return I; },
    diag: (d) => { const D = M.zeros(d.length, d.length); d.forEach((v, i) => D[i][i] = v); return D; },
    clone: (A) => A.map(r => r.slice()),
    T: (A) => A[0].map((_, j) => A.map(r => r[j])),
    add: (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j])),
    sub: (A, B) => A.map((r, i) => r.map((v, j) => v - B[i][j])),
    scale: (A, s) => A.map(r => r.map(v => v * s)),
    mul(A, B) {
      const n = A.length, k = B.length, p = B[0].length, C = M.zeros(n, p);
      for (let i = 0; i < n; i++) { const Ai = A[i], Ci = C[i];
        for (let t = 0; t < k; t++) { const a = Ai[t]; if (a === 0) continue; const Bt = B[t];
          for (let j = 0; j < p; j++) Ci[j] += a * Bt[j]; } }
      return C;
    },
    matvec: (A, v) => A.map(r => r.reduce((s, a, j) => s + a * v[j], 0)),
    vadd: (a, b) => a.map((v, i) => v + b[i]),
    vsub: (a, b) => a.map((v, i) => v - b[i]),
    vscale: (a, s) => a.map(v => v * s),
    dot: (a, b) => a.reduce((s, v, i) => s + v * b[i], 0),
    normInf: (A) => Math.max(...A.map(r => Math.max(...r.map(Math.abs)))),
    // Gauss–Jordan inverse with partial pivoting (fine for n <= ~12)
    inv(A) {
      const n = A.length, X = A.map((r, i) => [...r, ...M.eye(n)[i]]);
      for (let c = 0; c < n; c++) {
        let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(X[r][c]) > Math.abs(X[p][c])) p = r;
        [X[c], X[p]] = [X[p], X[c]];
        const piv = X[c][c]; if (Math.abs(piv) < 1e-14) throw new Error('singular');
        for (let j = 0; j < 2 * n; j++) X[c][j] /= piv;
        for (let r = 0; r < n; r++) if (r !== c) { const f = X[r][c]; if (f !== 0) for (let j = 0; j < 2 * n; j++) X[r][j] -= f * X[c][j]; }
      }
      return X.map(r => r.slice(n));
    },
  };
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  /* ---------------------------------------------------------------- */
  /* 3. MODEL A — 1-D hover rocket                                     */
  /*    state: z [m], v [m/s], T [N] (actual thrust, lags command), m  */
  /* ---------------------------------------------------------------- */
  function makeStateA(p, z0 = 0) { return { z: z0, v: 0, T: p.m * DEFAULTS.g, m: p.m, t: 0 }; }

  // semi-implicit (symplectic) Euler: velocity first, then position with the new velocity.
  // Tcmd: commanded thrust [N] (clamped to [Tmin,Tmax]); dist: extra vertical force [N] (gust)
  function stepA(s, Tcmd, p, dt, dist = 0) {
    const Tc = clamp(Tcmd, p.Tmin, p.Tmax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));      // exact 1st-order lag for a held command
    const a = (s.T - s.m * DEFAULTS.g + dist - p.drag * s.v) / s.m;
    s.v += a * dt;
    s.z += s.v * dt;
    if (s.z <= 0) { s.z = 0; if (s.v < 0) s.v = 0; }        // ground
    if (p.burn) s.m = Math.max(p.mDry, s.m - (s.T / (p.Isp * DEFAULTS.g)) * dt);
    s.t += dt;
    return s;
  }
  // Slider helper for S1: throttle fraction u∈[0,1] -> thrust
  const throttleToThrust = (u, p) => p.Tmin + clamp(u, 0, 1) * (p.Tmax - p.Tmin);
  const hoverThrottle = (p) => (p.m * DEFAULTS.g - p.Tmin) / (p.Tmax - p.Tmin);

  /* ---------------------------------------------------------------- */
  /* 4. MODEL B — 2-D planar rocket                                    */
  /*    state vector x = [x, z, th, vx, vz, om]                        */
  /*    th = tilt from vertical, positive toward +x. Thrust acts along */
  /*    the body axis rotated by gimbal delta; positive delta pushes   */
  /*    the base toward +x, which torques the nose toward -x:         */
  /*        ẍ = T sin(th+δ)/m,  z̈ = T cos(th+δ)/m − g,  ω̇ = −L T sin δ / I */
  /* ---------------------------------------------------------------- */
  function makeStateB(p, x0) {
    return { x: x0.slice(), T: p.m * DEFAULTS.g, dl: 0, t: 0 };   // T, dl = actual (lagged) actuator values
  }
  // u = [T_cmd (absolute N), delta_cmd (rad)];  windN = lateral force [N]
  function stepB(s, u, p, dt, windN = 0) {
    const Tc = clamp(u[0], p.Tmin, p.Tmax), dc = clamp(u[1], -p.deltaMax, p.deltaMax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    s.dl += (dc - s.dl) * (1 - Math.exp(-dt / p.tauServo));
    const X = s.x, ang = X[2] + s.dl;
    const ax = (s.T * Math.sin(ang) + windN) / p.m;
    const az = s.T * Math.cos(ang) / p.m - DEFAULTS.g;
    const aom = -p.L * s.T * Math.sin(s.dl) / p.I;
    X[3] += ax * dt; X[4] += az * dt; X[5] += aom * dt;
    X[0] += X[3] * dt; X[1] += X[4] * dt; X[2] += X[5] * dt;
    if (X[1] <= 0) { X[1] = 0; if (X[4] < 0) X[4] = 0; }
    s.t += dt;
    return s;
  }
  // Continuous linearization about hover (T0 = m g, δ = 0, θ = 0). Input u = [ΔT, δ].
  function linearizeB(p) {
    const g = DEFAULTS.g, A = M.zeros(6, 6), B = M.zeros(6, 2);
    A[0][3] = 1; A[1][4] = 1; A[2][5] = 1;   // ẋ = vx, ż = vz, θ̇ = ω
    A[3][2] = g;                             // v̇x = g θ (+ g δ)
    B[3][1] = g;                             // v̇x from gimbal
    B[4][0] = 1 / p.m;                       // v̇z = ΔT/m
    B[5][1] = -p.L * p.m * g / p.I;          // ω̇ = −(L m g / I) δ
    return { A, B };
  }
  // Discretize with the matrix exponential of [[A,B],[0,0]]·dt via Taylor series (exact here: A is nilpotent).
  function c2d(A, B, dt, terms = 20) {
    const n = A.length, m = B[0].length, N = n + m;
    const Mbig = M.zeros(N, N);
    for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) Mbig[i][j] = A[i][j] * dt; for (let j = 0; j < m; j++) Mbig[i][n + j] = B[i][j] * dt; }
    let E = M.eye(N), term = M.eye(N);
    for (let k = 1; k <= terms; k++) { term = M.scale(M.mul(term, Mbig), 1 / k); E = M.add(E, term); if (M.normInf(term) < 1e-16) break; }
    const Ad = E.slice(0, n).map(r => r.slice(0, n)), Bd = E.slice(0, n).map(r => r.slice(n));
    return { Ad, Bd };
  }

  /* ---------------------------------------------------------------- */
  /* 5. PID with anti-windup, derivative on filtered measurement        */
  /* ---------------------------------------------------------------- */
  class PID {
    // opts: {Kp, Ki, Kd, tauD, uMin, uMax, iMax}
    //   iMax = max |Ki·∫e| contribution (N). Default: full actuator range.
    constructor(o) {
      this.Kp = o.Kp; this.Ki = o.Ki; this.Kd = o.Kd;
      this.tauD = o.tauD ?? 0.05; this.uMin = o.uMin; this.uMax = o.uMax;
      this.iMax = o.iMax ?? (o.uMax - o.uMin);
      this.reset();
    }
    reset() { this.integ = 0; this.prevMeas = null; this.vFilt = 0; }
    // ref: setpoint; meas: measured position; dt; ff: feedforward [N]; vRef: reference velocity
    // (0 for step setpoints); vMeas: optional velocity from nav — if given, no differencing.
    update(ref, meas, dt, ff = 0, vRef = 0, vMeas = null) {
      const e = ref - meas;
      // --- D: derivative of the MEASUREMENT (no derivative kick on setpoint steps), low-passed
      let vRaw;
      if (vMeas !== null) vRaw = vMeas;
      else { vRaw = this.prevMeas === null ? 0 : (meas - this.prevMeas) / dt; this.prevMeas = meas; }
      this.vFilt += (vRaw - this.vFilt) * (dt / (this.tauD + dt));
      const P = this.Kp * e;
      const D = this.Kd * (vRef - this.vFilt);
      // --- I with anti-windup: clamp the I contribution and stop integrating while saturated in the same direction
      let I = this.Ki * this.integ;
      const uUnsat = ff + P + I + D;
      const sat = uUnsat > this.uMax ? 1 : uUnsat < this.uMin ? -1 : 0;
      if (!(sat === 1 && e > 0) && !(sat === -1 && e < 0)) this.integ += e * dt;
      if (this.Ki > 0) this.integ = clamp(this.integ, -this.iMax / this.Ki, this.iMax / this.Ki);
      I = this.Ki * this.integ;
      const u = clamp(ff + P + I + D, this.uMin, this.uMax);
      return { u, P, I, D, FF: ff, e, sat: u !== ff + P + I + D };
    }
  }

  /* ---------------------------------------------------------------- */
  /* 6. LQR — discrete Riccati iteration                                */
  /* ---------------------------------------------------------------- */
  // Returns K (m×n) such that u = −K x, plus the converged P.
  function dlqr(Ad, Bd, Q, R, tol = 1e-9, maxIter = 5000) {
    const At = M.T(Ad), Bt = M.T(Bd);
    let P = M.clone(Q), K = null, it = 0;
    for (; it < maxIter; it++) {
      const BtP = M.mul(Bt, P);
      const S = M.add(R, M.mul(BtP, Bd));            // R + Bᵀ P B
      K = M.mul(M.inv(S), M.mul(BtP, Ad));           // (R + BᵀPB)⁻¹ BᵀPA
      const Pn = M.add(Q, M.sub(M.mul(M.mul(At, P), Ad), M.mul(M.mul(M.mul(At, P), Bd), K)));
      const d = M.normInf(M.sub(Pn, P));
      P = Pn;
      if (d < tol * Math.max(1, M.normInf(P))) break;
    }
    return { K, P, iters: it + 1 };
  }
  // Eigenvalues of a small real matrix: Faddeev–LeVerrier characteristic polynomial + Durand–Kerner roots.
  function eigenvalues(A) {
    const n = A.length;
    // coefficients c[0..n] of det(λI − A) = λ^n + c1 λ^{n-1} + ... (Faddeev–LeVerrier)
    let Mk = M.zeros(n, n); const c = new Array(n + 1).fill(0); c[0] = 1;
    for (let k = 1; k <= n; k++) {
      Mk = M.add(M.mul(A, Mk), M.scale(M.eye(n), c[k - 1]));
      const AM = M.mul(A, Mk); let tr = 0; for (let i = 0; i < n; i++) tr += AM[i][i];
      c[k] = -tr / k;
    }
    // Durand–Kerner on monic polynomial p(λ) = Σ c[k] λ^{n-k}
    let roots = Array.from({ length: n }, (_, i) => ({ re: 0.4 * Math.cos(2 * Math.PI * i / n + 0.4), im: 0.9 * Math.sin(2 * Math.PI * i / n + 0.4) }));
    const cmul = (a, b) => ({ re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re });
    const cdiv = (a, b) => { const d = b.re * b.re + b.im * b.im; return { re: (a.re * b.re + a.im * b.im) / d, im: (a.im * b.re - a.re * b.im) / d }; };
    const peval = (z) => { let r = { re: c[0], im: 0 }; for (let k = 1; k <= n; k++) { r = cmul(r, z); r.re += c[k]; } return r; };
    for (let it = 0; it < 500; it++) {
      let maxd = 0;
      for (let i = 0; i < n; i++) {
        let den = { re: 1, im: 0 };
        for (let j = 0; j < n; j++) if (j !== i) den = cmul(den, { re: roots[i].re - roots[j].re, im: roots[i].im - roots[j].im });
        const d = cdiv(peval(roots[i]), den);
        roots[i] = { re: roots[i].re - d.re, im: roots[i].im - d.im };
        maxd = Math.max(maxd, Math.hypot(d.re, d.im));
      }
      if (maxd < 1e-13) break;
    }
    return roots.map(r => ({ re: r.re, im: r.im, abs: Math.hypot(r.re, r.im) }));
  }
  // Build Q,R for model B from the S5 sliders (multipliers on base weights)
  function lqrWeightsFromSliders(sl, base = DEFAULTS.S5) {
    const q = base.Q.slice(), r = base.R.slice();
    q[0] *= sl.posW; q[1] *= sl.posW; q[2] *= sl.angleW; r[0] *= sl.thrustR; r[1] *= sl.gimbalR;
    return { Q: M.diag(q), R: M.diag(r) };
  }

  /* ---------------------------------------------------------------- */
  /* 7. MPC — condensed linear MPC with box input constraints (PGD)     */
  /* ---------------------------------------------------------------- */
  // Prediction over the horizon (LTI version of the repo's build_Su):
  //   X = Sx x0 + Su U,   X = [x1;…;xN] (nN),  U = [u0;…;u_{N-1}] (mN)
  //   Su block (k,j) = A^{k-j} B  for j <= k
  function buildPrediction(Ad, Bd, N) {
    const n = Ad.length, m = Bd[0].length;
    const Sx = M.zeros(n * N, n), Su = M.zeros(n * N, m * N);
    const Apow = [M.eye(n)]; for (let k = 1; k <= N; k++) Apow.push(M.mul(Ad, Apow[k - 1]));
    for (let k = 0; k < N; k++) {
      const Ak1 = Apow[k + 1];
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) Sx[k * n + i][j] = Ak1[i][j];
      for (let j = 0; j <= k; j++) {
        const blk = M.mul(Apow[k - j], Bd);
        for (let i = 0; i < n; i++) for (let jj = 0; jj < m; jj++) Su[k * n + i][j * m + jj] = blk[i][jj];
      }
    }
    return { Sx, Su };
  }

  // opts: {Ad, Bd, Q (n×n), R (m×m), P (terminal n×n), N, uMin, uMax (length m, deviation units),
  //        uScale (length m: inputs are optimized in units of u/uScale for conditioning),
  //        cone: {k, margin} | null, rho (soft-constraint weight), maxIter}
  function makeMPC(o) {
    const n = o.Ad.length, m = o.Bd[0].length, N = o.N, mN = m * N, nN = n * N;
    const us = o.uScale || new Array(m).fill(1);
    // scale B columns so the decision variable is the normalized input
    const Bs = o.Bd.map(r => r.map((v, j) => v * us[j]));
    const { Sx, Su } = buildPrediction(o.Ad, Bs, N);
    // block-diagonal Qbar (Q…Q, P at the end) stored as a diagonal-block list, Rbar in scaled units
    const Qblocks = []; for (let k = 0; k < N - 1; k++) Qblocks.push(o.Q); Qblocks.push(o.P || o.Q);
    const Rs = o.R.map((r, i) => r.map((v, j) => v * us[i] * us[j]));
    // H = Suᵀ Qbar Su + Rbar   (dense mN×mN, built once — the model is LTI)
    const QSu = M.zeros(nN, mN);
    for (let k = 0; k < N; k++) { const Qk = Qblocks[k];
      for (let i = 0; i < n; i++) for (let j = 0; j < mN; j++) { let s = 0; for (let t = 0; t < n; t++) s += Qk[i][t] * Su[k * n + t][j]; QSu[k * n + i][j] = s; } }
    let H = M.mul(M.T(Su), QSu);
    for (let k = 0; k < N; k++) for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) H[k * m + i][k * m + j] += Rs[i][j];
    H = M.scale(M.add(H, M.T(H)), 0.5); for (let i = 0; i < mN; i++) H[i][i] += 1e-8;
    // Jacobi preconditioning: rescale each decision variable by 1/sqrt(H_ii) so the QP is well conditioned.
    // (A diagonal rescaling keeps box constraints as boxes, so projection stays a clamp.)
    const sJ = H.map((r, i) => 1 / Math.sqrt(r[i]));
    H = H.map((r, i) => r.map((v, j) => v * sJ[i] * sJ[j]));
    for (let i = 0; i < nN; i++) for (let j = 0; j < mN; j++) Su[i][j] *= sJ[j];
    const SuT = M.T(Su);
    const scaleOf = (j) => us[j % m] * sJ[j];                       // physical u = scaleOf(j) * decision variable
    // Lipschitz constant λmax(H) by power iteration -> gradient step size 1/L
    let v = new Array(mN).fill(1), L = 1;
    for (let it = 0; it < 100; it++) { const Hv = M.matvec(H, v); L = Math.sqrt(M.dot(Hv, Hv)); v = M.vscale(Hv, 1 / L); }
    const lo = Array.from({ length: mN }, (_, j) => o.uMin[j % m] / scaleOf(j)), hi = Array.from({ length: mN }, (_, j) => o.uMax[j % m] / scaleOf(j));
    const rho = o.rho ?? 0, cone = o.cone || null, maxIter = o.maxIter ?? 40;

    // stacked reference: Xref is an array of N states (for k = 1..N); a single state is broadcast
    function stackRef(Xref) {
      const r = new Array(nN);
      for (let k = 0; k < N; k++) { const xr = Array.isArray(Xref[0]) ? Xref[Math.min(k, Xref.length - 1)] : Xref; for (let i = 0; i < n; i++) r[k * n + i] = xr[i]; }
      return r;
    }
    // soft state-constraint penalty and its gradient wrt X (glide-slope cone |x| <= k z + margin, and z >= 0)
    function penalty(X, gradX) {
      if (!cone) return 0; let J = 0;
      for (let k = 0; k < N; k++) { const x = X[k * n], z = X[k * n + 1];
        const viol = Math.abs(x) - (cone.k * z + cone.margin);
        if (viol > 0) { J += rho * viol * viol; if (gradX) { gradX[k * n] += 2 * rho * viol * Math.sign(x); gradX[k * n + 1] -= 2 * rho * viol * cone.k; } }
        if (z < 0) { J += rho * z * z; if (gradX) gradX[k * n + 1] += 2 * rho * z; }
      }
      return J;
    }
    const project = (U) => U.map((u, i) => clamp(u, lo[i], hi[i]));

    // One receding-horizon step. x0: current state (deviation coords: absolute state − hover input is
    // handled by the caller adding T0). Xref: target state(s). Uwarm: previous solution (scaled units).
    function step(x0, Xref, Uwarm) {
      const t0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
      const r = stackRef(Xref);
      const Sxx0 = M.matvec(Sx, x0);
      const d = M.vsub(Sxx0, r);                         // free response minus reference
      // f = Suᵀ Qbar d  (linear term); quadratic part of the state cost is in H
      const Qd = new Array(nN); for (let k = 0; k < N; k++) { const Qk = Qblocks[k]; for (let i = 0; i < n; i++) { let s = 0; for (let t = 0; t < n; t++) s += Qk[i][t] * d[k * n + t]; Qd[k * n + i] = s; } }
      const f = M.matvec(SuT, Qd);
      const c0 = 0.5 * M.dot(d, Qd);                     // constant term (so reported cost = true cost)
      const cost = (U, X) => 0.5 * M.dot(U, M.matvec(H, U)) + M.dot(f, U) + c0 + penalty(X, null);
      const predict = (U) => M.vadd(Sxx0, M.matvec(Su, U));
      const gradAt = (U, X) => { const gX = new Array(nN).fill(0); penalty(X, gX); return M.vadd(M.vadd(M.matvec(H, U), f), M.matvec(SuT, gX)); };
      // warm start: shift previous solution one step, repeat the last input
      // (Uwarm is the previous solution in decision-variable units; shift it in PHYSICAL units because the
      //  Jacobi scale differs per stage.)
      let U = new Array(mN).fill(0);
      if (Uwarm) for (let j = 0; j < mN; j++) { const src = Math.min(j + m, mN - m + (j % m)); U[j] = Uwarm[src] * scaleOf(src) / scaleOf(j); }
      U = project(U);
      let X = predict(U), J = cost(U, X);
      // Accelerated projected gradient (FISTA with adaptive restart) + backtracking.
      // Same projected-gradient skeleton as the repo's solve_qp_box; the momentum term is what makes
      // it converge in tens of iterations instead of thousands on this badly scaled QP.
      let Y = U.slice(), tk = 1, a = 1 / L, it = 0;
      for (; it < maxIter; it++) {
        const Xy = predict(Y), Jy = cost(Y, Xy), gY = gradAt(Y, Xy);
        let Un, Xn, Jn;
        for (let ls = 0; ls < 30; ls++) {                                       // backtracking on the step a
          Un = project(M.vsub(Y, M.vscale(gY, a))); Xn = predict(Un); Jn = cost(Un, Xn);
          const dY = M.vsub(Un, Y);
          if (Jn <= Jy + M.dot(gY, dY) + M.dot(dY, dY) / (2 * a) + 1e-12) break;   // sufficient-decrease test
          a *= 0.5;
        }
        const dU = M.vsub(Un, U);
        if (M.dot(M.vsub(Y, Un), dU) > 0 || Jn > J) { tk = 1; Y = U.slice(); a = Math.min(a * 2, 1 / L); if (Jn > J) continue; } // adaptive restart
        const tn = 0.5 * (1 + Math.sqrt(1 + 4 * tk * tk));
        Y = M.vadd(Un, M.vscale(dU, (tk - 1) / tn)); tk = tn;
        U = Un; X = Xn; J = Jn;
        if (Math.sqrt(M.dot(dU, dU)) < 1e-6 * Math.max(1, Math.sqrt(M.dot(U, U)))) { it++; break; }
      }
      // unpack: first input in physical units, predicted trajectory (N+1 points incl. x0)
      const u0 = Array.from({ length: m }, (_, j) => U[j] * scaleOf(j));
      const Xpred = [x0.slice()]; for (let k = 0; k < N; k++) Xpred.push(X.slice(k * n, k * n + n));
      const Uphys = []; for (let k = 0; k < N; k++) Uphys.push(Array.from({ length: m }, (_, j) => U[k * m + j] * scaleOf(k * m + j)));
      let maxViol = 0; if (cone) for (const xk of Xpred) maxViol = Math.max(maxViol, Math.abs(xk[0]) - (cone.k * xk[1] + cone.margin));
      const ms = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
      return { u0, U: Uphys, Uscaled: U, Xpred, cost: J, iters: it, ms, coneViolPred: maxViol };
    }
    return { step, H, Su, Sx, L, N, n, m };
  }

  /* ---------------------------------------------------------------- */
  /* 8. REFERENCE PROFILES                                              */
  /* ---------------------------------------------------------------- */
  // Minimum-jerk rest-to-rest move from z0 to z1 over T seconds (what a guidance profile looks like).
  function minJerk(z0, z1, T) {
    return (t) => {
      if (t <= 0) return { z: z0, v: 0, a: 0, j: 0 };
      if (t >= T) return { z: z1, v: 0, a: 0, j: 0 };
      const s = t / T, d = z1 - z0;
      return { z: z0 + d * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5),
               v: d * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T,
               a: d * (60 * s - 180 * s ** 2 + 120 * s ** 3) / T ** 2,
               j: d * (60 - 360 * s + 360 * s ** 2) / T ** 3 };
    };
  }
  // Open-loop plan for S2: T_cmd = m(g + a_ref)  [+ tau·m·j_ref if invertLag — optional lag inversion; the
  // plain plan already lands within 0.03 m at 5 s and exactly at 10 s because it ends on T = m g]
  function openLoopThrust(prof, p, t, invertLag = false) {
    const r = prof(t);
    return p.m * (DEFAULTS.g + r.a) + (invertLag ? p.tau * p.m * r.j : 0);
  }

  /* ---------------------------------------------------------------- */
  /* 9. SCENARIOS                                                       */
  /* ---------------------------------------------------------------- */
  // Generic model-A runner: ctrl(t, meas, state) -> Tcmd; runs sim at dt, controller every ctrlDiv steps
  function runA(p, ctrl, opts) {
    const dt = DEFAULTS.sim.dt, div = opts.ctrlDiv ?? DEFAULTS.sim.ctrlDivA, dtc = dt * div;
    const s = makeStateA(p, opts.z0 ?? 0); if (opts.m0) s.m = opts.m0;
    const rng = makeRng(opts.seed ?? 1); const noise = opts.noise ? p.noiseZ : 0;
    const log = []; let Tcmd = p.m * DEFAULTS.g, meas = s.z, info = null;
    const steps = Math.round(opts.tEnd / dt);
    for (let i = 0; i <= steps; i++) {
      if (i % div === 0) { meas = s.z + noise * rng.gauss(); const out = ctrl(s.t, meas, s, dtc); if (typeof out === 'number') Tcmd = out; else { Tcmd = out.u; info = out; } }
      if (opts.log && i % (opts.logEvery ?? 4) === 0) log.push({ t: s.t, z: s.z, v: s.v, T: s.T, Tcmd, meas, m: s.m, info });
      const dist = opts.dist ? opts.dist(s.t) : 0;
      stepA(s, Tcmd, p, dt, dist);
    }
    return { s, log };
  }
  // Generic model-B runner: ctrl(t, x) -> [T, delta] absolute; ctrl every ctrlDiv steps
  function runB(p, ctrl, opts) {
    const dt = DEFAULTS.sim.dt, div = opts.ctrlDiv ?? DEFAULTS.sim.ctrlDivB;
    const s = makeStateB(p, opts.x0); let u = [p.m * DEFAULTS.g, 0], info = null; const log = [];
    const steps = Math.round(opts.tEnd / dt);
    for (let i = 0; i <= steps; i++) {
      if (i % div === 0) { const out = ctrl(s.t, s.x.slice(), s); if (Array.isArray(out)) u = out; else { u = out.u; info = out; } }
      if (opts.log && i % (opts.logEvery ?? 4) === 0) log.push({ t: s.t, x: s.x.slice(), T: s.T, dl: s.dl, uT: u[0], uD: u[1], info });
      stepB(s, u, p, dt, opts.wind ? opts.wind(s.t) : 0);
    }
    return { s, log };
  }
  // LQR controller factory (absolute inputs; optional clipping to actuator limits)
  function makeLQRController(K, target, p, clip = true) {
    return (t, x) => {
      const dx = M.vsub(x, typeof target === 'function' ? target(t) : target);
      const u = M.vscale(M.matvec(K, dx), -1);            // [ΔT, δ]
      let T = p.m * DEFAULTS.g + u[0], d = u[1];
      const raw = [T, d];
      if (clip) { T = clamp(T, p.Tmin, p.Tmax); d = clamp(d, -p.deltaMax, p.deltaMax); }
      return { u: [T, d], raw };
    };
  }

  return { DEFAULTS, makeRng, M, clamp, makeStateA, stepA, throttleToThrust, hoverThrottle,
           makeStateB, stepB, linearizeB, c2d, PID, dlqr, eigenvalues, lqrWeightsFromSliders,
           buildPrediction, makeMPC, minJerk, openLoopThrust, runA, runB, makeLQRController };
});
