/* GTPL controls guide: sim "mpc" — MPC vs clipped LQR on the planar rocket (model B) with input limits and a
   soft glide-slope constraint. Toy numbers from the repo: 1 kg, I = 0.2 kg m^2, L = 1 m, thrust 1..20 N, gimbal +/-10 deg.
   Physics 240 Hz; LQR every 5 steps (48 Hz, its own discretization step); MPC re-solved every 24 steps (10 Hz, model step 0.1 s).
   Solver: condensed linear MPC (X = Sx x0 + Su U), Jacobi-preconditioned accelerated projected gradient (FISTA + restart +
   backtracking), warm start = previous solution shifted one step. Same algorithm as research/simspec.js, ported to flat buffers. */
(function () {
  'use strict';
  const G = window.GTPL;

  /* ============================== pure math (no DOM) ============================== */
  const DEG = Math.PI / 180;
  const params = {
    g: 9.81,
    m: 1.0, I: 0.2, L: 1.0,                    // kg, kg m^2, m (CoM to nozzle pivot)
    Tmin: 1.0, Tmax: 20.0,                     // N (1 N = flameout floor)
    deltaMax: 10 * DEG,                        // rad
    tau: 0.2, tauServo: 0.05,                  // s thrust lag, servo lag (truth model only)
    dt: 1 / 240,                               // s physics step
    lqrDiv: 5,                                 // LQR every 5 steps = 48 Hz (dtCtrl = 5/240 s)
    mpcDiv: 24,                                // MPC every 24 steps = 10 Hz
    dtMpc: 0.1,                                // s internal model step of the MPC
    N: 10,                                     // default horizon (the repo uses 10)
    maxIter: 200, rho: 2000,                   // active-set iteration cap; soft cone penalty weight
    terminal: 'stage',                         // terminal cost: 'stage' (P = Q, what the horizon slider needs to mean something), 'lqr' (P = Riccati), or a number (P = k Q)
    cone: { k: 0.5, margin: 0.5 },             // |x| <= k z + margin  (26.6 deg half-angle)
    uScale: [3, 0.1],                          // N, rad: decision variables are u / uScale
    Q: [1, 1, 25, 1, 1, 4],                    // x z theta vx vz omega  (Bryson 1/max^2: 1 m, 1 m, 0.2 rad, 1 m/s, 1 m/s, 0.5 rad/s)
    R: [2, 300],                               // dT, delta: raised from Bryson (0.01, 33) so the toy rocket's default LQR stays inside its limits from 10 m
    xFrac: 0.45, vz0: -2.0,                    // start x = xFrac * z0, already descending at 2 m/s
    zRef: -0.25,                               // m: aim a quarter metre below the pad so touchdown is decisive; the ground stops the rocket
    gustN: 3.0, gustDur: 2.0,                  // lateral force toward -x (across the pad), seconds
    ring: 64,                                  // latency ring buffer (states), 64/240 s = 0.27 s > 200 ms
  };

  /* ---- small dense-matrix helpers (setup time only; arrays of arrays) ---- */
  const M = {
    zeros: (r, c) => Array.from({ length: r }, () => new Array(c).fill(0)),
    eye: (n) => { const I = M.zeros(n, n); for (let i = 0; i < n; i++) I[i][i] = 1; return I; },
    diag: (d) => { const D = M.zeros(d.length, d.length); d.forEach((v, i) => { D[i][i] = v; }); return D; },
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
    normInf: (A) => { let m = 0; for (const r of A) for (const v of r) if (Math.abs(v) > m) m = Math.abs(v); return m; },
    inv(A) {
      const n = A.length, X = A.map((r, i) => r.concat(M.eye(n)[i]));
      for (let c = 0; c < n; c++) {
        let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(X[r][c]) > Math.abs(X[p][c])) p = r;
        const tmp = X[c]; X[c] = X[p]; X[p] = tmp;
        const piv = X[c][c]; if (Math.abs(piv) < 1e-14) throw new Error('singular');
        for (let j = 0; j < 2 * n; j++) X[c][j] /= piv;
        for (let r = 0; r < n; r++) if (r !== c) { const f = X[r][c]; if (f !== 0) for (let j = 0; j < 2 * n; j++) X[r][j] -= f * X[c][j]; }
      }
      return X.map(r => r.slice(n));
    },
  };
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const now = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

  /* ---- model B truth plant: x = [x, z, th, vx, vz, om]; actuators lag; semi-implicit Euler ---- */
  function makePlant(x0) {
    return { x: Float64Array.from(x0), T: params.m * params.g, dl: 0, t: 0 };
  }
  /* u = [T (N), delta (rad)]; windN lateral force. Returns the pre-clamp vz if the ground was hit this step, else null. */
  function stepPlant(s, u, dt, windN) {
    const p = params;
    const Tc = clamp(u[0], p.Tmin, p.Tmax), dc = clamp(u[1], -p.deltaMax, p.deltaMax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    s.dl += (dc - s.dl) * (1 - Math.exp(-dt / p.tauServo));
    const X = s.x, ang = X[2] + s.dl;
    const ax = (s.T * Math.sin(ang) + (windN || 0)) / p.m;
    const az = s.T * Math.cos(ang) / p.m - p.g;
    const aom = -p.L * s.T * Math.sin(s.dl) / p.I;
    X[3] += ax * dt; X[4] += az * dt; X[5] += aom * dt;
    X[0] += X[3] * dt; X[1] += X[4] * dt; X[2] += X[5] * dt;
    s.t += dt;
    if (X[1] <= 0) { const vz = X[4]; X[1] = 0; if (X[4] < 0) X[4] = 0; return vz; }
    return null;
  }
  function coneViolation(x, z) { return Math.abs(x) - (params.cone.k * z + params.cone.margin); }

  /* ---- linearization about hover, exact discretization (A nilpotent) ---- */
  function linearize() {
    const p = params, g = p.g, A = M.zeros(6, 6), B = M.zeros(6, 2);
    A[0][3] = 1; A[1][4] = 1; A[2][5] = 1; A[3][2] = g;
    B[3][1] = g; B[4][0] = 1 / p.m; B[5][1] = -p.L * p.m * g / p.I;
    return { A, B };
  }
  function c2d(A, B, dt) {
    const n = A.length, m = B[0].length, N = n + m, Mb = M.zeros(N, N);
    for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) Mb[i][j] = A[i][j] * dt; for (let j = 0; j < m; j++) Mb[i][n + j] = B[i][j] * dt; }
    let E = M.eye(N), term = M.eye(N);
    for (let k = 1; k <= 20; k++) { term = M.scale(M.mul(term, Mb), 1 / k); E = M.add(E, term); if (M.normInf(term) < 1e-16) break; }
    return { Ad: E.slice(0, n).map(r => r.slice(0, n)), Bd: E.slice(0, n).map(r => r.slice(n)) };
  }
  function dlqr(Ad, Bd, Q, R) {
    const At = M.T(Ad), Bt = M.T(Bd);
    let P = M.clone(Q), K = null, it = 0;
    for (; it < 5000; it++) {
      const BtP = M.mul(Bt, P);
      K = M.mul(M.inv(M.add(R, M.mul(BtP, Bd))), M.mul(BtP, Ad));
      const AtP = M.mul(At, P);
      const Pn = M.add(Q, M.sub(M.mul(AtP, Ad), M.mul(M.mul(AtP, Bd), K)));
      const d = M.normInf(M.sub(Pn, P)); P = Pn;
      if (d < 1e-9 * Math.max(1, M.normInf(P))) break;
    }
    return { K, P, iters: it + 1 };
  }
  function weights(o) {
    const q = params.Q.slice(), r = params.R.slice();
    if (o) { q[0] *= o.posW || 1; q[1] *= o.posW || 1; q[2] *= o.angleW || 1; r[0] *= o.thrustR || 1; r[1] *= o.gimbalR || 1; }
    return { Q: M.diag(q), R: M.diag(r) };
  }

  /* ---- LQR lane controller: u = u_hover - K (x - target), clipped afterwards ---- */
  function makeLQR(o) {
    const p = params, dtc = p.dt * p.lqrDiv;
    const { A, B } = linearize(); const { Ad, Bd } = c2d(A, B, dtc);
    const { Q, R } = weights(o && o.weights);
    const { K } = dlqr(Ad, Bd, Q, R);
    const k = [Float64Array.from(K[0]), Float64Array.from(K[1])];
    const out = { u: [p.m * p.g, 0], raw: [p.m * p.g, 0], ms: 0 };
    function control(x, target) {
      const t0 = now();
      let dT = 0, dd = 0;
      for (let j = 0; j < 6; j++) { const e = x[j] - target[j]; dT -= k[0][j] * e; dd -= k[1][j] * e; }
      out.raw[0] = p.m * p.g + dT; out.raw[1] = dd;
      out.u[0] = clamp(out.raw[0], p.Tmin, p.Tmax); out.u[1] = clamp(dd, -p.deltaMax, p.deltaMax);
      out.ms = now() - t0;
      return out;
    }
    return { K, control, dtCtrl: dtc };
  }

  /* ---- MPC: condensed QP with box input constraints and a soft glide-slope penalty ---- */
  function makeMPC(o) {
    o = o || {};
    const p = params, n = 6, m = 2, N = o.N || p.N, mN = m * N, nN = n * N;
    const cone = o.cone === undefined ? p.cone : o.cone;      // null disables the state constraint
    const rho = o.rho === undefined ? p.rho : o.rho, maxIter = o.maxIter || p.maxIter;
    const { A, B } = linearize(); const { Ad, Bd } = c2d(A, B, p.dtMpc);
    const { Q, R } = weights(o.weights);
    const term = o.terminal || p.terminal;
    const P = term === 'lqr' ? dlqr(Ad, Bd, Q, R).P : M.scale(Q, term === 'stage' ? 1 : term);   // 'lqr': infinite-horizon value (unconstrained MPC == LQR); 'stage': plain stage cost, or a multiplier on Q
    const us = p.uScale, uMin = [p.Tmin - p.m * p.g, -p.deltaMax], uMax = [p.Tmax - p.m * p.g, p.deltaMax];
    /* prediction matrices with B columns pre-scaled by uScale */
    const Bs = Bd.map(r => r.map((v, j) => v * us[j]));
    const Apow = [M.eye(n)]; for (let k = 1; k <= N; k++) Apow.push(M.mul(Ad, Apow[k - 1]));
    const Sx = new Float64Array(nN * n), Su = new Float64Array(nN * mN);
    for (let k = 0; k < N; k++) {
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) Sx[(k * n + i) * n + j] = Apow[k + 1][i][j];
      for (let j = 0; j <= k; j++) { const blk = M.mul(Apow[k - j], Bs);
        for (let i = 0; i < n; i++) for (let jj = 0; jj < m; jj++) Su[(k * n + i) * mN + j * m + jj] = blk[i][jj]; }
    }
    /* Qbar blocks (Q ... Q, P) flat, and H = Su' Qbar Su + Rbar (Rbar in scaled units) */
    const Qb = new Float64Array(N * n * n);
    for (let k = 0; k < N; k++) { const Qk = k === N - 1 ? P : Q; for (let i = 0; i < n; i++) for (let t = 0; t < n; t++) Qb[k * n * n + i * n + t] = Qk[i][t]; }
    const QSu = new Float64Array(nN * mN);
    for (let k = 0; k < N; k++) for (let i = 0; i < n; i++) for (let j = 0; j < mN; j++) {
      let s = 0; for (let t = 0; t < n; t++) s += Qb[k * n * n + i * n + t] * Su[(k * n + t) * mN + j]; QSu[(k * n + i) * mN + j] = s; }
    const H = new Float64Array(mN * mN);
    for (let i = 0; i < mN; i++) for (let j = 0; j < mN; j++) { let s = 0; for (let r = 0; r < nN; r++) s += Su[r * mN + i] * QSu[r * mN + j]; H[i * mN + j] = s; }
    for (let k = 0; k < N; k++) for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) H[(k * m + i) * mN + k * m + j] += R[i][j] * us[i] * us[j];
    for (let i = 0; i < mN; i++) { for (let j = i + 1; j < mN; j++) { const v = 0.5 * (H[i * mN + j] + H[j * mN + i]); H[i * mN + j] = v; H[j * mN + i] = v; } H[i * mN + i] += 1e-8; }
    /* Jacobi preconditioning (diagonal rescale keeps the box a box) */
    const sJ = new Float64Array(mN); for (let i = 0; i < mN; i++) sJ[i] = 1 / Math.sqrt(H[i * mN + i]);
    for (let i = 0; i < mN; i++) for (let j = 0; j < mN; j++) H[i * mN + j] *= sJ[i] * sJ[j];
    for (let r = 0; r < nN; r++) for (let j = 0; j < mN; j++) Su[r * mN + j] *= sJ[j];
    const scale = new Float64Array(mN), lo = new Float64Array(mN), hi = new Float64Array(mN);
    for (let j = 0; j < mN; j++) { scale[j] = us[j % m] * sJ[j]; lo[j] = uMin[j % m] / scale[j]; hi[j] = uMax[j % m] / scale[j]; }
    /* ---- solver: primal active-set method on the box-constrained QP (exact: the unconstrained answer is the LQR input, verified
       in test-mpc.js). Variables sit either free or pinned at a bound; each iteration solves the free block by Cholesky, walks
       toward that answer until a bound blocks, or releases the pinned variable with the worst multiplier. The soft cone penalty is
       piecewise quadratic in U (given the sign of x_k and the set of violated stages), so an outer loop adds 2 rho a_k a_k' to H
       for every violated stage and repeats until that set is stable. The repo's plain projected gradient cannot converge on this
       horizon (gimbal -> x is a triple integrator, so H is very badly conditioned); this is the "solver you need" take-home. ---- */
    const maxPass = 6;
    const Heff = new Float64Array(mN * mN), feff = new Float64Array(mN), Hr = new Float64Array(mN * mN), rr = new Float64Array(mN), uF = new Float64Array(mN);
    const status = new Int8Array(mN), idxF = new Int32Array(mN), g = new Float64Array(mN);
    /* Cholesky of the reduced free block of Heff, then solve  Heff_FF uF = rhs  (rr on input, uF on output) */
    function solveFree(nF) {
      for (let a = 0; a < nF; a++) { const ia = idxF[a] * mN; for (let b = 0; b <= a; b++) Hr[a * mN + b] = Heff[ia + idxF[b]]; }
      for (let i = 0; i < nF; i++) for (let j = 0; j <= i; j++) {
        let s = Hr[i * mN + j]; for (let k = 0; k < j; k++) s -= Hr[i * mN + k] * Hr[j * mN + k];
        if (i === j) { if (s <= 1e-300) return false; Hr[i * mN + i] = Math.sqrt(s); } else Hr[i * mN + j] = s / Hr[j * mN + j]; }
      for (let i = 0; i < nF; i++) { let s = rr[i]; for (let k = 0; k < i; k++) s -= Hr[i * mN + k] * uF[k]; uF[i] = s / Hr[i * mN + i]; }
      for (let i = nF - 1; i >= 0; i--) { let s = uF[i]; for (let k = i + 1; k < nF; k++) s -= Hr[k * mN + i] * uF[k]; uF[i] = s / Hr[i * mN + i]; }
      return true;
    }
    /* minimize 0.5 Z' Heff Z + feff' Z  s.t. lo <= Z <= hi, starting from the feasible Z. Returns iterations used. */
    function boxQP() {
      const eps = 1e-10;
      for (let i = 0; i < mN; i++) status[i] = Z[i] <= lo[i] + eps ? -1 : Z[i] >= hi[i] - eps ? 1 : 0;
      let it = 0;
      for (; it < maxIter; it++) {
        let nF = 0; for (let i = 0; i < mN; i++) if (status[i] === 0) idxF[nF++] = i;
        if (nF > 0) {
          for (let a = 0; a < nF; a++) { const i = idxF[a], row = i * mN; let s = -feff[i];
            for (let j = 0; j < mN; j++) if (status[j] !== 0) s -= Heff[row + j] * Z[j]; rr[a] = s; }
          if (!solveFree(nF)) break;
          /* walk from Z_F toward uF until the first bound blocks */
          let t = 1, block = -1, side = 0;
          for (let a = 0; a < nF; a++) { const i = idxF[a], dz = uF[a] - Z[i];
            if (dz < -eps && uF[a] < lo[i]) { const ti = (lo[i] - Z[i]) / dz; if (ti < t) { t = ti; block = i; side = -1; } }
            else if (dz > eps && uF[a] > hi[i]) { const ti = (hi[i] - Z[i]) / dz; if (ti < t) { t = ti; block = i; side = 1; } } }
          for (let a = 0; a < nF; a++) { const i = idxF[a]; Z[i] = clamp(Z[i] + t * (uF[a] - Z[i]), lo[i], hi[i]); }
          if (block >= 0) { status[block] = side; Z[block] = side < 0 ? lo[block] : hi[block]; continue; }
        }
        /* free block optimal: check the multipliers of the pinned variables */
        let gmax = 0; for (let i = 0; i < mN; i++) { let s = feff[i]; const row = i * mN; for (let j = 0; j < mN; j++) s += Heff[row + j] * Z[j]; g[i] = s; if (Math.abs(s) > gmax) gmax = Math.abs(s); }
        const tolK = 1e-7 * (1 + gmax); let worst = -1, wv = tolK;
        for (let i = 0; i < mN; i++) { const v = status[i] === -1 ? -g[i] : status[i] === 1 ? g[i] : 0; if (v > wv) { wv = v; worst = i; } }
        if (worst < 0) { it++; break; }
        status[worst] = 0;
      }
      return it;
    }

    /* work buffers, allocated once */
    const Sxx0 = new Float64Array(nN), d = new Float64Array(nN), Qd = new Float64Array(nN), f = new Float64Array(mN), tmp = new Float64Array(mN);
    const Z = new Float64Array(mN), X = new Float64Array(nN), ak = new Float64Array(mN), active = new Int8Array(N), activePrev = new Int8Array(N), sgn = new Float64Array(N);
    const result = { u0: [0, 0], Xpred: new Float64Array((N + 1) * n), Uplan: new Float64Array(mN), Uscaled: new Float64Array(mN), cost: 0, iters: 0, passes: 0, ms: 0, coneViolPred: 0, N, ok: true };

    function predict(Uv, Xout) { for (let r = 0; r < nN; r++) { let s = Sxx0[r]; const row = r * mN; for (let j = 0; j < mN; j++) s += Su[row + j] * Uv[j]; Xout[r] = s; } }
    function penalty(Xv) {
      if (!cone) return 0; let J = 0;
      for (let k = 0; k < N; k++) { const viol = Math.abs(Xv[k * n]) - (cone.k * Xv[k * n + 1] + cone.margin); if (viol > 0) J += rho * viol * viol; }
      return J;
    }
    let c0 = 0;
    function matvecH(v, out) { for (let i = 0; i < mN; i++) { let s = 0; const row = i * mN; for (let j = 0; j < mN; j++) s += H[row + j] * v[j]; out[i] = s; } }
    function cost(Uv, Xv) { matvecH(Uv, tmp); let s = 0; for (let i = 0; i < mN; i++) s += Uv[i] * (0.5 * tmp[i] + f[i]); return s + c0 + penalty(Xv); }
    /* violated stages at the current prediction; returns true if the set (or a sign) changed */
    function updateActive(Xv) {
      let changed = false;
      for (let k = 0; k < N; k++) { activePrev[k] = active[k];
        const x = Xv[k * n], viol = Math.abs(x) - (cone.k * Xv[k * n + 1] + cone.margin);
        const a = viol > 0 ? 1 : 0, s = x >= 0 ? 1 : -1;
        if (a !== activePrev[k] || (a && s !== sgn[k])) changed = true;
        active[k] = a; if (a) sgn[k] = s; }
      return changed;
    }
    /* Heff = H + sum_active 2 rho a_k a_k',  feff = f + sum_active 2 rho b_k a_k;  a_k = s_k Su[x_k] - kc Su[z_k] */
    function buildEffective() {
      Heff.set(H); feff.set(f);
      for (let k = 0; k < N; k++) { if (!active[k]) continue;
        const rx = (k * n) * mN, rz = (k * n + 1) * mN, s = sgn[k];
        for (let j = 0; j < mN; j++) ak[j] = s * Su[rx + j] - cone.k * Su[rz + j];
        const bk = s * Sxx0[k * n] - cone.k * Sxx0[k * n + 1] - cone.margin;
        for (let i = 0; i < mN; i++) { const ai = 2 * rho * ak[i]; feff[i] += ai * bk; const row = i * mN; for (let j = 0; j < mN; j++) Heff[row + j] += ai * ak[j]; } }
    }

    /* x0: current state (6). target: state array (6), broadcast over the horizon. Uwarm: previous Uscaled or null. */
    function solve(x0, target, Uwarm) {
      const t0 = now();
      for (let r = 0; r < nN; r++) { let s = 0; for (let j = 0; j < n; j++) s += Sx[r * n + j] * x0[j]; Sxx0[r] = s; d[r] = s - target[r % n]; }
      for (let k = 0; k < N; k++) for (let i = 0; i < n; i++) { let s = 0; for (let t = 0; t < n; t++) s += Qb[k * n * n + i * n + t] * d[k * n + t]; Qd[k * n + i] = s; }
      for (let j = 0; j < mN; j++) { let s = 0; for (let r = 0; r < nN; r++) s += Su[r * mN + j] * Qd[r]; f[j] = s; }
      c0 = 0; for (let r = 0; r < nN; r++) c0 += 0.5 * d[r] * Qd[r];
      /* warm start: shift one stage in physical units, repeat the last input */
      if (Uwarm) for (let j = 0; j < mN; j++) { const src = Math.min(j + m, mN - m + (j % m)); Z[j] = clamp(Uwarm[src] * scale[src] / scale[j], lo[j], hi[j]); }
      else for (let j = 0; j < mN; j++) Z[j] = clamp(0, lo[j], hi[j]);
      let iters = 0, passes = 0;
      active.fill(0);
      if (cone) { predict(Z, X); updateActive(X); }
      for (let pass = 0; pass < maxPass; pass++) {
        passes++;
        buildEffective();
        iters += boxQP();
        predict(Z, X);
        if (!cone || !updateActive(X)) break;
      }
      /* unpack */
      let ok = true;
      for (let j = 0; j < mN; j++) { const v = Z[j] * scale[j]; result.Uplan[j] = v; result.Uscaled[j] = Z[j]; if (!isFinite(v)) ok = false; }
      result.u0[0] = result.Uplan[0]; result.u0[1] = result.Uplan[1];
      for (let i = 0; i < n; i++) result.Xpred[i] = x0[i];
      for (let r = 0; r < nN; r++) { result.Xpred[n + r] = X[r]; if (!isFinite(X[r])) ok = false; }
      let mv = 0; for (let k = 0; k <= N; k++) mv = Math.max(mv, coneViolation(result.Xpred[k * n], result.Xpred[k * n + 1]));
      const J = cost(Z, X);
      result.coneViolPred = mv; result.cost = J; result.iters = iters; result.passes = passes; result.ok = ok && isFinite(J); result.ms = now() - t0;
      return result;
    }
    return { solve, N, n, m, Ad, Bd, P, cone: !!cone };
  }

  /* ---- scenario ---- */
  function scenario(z0) {
    return { x0: [params.xFrac * z0, z0, 0, 0, params.vz0, 0], target: [0, params.zRef, 0, 0, 0, 0], z0 };
  }

  /* ---- a lane: one plant + one controller + latency ring + metrics. kind: 'lqr' | 'mpc' ---- */
  function makeLane(kind, cfg) {
    cfg = cfg || {};
    const p = params, n = 6;
    const lane = { kind, started: false, landed: false, t: 0 };
    let plant = null, ctrl = null, mpc = null, sc = null, steps = 0;
    const ring = new Float64Array(p.ring * n); let ringLen = 0, ringHead = 0;
    const delayed = new Float64Array(n);
    lane.u = [p.m * p.g, 0]; lane.raw = [p.m * p.g, 0];
    lane.latencyMs = cfg.latencyMs || 0;
    lane.pred = null;           // Float64Array((N+1)*6) from the last MPC solve
    lane.predN = 0; lane.predViol = 0;
    lane.metrics = { solveMs: 0, solves: 0, violations: 0, maxViol: 0, touchdownSpeed: NaN, padError: NaN, tTouch: NaN, nanCount: 0, peakGimbalCmd: 0, peakRawGimbal: 0, minThrust: Infinity, maxThrust: -Infinity, maxTilt: 0, minZ: Infinity, maxAbsX: 0, gimbalSatSteps: 0, ctrlSteps: 0 };
    let Uwarm = null, haveWarm = false;

    lane.design = function (o) {   // o: {N, cone (bool), weights, rho}
      if (kind === 'lqr') { ctrl = makeLQR({ weights: o && o.weights }); }
      else { mpc = makeMPC({ N: o && o.N, cone: o && o.cone === false ? null : p.cone, weights: o && o.weights, rho: o && o.rho }); haveWarm = false; Uwarm = new Float64Array(mpc.N * 2); lane.pred = new Float64Array((mpc.N + 1) * n); lane.predN = 0; }
    };
    lane.arm = function (z0) {     // put the rocket at the start state, hovering, not yet descending
      sc = scenario(z0); plant = makePlant(sc.x0);
      lane.started = false; lane.landed = false; lane.t = 0; steps = 0; ringLen = 0; ringHead = 0; haveWarm = false;
      lane.u[0] = p.m * p.g; lane.u[1] = 0; lane.raw[0] = p.m * p.g; lane.raw[1] = 0;
      lane.predN = 0; lane.predViol = 0;
      const mm = lane.metrics; mm.solveMs = 0; mm.solves = 0; mm.violations = 0; mm.maxViol = 0; mm.touchdownSpeed = NaN; mm.padError = NaN; mm.tTouch = NaN;
      mm.nanCount = 0; mm.peakGimbalCmd = 0; mm.peakRawGimbal = 0; mm.minThrust = Infinity; mm.maxThrust = -Infinity; mm.maxTilt = 0; mm.minZ = Infinity; mm.maxAbsX = 0; mm.gimbalSatSteps = 0; mm.ctrlSteps = 0;
    };
    lane.start = function () { lane.started = true; };
    lane.state = () => plant.x;
    lane.plant = () => plant;
    lane.scenario = () => sc;
    lane.delayedState = () => delayed;
    lane.controlPeriod = () => (kind === 'lqr' ? p.lqrDiv : p.mpcDiv) * p.dt;

    function pushRing() {
      ring.set(plant.x, ringHead * n);
      ringHead = (ringHead + 1) % p.ring; if (ringLen < p.ring) ringLen++;
    }
    function readDelayed() {
      let lag = Math.round(lane.latencyMs / 1000 / p.dt); if (lag > ringLen - 1) lag = ringLen - 1; if (lag < 0) lag = 0;
      const idx = (ringHead - 1 - lag + 2 * p.ring) % p.ring;
      for (let i = 0; i < n; i++) delayed[i] = ring[idx * n + i];
    }
    lane.step = function (dt, windN) {
      if (!lane.started || lane.landed) return;
      const mm = lane.metrics;
      pushRing(); readDelayed();
      const div = kind === 'lqr' ? p.lqrDiv : p.mpcDiv;
      if (steps % div === 0) {
        if (kind === 'lqr') {
          const o = ctrl.control(delayed, sc.target);
          lane.u[0] = o.u[0]; lane.u[1] = o.u[1]; lane.raw[0] = o.raw[0]; lane.raw[1] = o.raw[1];
          mm.solveMs += (o.ms - mm.solveMs) * 0.1;
        } else {
          const r = mpc.solve(delayed, sc.target, haveWarm ? Uwarm : null);
          if (r.ok) {
            lane.u[0] = p.m * p.g + r.u0[0]; lane.u[1] = r.u0[1]; lane.raw[0] = lane.u[0]; lane.raw[1] = lane.u[1];
            Uwarm.set(r.Uscaled); haveWarm = true;
            lane.pred.set(r.Xpred); lane.predN = r.N; lane.predViol = r.coneViolPred;
          } else { mm.nanCount++; haveWarm = false; }        // keep the previous command
          mm.solveMs += (r.ms - mm.solveMs) * 0.1;
        }
        mm.solves++; mm.ctrlSteps++;
        if (Math.abs(lane.raw[1]) >= p.deltaMax - 1e-9) mm.gimbalSatSteps++;
        mm.peakGimbalCmd = Math.max(mm.peakGimbalCmd, Math.abs(lane.u[1])); mm.peakRawGimbal = Math.max(mm.peakRawGimbal, Math.abs(lane.raw[1]));
        mm.minThrust = Math.min(mm.minThrust, lane.u[0]); mm.maxThrust = Math.max(mm.maxThrust, lane.u[0]);
      }
      if (steps % p.mpcDiv === 0) {   // cone bookkeeping at a common 10 Hz for both lanes
        const v = coneViolation(plant.x[0], plant.x[1]);
        if (v > 0) { mm.violations++; if (v > mm.maxViol) mm.maxViol = v; }
      }
      const hit = stepPlant(plant, lane.u, dt, windN);
      steps++; lane.t = plant.t;
      const X = plant.x;
      if (!isFinite(X[0]) || !isFinite(X[1]) || !isFinite(X[2]) || !isFinite(X[3]) || !isFinite(X[4]) || !isFinite(X[5])) { lane.arm(sc.z0); mm.nanCount++; return; }
      mm.maxTilt = Math.max(mm.maxTilt, Math.abs(X[2])); mm.minZ = Math.min(mm.minZ, X[1]); mm.maxAbsX = Math.max(mm.maxAbsX, Math.abs(X[0]));
      if (hit !== null) {
        lane.landed = true; mm.touchdownSpeed = Math.abs(hit); mm.padError = Math.abs(X[0]); mm.tTouch = plant.t;
        X[3] = 0; X[4] = 0; X[5] = 0; lane.u[0] = p.Tmin; lane.u[1] = 0; plant.T = p.Tmin;
      }
    };
    lane.design(cfg); lane.arm(cfg.z0 || 10);
    return lane;
  }

  /* ---- headless run for tests: returns the lane metrics ---- */
  function runScenario(o) {
    const lane = makeLane(o.controller, { N: o.N, cone: o.cone !== false, weights: o.weights, latencyMs: o.latencyMs || 0, z0: o.z0 || 10 });
    lane.start();
    const dt = params.dt, tEnd = o.tEnd || 25; let gustLeft = 0, gustFired = false;
    const log = o.log ? [] : null;
    for (let i = 0; lane.t < tEnd && !lane.landed; i++) {
      if (!gustFired && o.gustAt !== undefined && lane.t >= o.gustAt) { gustFired = true; gustLeft = params.gustDur; }
      const w = gustLeft > 0 ? -params.gustN : 0; if (gustLeft > 0) gustLeft -= dt;
      lane.step(dt, w);
      if (log && i % 12 === 0) log.push({ t: lane.t, x: Array.from(lane.state()), u: lane.u.slice(), raw: lane.raw.slice() });
    }
    return { lane, metrics: lane.metrics, landed: lane.landed, t: lane.t, x: Array.from(lane.state()), log };
  }

  const math = { params, M, clamp, makePlant, stepPlant, coneViolation, linearize, c2d, dlqr, weights, makeLQR, makeMPC, scenario, makeLane, runScenario, DEG };
  G.math = G.math || {}; G.math.mpc = math;

  /* ============================== mount (all DOM work) ============================== */
  G.sims = G.sims || {};
  G.sims.mpc = function mount(root) {
    const p = math.params, el = G.el;
    const DEF = { mode: 'mpc', N: p.N, latencyMs: 0, cone: true, z0: 10, w: { posW: 1, angleW: 1, gimbalR: 1, thrustR: 1 }, rho: p.rho };
    let mode = DEF.mode, N = DEF.N, latencyMs = DEF.latencyMs, coneOn = DEF.cone, z0 = DEF.z0;
    const w = Object.assign({}, DEF.w); let rho = DEF.rho;
    const mpcDesign = () => ({ N, cone: coneOn, weights: w, rho });
    let gustLeft = 0, chartTick = 0;
    const rand = G.rng(11);

    const ui = G.scaffold(root, { wide: false });
    const stage = G.canvas(ui.stage, { aspect: 1.5, minHeight: 380, maxHeight: 640 });
    const ctx = stage.ctx;

    const lanes = { lqr: math.makeLane('lqr', { z0 }), mpc: math.makeLane('mpc', { N, cone: true, z0 }) };
    const activeLanes = () => (mode === 'both' ? [lanes.lqr, lanes.mpc] : [lanes[mode]]);
    const LANE_TITLE = { lqr: 'LQR (clipped)', mpc: 'MPC' };

    /* ---- charts (rebuilt on mode change: the series set differs) ---- */
    let chGimbal = null, chAlt = null;
    function buildCharts() {
      let gs, as;
      if (mode === 'both') {
        gs = [{ key: 'lqr', color: '--sig-fb', label: 'LQR' }, { key: 'mpc', color: '--sig-act', label: 'MPC' }];
        as = [{ key: 'lqr', color: '--sig-fb', label: 'LQR' }, { key: 'mpc', color: '--sig-act', label: 'MPC' }];
      } else if (mode === 'lqr') {
        gs = [{ key: 'raw', color: '--sig-fb', dash: [3, 3], label: 'unclipped', endDot: false }, { key: 'cmd', color: '--sig-act', label: 'command' }];
        as = [{ key: 'z', color: '--sig-act', label: 'altitude' }];
      } else {
        gs = [{ key: 'cmd', color: '--sig-act', label: 'command' }];
        as = [{ key: 'z', color: '--sig-act', label: 'altitude' }];
      }
      chGimbal = new G.StripChart({ duration: 12, autoscale: true, ymin: -12, ymax: 12, padding: 0.12, yLabel: 'gimbal (deg)', xLabel: 't (s)', series: gs,
        thresholds: [{ y: 10, color: '--bad', label: '+10 deg limit' }, { y: -10, color: '--bad', label: '−10 deg limit' }] });
      chAlt = new G.StripChart({ duration: 12, autoscale: true, floor: 0, ymin: 0, ymax: 12, yLabel: 'altitude (m)', xLabel: 't (s)', series: as,
        thresholds: [{ y: 0, color: '--sig-ref', dash: [6, 5], label: 'pad' }] });
    }
    buildCharts();

    /* ---- HUD: one block of readouts per lane ---- */
    ui.hud.style.cssText = 'display:flex;flex-direction:column;gap:.5rem';
    const hud = {};
    ['lqr', 'mpc'].forEach(k => {
      const ro = {
        solve: G.readout({ label: 'Solve time', unit: 'ms', digits: 2 }),
        viol: G.readout({ label: 'Cone violations', unit: 'steps', digits: 0 }),
        maxViol: G.readout({ label: 'Max violation', unit: 'm', digits: 2 }),
        vtd: G.readout({ label: 'Touchdown speed', unit: 'm/s', digits: 2 }),
        xerr: G.readout({ label: 'Pad error', unit: 'm', digits: 2 }),
      };
      const title = el('div', { class: 'ctl-title', text: LANE_TITLE[k] });
      const grid = el('div', { class: 'hud' }, [ro.solve.root, ro.viol.root, ro.maxViol.root, ro.vtd.root, ro.xerr.root]);
      const block = el('div', {}, [title, grid]);
      ui.hud.appendChild(block);
      hud[k] = { ro, block, title };
    });
    function syncHud() {
      ['lqr', 'mpc'].forEach(k => { hud[k].block.hidden = !(mode === 'both' || mode === k); hud[k].title.hidden = mode !== 'both'; });
    }

    /* ---- controls ---- */
    const segMode = G.segmented({ label: 'Controller', value: mode, options: [{ label: 'LQR (clipped)', value: 'lqr' }, { label: 'MPC', value: 'mpc' }, { label: 'Both', value: 'both' }],
      onChange: v => { mode = v; buildCharts(); rearm(); syncHud(); syncCaption(); loop.renderOnce(); } });
    const sN = G.slider({ label: 'Horizon N', unit: 'steps', min: 5, max: 30, step: 1, value: N, digits: 0,
      onInput: v => { N = v; lanes.mpc.design(mpcDesign()); } });
    const sLat = G.slider({ label: 'State latency', unit: 'ms', min: 0, max: 200, step: 10, value: latencyMs, digits: 0,
      onInput: v => { latencyMs = v; lanes.lqr.latencyMs = v; lanes.mpc.latencyMs = v; } });
    const tCone = G.toggle({ label: 'Glide-slope constraint', value: coneOn, onChange: v => { coneOn = v; lanes.mpc.design(mpcDesign()); } });
    /* Q and R weights, shared by both controllers so the same prices can be compared; log sliders in "× default" */
    const fmtX = v => G.fmt(Math.pow(10, v), Math.pow(10, v) < 1 ? 2 : 1) + ' × default';
    function weightSlider(label, key) {
      return G.slider({ label, min: -1, max: 1.5, step: 0.05, value: 0, format: fmtX,
        onInput: v => { w[key] = Math.pow(10, v); lanes.lqr.design({ weights: w }); lanes.mpc.design(mpcDesign()); } });
    }
    const sPos = weightSlider('Care about position (Q)', 'posW');
    const sAng = weightSlider('Care about tilt (Q)', 'angleW');
    const sGim = weightSlider('Gimbal effort (R)', 'gimbalR');
    const sThr = weightSlider('Thrust effort (R)', 'thrustR');
    const sRho = G.slider({ label: 'Cone penalty', min: 1, max: 5, step: 0.1, value: Math.log10(DEF.rho),
      format: v => G.fmt(Math.pow(10, v), 0),
      onInput: v => { rho = Math.pow(10, v); lanes.mpc.design(mpcDesign()); } });
    const segAlt = G.segmented({ label: 'Start altitude', value: z0, options: [{ label: '10 m', value: 10 }, { label: '20 m', value: 20 }],
      onChange: v => { z0 = v; rearm(); loop.renderOnce(); } });
    const bStart = G.button({ label: 'Start descent', kind: 'primary', onClick: startDescent });
    const bGust = G.button({ label: 'Gust', onClick: () => { if (activeLanes().some(l => l.started && !l.landed)) gustLeft = p.gustDur; } });
    const bReset = G.button({ label: 'Reset', onClick: reset });
    ui.controls.append(
      G.group('Controller', [segMode.root]),
      G.group('MPC', [sN.root, tCone.root, sRho.root]),
      G.group('Weights, both controllers', [sPos.root, sAng.root, sGim.root, sThr.root]),
      G.group('Scenario', [segAlt.root, sLat.root]),
      el('div', { class: 'btn-row' }, [bStart.root, bGust.root, bReset.root]));

    const caption = el('p', { class: 'caption' });
    const CAPTIONS = {
      mpc: 'MPC on the planar rocket. The faint trace ahead of the vehicle is the predicted path the solver just chose. Only its first command is applied.',
      lqr: 'LQR on the same rocket. The dashed gimbal trace is what K asks for before the command is clipped to the 10 deg limit.',
      both: 'Same vehicle, same landing, same gust. Left is LQR with commands clipped at the limits. Right is MPC with the limits inside the solver.',
    };
    function syncCaption() { caption.textContent = CAPTIONS[mode]; }
    ui.foot.append(caption, G.legend([{ label: 'predicted path', color: '--sig-act' }, { label: 'glide slope', color: '--sig-ref', dash: true }, { label: 'limit', color: '--bad', dash: true }]));
    root.appendChild(G.hidden('Side view of a planar rocket descending to a landing pad inside a glide-slope cone, with the controller’s predicted path drawn ahead of it and gauges for the gimbal and thrust limits.'));
    syncHud(); syncCaption();

    /* ---- run control ---- */
    function rearm() {
      lanes.lqr.arm(z0); lanes.mpc.arm(z0);
      gustLeft = 0; chartTick = 0;
      chGimbal.clear(); chAlt.clear();
      /* seed the altitude chart with the start altitude so its scale reads 0..z0 before the run starts */
      if (mode === 'both') chAlt.push(0, { lqr: z0, mpc: z0 }); else chAlt.push(0, { z: z0 });
    }
    function startDescent() {
      const act = activeLanes();
      if (act.some(l => l.started)) rearm();
      act.forEach(l => l.start());
      loop.start();
    }
    function reset() {
      mode = DEF.mode; N = DEF.N; latencyMs = DEF.latencyMs; coneOn = DEF.cone; z0 = DEF.z0;
      Object.assign(w, DEF.w); rho = DEF.rho;
      segMode.set(mode, true); sN.set(N, true); sLat.set(latencyMs, true); tCone.set(coneOn, true); segAlt.set(z0, true);
      [sPos, sAng, sGim, sThr].forEach(s => s.set(0, true)); sRho.set(Math.log10(DEF.rho), true);
      lanes.lqr.latencyMs = 0; lanes.mpc.latencyMs = 0;
      lanes.lqr.design({ weights: w }); lanes.mpc.design(mpcDesign());
      buildCharts(); rearm(); syncHud(); syncCaption(); loop.renderOnce();
    }

    /* ---- physics step (fixed dt from GTPL.loop); the MPC solve happens inside lane.step at 10 Hz ---- */
    function step(dt) {
      const act = activeLanes();
      const w = gustLeft > 0 ? -p.gustN : 0;
      if (gustLeft > 0) gustLeft -= dt;
      let t = 0, any = false;
      for (let i = 0; i < act.length; i++) { const l = act[i]; l.step(dt, w); if (l.started) { any = true; if (l.t > t) t = l.t; } }
      if (!any) return;
      chartTick++;
      if (chartTick % 4 === 0) {
        if (mode === 'both') {
          const a = lanes.lqr, b = lanes.mpc;
          chGimbal.push(t, { lqr: a.landed ? undefined : a.u[1] / DEG, mpc: b.landed ? undefined : b.u[1] / DEG });
          chAlt.push(t, { lqr: a.landed ? undefined : a.state()[1], mpc: b.landed ? undefined : b.state()[1] });
        } else {
          const l = act[0];
          if (!l.landed) { chGimbal.push(t, { cmd: l.u[1] / DEG, raw: l.raw[1] / DEG }); chAlt.push(t, { z: l.state()[1] }); }
        }
      }
    }

    /* ---- drawing ---- */
    function drawGauge(x, y, w, o) {
      const g = G.theme.get;
      ctx.font = '10px ' + g('--font-ui'); ctx.textBaseline = 'middle'; ctx.textAlign = 'left'; ctx.fillStyle = g('--ink-3');
      ctx.fillText(o.label, x, y);
      ctx.font = '10px ' + g('--font-mono'); ctx.textAlign = 'right'; ctx.fillStyle = g('--ink-2');
      ctx.fillText(G.fmt(o.value, o.digits) + ' ' + o.unit, x + w, y);
      const ty = y + 10, th = 6;
      const sx = (v) => x + (G.clamp(v, o.min, o.max) - o.min) / (o.max - o.min) * w;
      ctx.fillStyle = g('--bg-3'); ctx.fillRect(x, ty, w, th);
      if (o.fill !== undefined) { ctx.fillStyle = G.theme.alpha('--sig-act', 0.35); const a = sx(o.zero === undefined ? o.min : o.zero), b = sx(o.fill); ctx.fillRect(Math.min(a, b), ty, Math.abs(b - a), th); }
      (o.marks || []).forEach(mk => { ctx.strokeStyle = g('--line-2'); ctx.lineWidth = 1; const mx = Math.round(sx(mk)) + 0.5; ctx.beginPath(); ctx.moveTo(mx, ty - 2); ctx.lineTo(mx, ty + th + 2); ctx.stroke(); });
      /* limits at both ends */
      ctx.strokeStyle = g('--bad'); ctx.lineWidth = 1.5;
      [x + 0.5, x + w - 0.5].forEach(lx => { ctx.beginPath(); ctx.moveTo(lx, ty - 3); ctx.lineTo(lx, ty + th + 3); ctx.stroke(); });
      /* raw (unclipped) marker, hollow, red when outside */
      if (o.raw !== undefined && Math.abs(o.raw - o.value) > 1e-9) {
        const out = o.raw > o.max || o.raw < o.min; const rx = sx(o.raw);
        ctx.strokeStyle = g(out ? '--bad' : '--ink-3'); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(rx, ty + th / 2, 4, 0, Math.PI * 2); ctx.stroke();
        if (out) { ctx.font = '10px ' + g('--font-mono'); ctx.fillStyle = g('--bad'); ctx.textAlign = 'right'; ctx.fillText('asks ' + G.fmt(o.raw, o.digits) + ' ' + o.unit, x + w, ty + th + 9); }
      }
      /* command marker */
      const cx = sx(o.value), atLimit = o.value >= o.max - 1e-9 || o.value <= o.min + 1e-9;
      ctx.fillStyle = g(atLimit ? '--bad' : '--sig-act'); ctx.beginPath(); ctx.arc(cx, ty + th / 2, 4, 0, Math.PI * 2); ctx.fill();
      ctx.font = '9px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3'); ctx.textBaseline = 'top'; ctx.textAlign = 'left'; ctx.fillText(G.fmt(o.min, 0), x, ty + th + 2);
      ctx.textAlign = 'right'; ctx.fillText(G.fmt(o.max, 0), x + w, ty + th + 2);
      return ty + th + 24;
    }

    function drawPanel(rect, lane) {
      const g = G.theme.get, s = lane.plant(), X = s.x, sc = lane.scenario();
      ctx.save(); ctx.beginPath(); ctx.rect(rect.x, rect.y, rect.w, rect.h); ctx.clip();
      /* camera: fixed horizontal scale from the start offset; vertical follows the rocket until the ground reaches the bottom */
      const xr = Math.max(6, Math.abs(sc.x0[0]) + 2.5);
      const pxm = rect.w / (2 * xr), viewH = rect.h / pxm;
      let cz = isFinite(X[1]) ? X[1] : 0; const minC = viewH * 0.5 - 1.5; if (cz < minC) cz = minC;
      const sx = (x) => rect.x + rect.w / 2 + x * pxm, sy = (z) => rect.y + rect.h / 2 - (z - cz) * pxm;
      G.drawGround(ctx, rect, { y0: sy(0), mpp: 1 / pxm, ticks: [5, 10, 15, 20, 25, 30] });
      /* pad */
      ctx.strokeStyle = g('--ink'); ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(sx(-1), sy(0) + 0.5); ctx.lineTo(sx(1), sy(0) + 0.5); ctx.stroke();
      /* glide-slope cone: |x| = k z + margin */
      const zTop = cz + viewH / 2 + 2, k = p.cone.k, mg = p.cone.margin;
      ctx.setLineDash([5, 4]); ctx.strokeStyle = g(coneOn || lane.kind === 'lqr' ? '--sig-ref' : '--ink-3'); ctx.lineWidth = 1.25;
      ctx.beginPath(); ctx.moveTo(sx(-mg), sy(0)); ctx.lineTo(sx(-(k * zTop + mg)), sy(zTop)); ctx.moveTo(sx(mg), sy(0)); ctx.lineTo(sx(k * zTop + mg), sy(zTop)); ctx.stroke(); ctx.setLineDash([]);
      ctx.font = '10px ' + g('--font-ui'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
      const lz = Math.min(zTop - 1, Math.max(3, cz + viewH * 0.25)); ctx.fillText('glide slope', sx(k * lz + mg) + 5, sy(lz));
      /* what the controller sees, when latency is on */
      if (lane.started && !lane.landed && lane.latencyMs > 0) {
        const D = lane.delayedState(); ctx.strokeStyle = g('--ink-3'); ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.arc(sx(D[0]), sy(D[1]), 6, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
        ctx.font = '10px ' + g('--font-ui'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.fillText('seen', sx(D[0]) + 9, sy(D[1]));
      }
      /* predicted horizon: a fading fan of the N+1 predicted positions from the last solve */
      if (lane.kind === 'mpc' && lane.started && !lane.landed && lane.predN > 0) {
        const Pr = lane.pred, n = 6, NN = lane.predN;
        for (let k2 = 0; k2 < NN; k2++) {
          const x0 = Pr[k2 * n], z0p = Pr[k2 * n + 1], x1 = Pr[(k2 + 1) * n], z1 = Pr[(k2 + 1) * n + 1];
          if (!isFinite(x1) || !isFinite(z1)) break;
          const a = 0.15 + 0.75 * (1 - (k2 + 1) / (NN + 1));
          const viol = math.coneViolation(x1, z1) > 0 && coneOn;
          ctx.strokeStyle = G.theme.alpha(viol ? '--bad' : '--sig-act', a); ctx.lineWidth = 2;
          ctx.beginPath(); ctx.moveTo(sx(x0), sy(z0p)); ctx.lineTo(sx(x1), sy(z1)); ctx.stroke();
          ctx.fillStyle = G.theme.alpha(viol ? '--bad' : '--sig-act', a); ctx.beginPath(); ctx.arc(sx(x1), sy(z1), 2.2, 0, Math.PI * 2); ctx.fill();
        }
      }
      /* rocket: the helper's origin is the nozzle pivot with the CoM 0.45 scale above it; our state is the CoM */
      const scale = Math.max(26, p.L / 0.45 * pxm);
      if (isFinite(X[0]) && isFinite(X[1])) G.drawRocket(ctx, { x: sx(X[0]), y: sy(X[1]) + 0.45 * scale, scale, tilt: X[2], gimbal: s.dl, thrust01: (s.T - p.Tmin) / (p.Tmax - p.Tmin), rand });
      /* gust arrow */
      if (gustLeft > 0 && lane.started && !lane.landed) {
        const ax = sx(X[0]) + 0.9 * scale, ay = sy(X[1]) - 0.3 * scale; ctx.strokeStyle = g('--sig-fb'); ctx.fillStyle = g('--sig-fb'); ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(ax + 26, ay); ctx.lineTo(ax + 4, ay); ctx.stroke(); ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax + 7, ay - 4); ctx.lineTo(ax + 7, ay + 4); ctx.closePath(); ctx.fill();
        ctx.font = '10px ' + g('--font-ui'); ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.fillText('gust ' + G.fmt(p.gustN, 0) + ' N', ax + 30, ay);
      }
      /* title + clock (top-left) */
      ctx.font = '11px ' + g('--font-ui'); ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.fillStyle = g('--ink-2');
      ctx.fillText(LANE_TITLE[lane.kind], rect.x + 8, rect.y + 7);
      ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3');
      ctx.fillText('t ' + G.fmt(lane.t, 1) + ' s', rect.x + 8, rect.y + 21);
      /* gauges (top-right) */
      const gw = Math.min(120, rect.w - 24), gx = rect.x + rect.w - gw - 8; let gy = rect.y + 10;
      ctx.fillStyle = G.theme.alpha('--bg-2', 0.85); G.roundRect(ctx, gx - 6, gy - 6, gw + 12, 86, 4); ctx.fill();
      gy = drawGauge(gx, gy, gw, { label: 'Gimbal', unit: 'deg', digits: 1, min: -p.deltaMax / DEG, max: p.deltaMax / DEG, value: lane.u[1] / DEG, raw: lane.raw[1] / DEG, marks: [0] });
      drawGauge(gx, gy, gw, { label: 'Thrust', unit: 'N', digits: 1, min: p.Tmin, max: p.Tmax, value: lane.u[0], raw: lane.kind === 'lqr' ? lane.raw[0] : undefined, marks: [p.m * p.g], fill: s.T, zero: p.Tmin });
      /* status (bottom-left, above the ground fill) */
      ctx.font = '11px ' + g('--font-ui'); ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillStyle = g('--ink-2');
      const mm = lane.metrics;
      let msg = !lane.started ? 'Press Start descent' : lane.landed ? 'Touchdown ' + G.fmt(mm.touchdownSpeed, 2) + ' m/s, ' + G.fmt(mm.padError, 2) + ' m off the pad' : '';
      if (mm.nanCount > 0) msg += (msg ? ' · ' : '') + 'solver failures ' + mm.nanCount;
      if (msg) { ctx.fillStyle = G.theme.alpha('--bg-2', 0.85); const tw = ctx.measureText(msg).width; ctx.fillRect(rect.x + 4, rect.y + rect.h - 22, tw + 8, 18); ctx.fillStyle = g('--ink-2'); ctx.fillText(msg, rect.x + 8, rect.y + rect.h - 6); }
      ctx.restore();
    }

    function render() {
      const W = stage.width, H = stage.height, g = G.theme.get;
      ctx.clearRect(0, 0, W, H);
      const narrow = W < 620;
      let stageRect, gRect, aRect;
      if (!narrow) {
        const sw = Math.round(W * 0.56);
        stageRect = { x: 0, y: 0, w: sw, h: H };
        gRect = { x: sw + 4, y: 4, w: W - sw - 8, h: Math.round(H / 2) - 6 };
        aRect = { x: sw + 4, y: Math.round(H / 2) + 2, w: W - sw - 8, h: Math.round(H / 2) - 6 };
      } else {
        const sh = Math.round(H * 0.6);
        stageRect = { x: 0, y: 0, w: W, h: sh };
        gRect = { x: 2, y: sh + 4, w: Math.round(W / 2) - 4, h: H - sh - 8 };
        aRect = { x: Math.round(W / 2) + 2, y: sh + 4, w: Math.round(W / 2) - 4, h: H - sh - 8 };
      }
      const act = activeLanes();
      if (act.length === 2) {
        const pw = Math.floor(stageRect.w / 2);
        drawPanel({ x: stageRect.x, y: stageRect.y, w: pw - 1, h: stageRect.h }, act[0]);
        drawPanel({ x: stageRect.x + pw + 1, y: stageRect.y, w: stageRect.w - pw - 1, h: stageRect.h }, act[1]);
        ctx.strokeStyle = g('--line'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(stageRect.x + pw, stageRect.y); ctx.lineTo(stageRect.x + pw, stageRect.y + stageRect.h); ctx.stroke();
      } else drawPanel(stageRect, act[0]);
      ctx.strokeStyle = g('--line'); ctx.lineWidth = 1;
      if (!narrow) { ctx.beginPath(); ctx.moveTo(stageRect.w + 0.5, 0); ctx.lineTo(stageRect.w + 0.5, H); ctx.stroke(); }
      else { ctx.beginPath(); ctx.moveTo(0, stageRect.h + 0.5); ctx.lineTo(W, stageRect.h + 0.5); ctx.stroke(); }
      chGimbal.draw(ctx, gRect); chAlt.draw(ctx, aRect);
      /* readouts */
      act.forEach(l => {
        const r = hud[l.kind].ro, mm = l.metrics;
        r.solve.set(mm.solves ? mm.solveMs : NaN);
        r.viol.set(mm.violations, mm.violations === 0 ? 'good' : 'warn');
        r.maxViol.set(mm.maxViol, mm.maxViol < 0.05 ? 'good' : mm.maxViol < 0.5 ? 'warn' : 'bad');
        r.vtd.set(mm.touchdownSpeed, !isFinite(mm.touchdownSpeed) ? '' : mm.touchdownSpeed < 1 ? 'good' : mm.touchdownSpeed < 2 ? 'warn' : 'bad');
        r.xerr.set(mm.padError, !isFinite(mm.padError) ? '' : mm.padError < 0.5 ? 'good' : mm.padError < 1 ? 'warn' : 'bad');
      });
    }

    const loop = G.loop({ step, render, dt: p.dt, maxSubsteps: 12, root });
    loop.renderOnce();
    return {
      reset, loop,
      destroy() { loop.destroy(); stage.destroy(); root.textContent = ''; },
    };
  };
})();
