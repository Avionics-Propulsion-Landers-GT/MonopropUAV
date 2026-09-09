/* node sims/test-lqr.js : asserts the S5 (LQR on model B) behaviors from research/simspec.md, rescaled to the repo toy rocket. */
'use strict';
const assert = require('node:assert');
const path = require('path');
const G = require(path.join(__dirname, 'node-stub.js'));
require(path.join(__dirname, 'sim-lqr.js'));
const m = G.math.lqr;
const DEG = Math.PI / 180;

assert.strictEqual(typeof G.sims.lqr, 'function', 'mount registered');
assert.ok(m && typeof m.design === 'function', 'math registered');

/* ---- 0. constants from the build brief ---- */
const p = m.params;
assert.strictEqual(p.m, 1.0); assert.strictEqual(p.g, 9.81); assert.strictEqual(p.I, 0.2); assert.strictEqual(p.L, 1.0);
assert.strictEqual(p.Tmin, 1); assert.strictEqual(p.Tmax, 20); assert.ok(Math.abs(p.deltaMax - 10 * DEG) < 1e-12);
assert.ok(Math.abs(p.dt - 1 / 240) < 1e-15); assert.strictEqual(p.ctrlDiv, 5);
assert.ok(Math.abs(p.hover - 9.81) < 1e-12);

/* ---- 1. the Riccati solver reproduces simspec's verified K on simspec's own 50 kg plant ---- */
const S = require(path.join(__dirname, '..', 'research', 'simspec.js'));
{
  const { A, B } = S.linearizeB(S.DEFAULTS.B);
  const { Ad, Bd } = S.c2d(A, B, 0.02);
  const { Q, R } = S.lqrWeightsFromSliders({ posW: 1, angleW: 1, gimbalR: 1, thrustR: 1 });
  const ours = m.dlqr(Ad, Bd, Q, R).K;
  const ref = [[0, 190.4403, 0, 0, 235.1841, 0], [-0.0302, 0, -0.6440, -0.0698, 0, -0.3314]];
  for (let i = 0; i < 2; i++) for (let j = 0; j < 6; j++) assert.ok(Math.abs(ours[i][j] - ref[i][j]) < 6e-5, `simspec K[${i}][${j}] ${ours[i][j]} vs ${ref[i][j]}`);
  const theirs = S.dlqr(Ad, Bd, Q, R).K;
  for (let i = 0; i < 2; i++) for (let j = 0; j < 6; j++) assert.ok(Math.abs(ours[i][j] - theirs[i][j]) < 1e-7, 'dlqr matches simspec.dlqr');
}

/* ---- 2. linear model of the toy rocket ---- */
const d0 = m.design(m.defaults);
assert.ok(Math.abs(d0.B[3][1] - 9.81) < 1e-12, 'vx from gimbal = g');
assert.ok(Math.abs(d0.B[4][0] - 1.0) < 1e-12, 'vz from thrust = 1/m');
assert.ok(Math.abs(d0.B[5][1] + 49.05) < 1e-9, 'omega from gimbal = -L m g / I = -49.05');
assert.ok(Math.abs(d0.A[3][2] - 9.81) < 1e-12);
assert.ok(Math.abs(d0.Ad[0][3] - p.dtCtrl) < 1e-12, 'Ad x<-vx = dt');
assert.ok(Math.abs(d0.Ad[3][2] - 9.81 * p.dtCtrl) < 1e-12, 'Ad vx<-theta = g dt');

/* ---- 3. K structure: thrust row sees only z, vz; gimbal row sees only x, theta, vx, omega; signs ---- */
const K0 = d0.K;
[0, 2, 3, 5].forEach(j => assert.ok(Math.abs(K0[0][j]) < 1e-9, 'thrust row zero at col ' + j));
[1, 4].forEach(j => assert.ok(Math.abs(K0[1][j]) < 1e-9, 'gimbal row zero at col ' + j));
assert.ok(K0[0][1] > 0 && K0[0][4] > 0, 'thrust gains positive (u = -K dx: below target -> more thrust)');
[0, 2, 3, 5].forEach(j => assert.ok(K0[1][j] < 0, 'gimbal gains negative at col ' + j));
assert.ok(d0.iters > 100 && d0.iters < 3000, 'Riccati iterations ' + d0.iters);

/* ---- 4. only the ratio of Q to R matters ---- */
{
  const Q10 = d0.Q.map(v => v * 10), R10 = d0.R.map(v => v * 10);
  const K10 = m.dlqr(d0.Ad, d0.Bd, Q10, R10).K;
  for (let i = 0; i < 2; i++) for (let j = 0; j < 6; j++) assert.ok(Math.abs(K10[i][j] - K0[i][j]) < 1e-6 * Math.max(1, Math.abs(K0[i][j])), 'scale invariance');
}

/* ---- 5. closed loop stable at defaults and at all 16 slider corners ---- */
assert.ok(m.closedLoopDecay(d0.Ad, d0.Bd, K0, 3000) < 1e-6, 'default closed loop decays');
for (const posW of m.sliderRanges.posW) for (const angleW of m.sliderRanges.angleW) for (const gimbalR of m.sliderRanges.gimbalR) for (const thrustR of m.sliderRanges.thrustR) {
  const d = m.design({ posW, angleW, gimbalR, thrustR });
  assert.ok(d.K.every(r => r.every(Number.isFinite)), 'finite K at corner');
  assert.ok(m.closedLoopDecay(d.Ad, d.Bd, d.K, 4000) < 1e-6, `corner ${posW} ${angleW} ${gimbalR} ${thrustR} stable`);
}

/* ---- 6. solve time: a slider drag must stay under a few ms ---- */
{
  m.design(m.defaults);
  const t0 = process.hrtime.bigint();
  const N = 20;
  for (let i = 0; i < N; i++) m.design({ posW: 1 + i * 0.1, angleW: 1, gimbalR: 1, thrustR: 1 });
  const ms = Number(process.hrtime.bigint() - t0) / 1e6 / N;
  console.log('design() average', ms.toFixed(2), 'ms');
  assert.ok(ms < 25, 'design under 25 ms (' + ms.toFixed(2) + ')');
}

/* ---- 7. default 5 m move: tilt toward target, translate, straighten; gimbal stays inside 10 deg; altitude barely moves ---- */
const r5 = m.runScenario({ offset: 5, tEnd: 15 });
{
  const M = r5.metrics;
  assert.ok(r5.ok, 'finite');
  assert.ok(M.peakGimbalCmd < 10, 'peak gimbal command under 10 deg: ' + M.peakGimbalCmd.toFixed(2));
  assert.strictEqual(M.satTime, 0, 'never clipped');
  assert.strictEqual(M.thrustClipTime, 0, 'thrust never clipped');
  assert.ok(M.peakTilt > 8 && M.peakTilt < 25, 'peak tilt ' + M.peakTilt.toFixed(1));
  assert.ok(M.settleTime !== null && M.settleTime < 6, 'settles under 6 s: ' + M.settleTime);
  assert.ok(10 - M.minZ < 0.2, 'altitude dip small: ' + (10 - M.minZ).toFixed(3));
  assert.strictEqual(M.firstGimbalSign, -1, 'from x0 = -5, gimbal first goes negative (kicks the base away from the target)');
  assert.strictEqual(M.firstTiltSign, 1, 'tilt goes positive (nose toward the target)');
  assert.ok(M.overshoot < 0.05, 'no overshoot: ' + M.overshoot.toFixed(3));
  assert.ok(Math.abs(r5.x[0]) < 0.05 && Math.abs(r5.x[1] - 10) < 0.05 && Math.abs(r5.x[2]) < 0.5 * DEG, 'ends at the target, upright');
  console.log('5 m default: tilt', M.peakTilt.toFixed(1), 'deg, gimbal cmd', M.peakGimbalCmd.toFixed(2), 'deg, settle', M.settleTime.toFixed(2), 's, dip', (10 - M.minZ).toFixed(3), 'm');
}

/* ---- 8. 30 m move: tilt past 20 deg, altitude dips, still recovers ---- */
{
  const r = m.runScenario({ offset: 30, tEnd: 20 });
  const M = r.metrics;
  assert.ok(r.ok, 'finite');
  assert.ok(M.peakTilt > 20, 'tilt past 20 deg: ' + M.peakTilt.toFixed(1));
  assert.ok(10 - M.minZ > 0.5, 'altitude dips: ' + (10 - M.minZ).toFixed(2));
  assert.ok(M.satTime > 0, 'gimbal saturates');
  assert.ok(M.settleTime !== null && M.settleTime < 12, 'still settles: ' + M.settleTime);
  assert.ok(Math.abs(r.x[0]) < 0.1 && Math.abs(r.x[1] - 10) < 0.1, 'ends at the target');
  console.log('30 m: tilt', M.peakTilt.toFixed(1), 'deg, gimbal cmd', M.peakGimbalCmd.toFixed(1), 'deg, saturated', M.satTime.toFixed(2), 's, dip', (10 - M.minZ).toFixed(2), 'm, settle', M.settleTime.toFixed(2), 's');
}

/* ---- 9. position weight up: faster, bigger gimbal, crosses the 10 deg line ---- */
{
  const r3 = m.runScenario({ offset: 5, sliders: { posW: 3 } });
  const r10 = m.runScenario({ offset: 5, sliders: { posW: 10 } });
  assert.ok(r3.metrics.peakGimbalCmd > 10, 'posW x3 crosses 10 deg: ' + r3.metrics.peakGimbalCmd.toFixed(1));
  assert.ok(r10.metrics.peakGimbalCmd > r3.metrics.peakGimbalCmd, 'posW x10 asks for more');
  assert.ok(r10.metrics.satTime > 0, 'posW x10 saturates');
  assert.ok(r10.metrics.settleTime < r5.metrics.settleTime, 'posW x10 is faster');
  const d10 = m.design(Object.assign({}, m.defaults, { posW: 10 }));
  assert.ok(Math.abs(d10.K[1][0]) > Math.abs(K0[1][0]), 'gimbal-from-x gain grows');
}

/* ---- 10. gimbal effort up: lazy gimbal, slower, K gimbal row shrinks ---- */
{
  const r = m.runScenario({ offset: 5, sliders: { gimbalR: 10 } });
  assert.ok(r.metrics.peakGimbalCmd < r5.metrics.peakGimbalCmd, 'less gimbal');
  assert.ok(r.metrics.settleTime > r5.metrics.settleTime, 'slower');
  const d = m.design(Object.assign({}, m.defaults, { gimbalR: 10 }));
  [0, 2, 3, 5].forEach(j => assert.ok(Math.abs(d.K[1][j]) < Math.abs(K0[1][j]), 'gimbal gain shrinks at col ' + j));
  const rAng = m.runScenario({ offset: 5, sliders: { angleW: 10 } });
  assert.ok(rAng.metrics.peakTilt < r5.metrics.peakTilt, 'angle weight up refuses to lean');
}

/* ---- 11. nudge: lateral impulse is rejected ---- */
{
  const r = m.runScenario({ offset: 5, tEnd: 16, nudgeAt: 8 });
  assert.ok(r.ok);
  assert.ok(r.metrics.settleTime !== null && r.metrics.settleTime < 5, 'settles within 5 s of the nudge: ' + r.metrics.settleTime);
  assert.ok(Math.abs(r.x[0]) < 0.05, 'back on target');
}

/* ---- 12. controller helper and NaN guard ---- */
{
  const out = {};
  m.control(K0, [-5, 10, 0, 0, 0, 0], m.target, p, out);
  assert.ok(out.rawDelta < 0 && out.delta === out.rawDelta && !out.clippedDelta, 'small error: not clipped');
  assert.ok(Math.abs(out.dT) < 1e-9 && out.T === p.hover, 'no thrust correction for a pure x error');
  m.control(K0, [-30, 10, 0, 0, 0, 0], m.target, p, out);
  assert.ok(out.clippedDelta && out.delta === -p.deltaMax, 'big error clips to -10 deg');
  const sim = new m.Sim(p); sim.setK(K0); sim.reset(5);
  for (let i = 0; i < 48; i++) assert.ok(sim.tick(0));
  assert.ok(Math.abs(sim.s.x[0] + 5) < 1e-9 && Math.abs(sim.s.x[1] - 10) < 1e-9, 'holds the start point before go()');
  sim.s.x[0] = NaN;
  assert.strictEqual(sim.tick(0), false, 'non-finite state reported');
}

/* ---- 13. display helpers ---- */
assert.strictEqual(m.sig2(190.4403), '190');
assert.strictEqual(m.sig2(-0.0302), '−0.03');
assert.strictEqual(m.sig2(0), '0');
assert.strictEqual(m.sig2(4.6957), '4.7');
{
  const Kd = m.displayK(K0);
  assert.ok(Math.abs(Kd[1][0] - K0[1][0] / DEG) < 1e-9, 'gimbal row x entry in deg per m');
  assert.ok(Math.abs(Kd[1][2] - K0[1][2]) < 1e-9, 'deg per deg unchanged');
  assert.ok(Math.abs(Kd[0][1] - K0[0][1]) < 1e-9, 'thrust row N per m unchanged');
}

console.log('K (rows thrust N, gimbal rad):', K0.map(r => r.map(v => +v.toFixed(4))));
console.log('PASS lqr');
