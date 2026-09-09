/* node sims/test-mpc.js — asserts the S6 behaviors (rescaled to the repo's toy rocket) for sim-mpc.js */
const assert = require('node:assert');
const GTPL = require('./node-stub.js');
require('./sim-mpc.js');
const m = GTPL.math.mpc, P = m.params, DEG = m.DEG;
assert.strictEqual(typeof GTPL.sims.mpc, 'function', 'sim registered');
assert.ok(m && typeof m.makeMPC === 'function', 'math registered');

/* 1. constants from the build brief */
assert.strictEqual(P.m, 1); assert.strictEqual(P.I, 0.2); assert.strictEqual(P.L, 1); assert.strictEqual(P.Tmin, 1); assert.strictEqual(P.Tmax, 20);
assert.ok(Math.abs(P.deltaMax - 10 * DEG) < 1e-12); assert.ok(Math.abs(P.dt - 1 / 240) < 1e-15);
assert.ok(Math.abs(P.mpcDiv * P.dt - 0.1) < 1e-12, 'MPC at 10 Hz'); assert.ok(Math.abs(P.lqrDiv * P.dt - 1 / 48) < 1e-12, 'LQR at 48 Hz');

/* 2. linearization and exact discretization */
{
  const { A, B } = m.linearize();
  assert.ok(Math.abs(B[5][1] - (-P.L * P.m * P.g / P.I)) < 1e-12, 'omega_dot = -L m g / I delta');
  assert.ok(Math.abs(B[5][1] + 49.05) < 1e-9);
  assert.ok(Math.abs(B[4][0] - 1) < 1e-12 && Math.abs(B[3][1] - P.g) < 1e-12 && Math.abs(A[3][2] - P.g) < 1e-12);
  const { Ad, Bd } = m.c2d(A, B, 0.1);
  assert.ok(Math.abs(Ad[3][2] - 0.981) < 1e-9, 'Ad[vx][theta] = g dt');
  assert.ok(Math.abs(Ad[0][2] - 0.5 * P.g * 0.01) < 1e-9, 'Ad[x][theta] = g dt^2/2');
  assert.ok(Math.abs(Bd[4][0] - 0.1) < 1e-9 && Math.abs(Bd[5][1] - (-4.905)) < 1e-9);
}

/* 3. LQR gain: decoupled block structure, plausible magnitudes */
{
  const lq = m.makeLQR(); const K = lq.K;
  assert.ok(Math.abs(K[0][0]) < 1e-9 && Math.abs(K[0][2]) < 1e-9 && Math.abs(K[0][3]) < 1e-9 && Math.abs(K[0][5]) < 1e-9, 'thrust row only sees z, vz');
  assert.ok(Math.abs(K[1][1]) < 1e-9 && Math.abs(K[1][4]) < 1e-9, 'gimbal row does not see z, vz');
  assert.ok(K[0][1] > 0 && K[0][4] > 0, 'thrust gains positive');
  assert.ok(K[1][0] < 0 && K[1][2] < 0, 'gimbal: to move toward -x it first commands +delta (rocket leans toward the target)');
  const o = lq.control([1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
  assert.ok(o.raw[1] > 0 && Math.abs(o.u[1] - o.raw[1]) < 1e-12, 'small offset is not clipped');
  const o2 = lq.control([20, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
  assert.ok(o2.raw[1] > P.deltaMax && Math.abs(o2.u[1] - P.deltaMax) < 1e-12, 'big offset: raw exceeds 10 deg, command clipped exactly to 10 deg');
}

/* 4. solver exactness: with the Riccati terminal cost, unconstrained MPC == LQR at dt 0.1 (simspec's check), for any N */
{
  const { A, B } = m.linearize(); const { Ad, Bd } = m.c2d(A, B, 0.1); const { Q, R } = m.weights(); const K = m.dlqr(Ad, Bd, Q, R).K;
  for (const N of [5, 10, 30]) for (const x of [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0.3, 0.5, 0.05, 0.2, -0.3, 0.1]]) {
    const r = m.makeMPC({ N, cone: null, terminal: 'lqr' }).solve(x, [0, 0, 0, 0, 0, 0], null);
    for (let i = 0; i < 2; i++) { let u = 0; for (let j = 0; j < 6; j++) u -= K[i][j] * x[j]; assert.ok(Math.abs(r.u0[i] - u) < 1e-6, `MPC == LQR N=${N} input ${i}: ${r.u0[i]} vs ${u}`); }
    assert.ok(r.ok && r.iters >= 1);
  }
  /* input limits inside the solver: every planned input inside the box, exactly on it when it binds */
  const mp = m.makeMPC({ N: 10, cone: null, terminal: 'lqr' });
  const r = mp.solve([15, 10, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], null);
  for (let k = 0; k < 10; k++) { assert.ok(r.Uplan[2 * k] >= P.Tmin - P.m * P.g - 1e-9 && r.Uplan[2 * k] <= P.Tmax - P.m * P.g + 1e-9); assert.ok(Math.abs(r.Uplan[2 * k + 1]) <= P.deltaMax + 1e-9); }
  assert.ok(Math.abs(Math.abs(r.u0[1]) - P.deltaMax) < 1e-9, 'a 15 m offset pins the first gimbal command to the limit');
  assert.strictEqual(r.Xpred.length, 11 * 6, 'N+1 predicted states');
  /* NaN guard */
  const bad = m.makeMPC({ N: 10 }).solve([NaN, 5, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], null);
  assert.strictEqual(bad.ok, false, 'NaN state -> solver reports failure');
}

/* 5. Scenario B (glide slope) from 20 m: LQR + clipping violates the cone, MPC with the soft cone does not, MPC without it is as blind as LQR */
const lqr20 = m.runScenario({ controller: 'lqr', z0: 20 });
const mpc20 = m.runScenario({ controller: 'mpc', z0: 20, N: 10 });
const mpc20nc = m.runScenario({ controller: 'mpc', z0: 20, N: 10, cone: false });
assert.ok(lqr20.landed && mpc20.landed && mpc20nc.landed, 'all three touch down');
assert.ok(lqr20.metrics.maxViol > 1.0, 'LQR violates the cone by more than 1 m: ' + lqr20.metrics.maxViol);
assert.ok(mpc20.metrics.maxViol < 0.15, 'MPC + cone stays within 0.15 m of the cone: ' + mpc20.metrics.maxViol);
assert.ok(mpc20nc.metrics.maxViol > 1.0, 'MPC without the penalty violates like LQR: ' + mpc20nc.metrics.maxViol);
assert.ok(lqr20.metrics.peakRawGimbal > P.deltaMax && lqr20.metrics.peakGimbalCmd <= P.deltaMax + 1e-9, 'LQR asks for more than 10 deg and is clipped');
assert.ok(lqr20.metrics.minThrust <= P.Tmin + 1e-9, 'LQR drives thrust to the flameout floor from 20 m');
assert.ok(mpc20.metrics.peakGimbalCmd <= P.deltaMax + 1e-9 && mpc20.metrics.minThrust >= P.Tmin - 1e-9 && mpc20.metrics.maxThrust <= P.Tmax + 1e-9, 'MPC never leaves the input box');
assert.ok(mpc20.metrics.padError < 0.6 && isFinite(mpc20.metrics.touchdownSpeed), 'MPC lands near the pad');
assert.ok(mpc20.metrics.nanCount === 0 && lqr20.metrics.nanCount === 0);
assert.ok(mpc20.metrics.solveMs >= 0 && isFinite(mpc20.metrics.solveMs));

/* 6. Gust late in a 10 m descent: LQR clips and overshoots the pad, MPC recovers inside the cone and lands softer, later */
const lqrG = m.runScenario({ controller: 'lqr', z0: 10, gustAt: 1.5 });
const mpcG = m.runScenario({ controller: 'mpc', z0: 10, N: 10, gustAt: 1.5 });
assert.ok(lqrG.landed && mpcG.landed);
assert.ok(lqrG.metrics.padError > 1.5, 'LQR overshoots the pad after the gust: ' + lqrG.metrics.padError);
assert.ok(mpcG.metrics.padError < 0.3, 'MPC lands on the pad after the gust: ' + mpcG.metrics.padError);
assert.ok(mpcG.metrics.touchdownSpeed < lqrG.metrics.touchdownSpeed, 'MPC touches down slower');
assert.ok(mpcG.t > lqrG.t, 'MPC lands later');
assert.ok(lqrG.metrics.maxViol > mpcG.metrics.maxViol, 'LQR leaves the cone further');

/* 7. Horizon: a 5-step horizon with plain stage cost is short-sighted (ignores x without the cone); 30 steps fixes x */
const short = m.runScenario({ controller: 'mpc', z0: 10, N: 5, cone: false });
const long = m.runScenario({ controller: 'mpc', z0: 10, N: 30, cone: false });
assert.ok(short.metrics.padError > 2 && short.metrics.maxTilt < 5 * DEG, 'N=5 barely tilts and misses the pad: ' + short.metrics.padError);
assert.ok(long.metrics.padError < 0.5, 'N=30 lands near the pad: ' + long.metrics.padError);
const shortC = m.runScenario({ controller: 'mpc', z0: 20, N: 5 });
const longC = m.runScenario({ controller: 'mpc', z0: 20, N: 30 });
assert.ok(shortC.metrics.violations > longC.metrics.violations, 'with the cone on, N=5 hugs the edge (more samples at/over it) while N=30 turns earlier');
const mid20 = mpc20;
assert.ok(longC.metrics.touchdownSpeed < mid20.metrics.touchdownSpeed, 'from 20 m the 3 s horizon touches down softer than the 1 s horizon');
assert.ok(mid20.metrics.maxThrust < longC.metrics.maxThrust || mid20.metrics.touchdownSpeed > 1, 'the short horizon arrives fast');

/* 8. Latency turns MPC back into the shower */
const lat0 = m.runScenario({ controller: 'mpc', z0: 10, N: 10, latencyMs: 0 });
const lat200 = m.runScenario({ controller: 'mpc', z0: 10, N: 10, latencyMs: 200 });
assert.ok(lat0.metrics.touchdownSpeed < 1 && lat0.metrics.maxTilt < 30 * DEG);
assert.ok(lat200.metrics.touchdownSpeed > 3 || lat200.metrics.maxTilt > 60 * DEG, '200 ms latency: crash or flip');

/* 9. Latency ring buffer returns the state from exactly latency/dt steps ago */
{
  const lane = m.makeLane('mpc', { N: 10, cone: true, latencyMs: 100, z0: 10 });
  lane.start();
  const hist = [];
  for (let i = 0; i < 40; i++) { hist.push(Array.from(lane.state())); lane.step(P.dt, 0); }
  const D = lane.delayedState(), lag = Math.round(0.1 / P.dt);
  for (let j = 0; j < 6; j++) assert.ok(Math.abs(D[j] - hist[hist.length - 1 - lag][j]) < 1e-12, 'delayed state is the sample taken lag steps before the latest one');
}

/* 10. Determinism / lockstep: two lanes of the same kind with the same inputs stay identical (pause/resume cannot desync them) */
{
  const a = m.makeLane('lqr', { z0: 10 }), b = m.makeLane('lqr', { z0: 10 }); a.start(); b.start();
  for (let i = 0; i < 600; i++) { a.step(P.dt, i > 300 ? -3 : 0); b.step(P.dt, i > 300 ? -3 : 0); }
  for (let j = 0; j < 6; j++) assert.strictEqual(a.state()[j], b.state()[j]);
}

/* 11. NaN in the plant state resets the lane and counts it */
{
  const lane = m.makeLane('mpc', { N: 10, cone: true, z0: 10 }); lane.start();
  for (let i = 0; i < 30; i++) lane.step(P.dt, 0);
  lane.state()[3] = NaN; lane.step(P.dt, 0);
  assert.strictEqual(lane.started, false, 'lane re-armed after NaN');
  assert.ok(isFinite(lane.state()[0]) && lane.state()[1] === 10);
  assert.ok(lane.metrics.nanCount >= 1);
}

/* 12. armed lanes do not move; reset clears metrics */
{
  const lane = m.makeLane('lqr', { z0: 20 });
  for (let i = 0; i < 100; i++) lane.step(P.dt, 0);
  assert.strictEqual(lane.t, 0); assert.strictEqual(lane.state()[1], 20); assert.ok(Math.abs(lane.state()[0] - 9) < 1e-12, 'x0 = 0.45 z0');
  lane.start(); for (let i = 0; i < 2400; i++) lane.step(P.dt, 0);
  assert.ok(lane.landed); lane.arm(10); assert.ok(!lane.landed && Number.isNaN(lane.metrics.touchdownSpeed) && lane.metrics.violations === 0);
}

console.log('scenario numbers (toy rocket): LQR 20 m: maxViol', lqr20.metrics.maxViol.toFixed(2), 'm, vTD', lqr20.metrics.touchdownSpeed.toFixed(2), 'm/s, pad err', lqr20.metrics.padError.toFixed(2), 'm; MPC N=10 20 m: maxViol', mpc20.metrics.maxViol.toFixed(2), 'vTD', mpc20.metrics.touchdownSpeed.toFixed(2), 'pad err', mpc20.metrics.padError.toFixed(2), '; MPC N=30 20 m vTD', longC.metrics.touchdownSpeed.toFixed(2));
console.log('gust 3 N at 1.5 s from 10 m: LQR pad err', lqrG.metrics.padError.toFixed(2), 'vTD', lqrG.metrics.touchdownSpeed.toFixed(2), '| MPC pad err', mpcG.metrics.padError.toFixed(2), 'vTD', mpcG.metrics.touchdownSpeed.toFixed(2), 't', mpcG.t.toFixed(1), 'vs', lqrG.t.toFixed(1));
console.log('latency 200 ms MPC: vTD', lat200.metrics.touchdownSpeed.toFixed(2), 'tilt', (lat200.metrics.maxTilt / DEG).toFixed(0), 'deg; solve ms N=10', mpc20.metrics.solveMs.toFixed(3), 'N=30', longC.metrics.solveMs.toFixed(3));
console.log('PASS mpc');
