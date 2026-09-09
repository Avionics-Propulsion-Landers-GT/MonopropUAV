/* node sims/test-openloop.js
   Asserts the S2 (open loop / feedforward) behaviors from research/simspec.md, rescaled to the repo's
   toy model A (1 kg, 1..20 N, tau 0.2 s, dt 1/240). Mass-error results are mass-invariant while the
   thrust clip is not binding, so simspec's verified numbers carry over unchanged. */
'use strict';
const assert = require('node:assert');
const path = require('node:path');
const GTPL = require(path.join(__dirname, 'node-stub.js'));
require(path.join(__dirname, 'sim-openloop.js'));
const M = GTPL.math.openloop;
const p = M.params;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, want ${b} +/- ${tol}`);

/* constants the brief fixes */
assert.strictEqual(p.m, 1.0); assert.strictEqual(p.g, 9.81);
assert.strictEqual(p.Tmin, 1.0); assert.strictEqual(p.Tmax, 20.0);
assert.strictEqual(p.tau, 0.2); near(p.dt, 1 / 240, 1e-12, 'dt');
near(M.feedforward('hover', 3.0, p), 9.81, 1e-9, 'hover feedforward = m g');
assert.ok(typeof GTPL.sims.openloop === 'function', 'mount registered');

/* plan: min-jerk 0 -> 10 m in 5 s, peak accel 2.31 m/s^2, peak speed 3.75 m/s (simspec S2) */
{
  const ref = M.profiles.climb.ref;
  let aMax = 0, vMax = 0;
  for (let t = 0; t <= 5; t += 0.001) { const r = ref(t); aMax = Math.max(aMax, r.a); vMax = Math.max(vMax, r.v); }
  near(aMax, 2.31, 0.01, 'peak plan accel'); near(vMax, 3.75, 0.01, 'peak plan speed');
  near(ref(0).z, 0, 1e-12, 'plan start'); near(ref(5).z, 10, 1e-12, 'plan end'); near(ref(7).z, 10, 1e-12, 'plan holds');
  /* thrust profile stays inside the clip: 1*(9.81 +/- 2.31) = 7.50..12.12 N */
  let Tlo = Infinity, Thi = -Infinity;
  for (let t = 0; t <= 6; t += 0.001) { const T = M.feedforward('climb', t, p); Tlo = Math.min(Tlo, T); Thi = Math.max(Thi, T); }
  near(Tlo, 7.50, 0.02, 'min plan thrust'); near(Thi, 12.12, 0.02, 'max plan thrust');
  assert.ok(Tlo > p.Tmin && Thi < p.Tmax, 'plan thrust inside limits, so clipping never bites');
}

/* 1. perfect model, climb: z(5) = 9.97, z(10) = 10.00, v(10) = 0 */
{
  const r = M.run('climb', 0, 10);
  near(r.at(5).z, 9.97, 0.02, 'perfect climb z(5)');
  near(r.s.z, 10.0, 0.005, 'perfect climb z(10)');
  near(r.s.v, 0, 0.005, 'perfect climb v(10)');
  near(r.s.T, 9.81, 0.005, 'thrust back at hover');
}

/* 2. perfect model, hover: nothing moves */
{
  const r = M.run('hover', 0, 10);
  near(r.s.z, 10, 1e-9, 'perfect hover z(10)'); near(r.s.v, 0, 1e-9, 'perfect hover v(10)');
  r.log.forEach(e => near(e.z, 10, 1e-9, 'hover stays at 10 m'));
}

/* 3. real rocket 5 % lighter than the model: z(5) = 17.0, z(10) = 36.4, still climbing 5.2 m/s */
{
  const r = M.run('climb', -0.05, 10);
  near(r.at(5).z, 17.0, 0.15, 'mass -5 % z(5)');
  near(r.s.z, 36.4, 0.3, 'mass -5 % z(10)');
  near(r.s.v, 5.2, 0.1, 'mass -5 % v(10)');
}

/* 4. real rocket 5 % heavier: on the ground before 10 s. simspec.md quotes z(5) = 3.75 m, but running
      research/simspec.js itself (runA, m0 = 52.5, ctrlDiv 1 or 2) gives z(5) = 4.02..4.03 m; the code wins. */
{
  const r = M.run('climb', 0.05, 10);
  near(r.at(5).z, 4.03, 0.05, 'mass +5 % z(5)');
  assert.strictEqual(r.s.z, 0, 'mass +5 % crashed by 10 s');
  assert.ok(r.log.some(e => e.z > 3.5), 'it did lift off first');
}

/* 5. +/- 2 %: +10.2 m and climbing 2.0 m/s; z = 0.36 m sinking 1.9 m/s (simspec.js gives 0.359; the .md says 0.2) */
{
  const lo = M.run('climb', -0.02, 10);
  near(lo.s.z - 10, 10.2, 0.2, 'mass -2 % error at 10 s'); near(lo.s.v, 2.0, 0.1, 'mass -2 % v(10)');
  const hi = M.run('climb', 0.02, 10);
  near(hi.s.z, 0.36, 0.1, 'mass +2 % z(10)'); near(hi.s.v, -1.9, 0.15, 'mass +2 % v(10)');
}

/* 6. gust during a perfect hover: 2 N down for 2 s on 1 kg = -2 m/s^2 -> after the gust the rocket is
      4 m lower, falling at 4 m/s, and nothing brings it back (thrust never changed) */
{
  const r = M.run('hover', 0, 12, { gustAt: 5 });
  near(r.at(5).z, 10, 1e-6, 'level before gust');
  near(r.at(7).z, 6.0, 0.05, 'z after 2 s gust'); near(r.at(7).v, -4.0, 0.02, 'v after gust');
  near(r.at(7).T, 9.81, 1e-6, 'thrust did not react to the gust');
  assert.ok(r.at(8).z < r.at(7).z, 'keeps sinking after the gust ends');
  assert.strictEqual(r.s.z, 0, 'never comes back: on the ground by 12 s');
  assert.ok(r.log.every(e => Math.abs(e.Tcmd - 9.81) < 1e-9), 'controller command constant throughout');
}

/* 7. the gust on a perfect climb (pressed halfway up, t = 2.5 s) leaves the rocket 4 m/s slower than the
      plan for good; by the time the plan stops (5 s) it is falling at 4 m/s and it is on the ground by 10 s */
{
  const r = M.run('climb', 0, 10, { gustAt: 2.5 });
  near(r.at(5.5).v, -4.0, 0.05, 'v after gust = plan v - 4 m/s');
  assert.ok(r.at(5.5).z < r.at(5.5).zPlan - 3.5, 'well below plan after the gust');
  assert.strictEqual(r.s.z, 0, 'on the ground by 10 s, never came back');
}

/* 8. clip and lag: a 100 N command is held at 20 N; a 0 N command floors at 1 N; lag is first order with tau 0.2 s */
{
  const s = M.makeState('hover', 0, p);
  for (let i = 0; i < 480; i++) M.stepModel(s, 100, p.dt, p, 0);
  near(s.T, 20, 1e-3, 'clip high');
  const s2 = M.makeState('hover', 0, p);
  for (let i = 0; i < 480; i++) M.stepModel(s2, 0, p.dt, p, 0);
  near(s2.T, 1, 1e-3, 'clip low');
  const s3 = M.makeState('hover', 0, p);
  const n = Math.round(0.2 / p.dt);
  for (let i = 0; i < n; i++) M.stepModel(s3, 19.81, p.dt, p, 0);
  near((s3.T - 9.81) / 10, 1 - Math.exp(-1), 0.01, 'one tau reaches 63 %');
}

/* 9. ground clamp and state validity */
{
  const s = M.makeState('climb', 0, p);
  for (let i = 0; i < 240; i++) M.stepModel(s, 1, p.dt, p, 0);
  assert.strictEqual(s.z, 0); assert.strictEqual(s.v, 0);
  const r = M.run('climb', 0.10, 12);
  r.log.forEach(e => assert.ok(isFinite(e.z) && isFinite(e.v) && isFinite(e.T), 'finite state'));
}

/* 10. the mass slider changes the plant only: feedforward is the same for every mass error */
{
  const a = M.run('climb', -0.10, 6), b = M.run('climb', 0.10, 6);
  for (let i = 0; i < a.log.length; i++) near(a.log[i].Tcmd, b.log[i].Tcmd, 1e-12, 'same command regardless of real mass');
  assert.ok(a.s.z > 10 && b.s.z < 10, 'lighter climbs past the plan, heavier falls short');
}

console.log('perfect climb z(10) = ' + M.run('climb', 0, 10).s.z.toFixed(3) + ' m');
console.log('mass -5 % z(10) = ' + M.run('climb', -0.05, 10).s.z.toFixed(2) + ' m; mass +5 % z(10) = ' + M.run('climb', 0.05, 10).s.z.toFixed(2) + ' m');
console.log('hover + gust at 5 s: z(7) = ' + M.run('hover', 0, 7, { gustAt: 5 }).s.z.toFixed(2) + ' m');
console.log('PASS openloop');
