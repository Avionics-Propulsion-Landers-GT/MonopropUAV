/* node sims/test-handfly.js : asserts the S1 behaviours from research/simspec.md, rescaled to the
   repo's toy numbers (1 kg, 1..20 N, tau 0.2 s, dt 1/240, controller every step). */
'use strict';
const assert = require('node:assert');
const G = require('./node-stub.js');
require('./sim-handfly.js');

const m = G.math.handfly;
assert.ok(m, 'GTPL.math.handfly registered');
assert.ok(typeof G.sims.handfly === 'function', 'GTPL.sims.handfly registered');
const p = m.params;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b} (tol ${tol})`);

/* constants (override wins over simspec's 50 kg lander) */
assert.strictEqual(p.m, 1.0); assert.strictEqual(p.g, 9.81);
assert.strictEqual(p.Tmin, 1.0); assert.strictEqual(p.Tmax, 20.0);
assert.strictEqual(p.tau, 0.2); near(p.dt, 1 / 240, 1e-12, 'dt');
assert.strictEqual(p.target, 10); assert.strictEqual(p.band, 0.5);
near(m.hoverThrust(p), 9.81, 1e-12, 'hover thrust');
near(m.hoverThrottle(p), 0.4637, 1e-3, 'hover throttle fraction');

/* throttle mapping */
assert.strictEqual(m.throttleToThrust(0, p), 1);
assert.strictEqual(m.throttleToThrust(1, p), 20);
assert.strictEqual(m.throttleToThrust(1.7, p), 20);
assert.strictEqual(m.throttleToThrust(-3, p), 1);
near(m.thrustToThrottle(m.throttleToThrust(0.3, p), p), 0.3, 1e-12, 'throttle round trip');

/* exact first-order lag: after one tau of a held step the thrust is 63.2 % of the way */
{
  const s = m.makeState(p); s.z = 50;                          // off the ground so the clamp does not interfere
  const n = Math.round(p.tau / p.dt);
  for (let i = 0; i < n; i++) m.step(s, p.Tmax, p.dt, p, 0);
  near((s.T - p.Tmin) / (p.Tmax - p.Tmin), 1 - Math.exp(-1), 1e-9, 'lag reaches 1 - 1/e after tau');
}

/* pad: floor thrust below weight, rocket stays put */
{
  const s = m.makeState(p);
  assert.strictEqual(s.z, 0); assert.strictEqual(s.v, 0); assert.strictEqual(s.T, p.Tmin);
  for (let i = 0; i < 5 * 240; i++) m.step(s, m.throttleToThrust(0, p), p.dt, p, 0);
  assert.strictEqual(s.z, 0, 'stays on the pad'); assert.strictEqual(s.v, 0, 'no velocity on the pad');
  near(s.t, 5, 1e-9, 'time advances');
}

/* NaN guard */
{
  const s = m.makeState(p);
  assert.ok(m.finite(s));
  m.step(s, NaN, p.dt, p, 0);          // clamp(NaN) is NaN -> state goes non-finite
  assert.ok(!m.finite(s), 'non-finite state is detected');
}

/* fixed step is deterministic: one run of N steps equals two chunks (pause/resume) */
{
  const a = m.makeState(p), b = m.makeState(p);
  const ap = m.makeAutopilot(p), bp = m.makeAutopilot(p);
  for (let i = 0; i < 2000; i++) m.step(a, G.clamp(ap.update(a.z, p.dt), p.Tmin, p.Tmax), p.dt, p, 0);
  for (let i = 0; i < 700; i++) m.step(b, G.clamp(bp.update(b.z, p.dt), p.Tmin, p.Tmax), p.dt, p, 0);
  for (let i = 0; i < 1300; i++) m.step(b, G.clamp(bp.update(b.z, p.dt), p.Tmin, p.Tmax), p.dt, p, 0);
  assert.strictEqual(a.z, b.z); assert.strictEqual(a.v, b.v); assert.strictEqual(a.T, b.T);
}

/* autopilot at rest on target commands exactly hover thrust */
{
  const ap = m.makeAutopilot(p);
  ap.update(p.target, p.dt);
  near(ap.update(p.target, p.dt), m.hoverThrust(p), 1e-12, 'PD at rest on target = m g');
}

/* S1 autopilot check (simspec: peak 10.01 m, in band from 3.2 s and never leaves, 89 % of 30 s in band,
   max |e| after 10 s 0.014 m on the 50 kg plant). Rescaled plant: same Kp/m, Kd/m, but the floor is 1 m/s^2
   instead of 7, so the PD brakes harder on approach and arrives a little later. Verified numbers printed below. */
const auto = m.runAutopilot(p);
assert.ok(auto.peak <= p.target + 0.02, `no overshoot (peak ${auto.peak})`);
assert.ok(auto.tEnterForGood <= 6.0, `enters the band for good by 6 s (${auto.tEnterForGood})`);
assert.ok(auto.fracInBand >= 0.80, `at least 80 % of 30 s in band (${auto.fracInBand})`);
assert.ok(auto.maxErrAfter10 <= 0.05, `max |error| after 10 s under 5 cm (${auto.maxErrAfter10})`);
near(auto.zEnd, p.target, 0.01, 'settles at target');
assert.ok(Math.abs(auto.vEnd) < 1e-3, 'settles at rest');
assert.ok(auto.maxT === p.Tmax, 'launch from pad saturates the ceiling');

/* why hand flying is hard: full throttle to 10 m, floor throttle until the climb stops, then hover */
const bang = m.runBangBang(p);
assert.ok(bang.vAtTarget > 10, `passes the target fast (${bang.vAtTarget} m/s)`);
assert.ok(bang.overshoot > 10, `overshoots by more than 10 m (${bang.overshoot})`);

/* gust while the autopilot hovers: -0.6 N for 2 s. It must stay in the band and come back. */
const gust = m.runGust(p);
assert.ok(gust.sag > 0.05, `gust is visible (sag ${gust.sag})`);
assert.ok(gust.worst < p.band, `stays inside the band under the gust (${gust.worst})`);
assert.ok(gust.tBack !== null && gust.tBack <= 2 + p.gust.T + 5, `back within 5 cm inside 5 s of the gust ending (${gust.tBack})`);
near(gust.zEnd, p.target, 0.01, 'recovers to target');

console.log('hover thrust', m.hoverThrust(p).toFixed(2), 'N =', (m.hoverThrottle(p) * 100).toFixed(1), '% throttle');
console.log('autopilot: peak', auto.peak.toFixed(3), 'm; in band for good at', auto.tEnterForGood.toFixed(2), 's;',
  (auto.fracInBand * 100).toFixed(1), '% of 30 s in band; max |e| after 10 s', auto.maxErrAfter10.toFixed(3), 'm; min cmd', auto.minT.toFixed(2), 'N');
console.log('bang-bang: v at 10 m', bang.vAtTarget.toFixed(2), 'm/s; peak', bang.peak.toFixed(1), 'm; overshoot', bang.overshoot.toFixed(1), 'm at t =', bang.tPeak.toFixed(2), 's');
console.log('gust -0.6 N x 2 s under autopilot: sag', gust.sag.toFixed(3), 'm; back within 5 cm at t =', gust.tBack.toFixed(2), 's');
console.log('PASS handfly');
