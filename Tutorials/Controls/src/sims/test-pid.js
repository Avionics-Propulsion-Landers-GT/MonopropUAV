/* node sims/test-pid.js : asserts the S3 PID-sandbox behaviours from research/simspec.md,
   rescaled to the page's toy model A (1 kg, 1..20 N, tau 0.2 s, dt 1/240, controller every step). */
'use strict';
const assert = require('node:assert');
const GTPL = require('./node-stub.js');
require('./sim-pid.js');
const m = GTPL.math.pid;
const P = m.params;

const sd = (a) => { const mu = a.reduce((x, y) => x + y, 0) / a.length; return Math.sqrt(a.reduce((x, y) => x + (y - mu) * (y - mu), 0) / a.length); };
const tail = (r, seconds) => r.log.filter(l => l.t >= r.state.t - seconds);
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, expected ${b} +/- ${tol}`);

/* 0. constants the page copy quotes */
assert.strictEqual(P.m, 1.0); assert.strictEqual(P.g, 9.81); assert.strictEqual(P.Tmin, 1); assert.strictEqual(P.Tmax, 20);
assert.strictEqual(P.tau, 0.2); near(P.dt, 1 / 240, 1e-12, 'dt'); near(m.hover, 9.81, 1e-9, 'hover thrust');
assert.deepStrictEqual([m.defaults.Kp, m.defaults.Ki, m.defaults.Kd, m.defaults.ff], [2, 0.5, 3, false]);
assert.deepStrictEqual(m.ranges, { Kp: [0, 20], Ki: [0, 5], Kd: [0, 10], noise: [0, 0.3] });
assert.deepStrictEqual(m.setpoints, [5, 10, 15, 40]);
assert.deepStrictEqual([m.presets.ponly.Kp, m.presets.ponly.Ki, m.presets.ponly.Kd, m.presets.ponly.ff], [2, 0, 0, false]);
assert.deepStrictEqual([m.presets.pd.Kp, m.presets.pd.Ki, m.presets.pd.Kd, m.presets.pd.ff], [2, 0, 3, false]);
assert.deepStrictEqual([m.presets.pid.Kp, m.presets.pid.Ki, m.presets.pid.Kd, m.presets.pid.ff], [2, 0.5, 3, false]);
assert.deepStrictEqual([m.presets.pidff.Kp, m.presets.pidff.Ki, m.presets.pidff.Kd, m.presets.pidff.ff], [2, 0.5, 3, true]);

/* 1. plant: exact first-order lag, hover balance, ground clamp, command clipping */
{
  const s = m.makeState(50);
  for (let i = 0; i < 48; i++) m.stepModel(s, 20, 1 / 240);           // 48 steps = 0.2 s = one tau
  near(s.T, 1 + 19 * (1 - Math.exp(-1)), 1e-6, 'thrust after one tau');
  const h = m.makeState(10); h.T = m.hover;
  for (let i = 0; i < 2400; i++) m.stepModel(h, m.hover, 1 / 240);
  near(h.z, 10, 1e-9, 'hover holds altitude'); near(h.v, 0, 1e-9, 'hover holds speed');
  const g = m.makeState(0);
  for (let i = 0; i < 480; i++) m.stepModel(g, 1, 1 / 240);
  assert.strictEqual(g.z, 0, 'ground clamp'); assert.strictEqual(g.v, 0, 'ground kills downward speed');
  const c = m.makeState(0); for (let i = 0; i < 2400; i++) m.stepModel(c, 500, 1 / 240);
  assert.ok(c.T <= 20 + 1e-9, 'command clipped to Tmax');
}

/* 2. P only never settles: undamped mass + lag -> sustained bounce whose centre sits below the setpoint */
{
  const r = m.scenario({ gains: m.presets.ponly, ref: 10, tEnd: 40, logEvery: 4 });
  const z = tail(r, 10).map(l => l.z);
  assert.strictEqual(r.metrics.settle, null, 'P only must not settle');
  assert.ok(Math.max(...z) - Math.min(...z) > 5, 'P only keeps bouncing (>5 m swing in the last 10 s)');
  const mean = z.reduce((a, b) => a + b, 0) / z.length;
  assert.ok(mean < 9 && mean > 3, `P only bounce centred below the setpoint (mean ${mean.toFixed(2)} m)`);
  const r2 = m.scenario({ gains: { Kp: 4 }, ref: 10, tEnd: 40, logEvery: 4 });
  assert.ok(r2.metrics.peak > r.metrics.peak, 'doubling Kp makes the bounce bigger');
}

/* 3. PD, FF off: settles exactly m g / Kp below the setpoint (steady-state error) */
for (const ref of [10, 15]) {
  const r = m.scenario({ gains: m.presets.pd, ref, tEnd: 30, logEvery: 4 });
  near(r.state.z, ref - m.hover / 2, 0.02, `PD droop at ref ${ref}`);
  near(Math.abs(r.state.v), 0, 1e-3, 'PD at rest');
  assert.strictEqual(r.metrics.settle, null, 'PD with FF off never enters the 0.25 m band');
}

/* 4. PID, FF off: I term winds up to exactly hover thrust and the droop closes */
{
  const r = m.scenario({ gains: m.presets.pid, ref: 10, tEnd: 40, logEvery: 4 });
  near(r.state.z, 10, 0.02, 'PID reaches the setpoint');
  near(r.last.I, 9.81, 0.05, 'I term settles at 9.81 N when FF is off');
  assert.ok(r.metrics.settle !== null && r.metrics.settle < 15, 'PID settles');
  /* Ki x10: bigger overshoot, integral hits the clamp */
  const hot = m.scenario({ gains: { Kp: 2, Ki: 5, Kd: 3 }, ref: 10, tEnd: 40, logEvery: 4 });
  assert.ok(hot.metrics.peak > r.metrics.peak + 1, 'Ki x10 overshoots more');
  near(hot.iPeak, P.iMax, 1e-9, 'I contribution is clamped at +/- iMax');
  /* Ki = 0 keeps the integral at zero */
  const noI = m.scenario({ gains: m.presets.pd, ref: 10, tEnd: 10 });
  assert.strictEqual(noI.iPeak, 0, 'Ki = 0 -> I term stays 0');
}

/* 5. PID + FF: no droop, I term ends near zero; PD + FF alone is already offset free */
{
  const r = m.scenario({ gains: m.presets.pidff, ref: 10, tEnd: 30, logEvery: 4 });
  near(r.state.z, 10, 0.02, 'PID+FF reaches the setpoint'); near(r.last.I, 0, 0.3, 'I term near 0 with FF on');
  assert.ok(r.metrics.settle !== null, 'PID+FF settles');
  const pdff = m.scenario({ gains: { Kp: 2, Ki: 0, Kd: 3, ff: true }, ref: 10, tEnd: 30 });
  near(pdff.state.z, 10, 0.02, 'PD+FF has no droop'); assert.strictEqual(pdff.metrics.peak, 0, 'PD+FF does not overshoot');
}

/* 6. Kd sweep with FF on (5 m step from hover): Kd = 0 explodes, Kd rising damps, too much Kd creeps */
{
  const run = (Kd) => m.scenario({ gains: { Kp: 2, Ki: 0, Kd, ff: true }, refAt: t => t < 15 ? 10 : 15, hoverStart: true, z0: 10, tEnd: 40, logEvery: 4 });
  const k0 = run(0), k1 = run(1), k3 = run(3), k10 = run(10);
  assert.ok(k0.metrics.peak > 10 && k0.metrics.settle === null, 'Kd = 0: huge sustained overshoot');
  assert.ok(k1.metrics.peak > 1 && k1.metrics.peak < k0.metrics.peak, 'Kd = 1: smaller overshoot');
  assert.ok(k3.metrics.peak < 0.05 && k3.metrics.settle !== null && k3.metrics.settle < 5, 'Kd = 3: no overshoot, settles < 5 s');
  assert.ok(k10.metrics.settle > k3.metrics.settle, 'Kd = 10: overdamped, slower to settle');
  const zeta = 3 / (2 * Math.sqrt(2 * P.m));
  assert.ok(zeta > 1, 'Kd = 3 is just past critical damping');
}

/* 7. Saturation and windup: 40 m step pins the bar, integral climbs to the clamp, big overshoot follows */
{
  const r = m.scenario({ gains: { Kp: 6, Ki: 0.5, Kd: 3 }, ref: 40, tEnd: 40, logEvery: 4 });
  assert.ok(r.satFrac > 0.3, 'stiff Kp at 40 m saturates most of the run');
  near(r.iPeak, P.iMax, 1e-9, 'integral winds to the clamp while pinned');
  assert.ok(r.metrics.peak > 5, 'windup produces a large overshoot');
  const pinned = r.log.filter(l => l.u >= 20 - 1e-9 && l.t < 2);
  assert.ok(pinned.length > 0 && pinned[pinned.length - 1].I > pinned[0].I, 'I term climbs while the command is pinned at max');
}

/* 8. Derivative kick: on error, a setpoint jump spikes D; on measurement, it does not */
{
  const kick = (dmode) => {
    const r = m.scenario({ gains: Object.assign({}, m.presets.pidff, { dmode }), refAt: t => t < 20 ? 10 : 15, tEnd: 30 });
    const win = r.log.filter(l => l.t >= 20 - 0.01 && l.t < 20.3);
    return Math.max(...win.map(l => Math.abs(l.D)));
  };
  const e = kick('error'), me = kick('measurement');
  assert.ok(e > 100, `derivative on error kicks (${e.toFixed(0)} N)`);
  assert.ok(me < 10, `derivative on measurement removes the kick (${me.toFixed(1)} N)`);
}

/* 9. Noise: D amplifies measurement jitter, scaling with Kd and sigma; P barely notices */
{
  const noisy = (Kd, sig) => { const r = m.scenario({ gains: { Kp: 2, Ki: 0.5, Kd, ff: true }, ref: 10, tEnd: 30, noise: sig, seed: 5 }); const w = r.log.filter(l => l.t >= 15); return { D: sd(w.map(l => l.D)), P: sd(w.map(l => l.P)) }; };
  const a = noisy(3, 0.05), b = noisy(3, 0.1), c = noisy(6, 0.1);
  assert.ok(a.D > 1, 'sensor noise of 5 cm already shakes the D term by > 1 N');
  assert.ok(b.D > 1.6 * a.D && c.D > 1.6 * b.D, 'D jitter scales with sigma and with Kd');
  assert.ok(a.P < 0.5 && b.P < 0.5, 'P term barely reacts to noise');
  const quiet = noisy(3, 0); assert.ok(quiet.D < 0.3 && quiet.D < a.D / 3, `no noise -> steady D (${quiet.D.toFixed(3)} N)`);
}

/* 10. Gust: 2 N down for 2 s dips a settled PID+FF rocket, then it recovers */
{
  const r = m.scenario({ gains: m.presets.pidff, ref: 10, tEnd: 40, dist: t => (t >= 20 && t < 22) ? P.gustN : 0 });
  const win = r.log.filter(l => l.t >= 20 && l.t < 30);
  const dip = 10 - Math.min(...win.map(l => l.z));
  assert.ok(dip > 0.3 && dip < 2, `gust dips the rocket (${dip.toFixed(2)} m)`);
  near(r.state.z, 10, 0.02, 'recovers after the gust');
}

/* 11. Metrics helpers */
{
  const met = m.makeMetrics(); m.metricsMark(met, 0, 0, 10, true);
  m.metricsStep(met, 0.1, 11, 10); near(m.overshootPct(met), 10, 1e-9, 'overshoot % of the step');
  const met2 = m.makeMetrics(); m.metricsMark(met2, 5, 10, 10, false);
  assert.ok(Number.isNaN(m.overshootPct(met2)), 'no step -> overshoot undefined');
  const met3 = m.makeMetrics(); m.metricsMark(met3, 0, 0, 10, true);
  for (let t = 0.01; t < 3; t += 0.01) m.metricsStep(met3, t, 10.1, 10);
  near(met3.settle, 0.01, 1e-6, 'settle time = first in-band time after the change');
}

/* 12. Controller never returns non-finite numbers and clips its output */
{
  const c = m.makeController();
  const o = m.controller(c, { Kp: 20, Ki: 5, Kd: 10, ff: true, dmode: 'error' }, 40, 0, P.dt);
  assert.ok([o.u, o.uRaw, o.P, o.I, o.D, o.FF].every(Number.isFinite), 'finite outputs');
  assert.strictEqual(o.u, 20, 'output clipped to Tmax'); assert.strictEqual(o.sat, 1, 'saturation flag');
}

console.log('PASS pid');
