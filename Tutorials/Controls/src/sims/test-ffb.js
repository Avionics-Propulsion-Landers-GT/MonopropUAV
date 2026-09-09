/* node sims/test-ffb.js — asserts the S4 "anticipation vs correction" behaviours from research/simspec.md,
   re-verified at the page's toy numbers (model A: 1.0 kg, 1..20 N, tau 0.2 s, dt 1/240, PID 2 / 0.5 / 3). */
'use strict';
const assert = require('node:assert');
const path = require('node:path');
const G = require(path.join(__dirname, 'node-stub.js'));
require(path.join(__dirname, 'sim-ffb.js'));
const m = G.math.ffb;
const P = m.params;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, want ${b} +/- ${tol}`);

/* ---- registration and constants ---- */
assert.ok(typeof G.sims.ffb === 'function', 'mount registered');
assert.strictEqual(P.mModel, 1.0); assert.strictEqual(P.g, 9.81);
assert.strictEqual(P.Tmin, 1); assert.strictEqual(P.Tmax, 20); assert.strictEqual(P.tau, 0.2);
near(P.dt, 1 / 240, 1e-12, 'dt');
assert.deepStrictEqual([m.defaults.Kp, m.defaults.Ki, m.defaults.Kd], [2, 0.5, 3]);
assert.deepStrictEqual(m.ranges, { Kp: [0, 20], Ki: [0, 5], Kd: [0, 10], massErr: [-10, 10] });

/* ---- reference: min-jerk rest-to-rest climb 0 -> 10 m in 6 s, then hold ---- */
const r0 = m.reference(0), r3 = m.reference(3), r6 = m.reference(6), r9 = m.reference(9);
near(r0.z, 0, 1e-12, 'ref z(0)'); near(r0.v, 0, 1e-12, 'ref v(0)'); near(r0.a, 0, 1e-12, 'ref a(0)');
near(r3.z, 5, 1e-9, 'ref z(3) midpoint'); near(r3.v, 1.875 * 10 / 6, 1e-9, 'ref peak v = 1.875 d/T');
near(r6.z, 10, 1e-12, 'ref z(6)'); near(r6.v, 0, 1e-12, 'ref v(6)'); near(r6.a, 0, 1e-12, 'ref a(6)');
near(r9.z, 10, 1e-12, 'ref holds after climb');
let pv = 0, pa = 0;
for (let t = 0; t <= 6; t += 1e-3) { const q = m.reference(t); pv = Math.max(pv, q.v); pa = Math.max(pa, q.a); }
near(pv, 3.125, 1e-3, 'peak reference speed'); near(pa, 5.7735 * 10 / 36, 2e-3, 'peak reference accel = 5.7735 d/T^2');
near(m.feedforward(r0), 9.81, 1e-12, 'FF at rest = m g (hover thrust)');
near(m.feedforward({ a: 1.0 }), 10.81, 1e-12, 'FF = m (g + a_ref)');

/* ---- plant: clamp, exact lag, symplectic Euler, ground ---- */
{
  const s = m.makeState(0);
  near(s.T, 9.81, 1e-12, 'starts at hover thrust'); assert.strictEqual(s.z, 0);
  m.stepModel(s, 100, P.dt, 0);                                           // command above Tmax is clipped to 20
  near(s.T, 9.81 + (20 - 9.81) * (1 - Math.exp(-P.dt / 0.2)), 1e-12, 'exact first-order lag toward the clipped 20 N');
  const s2 = m.makeState(0); m.stepModel(s2, -50, P.dt, 0);
  near(s2.T, 9.81 + (1 - 9.81) * (1 - Math.exp(-P.dt / 0.2)), 1e-12, 'clipped to the 1 N flameout floor');
  assert.strictEqual(s2.z, 0, 'ground clamp holds the rocket on the pad'); assert.strictEqual(s2.v, 0, 'no downward speed on the pad');
  const s3 = m.makeState(0); s3.z = 5; for (let i = 0; i < 240; i++) m.stepModel(s3, 9.81, P.dt, 0);
  near(s3.z, 5, 1e-9, 'hover thrust holds altitude');
  const s4 = m.makeState(10); near(s4.m, 1.1, 1e-12, 'mass error +10 % makes the plant 1.1 kg'); m.setMass(s4, -10); near(s4.m, 0.9, 1e-12, 'setMass');
}

/* ---- PID: anti-windup and clamp ---- */
{
  const pid = new m.PID({ Kp: 2, Ki: 0.5, Kd: 3, uMin: 1, uMax: 20 });
  for (let i = 0; i < 240 * 200; i++) pid.update(100, 0, P.dt, 0, 0);        // 100 m error, saturated for 200 s
  assert.ok(pid.integ * 0.5 <= 19 + 1e-9, 'I contribution clamped to Tmax - Tmin');
  const o = pid.update(100, 0, P.dt, 0, 0);
  assert.strictEqual(o.u, 20, 'output clipped at Tmax'); assert.ok(o.sat, 'saturation flagged');
  const pid2 = new m.PID({ Kp: 2, Ki: 0.5, Kd: 3, uMin: 1, uMax: 20 });
  pid2.update(10, 0, P.dt, 0, 0); const o2 = pid2.update(10, 0, P.dt, 0, 0);
  near(o2.D, 0, 1e-12, 'derivative on measurement: a setpoint step gives no kick');
}

/* ---- controller modes ---- */
{
  const pid = new m.PID({ Kp: 2, Ki: 0.5, Kd: 3, uMin: 1, uMax: 20 });
  const ff = m.controller('ff', pid, r3, 4.5, P.dt);
  near(ff.ff, m.feedforward(r3), 1e-12, 'FF only: slice equals m (g + a_ref)'); assert.strictEqual(ff.fb, 0, 'FF only: no feedback slice');
  const fb = m.controller('fb', pid, r3, 4.5, P.dt);
  assert.strictEqual(fb.ff, 0, 'FB only: no feedforward slice'); near(fb.P, 2 * 0.5, 1e-12, 'FB only: P = Kp e');
  const both = m.controller('both', pid, r3, 4.5, P.dt);
  near(both.ff, m.feedforward(r3), 1e-12, 'Both: FF slice present'); near(both.fb, both.P + both.I + both.D, 1e-12, 'Both: FB slice = P + I + D');
  near(both.u, G.clamp(both.ff + both.fb, 1, 20), 1e-12, 'slices add to the clipped command');
}

/* ---- S4 scenarios (same gains everywhere) ---- */
const FB = m.simulate({ mode: 'fb' }), FF = m.simulate({ mode: 'ff' }), BOTH = m.simulate({ mode: 'both' });
// feedback alone is always late: large RMS, peak error near the moment of peak reference speed, integrator still hunting at 12 s
assert.ok(FB.rms > 2.5 && FB.rms < 3.5, `FB only RMS ${FB.rms}`);
assert.ok(FB.peak > 6 && FB.peak < 7.5, `FB only peak ${FB.peak}`);
assert.ok(FB.tAtPeak > 2.5 && FB.tAtPeak < 5, `FB only peak error during the fast part of the climb (t=${FB.tAtPeak})`);
assert.ok(FB.peakSigned > 0, 'FB only lags below the reference');
assert.ok(Math.abs(FB.finalErr) > 0.3, `FB only still off at 12 s (${FB.finalErr})`);
assert.ok(FB.satFrac > 0.05, 'FB only saturates during the catch-up');
// feedforward alone is blind but on time: small error from the thrust lag only, exact at the end
assert.ok(FF.rms > 0.15 && FF.rms < 0.4, `FF only RMS ${FF.rms}`);
assert.ok(FF.peak > 0.4 && FF.peak < 0.8, `FF only peak ${FF.peak}`);
near(FF.peak, P.tau * 3.125, 0.1, 'FF only peak error ~ tau * peak reference speed (pure lag)');
near(FF.finalErr, 0, 1e-3, 'FF only ends exactly on the reference with a perfect model');
assert.strictEqual(FF.satFrac, 0, 'FF only never saturates');
// both: best of the three
assert.ok(BOTH.rms < 0.12, `Both RMS ${BOTH.rms}`);
assert.ok(BOTH.peak < 0.25, `Both peak ${BOTH.peak}`);
assert.ok(BOTH.rms < FF.rms && FF.rms < FB.rms, 'ordering: Both < FF only < FB only (RMS)');
assert.ok(Math.abs(BOTH.finalErr) < 0.05, `Both final error ${BOTH.finalErr}`);
// the feedback slice gets small when feedforward carries the load
assert.ok(BOTH.meanAbsFbClimb < 0.1 * FB.meanAbsFbClimb, `FB slice during climb: both ${BOTH.meanAbsFbClimb} vs fb-only ${FB.meanAbsFbClimb}`);
assert.ok(Math.abs(BOTH.finalI) < 0.5, 'with FF on the integral stays near zero');

/* ---- model error: heavier (+10 %) and lighter (-10 %) plant ---- */
const FFh = m.simulate({ mode: 'ff', massErr: 10 }), BOTHh = m.simulate({ mode: 'both', massErr: 10 }), FBh = m.simulate({ mode: 'fb', massErr: 10 });
assert.ok(FFh.finalZ < 0.5 && FFh.finalErr > 9, 'FF only with a heavier rocket never leaves the ground');
assert.ok(BOTHh.rms < 0.3 && Math.abs(BOTHh.finalErr) < 0.05, `Both with +10 % mass still tracks (RMS ${BOTHh.rms})`);
assert.ok(FBh.rms > 2.5, 'FB only barely notices mass error (it was already bad)');
const FFl = m.simulate({ mode: 'ff', massErr: -10 }), BOTHl = m.simulate({ mode: 'both', massErr: -10 });
assert.ok(FFl.finalZ > 40 && FFl.peakSigned < 0, `FF only with a lighter rocket climbs away (z=${FFl.finalZ})`);
assert.ok(BOTHl.rms < 0.3 && Math.abs(BOTHl.finalErr) < 0.05, `Both with -10 % mass still tracks (RMS ${BOTHl.rms})`);

/* ---- gust: 2 N down for 2 s at t = 7 s (during the hold) ---- */
const FFg = m.simulate({ mode: 'ff', gustAt: 7 }), BOTHg = m.simulate({ mode: 'both', gustAt: 7 }), FBg = m.simulate({ mode: 'fb', gustAt: 7 });
assert.ok(FFg.finalZ < 0.5, 'FF only does nothing about the gust and falls to the ground');
assert.ok(BOTHg.rms < 0.4 && BOTHg.peak < 1.0 && Math.abs(BOTHg.finalErr) < 0.4, `Both rides out the gust (peak ${BOTHg.peak})`);
assert.ok(BOTHg.peak > BOTH.peak, 'the gust shows up as extra error in Both');
{ // the feedback slice does the gust work: FB slice is larger during the gust than before it
  const run = m.createRun({ mode: 'both' }); let fbBefore = 0, fbDuring = 0, nb = 0, nd = 0;
  for (let i = 0; i < Math.round(9.5 / P.dt); i++) {
    if (Math.abs(run.t - 7) < P.dt / 2) m.triggerGust(run);
    m.stepRun(run, P.dt);
    if (run.t > 6.5 && run.t < 7) { fbBefore += run.ctl.fb; nb++; }
    if (run.t > 8.5 && run.t < 9) { fbDuring += run.ctl.fb; nd++; }
  }
  assert.ok(fbDuring / nd > fbBefore / nb + 1.0, `feedback slice rises to push back on the gust (${(fbDuring / nd).toFixed(2)} N vs ${(fbBefore / nb).toFixed(2)} N)`);
  near(run.ctl.ff, 9.81, 1e-9, 'feedforward slice ignores the gust');
}
assert.ok(FBg.rms > 2.5, 'FB only with gust');

/* ---- halved Kp and Ki: fine with FF, struggles without ---- */
const BOTHhalf = m.simulate({ mode: 'both', gains: { Kp: 1, Ki: 0.25 } }), FBhalf = m.simulate({ mode: 'fb', gains: { Kp: 1, Ki: 0.25 } });
assert.ok(BOTHhalf.rms < 0.15 && BOTHhalf.peak < 0.3, `Both with halved gains still tracks (RMS ${BOTHhalf.rms})`);
assert.ok(FBhalf.rms > FB.rms && FBhalf.peak > 8, `FB only with halved gains struggles more (RMS ${FBhalf.rms}, peak ${FBhalf.peak})`);

/* ---- the integral weighs the rocket ---- */
const FB60 = m.simulate({ mode: 'fb', tEnd: 60 });
near(FB60.finalI, 9.81, 0.05, 'FB only: I term settles at the weight, 9.81 N');
const FB60h = m.simulate({ mode: 'fb', tEnd: 90, massErrAt: [30, 10] });
near(FB60h.finalI, 1.1 * 9.81, 0.05, 'FB only: after mass +10 % the I term crawls to the new weight');
const BOTH60h = m.simulate({ mode: 'both', tEnd: 60, massErr: 10 });
near(BOTH60h.finalI, 0.1 * 9.81, 0.05, 'Both: I term only carries the model error (0.981 N)');
near(m.simulate({ mode: 'both', tEnd: 60 }).finalI, 0, 0.05, 'Both: I term near zero with a perfect model');

/* ---- run bookkeeping: restart, score freeze, NaN guard ---- */
{
  const run = m.createRun({ mode: 'fb' });
  for (let i = 0; i < Math.round(14 / P.dt); i++) m.stepRun(run, P.dt);
  near(run.tScored, P.tRun, P.dt + 1e-9, 'RMS/peak window stops at 12 s (within one step)');
  const rmsAt14 = run.rms; for (let i = 0; i < 240; i++) m.stepRun(run, P.dt);
  assert.strictEqual(run.rms, rmsAt14, 'RMS frozen after the scoring window');
  m.restartRun(run);
  assert.strictEqual(run.t, 0); assert.strictEqual(run.rms, 0); assert.strictEqual(run.peak, 0); assert.strictEqual(run.state.z, 0);
  assert.strictEqual(run.pid.integ, 0, 'restart resets the PID');
  m.setGains(run, { Kp: 4 }); assert.strictEqual(run.pid.Kp, 4); assert.strictEqual(run.gains.Kp, 4);
  m.setMassErr(run, 5); near(run.state.m, 1.05, 1e-12, 'mass error applies live');
  assert.ok(m.isFiniteRun(run)); run.state.v = NaN; assert.ok(!m.isFiniteRun(run), 'NaN detected');
  // pause/resume: stepping is a pure function of dt, so a gap in wall time changes nothing
  const a = m.createRun({}), b = m.createRun({});
  for (let i = 0; i < 1000; i++) m.stepRun(a, P.dt);
  for (let i = 0; i < 500; i++) m.stepRun(b, P.dt); for (let i = 0; i < 500; i++) m.stepRun(b, P.dt);
  near(a.state.z, b.state.z, 1e-12, 'fixed-step integrator is independent of frame timing');
}

console.log('S4 at toy numbers: FB only RMS %s m, peak %s m at %s s; FF only RMS %s m, peak %s m; Both RMS %s m, peak %s m',
  FB.rms.toFixed(3), FB.peak.toFixed(2), FB.tAtPeak.toFixed(1), FF.rms.toFixed(3), FF.peak.toFixed(2), BOTH.rms.toFixed(3), BOTH.peak.toFixed(3));
console.log('mass +10 %: FF only on the ground, Both RMS %s m; mass -10 %: FF only at %s m by 12 s, Both RMS %s m; gust: Both peak %s m',
  BOTHh.rms.toFixed(3), FFl.finalZ.toFixed(0), BOTHl.rms.toFixed(3), BOTHg.peak.toFixed(2));
console.log('PASS ffb');
