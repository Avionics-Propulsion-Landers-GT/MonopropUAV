/* Minimal GTPL stand-in for running sim math in node (no DOM, no canvas).
   Usage in a test:  const GTPL = require('./node-stub.js'); require('./sim-pid.js'); const m = GTPL.math.pid; ...
   Sim modules must keep their physics/controller math in pure functions registered on GTPL.math.<name>
   and must not touch document/window at load time (only inside the mount function). */
const GTPL = { sims: {}, math: {}, instances: [], reducedMotion: false };
GTPL.clamp = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
GTPL.lerp = (a, b, t) => a + (b - a) * t;
GTPL.fmt = function (v, digits) {
  if (digits === undefined) digits = 2;
  if (!isFinite(v)) return '—';
  const s = Math.abs(v).toFixed(digits);
  return (v < 0 && Number(s) !== 0 ? '−' : '') + s;
};
GTPL.rng = function (seed) {
  let a = (seed >>> 0) || 1;
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
};
GTPL.gauss = function (rand) {
  let u = 0, v = 0;
  while (u === 0) u = rand();
  while (v === 0) v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
};
GTPL.niceStep = function (range, target) {
  const raw = range / Math.max(1, target);
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  const r = raw / p;
  const m = r < 1.5 ? 1 : r < 3.5 ? 2 : r < 7.5 ? 5 : 10;
  return m * p;
};
/* DOM-dependent helpers are stubbed so a module can be required without crashing if it touches them accidentally */
const noop = () => ({ root: null, get: () => 0, set: () => {}, setLabel: () => {}, setRange: () => {}, setDisabled: () => {} });
['slider', 'toggle', 'button', 'segmented', 'readout', 'legend', 'group', 'hidden'].forEach(k => { GTPL[k] = noop; });
GTPL.el = () => ({ appendChild() {}, append() {}, setAttribute() {}, addEventListener() {}, style: {}, dataset: {}, classList: { add() {}, remove() {} } });
GTPL.theme = { get: () => '#000000', isDark: () => true, onChange: () => () => {}, alpha: () => 'rgba(0,0,0,1)' };
globalThis.window = globalThis;
globalThis.GTPL = GTPL;
module.exports = GTPL;
