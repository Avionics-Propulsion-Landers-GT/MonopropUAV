/* GTPL controls guide: shared helpers. Plain browser JS, no dependencies.
   Everything hangs off window.GTPL. Sims register themselves on GTPL.sims. */
(function () {
  'use strict';
  const GTPL = { sims: {}, instances: [] };
  const root = document.documentElement;

  /* ---------- ticker: one rAF loop for the whole page ---------- */
  const ticker = { frame: 0, now: 0, clients: new Set(), running: false };
  function tick(ts) {
    ticker.frame++;
    ticker.now = ts / 1000;
    for (const fn of ticker.clients) { try { fn(ts / 1000); } catch (e) { console.error(e); } }
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
  GTPL.ticker = ticker;

  /* ---------- theme ---------- */
  const mq = matchMedia('(prefers-color-scheme: dark)');
  const theme = (function () {
    const cache = new Map();
    let cacheFrame = -1;
    const listeners = new Set();
    function get(name) {
      if (cacheFrame !== ticker.frame) { cache.clear(); cacheFrame = ticker.frame; }
      let v = cache.get(name);
      if (v === undefined) {
        v = getComputedStyle(root).getPropertyValue(name).trim();
        cache.set(name, v);
      }
      return v;
    }
    function isDark() {
      const t = root.getAttribute('data-theme');
      if (t === 'dark') return true;
      if (t === 'light') return false;
      return mq.matches;
    }
    function fire() { cache.clear(); cacheFrame = -1; listeners.forEach(fn => { try { fn(isDark()); } catch (e) { console.error(e); } }); }
    new MutationObserver(fire).observe(root, { attributes: true, attributeFilter: ['data-theme'] });
    mq.addEventListener('change', fire);
    function onChange(fn) { listeners.add(fn); return () => listeners.delete(fn); }
    /* rgba(token, alpha): token must resolve to #rrggbb */
    function alpha(name, a) {
      const hex = get(name);
      const m = /^#([0-9a-f]{6})$/i.exec(hex);
      if (!m) return hex;
      const n = parseInt(m[1], 16);
      return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
    }
    return { get, isDark, onChange, fire, alpha };
  })();
  GTPL.theme = theme;
  GTPL.reducedMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* ---------- small utilities ---------- */
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
  /* approx standard normal from a uniform rng */
  GTPL.gauss = function (rand) {
    let u = 0, v = 0;
    while (u === 0) u = rand();
    while (v === 0) v = rand();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };

  /* ---------- DOM builder ---------- */
  GTPL.el = function (tag, attrs, children) {
    const e = document.createElement(tag);
    if (attrs) {
      for (const k in attrs) {
        const v = attrs[k];
        if (v === undefined || v === null || v === false) continue;
        if (k === 'class') e.className = v;
        else if (k === 'text') e.textContent = v;
        else if (k === 'html') e.innerHTML = v;
        else if (k === 'style') e.style.cssText = v;
        else if (k.slice(0, 2) === 'on' && typeof v === 'function') e.addEventListener(k.slice(2).toLowerCase(), v);
        else e.setAttribute(k, v === true ? '' : v);
      }
    }
    if (children !== undefined && children !== null) {
      (Array.isArray(children) ? children : [children]).forEach(c => {
        if (c === null || c === undefined || c === false) return;
        e.appendChild(typeof c === 'string' ? document.createTextNode(c) : c);
      });
    }
    return e;
  };
  const el = GTPL.el;

  /* ---------- controls ---------- */
  GTPL.slider = function (o) {
    const digits = o.digits === undefined ? 2 : o.digits;
    const unit = o.unit ? ' ' + o.unit : '';
    const out = el('output', { class: 'ctl-val' });
    const label = el('span', { class: 'ctl-label', text: o.label });
    const input = el('input', { type: 'range', min: o.min, max: o.max, step: o.step === undefined ? 'any' : o.step, value: o.value });
    if (o.title) input.title = o.title;
    const rootEl = el('label', { class: 'ctl' + (o.vertical ? ' ctl-vertical' : '') }, [
      el('span', { class: 'ctl-row' }, [label, out]), input,
    ]);
    function show() { out.textContent = (o.format ? o.format(Number(input.value)) : GTPL.fmt(Number(input.value), digits)) + unit; }
    input.addEventListener('input', () => { show(); if (o.onInput) o.onInput(Number(input.value)); });
    show();
    return {
      root: rootEl, input,
      get: () => Number(input.value),
      set: (v, silent) => { input.value = v; show(); if (!silent && o.onInput) o.onInput(Number(input.value)); },
      setLabel: (s) => { label.textContent = s; },
      setRange: (min, max) => { input.min = min; input.max = max; show(); },
    };
  };

  GTPL.toggle = function (o) {
    let val = !!o.value;
    const b = el('button', { type: 'button', class: 'tog', 'aria-pressed': String(val) }, [el('span', { class: 'tog-dot' }), el('span', { class: 'tog-label', text: o.label })]);
    b.addEventListener('click', () => { set(!val); });
    function set(v, silent) { val = !!v; b.setAttribute('aria-pressed', String(val)); if (!silent && o.onChange) o.onChange(val); }
    return { root: b, get: () => val, set };
  };

  GTPL.button = function (o) {
    const b = el('button', { type: 'button', class: 'btn' + (o.kind ? ' btn-' + o.kind : ''), text: o.label });
    if (o.title) b.title = o.title;
    if (o.onClick) b.addEventListener('click', o.onClick);
    return { root: b, setLabel: (s) => { b.textContent = s; }, setDisabled: (d) => { b.disabled = !!d; } };
  };

  GTPL.segmented = function (o) {
    let val = o.value;
    const rootEl = el('div', { class: 'seg', role: 'radiogroup', 'aria-label': o.label || '' });
    const buttons = o.options.map(opt => {
      const b = el('button', { type: 'button', role: 'radio', class: 'seg-btn', text: opt.label, 'aria-checked': String(opt.value === val) });
      b.addEventListener('click', () => set(opt.value));
      rootEl.appendChild(b);
      return { b, opt };
    });
    function set(v, silent) {
      val = v;
      buttons.forEach(({ b, opt }) => b.setAttribute('aria-checked', String(opt.value === val)));
      if (!silent && o.onChange) o.onChange(val);
    }
    return { root: rootEl, get: () => val, set };
  };

  GTPL.readout = function (o) {
    const digits = o.digits === undefined ? 2 : o.digits;
    const val = el('span', { class: 'ro-num', text: '—' });
    const rootEl = el('div', { class: 'ro' + (o.big ? ' ro-big' : '') }, [
      el('span', { class: 'ro-label', text: o.label }),
      el('span', { class: 'ro-val' }, [val, o.unit ? el('span', { class: 'ro-unit', text: o.unit }) : null]),
    ]);
    return {
      root: rootEl,
      set: (v, cls) => { val.textContent = typeof v === 'string' ? v : GTPL.fmt(v, digits); rootEl.dataset.state = cls || ''; },
    };
  };

  GTPL.hidden = (text) => el('p', { class: 'visually-hidden', text });

  /* ---------- canvas with DPR + resize ---------- */
  GTPL.canvas = function (parent, o) {
    o = o || {};
    const aspect = o.aspect || 16 / 9;
    const minH = o.minHeight || 240;
    const canvas = el('canvas', { 'aria-hidden': 'true' });
    parent.appendChild(canvas);
    const ctx = canvas.getContext('2d');
    const api = { canvas, ctx, width: 0, height: 0, _cbs: [] };
    function fit() {
      const w = Math.max(200, parent.clientWidth);
      let h = o.height ? o.height : Math.round(w / aspect);
      if (h < minH) h = minH;
      if (o.maxHeight && h > o.maxHeight) h = o.maxHeight;
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      canvas.style.width = w + 'px';
      canvas.style.height = h + 'px';
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      api.width = w; api.height = h;
      api._cbs.forEach(fn => fn(w, h));
    }
    const ro = new ResizeObserver(() => fit());
    ro.observe(parent);
    fit();
    api.onResize = (fn) => { api._cbs.push(fn); };
    api.destroy = () => { ro.disconnect(); canvas.remove(); };
    api.clear = () => { ctx.clearRect(0, 0, api.width, api.height); };
    return api;
  };

  /* ---------- fixed-step loop, auto-pause offscreen ---------- */
  GTPL.loop = function (o) {
    const dt = o.dt || 1 / 240;
    const maxSub = o.maxSubsteps || 12;
    let acc = 0, last = null, running = false, visible = true, wanted = false, speed = o.speed || 1;
    const api = {};
    function frame(now) {
      if (!running) return;
      if (last === null) last = now;
      let elapsed = (now - last) * speed;
      last = now;
      if (elapsed > 0.25) elapsed = 0.25;
      acc += elapsed;
      let n = 0;
      while (acc >= dt && n < maxSub) { o.step(dt); acc -= dt; n++; }
      if (n === maxSub) acc = 0;
      if (o.render) o.render(acc / dt);
    }
    ticker.clients.add(frame);
    function apply() {
      const should = wanted && visible && !document.hidden;
      if (should && !running) { running = true; last = null; }
      else if (!should && running) { running = false; }
      if (o.onState) o.onState(running, wanted);
    }
    if (o.root && 'IntersectionObserver' in window) {
      new IntersectionObserver((entries) => { visible = entries[0].isIntersecting; apply(); }, { rootMargin: '80px' }).observe(o.root);
    }
    document.addEventListener('visibilitychange', apply);
    api.start = () => { wanted = true; apply(); };
    api.stop = () => { wanted = false; apply(); };
    api.toggle = () => { wanted ? api.stop() : api.start(); };
    api.setSpeed = (s) => { speed = s; };
    api.renderOnce = () => { if (o.render) o.render(0); };
    api.destroy = () => { ticker.clients.delete(frame); };
    Object.defineProperty(api, 'running', { get: () => running });
    Object.defineProperty(api, 'wanted', { get: () => wanted });
    if (o.autoplay !== false && !GTPL.reducedMotion) api.start();
    return api;
  };

  /* ---------- nice ticks ---------- */
  GTPL.niceStep = function (range, target) {
    const raw = range / Math.max(1, target);
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const r = raw / p;
    const m = r < 1.5 ? 1 : r < 3.5 ? 2 : r < 7.5 ? 5 : 10;
    return m * p;
  };

  /* ---------- strip chart ---------- */
  GTPL.StripChart = function (o) {
    this.o = Object.assign({ duration: 12, autoscale: false, ymin: 0, ymax: 1, padding: 0.1, legend: true, series: [], thresholds: [] }, o);
    this.data = new Map();
    this.o.series.forEach(s => this.data.set(s.key, { t: [], v: [] }));
    this.tmax = 0;
  };
  GTPL.StripChart.prototype.clear = function () {
    this.data.forEach(d => { d.t.length = 0; d.v.length = 0; });
    this.tmax = 0;
  };
  GTPL.StripChart.prototype.push = function (t, values) {
    const cutoff = t - this.o.duration - 0.5;
    for (const key in values) {
      const d = this.data.get(key);
      if (!d) continue;
      const v = values[key];
      if (v === undefined || v === null) continue;
      d.t.push(t); d.v.push(v);
      let drop = 0;
      while (drop < d.t.length && d.t[drop] < cutoff) drop++;
      if (drop > 64) { d.t.splice(0, drop); d.v.splice(0, drop); }
    }
    if (t > this.tmax) this.tmax = t;
  };
  GTPL.StripChart.prototype.draw = function (ctx, rect) {
    const o = this.o, g = theme.get;
    const padL = o.padLeft || 44, padR = 10, padT = o.legend ? 22 : 8, padB = o.xLabel ? 34 : 22;
    const x0 = rect.x + padL, y0 = rect.y + padT, w = rect.w - padL - padR, h = rect.h - padT - padB;
    if (w < 20 || h < 20) return;
    const t1 = Math.max(this.tmax, o.duration), t0 = t1 - o.duration;
    let ymin = o.ymin, ymax = o.ymax;
    if (o.autoscale) {
      let lo = Infinity, hi = -Infinity;
      this.data.forEach(d => { for (let i = 0; i < d.v.length; i++) { if (d.t[i] < t0) continue; const v = d.v[i]; if (v < lo) lo = v; if (v > hi) hi = v; } });
      (o.thresholds || []).forEach(th => { if (th.y < lo) lo = th.y; if (th.y > hi) hi = th.y; });
      if (!isFinite(lo)) { lo = o.ymin; hi = o.ymax; }
      const minSpan = o.minSpan === undefined ? 1 : o.minSpan;
      if (hi - lo < minSpan) { const c = (hi + lo) / 2; lo = c - minSpan / 2; hi = c + minSpan / 2; }
      const pad = (hi - lo) * o.padding;
      ymin = lo - pad; ymax = hi + pad;
      if (o.floor !== undefined && ymin < o.floor) ymin = o.floor;
    }
    const sx = (t) => x0 + (t - t0) / (t1 - t0) * w;
    const sy = (v) => y0 + (1 - (v - ymin) / (ymax - ymin)) * h;
    ctx.save();
    ctx.font = '11px ' + g('--font-mono');
    ctx.textBaseline = 'middle';
    /* grid */
    const step = GTPL.niceStep(ymax - ymin, 4);
    ctx.strokeStyle = g('--line'); ctx.lineWidth = 1; ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'right';
    for (let v = Math.ceil(ymin / step) * step; v <= ymax + 1e-9; v += step) {
      const y = Math.round(sy(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x0 + w, y); ctx.stroke();
      ctx.fillText(GTPL.fmt(v, step < 1 ? 1 : 0), x0 - 6, y);
    }
    /* time axis */
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    const tstep = GTPL.niceStep(o.duration, 6);
    for (let t = Math.ceil(t0 / tstep) * tstep; t <= t1 + 1e-9; t += tstep) {
      const x = Math.round(sx(t)) + 0.5;
      ctx.strokeStyle = g('--line'); ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y0 + h); ctx.stroke();
      ctx.fillText(GTPL.fmt(t, 0), x, y0 + h + 5);
    }
    /* axes */
    ctx.strokeStyle = g('--line-2'); ctx.beginPath(); ctx.moveTo(x0 + 0.5, y0); ctx.lineTo(x0 + 0.5, y0 + h + 0.5); ctx.lineTo(x0 + w, y0 + h + 0.5); ctx.stroke();
    if (o.yLabel) { ctx.save(); ctx.translate(rect.x + 10, y0 + h / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillStyle = g('--ink-3'); ctx.fillText(o.yLabel, 0, 0); ctx.restore(); }
    if (o.xLabel) { ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillStyle = g('--ink-3'); ctx.fillText(o.xLabel, x0 + w, rect.y + rect.h); }
    /* thresholds */
    ctx.beginPath(); ctx.rect(x0, y0, w, h); ctx.clip();
    (o.thresholds || []).forEach(th => {
      const y = Math.round(sy(th.y)) + 0.5;
      ctx.setLineDash(th.dash || [3, 4]); ctx.strokeStyle = g(th.color || '--ink-3'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x0 + w, y); ctx.stroke(); ctx.setLineDash([]);
      if (th.label) { ctx.fillStyle = g(th.color || '--ink-3'); ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillText(th.label, x0 + 4, y - 2); }
    });
    /* series */
    o.series.forEach(s => {
      const d = this.data.get(s.key);
      if (!d || d.t.length < 2) return;
      ctx.strokeStyle = g(s.color); ctx.lineWidth = s.width || 1.75; ctx.setLineDash(s.dash || []);
      ctx.lineJoin = 'round'; ctx.beginPath();
      let started = false;
      for (let i = 0; i < d.t.length; i++) {
        if (d.t[i] < t0 - 0.2) continue;
        const x = sx(d.t[i]), y = sy(d.v[i]);
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke(); ctx.setLineDash([]);
      if (s.endDot !== false) { const x = sx(d.t[d.t.length - 1]), y = sy(d.v[d.v.length - 1]); ctx.fillStyle = g(s.color); ctx.beginPath(); ctx.arc(x, y, 2.5, 0, Math.PI * 2); ctx.fill(); }
    });
    ctx.restore();
    /* legend */
    if (o.legend) {
      ctx.save(); ctx.font = '11px ' + g('--font-mono'); ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
      let x = x0 + w;
      for (let i = o.series.length - 1; i >= 0; i--) {
        const s = o.series[i];
        if (!s.label) continue;
        const tw = ctx.measureText(s.label).width;
        x -= tw + 26;
        ctx.strokeStyle = g(s.color); ctx.lineWidth = 2; ctx.setLineDash(s.dash || []);
        ctx.beginPath(); ctx.moveTo(x, rect.y + 9); ctx.lineTo(x + 14, rect.y + 9); ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = g('--ink-2'); ctx.fillText(s.label, x + 18, rect.y + 9);
      }
      ctx.restore();
    }
    return { x0, y0, w, h, sx, sy, ymin, ymax, t0, t1 };
  };

  /* ---------- drawing: rocket, ground, setpoint, bar stack ---------- */
  /* Stylized Monarch: central column, two side tanks, top frame, four legs, gimbaled nozzle, flame.
     Origin at the nozzle pivot (bottom of the column). Body extends upward (negative y). */
  GTPL.drawRocket = function (ctx, p) {
    const g = theme.get;
    const s = p.scale || 80;          // px per rocket height unit
    const tilt = p.tilt || 0;         // rad, positive = leaning to +x (clockwise on screen)
    const gimbal = p.gimbal || 0;     // rad, nozzle angle relative to body
    const thr = GTPL.clamp(p.thrust01 === undefined ? 0 : p.thrust01, 0, 1.4);
    const rand = p.rand || Math.random;
    const cmY = -(p.cm === undefined ? 0.45 : p.cm) * s; // center of mass above pivot
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.translate(0, cmY); ctx.rotate(tilt); ctx.translate(0, -cmY);
    const line = g('--ink-2'), fill = g('--bg-3'), dark = g('--ink');
    ctx.lineWidth = Math.max(1, s * 0.018); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    /* legs: from column base outward and down to feet */
    const legTopY = -0.30 * s, footX = 0.42 * s, footY = 0.16 * s;
    ctx.strokeStyle = g('--ink-3');
    [[-1, 0.6], [1, 0.6], [-1, 1], [1, 1]].forEach(([sx, k]) => {
      ctx.beginPath(); ctx.moveTo(sx * 0.10 * s, legTopY); ctx.lineTo(sx * footX * k, footY); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(sx * 0.10 * s, -0.05 * s); ctx.lineTo(sx * footX * k, footY); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(sx * footX * k - 0.04 * s, footY); ctx.lineTo(sx * footX * k + 0.04 * s, footY); ctx.stroke();
    });
    /* side tanks */
    ctx.strokeStyle = line; ctx.fillStyle = fill;
    [-1, 1].forEach(sx => { roundRect(ctx, sx * 0.19 * s - 0.07 * s, -0.78 * s, 0.14 * s, 0.34 * s, 0.05 * s); ctx.fill(); ctx.stroke(); });
    /* central column */
    roundRect(ctx, -0.075 * s, -0.95 * s, 0.15 * s, 0.85 * s, 0.03 * s); ctx.fill(); ctx.stroke();
    /* top frame */
    ctx.beginPath(); ctx.rect(-0.12 * s, -1.05 * s, 0.24 * s, 0.10 * s); ctx.fill(); ctx.stroke();
    /* nozzle, rotated by gimbal about the pivot */
    ctx.save(); ctx.rotate(gimbal);
    ctx.fillStyle = dark; ctx.strokeStyle = dark;
    ctx.beginPath(); ctx.moveTo(-0.04 * s, 0); ctx.lineTo(0.04 * s, 0); ctx.lineTo(0.075 * s, 0.11 * s); ctx.lineTo(-0.075 * s, 0.11 * s); ctx.closePath(); ctx.fill();
    /* flame */
    if (thr > 0.02) {
      const flick = 1 + (rand() - 0.5) * 0.18;
      const len = (0.12 + 0.55 * thr) * s * flick, wid = 0.075 * s * (0.8 + 0.4 * thr);
      ctx.globalAlpha = 0.9;
      ctx.fillStyle = g('--gold');
      ctx.beginPath(); ctx.moveTo(-wid, 0.11 * s); ctx.quadraticCurveTo(-wid * 0.9, 0.11 * s + len * 0.55, 0, 0.11 * s + len); ctx.quadraticCurveTo(wid * 0.9, 0.11 * s + len * 0.55, wid, 0.11 * s); ctx.closePath(); ctx.fill();
      ctx.globalAlpha = 0.85; ctx.fillStyle = g('--bg');
      ctx.beginPath(); ctx.moveTo(-wid * 0.45, 0.11 * s); ctx.quadraticCurveTo(-wid * 0.4, 0.11 * s + len * 0.3, 0, 0.11 * s + len * 0.5); ctx.quadraticCurveTo(wid * 0.4, 0.11 * s + len * 0.3, wid * 0.45, 0.11 * s); ctx.closePath(); ctx.fill();
      ctx.globalAlpha = 1;
    }
    ctx.restore();
    ctx.restore();
  };
  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath(); ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r); ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h); ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r); ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y); ctx.closePath();
  }
  GTPL.roundRect = roundRect;

  /* ground line + altitude ticks. rect: drawing area; y0: screen y of ground; mpp: meters per px; ticks: array of meters */
  GTPL.drawGround = function (ctx, rect, o) {
    const g = theme.get;
    ctx.save();
    ctx.strokeStyle = g('--line-2'); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(rect.x, o.y0 + 0.5); ctx.lineTo(rect.x + rect.w, o.y0 + 0.5); ctx.stroke();
    ctx.fillStyle = theme.alpha('--line', 0.5);
    ctx.fillRect(rect.x, o.y0 + 1, rect.w, Math.max(0, rect.y + rect.h - o.y0 - 1));
    ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.strokeStyle = g('--line'); ctx.lineWidth = 1;
    (o.ticks || []).forEach(m => {
      const y = Math.round(o.y0 - m / o.mpp) + 0.5;
      if (y < rect.y + 8) return;
      ctx.beginPath(); ctx.moveTo(rect.x, y); ctx.lineTo(rect.x + (o.tickLen || 8), y); ctx.stroke();
      ctx.fillText(m + ' m', rect.x + (o.tickLen || 8) + 4, y);
    });
    ctx.restore();
  };

  GTPL.drawSetpoint = function (ctx, rect, o) {
    const g = theme.get;
    ctx.save();
    const y = Math.round(o.y) + 0.5;
    ctx.setLineDash(o.dash || [6, 5]); ctx.strokeStyle = g(o.color || '--sig-ref'); ctx.lineWidth = o.width || 1.5;
    ctx.beginPath(); ctx.moveTo(rect.x, y); ctx.lineTo(rect.x + rect.w, y); ctx.stroke(); ctx.setLineDash([]);
    if (o.label) {
      ctx.font = '11px ' + g('--font-mono'); ctx.fillStyle = g(o.color || '--sig-ref'); ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
      ctx.fillText(o.label, rect.x + rect.w - 6, y - 3);
    }
    ctx.restore();
  };

  /* vertical stacked bar. segments stack from zero (positive up, negative down).
     o: {segments:[{value,color,label}], min, max, limitLo, limitHi, title, unit} */
  GTPL.drawBarStack = function (ctx, rect, o) {
    const g = theme.get;
    const padT = 18, padB = 16;
    const x = rect.x, w = rect.w, y0 = rect.y + padT, h = rect.h - padT - padB;
    const sy = (v) => y0 + (1 - (v - o.min) / (o.max - o.min)) * h;
    ctx.save();
    ctx.font = '11px ' + g('--font-mono'); ctx.textBaseline = 'middle';
    /* track */
    ctx.fillStyle = g('--bg-3'); ctx.fillRect(x, y0, w, h);
    /* segments */
    let up = 0, down = 0;
    o.segments.forEach(seg => {
      const v = seg.value || 0;
      if (Math.abs(v) < 1e-9) return;
      let a, b;
      if (v > 0) { a = up; b = up + v; up = b; } else { a = down + v; b = down; down = a; }
      const ya = sy(GTPL.clamp(b, o.min, o.max)), yb = sy(GTPL.clamp(a, o.min, o.max));
      ctx.fillStyle = g(seg.color); ctx.fillRect(x + 2, ya, w - 4, Math.max(0, yb - ya));
    });
    /* total marker */
    const total = up + down;
    const yt = sy(GTPL.clamp(total, o.min, o.max));
    ctx.strokeStyle = g('--ink'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, Math.round(yt) + 0.5); ctx.lineTo(x + w, Math.round(yt) + 0.5); ctx.stroke();
    /* limits; labels go on the side named by o.labelSide ('right' default, or 'left', or 'none') */
    const side = o.labelSide || 'right';
    [['limitLo', 'min'], ['limitHi', 'max']].forEach(([k, lbl]) => {
      if (o[k] === undefined) return;
      const y = Math.round(sy(o[k])) + 0.5;
      ctx.setLineDash([3, 3]); ctx.strokeStyle = g('--bad'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x - 4, y); ctx.lineTo(x + w + 4, y); ctx.stroke(); ctx.setLineDash([]);
      if (side === 'none') return;
      ctx.fillStyle = g('--bad'); ctx.textAlign = side === 'left' ? 'right' : 'left'; ctx.textBaseline = 'middle';
      ctx.fillText(lbl, side === 'left' ? x - 8 : x + w + 8, y);
    });
    /* saturation hint */
    const clippedHi = o.limitHi !== undefined && total > o.limitHi + 1e-9;
    const clippedLo = o.limitLo !== undefined && total < o.limitLo - 1e-9;
    if (clippedHi || clippedLo) { ctx.fillStyle = g('--bad'); ctx.textAlign = 'center'; ctx.textBaseline = 'bottom'; ctx.fillText('clipped', x + w / 2, y0 - 4); }
    else if (o.title) { ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'center'; ctx.textBaseline = 'bottom'; ctx.fillText(o.title, x + w / 2, y0 - 4); }
    /* zero line */
    const yz = Math.round(sy(0)) + 0.5;
    if (yz > y0 && yz < y0 + h) { ctx.strokeStyle = g('--line-2'); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, yz); ctx.lineTo(x + w, yz); ctx.stroke(); }
    /* value */
    ctx.fillStyle = g('--ink-2'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillText(GTPL.fmt(total, 0) + (o.unit ? ' ' + o.unit : ''), x + w / 2, y0 + h + 4);
    ctx.restore();
    return { sy, total };
  };

  /* tiny legend chips in DOM, for below-canvas legends */
  GTPL.legend = function (items) {
    return el('div', { class: 'legend' }, items.map(it => el('span', { class: 'legend-item' }, [
      el('span', { class: 'legend-swatch', style: 'background: var(' + it.color + ')' + (it.dash ? '; background: repeating-linear-gradient(90deg, var(' + it.color + ') 0 4px, transparent 4px 7px)' : '') }),
      it.label,
    ])));
  };

  /* ---------- instrument scaffold: stage + controls + hud ---------- */
  GTPL.scaffold = function (rootEl, o) {
    o = o || {};
    const stage = el('div', { class: 'stage' });
    const controls = el('div', { class: 'controls' });
    const hud = el('div', { class: 'hud' });
    const foot = el('div', { class: 'inst-foot' });
    rootEl.appendChild(el('div', { class: 'inst-grid' + (o.wide ? ' inst-wide' : '') }, [el('div', { class: 'inst-main' }, [stage, hud]), controls]));
    rootEl.appendChild(foot);
    return { stage, controls, hud, foot };
  };
  GTPL.group = function (title, children) {
    return el('div', { class: 'ctl-group' }, [title ? el('div', { class: 'ctl-title', text: title }) : null].concat(children));
  };

  /* ---------- mount all sims ---------- */
  GTPL.mountAll = function () {
    document.querySelectorAll('.instrument[data-sim]').forEach(node => {
      const name = node.dataset.sim;
      const fn = GTPL.sims[name];
      if (!fn) { console.warn('no sim registered for', name); return; }
      try { GTPL.instances.push({ name, api: fn(node, {}) }); }
      catch (e) { console.error('sim failed to mount', name, e); node.appendChild(el('p', { class: 'inst-error', text: 'This simulation failed to load. Reload the page or open the console for details.' })); }
    });
  };

  window.GTPL = GTPL;
})();
