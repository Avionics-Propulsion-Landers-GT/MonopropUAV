/* openloop: pure feedforward on model A (1 kg hover rocket). The controller never reads a sensor.
   Hover profile commands thrust = m_est * g; climb profile plays back m_est * (g + a_ref(t)) from a
   minimum-jerk 0 -> 10 m climb over 5 s. The mass error slider changes the real rocket only. */
(function () {
  'use strict';
  const G = window.GTPL;

  /* ---------------- pure math: no DOM ---------------- */
  const params = {
    m: 1.0,          // kg, the mass the controller believes (model)
    g: 9.81,         // m/s^2
    Tmin: 1.0,       // N, flameout floor
    Tmax: 20.0,      // N
    tau: 0.20,       // s, first-order thrust lag
    dt: 1 / 240,     // s, physics step; controller runs every step
    zTarget: 10.0,   // m
    tClimb: 5.0,     // s, climb duration
    gustN: 2.0,      // N, downward gust force
    gustT: 2.0,      // s, gust duration
    massErrPct: 10,  // slider range +/- %
  };

  /* minimum-jerk rest-to-rest move z0 -> z1 over T seconds; returns ref(t) -> {z, v, a, j} */
  function minJerk(z0, z1, T) {
    return function (t) {
      if (t <= 0) return { z: z0, v: 0, a: 0, j: 0 };
      if (t >= T) return { z: z1, v: 0, a: 0, j: 0 };
      const s = t / T, d = z1 - z0;
      const s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
      return {
        z: z0 + d * (10 * s3 - 15 * s4 + 6 * s5),
        v: d * (30 * s2 - 60 * s3 + 30 * s4) / T,
        a: d * (60 * s - 180 * s2 + 120 * s3) / (T * T),
        j: d * (60 - 360 * s + 360 * s2) / (T * T * T),
      };
    };
  }

  const hoverRef = { z: params.zTarget, v: 0, a: 0, j: 0 };
  const profiles = {
    hover: { label: 'Hover at 10 m', z0: params.zTarget, ref: function () { return hoverRef; } },
    climb: { label: 'Climb 0 to 10 m', z0: 0, ref: minJerk(0, params.zTarget, params.tClimb) },
  };
  const defaultProfile = 'climb';

  /* feedforward thrust: the plan and the model, nothing else. Returns the unclipped command. */
  function feedforward(profile, t, p) {
    const r = profiles[profile].ref(t);
    return p.m * (p.g + r.a);
  }

  /* massErr is a fraction: +0.05 means the real rocket is 5 % heavier than the model believes */
  function makeState(profile, massErr, p) {
    return { z: profiles[profile].z0, v: 0, T: p.m * p.g, m: p.m * (1 + massErr), t: 0 };
  }

  /* semi-implicit Euler, exact first-order lag for a held command. dist: extra vertical force [N]. */
  function stepModel(s, Tcmd, dt, p, dist) {
    const Tc = G.clamp(Tcmd, p.Tmin, p.Tmax);
    s.T += (Tc - s.T) * (1 - Math.exp(-dt / p.tau));
    const a = (s.T - s.m * p.g + (dist || 0)) / s.m;
    s.v += a * dt;
    s.z += s.v * dt;
    if (s.z <= 0) { s.z = 0; if (s.v < 0) s.v = 0; }
    s.t += dt;
    return s;
  }

  /* offline run for tests: opts { gustAt: seconds | undefined, logEvery: steps } -> { s, log, at(t) } */
  function run(profile, massErr, tEnd, opts, p) {
    opts = opts || {}; p = p || params;
    const s = makeState(profile, massErr, p);
    const steps = Math.round(tEnd / p.dt);
    const every = opts.logEvery || 24;
    const log = [];
    for (let i = 0; i <= steps; i++) {
      const Tcmd = feedforward(profile, s.t, p);
      const gust = opts.gustAt !== undefined && s.t >= opts.gustAt && s.t < opts.gustAt + p.gustT;
      if (i % every === 0) log.push({ t: s.t, z: s.z, v: s.v, T: s.T, Tcmd, zPlan: profiles[profile].ref(s.t).z, gust });
      if (i === steps) break;
      stepModel(s, Tcmd, p.dt, p, gust ? -p.gustN : 0);
    }
    function at(t) {
      let best = log[0];
      for (let i = 1; i < log.length; i++) if (Math.abs(log[i].t - t) < Math.abs(best.t - t)) best = log[i];
      return best;
    }
    return { s, log, at };
  }

  const math = { params, minJerk, profiles, defaultProfile, feedforward, makeState, stepModel, run };
  G.math = G.math || {};
  G.math.openloop = math;

  /* ---------------- mount: all DOM / canvas work ---------------- */
  G.sims.openloop = function mount(root) {
    const p = params;
    const ui = G.scaffold(root);
    const stage = G.canvas(ui.stage, { aspect: 2.2, minHeight: 260 });
    const chart = new G.StripChart({
      duration: 12, autoscale: true, floor: 0, ymin: 0, ymax: 12,
      yLabel: 'altitude (m)', xLabel: 't (s)',
      series: [
        { key: 'plan', color: '--sig-ref', dash: [6, 5], label: 'plan' },
        { key: 'z', color: '--sig-act', label: 'altitude' },
      ],
      /* also anchors the autoscale so the first frames of a climb do not zoom into a 1e-5 m range */
      thresholds: [{ y: p.zTarget, color: '--ink-3', dash: [2, 5], label: 'target 10 m' }],
    });
    const rand = G.rng(11);

    let profile = defaultProfile;
    let massErr = 0;            // fraction
    let s = makeState(profile, massErr, p);
    let flying = false;
    let gustLeft = 0;
    let Tcmd = p.m * p.g;
    let pushCount = 0;

    /* controls */
    const seg = G.segmented({
      label: 'Profile',
      options: [{ label: profiles.hover.label, value: 'hover' }, { label: profiles.climb.label, value: 'climb' }],
      value: profile,
      onChange: function (v) { profile = v; restart(); },
    });
    const sMass = G.slider({
      label: 'Mass estimate error', unit: '%', min: -p.massErrPct, max: p.massErrPct, step: 1, value: 0,
      format: function (v) { return (v > 0 ? '+' : '') + G.fmt(v, 0); },
      onInput: function (v) { massErr = v / 100; s.m = p.m * (1 + massErr); },
    });
    const bPlay = G.button({ label: 'Play', kind: 'primary', onClick: function () { flying = true; restart(); loop.start(); } });
    const bGust = G.button({ label: 'Gust', onClick: function () { if (flying) gustLeft = p.gustT; } });
    const bReset = G.button({ label: 'Reset', kind: 'ghost', onClick: reset });
    ui.controls.append(
      G.group('Profile', [seg.root]),
      G.group('Real rocket', [sMass.root]),
      G.el('div', { class: 'btn-row' }, [bPlay.root, bGust.root, bReset.root])
    );

    /* HUD */
    const roAlt = G.readout({ label: 'Altitude', unit: 'm' });
    const roErr = G.readout({ label: 'Error', unit: 'm' });
    const roThr = G.readout({ label: 'Thrust', unit: 'N', digits: 1 });
    const roTime = G.readout({ label: 'Time', unit: 's', digits: 1 });
    ui.hud.append(roAlt.root, roErr.root, roThr.root, roTime.root);

    ui.foot.append(
      G.el('p', { class: 'caption', text: 'Toy rocket: 1 kg, hover thrust 9.81 N, thrust limited to 1 to 20 N with a 0.2 s lag. The gust is 2 N down for 2 s.' }),
      G.legend([{ label: 'plan', color: '--sig-ref', dash: true }, { label: 'altitude', color: '--sig-act' }])
    );
    root.appendChild(G.hidden('The stage shows the rocket above the ground with altitude ticks and a dashed line at the planned altitude; the chart on the right plots planned and actual altitude against time.'));

    function restart() {
      s = makeState(profile, massErr, p);
      Tcmd = p.m * p.g;
      gustLeft = 0;
      pushCount = 0;
      chart.clear();
      bPlay.setLabel(flying ? 'Restart' : 'Play');
    }
    function reset() {
      flying = false;
      profile = defaultProfile;
      seg.set(profile, true);
      massErr = 0;
      sMass.set(0, true);
      restart();
    }

    function step(dt) {
      if (!flying) return;
      Tcmd = feedforward(profile, s.t, p);
      const dist = gustLeft > 0 ? -p.gustN : 0;
      if (gustLeft > 0) gustLeft -= dt;
      stepModel(s, Tcmd, dt, p, dist);
      if (!isFinite(s.z) || !isFinite(s.v) || !isFinite(s.T)) { restart(); return; }
      pushCount++;
      if (pushCount % 4 === 0) chart.push(s.t, { plan: profiles[profile].ref(s.t).z, z: s.z });
    }

    const stageRect = { x: 0, y: 0, w: 0, h: 0 };
    const chartRect = { x: 0, y: 0, w: 0, h: 0 };
    const ticks = [0, 5, 10, 15, 20];

    function render() {
      const ctx = stage.ctx, W = stage.width, H = stage.height, g = G.theme.get;
      stage.clear();
      stageRect.w = Math.round(W * 0.38); stageRect.h = H;
      chartRect.x = stageRect.w + 6; chartRect.y = 4; chartRect.w = W - stageRect.w - 10; chartRect.h = H - 8;

      const y0 = H - 26;
      const mpp = 22 / (H - 60);            // 22 m of sky above the ground line
      const zPlan = profiles[profile].ref(s.t).z;
      G.drawGround(ctx, stageRect, { y0, mpp, ticks });
      G.drawSetpoint(ctx, stageRect, { y: y0 - zPlan / mpp, label: 'plan ' + G.fmt(zPlan, 1) + ' m' });

      const scale = 60;
      const topLimit = scale * 1.1 + 6;
      let ry = y0 - s.z / mpp;
      let above = false;
      if (!(ry >= topLimit)) { ry = topLimit; above = true; }
      const rx = stageRect.w * 0.58;
      const thrust01 = G.clamp((s.T - p.Tmin) / (p.Tmax - p.Tmin), 0, 1);
      G.drawRocket(ctx, { x: rx, y: ry, scale, thrust01, tilt: 0, gimbal: 0, rand });

      ctx.save();
      ctx.font = '11px ' + g('--font-mono');
      ctx.textBaseline = 'middle';
      if (above) {
        ctx.fillStyle = g('--ink-3'); ctx.textAlign = 'left';
        ctx.fillText('above view', rx + scale * 0.5, ry - scale * 0.5);
      }
      if (gustLeft > 0) {
        ctx.strokeStyle = g('--sig-fb'); ctx.fillStyle = g('--sig-fb'); ctx.lineWidth = 1.5;
        for (let i = -1; i <= 1; i++) {
          const ax = rx + i * scale * 0.42, ay = ry - scale * 1.55;
          ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax, ay + scale * 0.3); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(ax - 4, ay + scale * 0.3 - 5); ctx.lineTo(ax, ay + scale * 0.3); ctx.lineTo(ax + 4, ay + scale * 0.3 - 5); ctx.stroke();
        }
        ctx.textAlign = 'center';
        ctx.fillText('gust 2 N', rx, ry - scale * 1.55 - 9);
      }
      if (!flying) {
        ctx.fillStyle = g('--ink-2'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ctx.fillText(stageRect.w < 330 ? 'Press Play' : 'Press Play to fly the plan', stageRect.w / 2, 10);
      } else if (s.z <= 0 && s.t > 1) {
        ctx.fillStyle = g('--bad'); ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ctx.fillText('on the ground', stageRect.w / 2, 10);
      }
      ctx.restore();

      chart.draw(ctx, chartRect);

      const err = s.z - zPlan;
      const ae = Math.abs(err);
      roAlt.set(s.z);
      roErr.set(err, ae < 0.25 ? 'good' : ae < 1 ? 'warn' : 'bad');
      roThr.set(s.T);
      roTime.set(s.t);
    }

    const loop = G.loop({ step, render, dt: p.dt, root });
    loop.renderOnce();

    return {
      reset,
      renderOnce: loop.renderOnce,
      destroy: function () { loop.stop(); loop.destroy(); stage.destroy(); },
    };
  };
})();
