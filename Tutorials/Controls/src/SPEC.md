# GTPL Controls Guide: shared design system and simulation-module contract

Everything in the page is one `index.html`, no build step, no external JS. The only external
resources allowed are Google Fonts (CSS from fonts.googleapis.com, files from fonts.gstatic.com).
All CSS and JS are inline. Simulations are hand-written canvas code against the helpers below.

## 1. Design plan

Subject: the GTPL Monarch lander (1000 N hybrid, N2O/paraffin, throttle valve + gimbal). Audience:
freshmen joining the GNC subteam. Page job: get them to *feel* control (hand-fly, then P, I, D, FF,
then LQR, then MPC), then send them to the repo.

Treatment: editorial explainer, instrument-panel flavor. Long-form prose column with inline
"instruments" (interactive simulations) that are wider than the prose. Dark theme reads like a
flight display; light theme is the GTPL site's cream. Gold is spent only on the thing that matters
in each view (the live signal, the active control, the one highlighted word).

### Color tokens

Light (bare `:root`, the GTPL site cream):
```
--bg:        #F0ECE2   /* GTPL site body */
--bg-2:      #E7E1D3   /* instrument panel ground on light */
--bg-3:      #DCD5C4   /* inset / track */
--ink:       #15161A   /* text */
--ink-2:     #4A4C55   /* secondary text */
--ink-3:     #7B7E8A   /* captions, ticks */
--line:      #C9C1AE   /* rules, plot grid */
--line-2:    #A79F8C   /* axes */
--gold:      #FFD600   /* GTPL --gt-gold: fills, live marks only, never text on cream */
--gold-ink:  #7A6300   /* gold that passes contrast as text on cream */
--gold-soft: rgba(255,214,0,.18)
--navy:      #1E3A5F   /* GTPL navy: setpoint/reference, links */
--sig-ref:   #1E3A5F   /* reference / setpoint series */
--sig-act:   #B8860B   /* actual / measured series (dark gold on cream) */
--sig-ff:    #2E7DA6   /* feedforward slice */
--sig-fb:    #C2452D   /* feedback slice */
--sig-p:     #C2452D   /* P term */
--sig-i:     #7A4EAB   /* I term */
--sig-d:     #2E7DA6   /* D term */
--ok:        #2F7D4F
--warn:      #B8860B
--bad:       #B2321F
--focus:     #1E3A5F
```
Dark (`@media (prefers-color-scheme: dark)` guarded `:root:not([data-theme="light"])`, and again
`:root[data-theme="dark"]`):
```
--bg:        #0E0F12   /* warm near-black, GTPL nav is rgba(10,10,10) */
--bg-2:      #15171C
--bg-3:      #1E2128
--ink:       #ECE8DD   /* warm off-white, not pure white */
--ink-2:     #B4B0A6
--ink-3:     #7E7B74
--line:      #2A2D35
--line-2:    #3D414B
--gold:      #FFD600
--gold-ink:  #FFD600   /* gold IS readable as text on near-black */
--gold-soft: rgba(255,214,0,.14)
--navy:      #6F9BD1   /* navy lifted so it reads on dark */
--sig-ref:   #C8CDD6   /* reference: light grey dashed on dark */
--sig-act:   #FFD600   /* actual: gold, the live signal */
--sig-ff:    #7FB8E6
--sig-fb:    #F0836B
--sig-p:     #F0836B
--sig-i:     #B79CE8
--sig-d:     #7FB8E6
--ok:        #6FCF97
--warn:      #FFD600
--bad:       #F26D5B
--focus:     #FFD600
```
`body { background: var(--bg); color: var(--ink); }` is mandatory. No color literal may appear
outside the token blocks; canvases read tokens via `GTPL.theme.get('--sig-act')` at draw time.

### Type

Google Fonts link (one tag, in `<head>`):
`https://fonts.googleapis.com/css2?family=B612:wght@400;700&family=B612+Mono:wght@400;700&family=IBM+Plex+Sans:ital,wght@0,400;0,500;1,400&family=Montserrat:wght@600;700;800&display=swap`

```
--font-display: "Montserrat", "Helvetica Neue", Arial, sans-serif;   /* GTPL brand face: h1, h2, h3 only */
--font-body:    "IBM Plex Sans", "Segoe UI", Roboto, system-ui, sans-serif;  /* prose */
--font-ui:      "B612", Verdana, sans-serif;                            /* Airbus cockpit face: anything the machine says: slider labels, buttons, HUD labels, axis titles, captions */
--font-mono:    "B612 Mono", "SFMono-Regular", Consolas, monospace;   /* every number that changes, ticks, eyebrows, code */
```
Rule: a font never leaves its role. Prose is Plex; instrument chrome is B612; changing numbers are B612 Mono.
Scale (rem): 0.75 (mono eyebrow / ticks), 0.875 (captions, UI), 1.0625 body (17px), 1.25 lead,
1.5 h3, 2.0 h2, 3.0 h1 (clamp to 2.25 on mobile). Body line-height 1.6, headings 1.1,
`text-wrap: balance` on headings. Prose measure `max-width: 65ch`. Eyebrows: mono, uppercase,
`letter-spacing: .08em`, color `--ink-3`. Numbers in readouts: `font-variant-numeric: tabular-nums`.

### Layout

Single column, left-aligned, not centered. Prose column 65ch, offset left of center on wide screens
(`margin-left: max(1.5rem, calc((100vw - 1100px)/2))`). Instruments break out to 1040px wide
(`.instrument { width: min(1040px, 100% - 3rem) }`). A thin left rail (2px, `--line`) runs beside
the prose in the LQR/MPC chapters only, with chapter numbers in mono, because those chapters are the
"advanced" arc and the rail marks where the difficulty steps up (structure encodes information).
Section spacing 6rem between chapters, 2.5rem inside. Sticky top bar: butterfly mark, "GTPL / GNC",
chapter links in mono, theme toggle. No hero image; the hero IS the first instrument (hand-fly).

Instruments: one flat panel `background: var(--bg-2)`, 1px `--line` border, radius 6px (only
instruments get a radius; text blocks never do). Inside: left canvas stage, right controls column
(stacks under on <760px). Controls: sliders with mono readouts, small toggle buttons, a Reset
button. A "Try this" block sits directly under each instrument as prose with a gold 3px left border
(the only accent-bar in the page, reserved for "do this now"). Captions under canvases in
`--ink-3`, 0.875rem.

### Motion

Sims run at fixed physics step, render on rAF, pause when scrolled out of view (IntersectionObserver)
and when `document.hidden`. Under `prefers-reduced-motion: reduce`: no autoplay; sims start paused
with a Play button and no decorative animation. No entrance animations for text. Hover states are
a 1px border change, not a lift.

## 2. Page skeleton (ids the copy uses)

```
header.topbar
main
  section#hero        .prose + .instrument[data-sim="handfly"]
  section#why-control .prose (+ svg block diagram: open vs closed loop)
  section#open-loop   .prose + .instrument[data-sim="openloop"]
  section#closed-loop .prose + .instrument[data-sim="pid"]
  section#videos      .prose + .videos (link cards with youtube thumbnails as <a>, NO iframes by default; a click loads the iframe)
  section#combine     .prose + .instrument[data-sim="ffb"]
  section#lqr         .prose + .instrument[data-sim="lqr"]
  section#mpc         .prose + .instrument[data-sim="mpc"]
  section#fits        .prose + svg GNC loop diagram (animated dots along arrows, respects reduced motion)
  section#exercises   .prose + exercise cards
  section#further     .prose list
footer
```

## 3. Shared JS helpers (`window.GTPL`), written once in the page before any sim

```js
GTPL.theme.get(varName)         // resolved CSS custom property string from documentElement, cached per frame
GTPL.theme.onChange(fn)          // called when data-theme changes or prefers-color-scheme flips
GTPL.rng(seed)                   // -> function returning uniform [0,1); mulberry32
GTPL.clamp(v, lo, hi)
GTPL.lerp(a, b, t)
GTPL.fmt(v, digits)              // number -> string with fixed digits, minus sign is U+2212
GTPL.el(tag, attrs, children)    // tiny DOM builder; attrs.class, attrs.text, attrs.html, event handlers as on*
GTPL.slider({label, unit, min, max, step, value, digits, onInput}) -> {root, get(), set(v), setLabel(s)}
   // renders: <label class="ctl"><span class="ctl-label">Kp</span><input type=range><output class="ctl-val mono">1.20</output></label>
GTPL.toggle({label, value, onChange}) -> {root, get(), set(v)}          // a button with aria-pressed
GTPL.button({label, onClick, kind})  -> {root}                         // kind: 'primary'|'ghost'
GTPL.segmented({options:[{label,value}], value, onChange}) -> {root, get(), set(v)}
GTPL.readout({label, unit, digits}) -> {root, set(v)}                  // mono big number tile for HUD
GTPL.canvas(parent, {aspect})  -> {canvas, ctx, width, height, onResize(fn)}  // DPR-aware, ResizeObserver, CSS size from parent
GTPL.loop({step, render, dt, maxSubsteps, root}) -> {start(), stop(), running, reset()}
   // fixed-step accumulator; step(dt) called 0..maxSubsteps times per frame; render(alpha) once;
   // auto-pauses when root not intersecting viewport or document.hidden; honors reduced motion (starts paused)
GTPL.StripChart(ctx-owner canvas, {duration, ymin, ymax, autoscale, series:[{key, color:'--sig-act', dash?, width?, label}], yLabel, xLabel, thresholds:[{y, color, label}]})
   .push(t, {key: value, ...})   // ring buffer by time window
   .draw(rect)                   // draws grid, axes, series, legend inside rect {x,y,w,h}; colors via GTPL.theme.get
   .clear()
GTPL.drawRocket(ctx, {x, y, scale, tilt, gimbal, thrust01, flameSeed})
   // rocket glyph: slender body, 4 legs, nozzle; flame length ∝ thrust01 with slight flicker (rng), rotated by tilt, nozzle rotated by gimbal about the base
GTPL.drawGround(ctx, rect, {y0, ticks})           // horizon line + altitude ticks in mono
GTPL.drawSetpoint(ctx, rect, {y, label})          // dashed reference line
GTPL.drawBarStack(ctx, rect, {segments:[{value, color, label}], min, max, zeroLine})  // stacked thrust bar (FF/P/I/D slices), shows saturation
```

Coordinate convention for stages: world z (altitude, m) maps to canvas y with a fixed meters-per-pixel chosen per sim; camera does not scroll except in MPC (follows rocket vertically).

## 4. Simulation module contract

Each sim is one function registered on `GTPL.sims`:

```js
GTPL.sims.pid = function mount(root, opts) {
  // root: the <div class="instrument" data-sim="pid"> element, already in the DOM, empty.
  // Build DOM with GTPL.el into root:  <div class="stage"> (canvas)  <div class="controls">  <div class="hud"> ...
  // Return { reset(), destroy(), setPreset?(name) }
}
```
Rules for every module:
- No globals other than the returned object; wrap in an IIFE. No external libraries. No `Date.now()` for physics (use accumulated sim time; rAF timestamps for real time only).
- Physics from the verified spec (models A and B); do not invent new constants without noting them.
- Every number the user can change has a slider with unit and mono readout. Every plot has axis labels with units and a dashed reference where one exists. Legend text uses `--ink-2`.
- Reset button restores defaults and clears charts. Presets (if any) are a segmented control.
- Colors only via `GTPL.theme.get('--token')`. Re-read on every draw (cheap; cached per frame by the helper).
- Keyboard: sliders are native `<input type=range>` so they already work; buttons are `<button>`; canvas is `aria-hidden` with an adjacent visually-hidden live description if useful.
- Mobile: stage and controls stack; canvas min height 260px; text never clipped.
- Performance: whole page must stay at 60 fps with all sims mounted but only visible ones running.
- The module file exports nothing else; it is pasted into the page inside `<script>` after helpers.

Per-sim briefs (physics parameters come from the verified spec; UI here):

- handfly (hero): stage shows rocket + ground + altitude ticks, setpoint dashed at 10 m. A vertical
  throttle slider (0..100% thrust) the user drags (pointer or arrow keys). HUD: altitude, vertical
  speed, "time on target" score, thrust %. Strip chart of altitude with the setpoint. Button:
  "Let the computer fly" toggles a PD autopilot so the reader sees the contrast. Reset.
- openloop: same stage. Controls: model error slider (mass estimate off by -10..+10%), "Gust" button
  (applies a 2 s downward force), profile selector (Hover at 10 m | Climb 0->10 m). The controller
  is pure feedforward from the plan. Chart: planned vs actual altitude. HUD: error at t.
- pid: stage + sliders Kp, Ki, Kd, toggle "Feedforward (gravity)", toggle "Sensor noise", setpoint
  segmented (5 m | 10 m | 15 m), Gust button, presets (Sluggish | Tuned | Too aggressive). Two
  charts: altitude vs setpoint; thrust command with saturation limits drawn. Stacked bar of the
  thrust command split into FF / P / I / D slices, live. HUD: error, overshoot %, settle time.
- ffb: moving setpoint (a smooth climb profile). Segmented: Feedback only | Feedforward only |
  Both. Slider: mass error %. Chart: reference vs actual; second chart: FF and FB slices of thrust
  over time. HUD: RMS tracking error for the current run (resets when mode changes).
- lqr: planar rocket stage (x to the right, z up), start offset 5 m left of target at 10 m up.
  Sliders: "Care about position" (Q pos), "Care about tilt" (Q theta), "Gimbal effort" (R delta),
  "Thrust effort" (R thrust). Live K heatmap (2x6 grid, cells labeled with state names and
  gains, color intensity by |k|). Chart: x, z, theta over time. Toggle "Show what LQR asks for
  without limits" (draw the unclipped gimbal command vs the 10 deg limit). Reset, "Nudge" button
  that applies a lateral impulse.
- mpc: planar stage with camera following altitude, glide-slope cone drawn from the landing pad,
  input limits shown as small gauges (gimbal +/-10 deg, thrust min/max). Segmented: LQR (clipped)
  | MPC. Slider: horizon N (5..30). Toggle: "Glide slope constraint". Button: Start descent
  (scenario from spec). Draw the predicted horizon as a fading fan of the planned trajectory
  every step. Charts: gimbal command vs limit; thrust vs limits. HUD: solve time (ms), constraint
  violations count, touchdown speed.

## 5. Voice for any text inside instruments
Labels are plain: "Kp", "Ki", "Kd", "Feedforward", "Sensor noise", "Gust", "Reset". No exclamation
marks, no emoji, no "Awesome". Units always shown (m, m/s, N, %, deg, s, ms).
