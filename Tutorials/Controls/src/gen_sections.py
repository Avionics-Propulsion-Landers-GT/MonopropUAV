import json, html, re, os

BUILD = os.path.dirname(os.path.abspath(__file__))
copy = json.load(open(os.path.join(BUILD, "copy.json"), encoding="utf-8"))
diagrams = open(os.path.join(BUILD, "diagrams.html"), encoding="utf-8").read()

S = {s["id"]: s for s in copy["sections"]}
esc = lambda t: html.escape(t, quote=False)

def fig(fid, caption=None):
    m = re.search(r'<figure class="diagram" id="%s">.*?</figure>' % fid, diagrams, re.S)
    f = m.group(0)
    if caption is not None:
        f = re.sub(r"<figcaption>.*?</figcaption>", "<figcaption>%s</figcaption>" % esc(caption), f, flags=re.S)
    return f

def paras(ps):
    return "\n".join("<p>%s</p>" % esc(p) for p in ps)

def instrument(name, captions):
    out = ['<div class="instrument" data-sim="%s"></div>' % name]
    for c in captions:
        out.append('<p class="caption">%s</p>' % esc(c))
    return "\n".join(out)

def try_block(items):
    lis = "\n".join("  <li>%s</li>" % esc(i) for i in items)
    return '<div class="try"><span class="eyebrow">Try this</span>\n<ol>\n%s\n</ol></div>' % lis

def check(cp):
    return ('<details class="check"><summary><span class="q">%s</span></summary>'
            '<div class="a"><p>%s</p></div></details>' % (esc(cp["q"]), esc(cp["a"])))

def head(n, label, heading):
    num = '<span class="n">%s</span>' % n if n else ""
    return ('<div class="chapter-head"><span class="eyebrow">%s%s</span>\n<h2>%s</h2></div>'
            % (num, esc(label), esc(heading)))

def repo_note(inner_html):
    return '<aside class="repo-note"><span class="eyebrow">In the repo</span>%s</aside>' % inner_html

parts = []
A = parts.append

# ---------------- topbar ----------------
A('''<header class="topbar"><div class="wrap topbar-in">
  <a class="brand" href="#hero"><img class="mark" src="ASSET_MARK" alt=""><span>Propulsive Landers <b>/ GNC</b></span></a>
  <nav class="topnav" aria-label="Chapters">
    <a href="#why-control">Why</a>
    <a href="#open-loop">Feedforward</a>
    <a href="#closed-loop">Feedback</a>
    <a href="#combine">Both</a>
    <a href="#lqr">LQR</a>
    <a href="#mpc">MPC</a>
    <a href="#exercises">Exercises</a>
  </nav>
  <button class="theme-btn" id="themeBtn" type="button">Theme: auto</button>
</div></header>''')

A('<main class="wrap">')

# ---------------- hero ----------------
h = copy["hero"]
wc = S["why-control"]
A('<section class="hero" id="hero">\n<div class="prose">')
A('<span class="eyebrow">%s</span>' % esc(h["subtitle"]))
A('<h1>%s</h1>' % esc(h["title"]))
lede_parts = [x.strip() for x in h["lede"].split("\n\n") if x.strip()]
A('<p class="lead">%s</p>' % esc(lede_parts[0]))
for _part in lede_parts[1:]:
    A('<p>%s</p>' % esc(_part))
if h.get('byline'):
    A('<p class="hero-meta">%s</p>' % esc(h['byline']))
A('<p>%s</p>' % esc(h["hook"]))
A('</div>')
A(instrument("handfly", [wc["captions"]["handHover"]]))
A('</section>')

# ---------------- 01 why-control ----------------
A('<section class="chapter" id="why-control">')
A(head("01", "Why control", wc["heading"]))
A('<div class="prose">')
A(paras(wc["paragraphs"][:4]))
A('</div>')
A(fig("fig-loops"))
A('<div class="prose">')
A(paras(wc["paragraphs"][4:]))
A('</div>')
A(try_block(wc["tryThis"]))
A(check(wc["checkpoint"]))
A('</section>')

# ---------------- 02 open-loop ----------------
s = S["open-loop"]
A('<section class="chapter" id="open-loop">')
A(head("02", "Feedforward", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"][:2]))
A('</div>')
A(instrument("openloop", [s["captions"]["feedforward"]]))
A(try_block(s["tryThis"]))
A('<div class="prose">')
A(paras(s["paragraphs"][2:]))
A('</div>')
A(check(s["checkpoint"]))
A('</section>')

# ---------------- 03 closed-loop ----------------
s = S["closed-loop"]
A('<section class="chapter" id="closed-loop">')
A(head("03", "Feedback", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"][:1]))
A('</div>')
A(instrument("pid", [s["captions"]["pid"]]))
A('<div class="prose">')
A(paras(s["paragraphs"][1:]))
A('</div>')
A(try_block(s["tryThis"]))
A(check(s["checkpoint"]))
A('</section>')

# ---------------- 04 videos ----------------
s = S["videos"]
videos = [
    ("4:40", "Controlling Self Driving Cars", "AerospaceControlsLab", "https://www.youtube.com/watch?v=4Y7zG48uHRo",
     "A car steering onto a lane line. P alone oscillates, PD damps it, PID removes the offset. No math."),
    ("7:44", "PID Control - A brief introduction", "Brian Douglas", "https://www.youtube.com/watch?v=UR0hOmjaHp0",
     "Hand-drawn intuition for what each of the three terms is for."),
    ("15:44", "What Is Feedforward Control? | Control Systems in Practice", "MATLAB", "https://www.youtube.com/watch?v=FW_ay7K4jPE",
     "Anticipation versus correction. Feedforward takes the predictable part so feedback only cleans up model error. Sets up the next section."),
    ("13:03", "Linear Quadratic Regulator (LQR) Control for the Inverted Pendulum on a Cart [Control Bootcamp]", "Steve Brunton", "https://www.youtube.com/watch?v=1_UobILf3cc",
     "Balancing a pole while moving the cart is tilting to translate, the closest textbook stand-in for a rocket."),
    ("4:50", "Why Use Model Predictive Control? | Understanding MPC, Part 1", "MATLAB", "https://www.youtube.com/watch?v=8U0xiOkDcmw",
     "Many inputs, hard limits, and looking ahead: the three reasons Monarch wants MPC."),
    ("1:36", "Grasshopper 744m Test | Single Camera (Hexacopter)", "SpaceX", "https://www.youtube.com/watch?v=9ZDkItO-0a4",
     "A real hop with a sideways move. Watch the gimbal tilt the vehicle before it translates."),
]
A('<section class="chapter" id="videos">')
A(head("04", "Videos", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"]))
A('</div>')
A('<ul class="videos">')
for dur, title, chan, url, why in videos:
    A('  <li><a class="vid" href="%s" target="_blank" rel="noopener"><span class="dur">%s</span>'
      '<span class="t">%s<span class="m">%s</span></span><span class="why">%s</span></a></li>'
      % (url, dur, esc(title), esc(chan), esc(why)))
A('</ul>')
A('</section>')

# ---------------- 05 combine ----------------
s = S["combine"]
A('<section class="chapter" id="combine">')
A(head("05", "Feedforward and feedback", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"][:1]))
A('</div>')
A(instrument("ffb", [s["captions"]["ffPlusFb"]]))
A(try_block(s["tryThis"]))
A('<div class="prose">')
A(paras(s["paragraphs"][1:]))
A('</div>')
A(check(s["checkpoint"]))
A('</section>')

# ---------------- 06 lqr ----------------
s = S["lqr"]
A('<section class="chapter advanced" id="lqr">')
A(head("06", "LQR", s["heading"]))
if s.get('note'):
    A('<p class="chapter-note">%s</p>' % esc(s['note']))
A('<div class="prose">')
A(paras(s["paragraphs"][:4]))
A('</div>')
A(instrument("lqr", []))
A(try_block(s["tryThis"]))
A('<div class="prose">')
A(paras(s["paragraphs"][4:]))
A('</div>')
A(check(s["checkpoint"]))
A('</section>')

# ---------------- 07 mpc ----------------
s = S["mpc"]
P = s["paragraphs"]
# paragraphs 4..9 (0-based) are the numbered function walk-through; render as an <ol>
first = next(i for i, x in enumerate(P) if re.match(r"^1\.\s", x))
last = first
while last + 1 < len(P) and re.match(r"^\d+\.\s", P[last + 1]): last += 1
steps = [re.sub(r"^\d+\.\s+", "", x) for x in P[first:last + 1]]
A('<section class="chapter advanced" id="mpc">')
A(head("07", "MPC", s["heading"]))
if s.get('note'):
    A('<p class="chapter-note">%s</p>' % esc(s['note']))
A('<div class="prose">')
A(paras(P[:first - 1]))
A('</div>')
A(instrument("mpc", []))
A(try_block(s["tryThis"]))
A('<div class="prose">')
A(paras(P[first - 1:first]))
A('<ol>\n%s\n</ol>' % "\n".join("  <li>%s</li>" % esc(x) for x in steps))
A(paras(P[last + 1:]))
A('</div>')
A(check(s["checkpoint"]))
A('</section>')

# ---------------- 08 fits ----------------
s = S["fits"]
A('<section class="chapter" id="fits">')
A(head("08", "GNC", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"]))
A('</div>')
A(fig("fig-gnc", s["captions"]["gncLoop"]))
A(try_block(s["tryThis"]))
A('''<figure class="monarch">
  <img src="ASSET_MONARCH" alt="Render of Monarch, GTPL's 1000 N hybrid lander: three tanks around a central column, a gimbaled engine below, four legs">
  <ol class="callouts">
    <li><span class="n">01</span><span><b>Throttle valve</b>on the oxidizer line. One servo. This is the thrust command.</span></li>
    <li><span class="n">02</span><span><b>Gimbaled engine</b>two servos. This is the pair of gimbal angle commands.</span></li>
    <li><span class="n">03</span><span><b>Three tanks</b>mass changes as they drain, so hover thrust changes during the burn.</span></li>
    <li><span class="n">04</span><span><b>Avionics</b>with the IMU up top. Where the state estimate starts.</span></li>
  </ol>
</figure>''')
A('</section>')

# ---------------- 09 exercises ----------------
s = S["exercises"]
files = {
    "exercises/ex1_hover_pid.py": "exercises/ex1_hover_pid.py",
    "exercises/ex2_lqr_tvc.py": "exercises/ex2_lqr_tvc.py",
    "exercises/ex3_mpc_receding_horizon.py": "exercises/ex3_mpc_receding_horizon.py",
}
def link_files(text):
    t = esc(text)
    for f in files:
        t = t.replace(f, '<a href="%s"><code>%s</code></a>' % (f, f))
    return t

A('<section class="chapter" id="exercises">')
A(head("09", "Exercises", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"]))
A('</div>')
A(repo_note('Starter files, a small grader and setup notes live in <code>exercises/</code>. '
            'Read <a href="exercises/README.md"><code>exercises/README.md</code></a> first.'))
A('<ol class="exercises">')
for ex in copy["exercises"]:
    A('  <li class="ex">')
    A('    <h3>%s</h3>' % esc(ex["title"]))
    A('    <p class="meta">%s</p>' % link_files(ex["meta"]))
    A('    <p class="goal">%s</p>' % esc(ex["goal"]))
    A('    <ol>')
    for st in ex["steps"]:
        A('      <li>%s</li>' % link_files(st))
    A('    </ol>')
    A('    <p class="submit"><b>Submit:</b> %s</p>' % esc(ex["submit"]))
    A('    <p class="stretch"><b>Stretch:</b> %s</p>' % esc(ex["stretch"]))
    A('  </li>')
A('</ol>')
A('</section>')

# ---------------- further ----------------
s = S["further"]
A('<section class="chapter" id="further">')
A(head(None, "Further reading", s["heading"]))
A('<div class="prose">')
A(paras(s["paragraphs"]))
A('</div>')
A('<ul class="further">')
for f in copy["further"]:
    A('  <li><div class="t"><a href="%s" target="_blank" rel="noopener">%s</a></div><div class="w">%s</div></li>'
      % (f["url"], esc(f["title"]), esc(f["why"])))
A('</ul>')
A('</section>')

A('</main>')

# ---------------- footer ----------------
A('''<footer><div class="wrap">
  <p>Propulsive Landers @ Georgia Tech · GNC subteam · questions in #gnc on Discord</p>
  <p><a href="https://gtpropulsivelanders.org/" target="_blank" rel="noopener">gtpropulsivelanders.org</a> · <a href="https://github.com/Avionics-Propulsion-Landers-GT/MonopropUAV" target="_blank" rel="noopener">MonopropUAV on GitHub</a></p>
  <p>The tune-feedforward-first progression and the noise and disturbance toggles are borrowed from the WPILib controls tuning tutorials (<a href="https://github.com/wpilibsuite/wpilib-docs" target="_blank" rel="noopener">wpilibsuite/wpilib-docs</a>, <a href="https://creativecommons.org/licenses/by/4.0/" target="_blank" rel="noopener">CC BY 4.0</a>). The simulations here are our own.</p>
</div></footer>''')

out = "\n".join(parts) + "\n"
open(os.path.join(BUILD, "sections.html"), "w", encoding="utf-8", newline="\n").write(out)
print("wrote", len(out), "bytes")
