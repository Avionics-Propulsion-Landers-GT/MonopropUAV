"""Assemble the GTPL controls guide.

Outputs:
  <dest>/index.html        full standalone document (Google Fonts link + inline CSS/JS + data-URI assets)
  <build>/artifact.html    the same page as an Artifact fragment (no doctype/html/head/body; <title> and <style> first)
"""
import base64, io, os, re, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

BUILD = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(BUILD, 'assets')
DEST = os.path.dirname(BUILD)
SIMS = ['handfly', 'openloop', 'pid', 'ffb', 'lqr', 'mpc']
FONTS = 'https://fonts.googleapis.com/css2?family=B612:wght@400;700&family=B612+Mono:wght@400;700&family=IBM+Plex+Sans:ital,wght@0,400;0,500;1,400&family=Montserrat:wght@600;700;800&display=swap'
TITLE = 'Control, from the ground up'
DESC = 'An interactive introduction to control theory for the GNC subteam of Propulsive Landers at Georgia Tech: hand-fly a rocket, tune PID and feedforward, then see why the team moved to LQR and MPC.'

def read(p):
    with open(p, encoding='utf-8') as f:
        return f.read()

def data_uri(path, mime):
    with open(path, 'rb') as f:
        return f'data:{mime};base64,' + base64.b64encode(f.read()).decode('ascii')

css = read(os.path.join(BUILD, 'styles.css'))
helpers = read(os.path.join(BUILD, 'helpers.js'))
page = read(os.path.join(BUILD, 'page.js'))
sections = read(os.path.join(BUILD, 'sections.html'))
sims_js = []
missing = []
for name in SIMS:
    p = os.path.join(BUILD, 'sims', f'sim-{name}.js')
    if os.path.exists(p):
        sims_js.append(f'/* ---- sim: {name} ---- */\n' + read(p))
    else:
        missing.append(name)
if missing:
    print('WARNING: missing sim modules:', missing)

mark_dark = data_uri(os.path.join(ASSETS, 'mark_dark.png'), 'image/png')
mark_light = data_uri(os.path.join(ASSETS, 'mark_light.png'), 'image/png')
monarch = data_uri(os.path.join(ASSETS, 'monarch_480.png'), 'image/png')

# brand mark: give the img both variants; page.js swaps by theme
sections = re.sub(r'<img class="mark" src="ASSET_MARK"([^>]*)>',
                  lambda m: f'<img class="mark" src="{mark_dark}" data-dark="{mark_dark}" data-light="{mark_light}"{m.group(1)}>', sections)
sections = sections.replace('ASSET_MARK', mark_dark).replace('ASSET_MONARCH', monarch)
leftover = re.findall(r'ASSET_[A-Z_]+', sections)
if leftover:
    print('WARNING: unreplaced asset placeholders:', set(leftover))

# guard: no closing script tags inside inline JS
for chunk in [helpers, page] + sims_js:
    if '</script' in chunk.lower():
        raise SystemExit('inline script contains </script>; escape it')

script_block = '<script>\n' + helpers + '\n' + '\n'.join(sims_js) + '\n' + page + '\n</script>'
style_block = '<style>\n' + css + '\n</style>'

full = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{TITLE}</title>
<meta name="description" content="{DESC}">
<meta name="color-scheme" content="light dark">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="{FONTS}">
{style_block}
</head>
<body>
{sections}
{script_block}
</body>
</html>
'''

fragment = f'''<title>{TITLE}</title>
<link rel="stylesheet" href="{FONTS}">
{style_block}
{sections}
{script_block}
'''

os.makedirs(DEST, exist_ok=True)
with open(os.path.join(DEST, 'index.html'), 'w', encoding='utf-8') as f:
    f.write(full)
with open(os.path.join(BUILD, 'artifact.html'), 'w', encoding='utf-8') as f:
    f.write(fragment)
print('index.html', len(full.encode('utf-8')) // 1024, 'KB ->', os.path.join(DEST, 'index.html'))
print('artifact.html', len(fragment.encode('utf-8')) // 1024, 'KB')
