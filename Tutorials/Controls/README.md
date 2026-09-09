# Control, from the ground up

An interactive introduction to control theory for new members of the GNC subteam at
Propulsive Landers @ Georgia Tech. It walks from "fly the rocket by hand" through
feedforward, PID, feedforward plus feedback, LQR and MPC, with a live simulation next to
each idea, then hands off to three take-home exercises that plug into the MonopropUAV repo.

## What is in this folder

| Path | What it is |
| --- | --- |
| `index.html` | The whole guide. One file, no build step. Open it in a browser or serve it statically. |
| `exercises/` | Starter files for the three take-home exercises, plus a README with setup and submission steps. |
| `solutions/` | Reference solutions. Try the exercises first; this isn't a quiz, but the point is the attempt. |
| `src/` | The unassembled parts (stylesheet, shared helpers, one file per simulation, page copy) and the assembler script, for anyone editing the page. |

The page is published from `main` by the `Controls guide to GitHub Pages` workflow whenever this folder changes.

## Running it

Double-clicking `index.html` works. Everything is inline except the fonts, which load from
Google Fonts; without internet the page falls back to system fonts and still works.

To host it, put `index.html` anywhere static (GitHub Pages on the repo, a Netlify drop,
the club site). No server code is needed.

## Editing it

The page is assembled from the parts in `src/`:

- `styles.css` holds the design tokens (GTPL gold `#FFD600`, cream `#F0ECE2`, navy `#1E3A5F`),
  both themes, and every class the page uses.
- `helpers.js` is the shared simulation toolkit: canvas sizing, the fixed-step loop, strip
  charts, the rocket glyph, sliders and readouts.
- `sims/sim-*.js` are the six instruments. Each keeps its physics in pure functions under
  `GTPL.math.<name>` so `node sims/test-<name>.js` can check them without a browser.
- `copy.json` is the prose. Edit words there, then rebuild `sections.html` or edit the HTML
  directly for small fixes.
- `assemble.py` inlines everything into `index.html`.

Voice rules for edits: short sentences, real numbers from the vehicle, no exclamation
marks, explain terms through what is on screen, never as a glossary.

## Credits

Simulations, diagrams and text are the team's own. The feedforward-first tuning progression
and the noise and disturbance toggles are borrowed from the
[WPILib controls tuning tutorials](https://github.com/wpilibsuite/wpilib-docs) (CC BY 4.0).
Typeface note: B612, used for the instrument labels, is the font Airbus designed for cockpit
displays.
