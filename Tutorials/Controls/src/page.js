/* Page-level behaviour: theme toggle, brand mark swap, active chapter in the top bar, mount sims. */
(function () {
  'use strict';
  const root = document.documentElement;
  const KEY = 'gtpl-controls-theme';
  const btn = document.getElementById('themeBtn');
  const order = ['auto', 'light', 'dark'];
  let saved = null;
  try { saved = localStorage.getItem(KEY); } catch (e) { saved = null; }
  let mode = order.indexOf(saved) >= 0 ? saved : 'auto';
  function applyTheme() {
    if (mode === 'auto') root.removeAttribute('data-theme'); else root.setAttribute('data-theme', mode);
    if (btn) btn.textContent = 'Theme: ' + mode;
    try { localStorage.setItem(KEY, mode); } catch (e) { /* storage may be unavailable */ }
  }
  if (btn) btn.addEventListener('click', () => { mode = order[(order.indexOf(mode) + 1) % order.length]; applyTheme(); });
  applyTheme();

  /* brand mark: dark variant on light ground, light variant on dark ground */
  const marks = document.querySelectorAll('img.mark');
  function swapMarks() {
    const dark = GTPL.theme.isDark();
    marks.forEach(img => {
      const want = dark ? img.dataset.light : img.dataset.dark;
      if (want && img.getAttribute('src') !== want) img.setAttribute('src', want);
    });
  }
  swapMarks();
  GTPL.theme.onChange(swapMarks);

  /* active chapter link */
  const links = Array.from(document.querySelectorAll('.topnav a[href^="#"]'));
  const byId = new Map(links.map(a => [a.getAttribute('href').slice(1), a]));
  const sections = Array.from(document.querySelectorAll('main section[id]'));
  if ('IntersectionObserver' in window && sections.length) {
    const visible = new Map();
    const io = new IntersectionObserver((entries) => {
      entries.forEach(e => visible.set(e.target.id, e.isIntersecting ? e.intersectionRatio : 0));
      let best = null, bestRatio = 0;
      sections.forEach(s => { const r = visible.get(s.id) || 0; if (r > bestRatio) { best = s.id; bestRatio = r; } });
      if (best === null) return;
      /* walk back to the nearest section that has a nav link */
      let idx = sections.findIndex(s => s.id === best);
      while (idx >= 0 && !byId.has(sections[idx].id)) idx--;
      links.forEach(a => a.classList.remove('active'));
      if (idx >= 0) byId.get(sections[idx].id).classList.add('active');
    }, { rootMargin: '-40% 0px -50% 0px', threshold: [0, 0.1, 0.5, 1] });
    sections.forEach(s => io.observe(s));
  }

  /* mount every instrument */
  GTPL.mountAll();
})();
