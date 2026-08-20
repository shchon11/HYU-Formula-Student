'use strict';

/* Cone labeler, browser side. Boxes live in image pixels as [cls, x1, y1, x2, y2]
   while being edited and are converted to normalized YOLO on save, which is the
   same contract cone_labeler.py uses -- the two tools share a dataset. */

const COLORS = {
  BLUE: '#2979ff', ORANGE_BIG: '#ff3d00', ORANGE: '#ff9800',
  UNDEFINED: '#b0bec5', YELLOW: '#ffea00',
};
const FALLBACK = ['#00e676', '#e040fb', '#00bcd4', '#ff4081', '#cddc39'];
const ROW_H = 22;
const HANDLE = 8;
const MIN_BOX = 3;

const S = {
  classes: [], files: [], pos: new Map(), view: [], vpos: 0,
  cls: 0, boxes: [], sel: new Set(), dirty: false,
  img: null, iw: 0, ih: 0, scale: 1, ox: 0, oy: 0, autofit: true,
  undo: [], clip: [], user: '', version: 0,
  showLabels: true, drag: null, labelCache: new Map(), imgCache: new Map(),
  saving: 0,
};

const $ = (id) => document.getElementById(id);
const canvas = $('canvas');
const ctx = canvas.getContext('2d');

/* ------------------------------------------------------------------ helpers */

const colorOf = (i) => COLORS[(S.classes[i] || '').toUpperCase()] || FALLBACK[i % FALLBACK.length];
const nameOf = (i) => S.classes[i] || ('#' + i);
const curFile = () => (S.view.length ? S.files[S.view[S.vpos]] : null);

function api(path, opts) {
  return fetch(path, Object.assign({ credentials: 'same-origin' }, opts)).then((r) => {
    if (r.status === 403) { location.reload(); throw new Error('forbidden'); }
    if (!r.ok) throw new Error(path + ' -> ' + r.status);
    return r.json();
  });
}

function post(path, body) {
  return api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(Object.assign({ user: S.user }, body)),
  });
}

function status(msg, warn) {
  const f = curFile();
  const base = f
    ? `${S.vpos + 1}/${S.view.length} of ${S.files.length} · ${f.n} · boxes <b>${S.boxes.length}</b>` +
      ` (sel ${S.sel.size}) · class <b>${S.cls + 1}:${nameOf(S.cls)}</b>` +
      (f.r ? ` · reviewed by ${f.r === true ? '?' : f.r}` : '') +
      (S.dirty ? ' · <b>unsaved</b>' : '')
    : 'no frame matches this filter';
  $('status').innerHTML = base + (msg ? ` <span class="${warn ? 'warn' : ''}">— ${msg}</span>` : '');
}

let flashTimer = null;
function flash(msg, warn) {
  status(msg, warn);
  clearTimeout(flashTimer);
  flashTimer = setTimeout(() => status(''), 5000);
}

/* -------------------------------------------------------------------- view */

const FILTERS = ['all', 'unreviewed', 'reviewed', 'no boxes', 'has boxes'];

function buildFilter() {
  const sel = $('filter');
  sel.innerHTML = '';
  FILTERS.concat(S.classes.map((c) => 'has ' + c)).forEach((f) => {
    const o = document.createElement('option');
    o.value = o.textContent = f;
    sel.appendChild(o);
  });
}

function passes(f) {
  const mode = $('filter').value;
  if (mode === 'all') return true;
  if (mode === 'unreviewed') return !f.r;
  if (mode === 'reviewed') return !!f.r;
  if (mode === 'no boxes') return !f.b;
  if (mode === 'has boxes') return f.b > 0;
  if (mode.startsWith('has ')) {
    const i = S.classes.indexOf(mode.slice(4));
    return i >= 0 && (f.c || []).indexOf(i) >= 0;
  }
  return true;
}

function rebuildView(keepName) {
  const want = keepName || (curFile() && curFile().n);
  S.view = [];
  for (let i = 0; i < S.files.length; i++) if (passes(S.files[i])) S.view.push(i);
  S.vpos = 0;
  if (want) {
    const target = S.pos.get(want);
    if (target !== undefined) {
      let best = 0, dist = Infinity;
      for (let k = 0; k < S.view.length; k++) {
        const d = Math.abs(S.view[k] - target);
        if (d < dist) { dist = d; best = k; }
      }
      S.vpos = best;
    }
  }
  renderList(true);
}

/* --------------------------------------------------------------- frame list */

const listEl = $('list');
const innerEl = $('list-inner');
let renderedRange = [-1, -1];

function renderList(force) {
  innerEl.style.height = S.view.length * ROW_H + 'px';
  const top = listEl.scrollTop;
  const first = Math.max(0, Math.floor(top / ROW_H) - 6);
  const last = Math.min(S.view.length, Math.ceil((top + listEl.clientHeight) / ROW_H) + 6);
  if (!force && first === renderedRange[0] && last === renderedRange[1]) return;
  renderedRange = [first, last];

  const frag = document.createDocumentFragment();
  for (let k = first; k < last; k++) {
    const f = S.files[S.view[k]];
    const row = document.createElement('div');
    row.className = 'row' + (k === S.vpos ? ' on' : '') + (f.r ? ' done' : '');
    row.style.top = k * ROW_H + 'px';
    row.dataset.k = k;
    row.innerHTML =
      `<span class="mark">${f.r ? '✓' : (f.b ? '·' : '')}</span>` +
      `<span class="name"></span>` +
      (f.h && f.h !== S.user ? `<span class="held">${f.h}</span>` : '') +
      `<span class="n">${f.b || ''}</span>`;
    row.querySelector('.name').textContent = f.n;
    frag.appendChild(row);
  }
  innerEl.replaceChildren(frag);
}

listEl.addEventListener('scroll', () => renderList(false));
innerEl.addEventListener('click', (e) => {
  const row = e.target.closest('.row');
  if (!row) return;
  const k = +row.dataset.k;
  if (k !== S.vpos) { save(false).then(() => { S.vpos = k; load(); }); }
});

function scrollToCurrent() {
  const y = S.vpos * ROW_H;
  if (y < listEl.scrollTop || y > listEl.scrollTop + listEl.clientHeight - ROW_H) {
    listEl.scrollTop = Math.max(0, y - listEl.clientHeight / 2);
  }
  renderList(true);
}

/* ------------------------------------------------------------------ legend */

function renderLegend() {
  const counts = S.classes.map(() => 0);
  S.boxes.forEach((b) => { counts[b[0]] = (counts[b[0]] || 0) + 1; });
  $('legend').replaceChildren(...S.classes.map((name, i) => {
    const el = document.createElement('div');
    el.className = 'cls' + (i === S.cls ? ' on' : '');
    el.innerHTML = `<span class="key">${i + 1}</span>` +
      `<span class="swatch" style="background:${colorOf(i)}"></span>` +
      `<span class="name"></span><span class="count">${counts[i] || 0}</span>`;
    el.querySelector('.name').textContent = name;
    el.onclick = () => setClass(i);
    return el;
  }));
}

function renderProgress() {
  const done = S.files.reduce((n, f) => n + (f.r ? 1 : 0), 0);
  $('progress').textContent = `${done}/${S.files.length} reviewed`;
}

/* ------------------------------------------------------------------ canvas */

function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  if (S.autofit) fit(); else draw();
}
window.addEventListener('resize', resizeCanvas);

const cw = () => canvas.clientWidth;
const ch = () => canvas.clientHeight;

function fit() {
  if (!S.iw) return;
  S.scale = Math.min(cw() / S.iw, ch() / S.ih);
  S.ox = (cw() - S.iw * S.scale) / 2;
  S.oy = (ch() - S.ih * S.scale) / 2;
  S.autofit = true;
  draw();
}

function zoomAt(factor, px, py) {
  const ix = (px - S.ox) / S.scale, iy = (py - S.oy) / S.scale;
  S.scale = Math.max(0.05, Math.min(40, S.scale * factor));
  S.ox = px - ix * S.scale;
  S.oy = py - iy * S.scale;
  S.autofit = false;
  draw();
}

const toImg = (px, py) => [(px - S.ox) / S.scale, (py - S.oy) / S.scale];
const toScr = (x, y) => [x * S.scale + S.ox, y * S.scale + S.oy];

function boxRect(b) {
  const [x1, y1] = toScr(Math.min(b[1], b[3]), Math.min(b[2], b[4]));
  const [x2, y2] = toScr(Math.max(b[1], b[3]), Math.max(b[2], b[4]));
  return [x1, y1, x2 - x1, y2 - y1];
}

function draw() {
  ctx.clearRect(0, 0, cw(), ch());
  ctx.fillStyle = '#16171a';
  ctx.fillRect(0, 0, cw(), ch());
  if (!S.img) return;
  ctx.imageSmoothingEnabled = S.scale < 1;
  ctx.drawImage(S.img, S.ox, S.oy, S.iw * S.scale, S.ih * S.scale);

  ctx.font = '600 11px system-ui, sans-serif';
  ctx.textBaseline = 'middle';
  S.boxes.forEach((b, i) => {
    const [x, y, w, h] = boxRect(b);
    const color = colorOf(b[0]);
    const on = S.sel.has(i);
    ctx.lineWidth = on ? 3 : 2;
    ctx.strokeStyle = color;
    ctx.strokeRect(x, y, w, h);
    if (on) {
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(255,255,255,.85)';
      ctx.strokeRect(x - 2.5, y - 2.5, w + 5, h + 5);
      ctx.setLineDash([]);
    }
    if (S.showLabels) {
      const text = nameOf(b[0]);
      const tw = ctx.measureText(text).width + 8;
      const ty = Math.max(0, y - 15);
      ctx.fillStyle = color;
      ctx.fillRect(x, ty, tw, 15);
      ctx.fillStyle = '#101112';
      ctx.fillText(text, x + 4, ty + 8);
    }
  });

  if (S.sel.size === 1) {
    const b = S.boxes[[...S.sel][0]];
    if (b) {
      const [x, y, w, h] = boxRect(b);
      ctx.fillStyle = '#14161a';
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1;
      handlePoints(x, y, w, h).forEach(([hx, hy]) => {
        ctx.fillRect(hx - HANDLE / 2, hy - HANDLE / 2, HANDLE, HANDLE);
        ctx.strokeRect(hx - HANDLE / 2, hy - HANDLE / 2, HANDLE, HANDLE);
      });
    }
  }

  if (S.drag && S.drag.mode === 'draw') {
    const [x1, y1] = toScr(S.drag.x1, S.drag.y1);
    const [x2, y2] = toScr(S.drag.x2, S.drag.y2);
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 2;
    ctx.strokeStyle = colorOf(S.cls);
    ctx.strokeRect(Math.min(x1, x2), Math.min(y1, y2), Math.abs(x2 - x1), Math.abs(y2 - y1));
    ctx.setLineDash([]);
  }
}

const HANDLE_NAMES = ['tl', 't', 'tr', 'r', 'br', 'b', 'bl', 'l'];
function handlePoints(x, y, w, h) {
  return [[x, y], [x + w / 2, y], [x + w, y], [x + w, y + h / 2],
          [x + w, y + h], [x + w / 2, y + h], [x, y + h], [x, y + h / 2]];
}

function hitHandle(px, py) {
  if (S.sel.size !== 1) return null;
  const b = S.boxes[[...S.sel][0]];
  if (!b) return null;
  const [x, y, w, h] = boxRect(b);
  const pts = handlePoints(x, y, w, h);
  for (let i = 0; i < pts.length; i++) {
    if (Math.abs(pts[i][0] - px) <= HANDLE && Math.abs(pts[i][1] - py) <= HANDLE) return HANDLE_NAMES[i];
  }
  return null;
}

function hitBox(px, py) {
  let best = null, area = Infinity;
  S.boxes.forEach((b, i) => {
    const [x, y, w, h] = boxRect(b);
    if (px >= x - 2 && px <= x + w + 2 && py >= y - 2 && py <= y + h + 2 && w * h < area) {
      area = w * h; best = i;
    }
  });
  return best;
}

/* -------------------------------------------------------------- mouse input */

canvas.addEventListener('contextmenu', (e) => e.preventDefault());

canvas.addEventListener('mousedown', (e) => {
  if (!S.img) return;
  const r = canvas.getBoundingClientRect();
  const px = e.clientX - r.left, py = e.clientY - r.top;

  // Pan is middle-drag only: Space already means confirm-and-advance.
  if (e.button === 1) {
    S.drag = { mode: 'pan', px, py };
    return;
  }
  if (e.button === 2) {
    const i = hitBox(px, py);
    if (i !== null) { pushUndo(); S.boxes.splice(i, 1); S.sel.clear(); changed(); }
    return;
  }
  if (e.button !== 0) return;

  const handle = e.ctrlKey ? null : hitHandle(px, py);
  if (handle) {
    pushUndo();
    S.drag = { mode: 'resize', handle };
    return;
  }
  const i = hitBox(px, py);
  if (i !== null) {
    if (e.ctrlKey || e.metaKey) {
      S.sel.has(i) ? S.sel.delete(i) : S.sel.add(i);
      draw(); status('');
      return;
    }
    if (!S.sel.has(i)) { S.sel = new Set([i]); }
    pushUndo();
    const [ix, iy] = toImg(px, py);
    S.drag = { mode: 'move', ix, iy, start: S.boxes.map((b) => b.slice()) };
    draw();
    return;
  }
  S.sel.clear();
  const [ix, iy] = toImg(px, py);
  S.drag = { mode: 'draw', x1: ix, y1: iy, x2: ix, y2: iy };
  draw();
});

window.addEventListener('mousemove', (e) => {
  if (!S.img) return;
  const r = canvas.getBoundingClientRect();
  const px = e.clientX - r.left, py = e.clientY - r.top;
  const d = S.drag;
  if (!d) {
    if (px >= 0 && py >= 0 && px <= r.width && py <= r.height) {
      canvas.style.cursor = hitHandle(px, py) ? 'nwse-resize'
        : (hitBox(px, py) !== null ? 'move' : 'crosshair');
    }
    return;
  }
  if (d.mode === 'pan') {
    S.ox += px - d.px; S.oy += py - d.py;
    d.px = px; d.py = py; S.autofit = false; draw();
    return;
  }
  const [ix, iy] = toImg(px, py);
  if (d.mode === 'draw') { d.x2 = ix; d.y2 = iy; draw(); return; }
  if (d.mode === 'move') {
    const dx = ix - d.ix, dy = iy - d.iy;
    S.sel.forEach((i) => {
      const s = d.start[i];
      S.boxes[i] = [s[0], s[1] + dx, s[2] + dy, s[3] + dx, s[4] + dy];
    });
    draw();
    return;
  }
  if (d.mode === 'resize' && S.sel.size === 1) {
    const b = S.boxes[[...S.sel][0]];
    if (d.handle.indexOf('l') >= 0) b[1] = ix;
    if (d.handle.indexOf('r') >= 0) b[3] = ix;
    if (d.handle.indexOf('t') >= 0) b[2] = iy;
    if (d.handle.indexOf('b') >= 0) b[4] = iy;
    draw();
  }
});

window.addEventListener('mouseup', () => {
  const d = S.drag;
  S.drag = null;
  if (!d) return;
  if (d.mode === 'draw') {
    if (Math.abs(d.x2 - d.x1) >= MIN_BOX && Math.abs(d.y2 - d.y1) >= MIN_BOX) {
      pushUndo();
      S.boxes.push([S.cls, Math.min(d.x1, d.x2), Math.min(d.y1, d.y2),
                    Math.max(d.x1, d.x2), Math.max(d.y1, d.y2)]);
      S.sel = new Set([S.boxes.length - 1]);
      changed();
    } else draw();
    return;
  }
  if (d.mode === 'move' || d.mode === 'resize') {
    S.boxes.forEach((b) => {
      const x1 = Math.min(b[1], b[3]), x2 = Math.max(b[1], b[3]);
      const y1 = Math.min(b[2], b[4]), y2 = Math.max(b[2], b[4]);
      b[1] = Math.max(0, x1); b[2] = Math.max(0, y1);
      b[3] = Math.min(S.iw, x2); b[4] = Math.min(S.ih, y2);
    });
    changed();
  }
});

canvas.addEventListener('wheel', (e) => {
  if (!S.img) return;
  e.preventDefault();
  const r = canvas.getBoundingClientRect();
  zoomAt(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX - r.left, e.clientY - r.top);
}, { passive: false });

/* ------------------------------------------------------------------- edits */

function pushUndo() {
  const f = curFile();
  if (!f) return;
  S.undo.push({ type: 'boxes', name: f.n, boxes: S.boxes.map((b) => b.slice()) });
  if (S.undo.length > 200) S.undo.shift();
}

function changed() {
  S.dirty = true;
  draw();
  renderLegend();
  status('');
}

function setClass(i) {
  if (i >= S.classes.length) return;
  S.cls = i;
  if (S.sel.size) {
    pushUndo();
    S.sel.forEach((k) => { S.boxes[k][0] = i; });
    changed();
    flash(`${S.sel.size} box(es) → ${nameOf(i)} (ctrl+Z undoes)`);
  } else {
    renderLegend();
    status('');
  }
}

function deleteSelected() {
  if (!S.sel.size) { flash('no box selected — Delete removes the image itself', true); return; }
  pushUndo();
  [...S.sel].sort((a, b) => b - a).forEach((i) => S.boxes.splice(i, 1));
  S.sel.clear();
  changed();
}

async function deleteImage() {
  const f = curFile();
  if (!f) return;
  const res = await post('/api/delete', { name: f.n });
  if (!res.ok) { flash(res.error || 'delete failed', true); return; }
  S.undo.push({ type: 'delete', name: f.n });
  dropFile(f.n);
  S.dirty = false;
  rebuildView();
  if (S.vpos >= S.view.length) S.vpos = Math.max(0, S.view.length - 1);
  await load();
  flash(`${f.n} → _trash (ctrl+Z restores)`);
}

async function undo() {
  const e = S.undo.pop();
  if (!e) { flash('nothing to undo'); return; }
  if (e.type === 'boxes') {
    const f = curFile();
    if (!f || f.n !== e.name) {
      const target = S.view.findIndex((i) => S.files[i].n === e.name);
      if (target < 0) { flash(`${e.name} is not in this filter`, true); return; }
      await save(false);
      S.vpos = target;
      await load();
    }
    S.boxes = e.boxes.map((b) => b.slice());
    S.sel.clear();
    changed();
    flash('undo: boxes');
    return;
  }
  const res = await post('/api/restore', { name: e.name });
  if (!res.ok) { flash(res.error || 'restore failed', true); return; }
  addFile(res.entry);
  rebuildView(e.name);
  await load();
  flash(`restored ${e.name}`);
}

function copyBoxes() {
  S.clip = S.boxes.map((b) => b.slice());
  flash(`copied ${S.clip.length} box(es)`);
}

function pasteBoxes() {
  if (!S.clip.length) { flash('clipboard is empty'); return; }
  pushUndo();
  const start = S.boxes.length;
  S.clip.forEach((b) => S.boxes.push(b.slice()));
  S.sel = new Set(S.boxes.map((_, i) => i).slice(start));
  changed();
  flash(`pasted ${S.clip.length} box(es)`);
}

/* ------------------------------------------------------------ file bookkeeping */

function reindex() {
  S.pos = new Map(S.files.map((f, i) => [f.n, i]));
}

function dropFile(name) {
  const i = S.pos.get(name);
  if (i === undefined) return;
  S.files.splice(i, 1);
  reindex();
}

function addFile(entry) {
  if (S.pos.has(entry.n)) return;
  S.files.push(entry);
  S.files.sort((a, b) => (a.n < b.n ? -1 : a.n > b.n ? 1 : 0));
  reindex();
}

/* ------------------------------------------------------------- load and save */

function parseLabels(text, w, h) {
  const out = [];
  text.split('\n').forEach((line) => {
    const p = line.trim().split(/\s+/);
    if (p.length < 5) return;
    const cls = parseInt(p[0], 10);
    const cx = +p[1], cy = +p[2], bw = +p[3], bh = +p[4];
    if (!isFinite(cx) || !isFinite(cy) || !isFinite(bw) || !isFinite(bh)) return;
    out.push([cls, (cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h]);
  });
  return out;
}

function toYolo() {
  return S.boxes.map((b) => {
    const x1 = Math.max(0, Math.min(b[1], b[3])), x2 = Math.min(S.iw, Math.max(b[1], b[3]));
    const y1 = Math.max(0, Math.min(b[2], b[4])), y2 = Math.min(S.ih, Math.max(b[2], b[4]));
    return [b[0], (x1 + x2) / 2 / S.iw, (y1 + y2) / 2 / S.ih, (x2 - x1) / S.iw, (y2 - y1) / S.ih];
  }).filter((b) => b[3] * S.iw >= 1 && b[4] * S.ih >= 1);
}

function fetchImage(name) {
  let p = S.imgCache.get(name);
  if (!p) {
    p = new Promise((resolve, reject) => {
      const im = new Image();
      im.onload = () => resolve(im);
      im.onerror = () => reject(new Error('image ' + name));
      im.src = '/api/image/' + encodeURIComponent(name);
    });
    S.imgCache.set(name, p);
    if (S.imgCache.size > 24) S.imgCache.delete(S.imgCache.keys().next().value);
  }
  return p;
}

function fetchLabels(name) {
  let p = S.labelCache.get(name);
  if (!p) {
    p = api('/api/labels/' + encodeURIComponent(name)).then((r) => r.text || '');
    S.labelCache.set(name, p);
    if (S.labelCache.size > 40) S.labelCache.delete(S.labelCache.keys().next().value);
  }
  return p;
}

function prefetch() {
  for (let k = S.vpos + 1; k <= S.vpos + 3 && k < S.view.length; k++) {
    const f = S.files[S.view[k]];
    if (f) { fetchImage(f.n); fetchLabels(f.n); }
  }
}

let loadToken = 0;
async function load() {
  const f = curFile();
  if (!f) {
    S.img = null; S.boxes = []; S.sel.clear();
    draw(); renderLegend(); status('');
    return;
  }
  const my = ++loadToken;
  try {
    const [im, text] = await Promise.all([fetchImage(f.n), fetchLabels(f.n)]);
    if (my !== loadToken) return;
    S.img = im; S.iw = im.naturalWidth; S.ih = im.naturalHeight;
    S.boxes = parseLabels(text, S.iw, S.ih);
  } catch (err) {
    if (my !== loadToken) return;
    S.img = null; S.boxes = [];
    flash('could not load ' + f.n, true);
  }
  S.sel.clear();
  S.dirty = false;
  if (S.autofit) fit(); else draw();
  renderLegend();
  scrollToCurrent();
  status('');
  prefetch();
  poll(f.n);
}

async function save(markReviewed) {
  const f = curFile();
  if (!f || !S.img) return;
  if (!S.dirty && !markReviewed) return;
  const boxes = toYolo();
  S.saving++;
  try {
    const res = await post('/api/save', { name: f.n, boxes, reviewed: !!markReviewed });
    if (res.ok && res.entry) {
      Object.assign(f, res.entry);
      S.labelCache.set(f.n, Promise.resolve(
        boxes.map((b) => b.map((v, i) => (i ? v.toFixed(6) : v)).join(' ')).join('\n')));
      S.dirty = false;
      renderList(true);
      renderProgress();
    } else if (!res.ok) {
      flash(res.error || 'save failed', true);
    }
  } catch (err) {
    flash('save failed: ' + err.message, true);
  } finally {
    S.saving--;
  }
}

/* -------------------------------------------------------------- navigation */

async function go(delta) {
  if (!S.view.length) return;
  await save(false);
  S.vpos = Math.max(0, Math.min(S.view.length - 1, S.vpos + delta));
  await load();
}

async function confirmNext() {
  await save(true);
  await go(1);
}

async function claim() {
  await save(false);
  const cur = curFile();
  const res = await post('/api/claim', { after: cur ? S.pos.get(cur.n) : -1 });
  if (!res.ok) { flash(res.error || 'nothing to claim', true); return; }
  const k = S.view.indexOf(S.pos.get(res.name));
  if (k < 0) {
    $('filter').value = 'all';
    rebuildView(res.name);
    S.vpos = S.view.indexOf(S.pos.get(res.name));
  } else S.vpos = k;
  await load();
  flash('claimed for you');
}

/* ------------------------------------------------------------------ polling */

let polling = false;
async function poll(holdName) {
  if (polling) return;
  polling = true;
  try {
    const q = `/api/changes?v=${S.version}&user=${encodeURIComponent(S.user)}` +
              (holdName ? `&hold=${encodeURIComponent(holdName)}` : '');
    const res = await api(q);
    S.version = res.version;
    let structural = false;
    (res.files || []).forEach((e) => {
      if (e.gone) { if (S.pos.has(e.n)) { dropFile(e.n); structural = true; } return; }
      const i = S.pos.get(e.n);
      if (i === undefined) { addFile(e); structural = true; return; }
      const cur = S.files[i];
      delete cur.r; delete cur.h;
      Object.assign(cur, e);
    });
    S.files.forEach((f) => { delete f.h; });
    Object.entries(res.holds || {}).forEach(([n, u]) => {
      const i = S.pos.get(n);
      if (i !== undefined && u !== S.user) S.files[i].h = u;
    });
    if (structural) rebuildView(); else renderList(true);
    renderProgress();
  } catch (err) {
    /* a poll that fails is not worth interrupting the labeler over */
  } finally {
    polling = false;
  }
}

setInterval(() => {
  const f = curFile();
  poll(f ? f.n : '');
}, 12000);

/* --------------------------------------------------------------- keyboard */

window.addEventListener('keydown', (e) => {
  if ($('gate').hidden === false) return;
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  const ctrl = e.ctrlKey || e.metaKey;
  const k = e.key;

  if (k === ' ' && !ctrl) {
    e.preventDefault();
    if (!e.repeat && !S.drag) confirmNext();
    return;
  }
  if (ctrl && (k === 'z' || k === 'Z')) { e.preventDefault(); undo(); return; }
  if (ctrl && (k === 'c' || k === 'C')) { e.preventDefault(); copyBoxes(); return; }
  if (ctrl && (k === 'v' || k === 'V')) { e.preventDefault(); pasteBoxes(); return; }
  if (ctrl && (k === 'a' || k === 'A')) {
    e.preventDefault();
    S.sel = new Set(S.boxes.map((_, i) => i));
    draw(); status('');
    return;
  }
  if (ctrl && (k === 's' || k === 'S')) { e.preventDefault(); save(true).then(() => flash('saved')); return; }
  if (ctrl) return;

  if (k === 'd' || k === 'D' || k === 'ArrowRight') { e.preventDefault(); go(1); }
  else if (k === 'a' || k === 'A' || k === 'ArrowLeft') { e.preventDefault(); go(-1); }
  else if (k === 'n' || k === 'N') claim();
  else if (k === 'g' || k === 'G') {
    const n = prompt(`go to frame 1..${S.view.length}`, S.vpos + 1);
    const v = parseInt(n, 10);
    if (v >= 1 && v <= S.view.length) save(false).then(() => { S.vpos = v - 1; load(); });
  } else if (k === 's' || k === 'S') save(true).then(() => flash('saved'));
  else if (k === 'l' || k === 'L') { S.showLabels = !S.showLabels; draw(); }
  else if (k === 'h' || k === 'H' || k === '?') $('help').hidden = !$('help').hidden;
  else if (k === 'Tab') {
    e.preventDefault();
    if (S.boxes.length) {
      const cur = S.sel.size ? Math.max(...S.sel) : -1;
      S.sel = new Set([(cur + 1) % S.boxes.length]);
      draw(); status('');
    }
  } else if (k === 'Escape') { S.sel.clear(); $('help').hidden = true; draw(); status(''); }
  else if (k === 'Backspace') { e.preventDefault(); deleteSelected(); }
  else if (k === 'Delete') { e.preventDefault(); deleteImage(); }
  else if (k === '+' || k === '=') zoomAt(1.25, cw() / 2, ch() / 2);
  else if (k === '-') zoomAt(1 / 1.25, cw() / 2, ch() / 2);
  else if (k === '0') fit();
  else if (k >= '1' && k <= '9') setClass(+k - 1);
});

window.addEventListener('beforeunload', (e) => {
  if (S.dirty || S.saving) { e.preventDefault(); e.returnValue = ''; }
});

/* ---------------------------------------------------------------- toolbar */

document.querySelector('header').addEventListener('click', (e) => {
  const act = e.target.dataset && e.target.dataset.act;
  if (!act) return;
  ({
    prev: () => go(-1),
    next: () => go(1),
    confirm: confirmNext,
    claim: claim,
    delete: deleteImage,
    undo: undo,
    help: () => { $('help').hidden = !$('help').hidden; },
    stats: showStats,
  }[act] || (() => {}))();
});

$('filter').addEventListener('change', () => { rebuildView(); load(); });

async function showStats() {
  const s = await api('/api/stats');
  const rows = s.classes.map((c, i) =>
    `<tr><td><span class="swatch" style="display:inline-block;width:10px;height:10px;border-radius:2px;background:${colorOf(i)}"></span> ${c}</td><td>${s.per_class[i]}</td></tr>`).join('');
  const users = Object.entries(s.by_user).sort((a, b) => b[1] - a[1])
    .map(([u, n]) => `<tr><td>${u}</td><td>${n}</td></tr>`).join('') || '<tr><td>nobody yet</td><td>0</td></tr>';
  $('modal-body').innerHTML =
    `<h2>dataset</h2><table>
       <tr><td>images</td><td>${s.images}</td></tr>
       <tr><td>reviewed</td><td>${s.reviewed}</td></tr>
       <tr><td>frames with no boxes</td><td>${s.empty}</td></tr></table>
     <h2>boxes per class</h2><table>${rows}</table>
     <h2>reviewed by</h2><table>${users}</table>`;
  $('modal').hidden = false;
}
$('modal-close').onclick = () => { $('modal').hidden = true; };

/* -------------------------------------------------------------------- boot */

async function start(user) {
  S.user = user;
  localStorage.setItem('labeler-user', user);
  $('me').textContent = user;
  $('gate').hidden = true;
  $('app').hidden = false;

  const snap = await api('/api/session?user=' + encodeURIComponent(user));
  S.classes = snap.classes;
  S.files = snap.files;
  S.version = snap.version;
  reindex();
  buildFilter();
  rebuildView();
  renderProgress();
  resizeCanvas();
  await load();
  if (!S.files.some((f) => f.r)) $('help').hidden = false;
}

$('enter').onclick = () => {
  const v = $('who').value.trim();
  if (!v) { $('gate-err').textContent = 'enter a name first'; return; }
  start(v).catch((err) => { $('gate-err').textContent = String(err.message || err); });
};
$('who').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('enter').click(); });
$('who').value = new URLSearchParams(location.search).get('user') ||
                 localStorage.getItem('labeler-user') || '';
$('who').focus();

// ?user=<name> skips the gate, so each person can be handed their own bookmark.
if (new URLSearchParams(location.search).get('user')) $('enter').click();
