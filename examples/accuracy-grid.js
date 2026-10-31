// Localization accuracy across the field: a 0.1 m grid, the robot at each point facing its target CELL, Monte Carlo
// corner noise, occlusion by the HIVE frame and the official CAD CELL skins. Compares two camera pitches and breaks the
// error down by viewing angle and distance.
// Run from the repository root:  node examples/accuracy-grid.js
// Writes examples/output/accuracy-grid.json, accuracy-heatmaps.svg and accuracy-by-angle.svg. With --charts-only it
// redraws the charts from an existing accuracy-grid.json instead of re-running the two-minute simulation.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.join(__dirname, '..'), out = path.join(__dirname, 'output');
const DEG = 180/Math.PI;
const STEP = .1, HALF = 72*.0254, SAMPLES = 16, NOISE_PX = 1, PITCHES = [10, 25];

// Chart styling: reference palette, light theme.
const INK = '#0b0b0b', INK2 = '#52514e', MUTED = '#8a8984', GRID = '#e4e3df', SURFACE = '#fcfcfb', NOFIX = '#ecebe7';
const SEQ = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5', '#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b'];
const SERIES = ['#2a78d6', '#eb6834', '#1baf7a'];
const CLUSTERS = ['#9ec5f4', '#3987e5', '#104281'];          // 1, 2, 3 clusters: ordinal steps of the sequential blue
const seq = (v, max) => SEQ[Math.min(SEQ.length - 1, Math.floor(Math.max(0, v)/max*SEQ.length))];
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
const font = 'font-family="system-ui, -apple-system, Segoe UI, Microsoft YaHei, sans-serif"';

// Load the page script headless, as the tests do, for its CAD-based occlusion test (sightBlocked).
const page = loadPage();
const L = page.Loc;
const tags = L.defaultTags({red:1, blue:-1}), layout = [{tags}];

const cached = process.argv.includes('--charts-only') && JSON.parse(fs.readFileSync(path.join(out, 'accuracy-grid.json'), 'utf8'));
const results = cached ? cached.grid : {};
for (const pitch of cached ? [] : PITCHES) {
  const camera = {...L.defaultCamera(), pitchDeg:pitch};
  const cells = [];
  for (let x = -HALF + STEP/2; x < HALF; x += STEP) for (let y = -HALF + STEP/2; y < HALF; y += STEP) {
    if (Math.abs(x) < .72 && Math.abs(y) < .62) continue;                // HIVE frame footprint and CELLs overhead
    const hive = x <= 0 ? 'red' : 'blue', up = hive === 'red' ? 1 : -1;  // nearest HIVE in X, as the simulator aims
    const target = L.cellFrame(hive, up, up).origin;
    const robot = {x, y, headingDeg:Math.atan2(target.y - y, target.x - x)*DEG};
    // Viewing angle around the target CELL: 0° straight in front of its opening, ±90° from the side, beyond is behind.
    const offAxis = Math.atan2((x - target.x)*up, (y - target.y)*up)*DEG*-1;
    const dist = Math.hypot(x - target.x, y - target.y);
    const seen = L.detect({robot, camera, tags, pixelNoise:0, occluded:page.sightBlocked}).detections.map(d => d.id);
    const cell = {x:+x.toFixed(2), y:+y.toFixed(2), hive, offAxisDeg:+offAxis.toFixed(1), distM:+dist.toFixed(2), tags:seen.length,
      clusters:new Set(seen.map(id => Math.floor((id - 30)/4))).size};
    if (seen.length) {
      const visible = tags.filter(t => seen.includes(t.id)), pos = [], aim = [], range = [];
      let usable = 0;
      for (let seed = 1; seed <= SAMPLES; seed++) {
        const r = L.localize({robot, camera, tags:visible, layouts:layout, pixelNoise:NOISE_PX, seed, pitchPriorDeg:3});
        if (!r.estimate) continue;
        const e = r.estimate, bearing = p => Math.atan2(target.y - p.y, target.x - p.x)*DEG;
        pos.push(r.error.position*100);
        aim.push(((robot.headingDeg + bearing(e) - e.headingDeg - bearing(robot)) % 360 + 540) % 360 - 180);
        range.push((Math.hypot(target.x - e.x, target.y - e.y) - dist)*100);
        if (L.qualityGate(e, {camera, tags}).ok) usable++;
      }
      Object.assign(cell, {posRmsCm:+rms(pos).toFixed(2), aimRmsDeg:+rms(aim).toFixed(3), rangeRmsCm:+rms(range).toFixed(2), gatePass:usable/SAMPLES});
    }
    cells.push(cell);
  }
  results[pitch] = cells;
  const fixed = cells.filter(c => c.posRmsCm != null);
  console.log(`pitch ${pitch}°: ${fixed.length}/${cells.length} grid points see a tag; median position RMS ${median(fixed.map(c => c.posRmsCm)).toFixed(2)} cm, ` +
    `median aim RMS ${median(fixed.map(c => c.aimRmsDeg)).toFixed(3)}°`);
}

const bands = [[.6, 1.2], [1.2, 1.8], [1.8, 2.6]], angleBins = [];
for (let a = -150; a < 150; a += 15) angleBins.push([a, a + 15]);
const byAngle = {};
for (const pitch of PITCHES) byAngle[pitch] = bands.map(([d0, d1]) => angleBins.map(([a0, a1]) => {
  const inBin = results[pitch].filter(c => c.distM >= d0 && c.distM < d1 && c.offAxisDeg >= a0 && c.offAxisDeg < a1);
  const fixed = inBin.filter(c => c.posRmsCm != null);
  return {angle:(a0 + a1)/2, n:inBin.length, fixed:fixed.length, posRmsCm:fixed.length ? rms(fixed.map(c => c.posRmsCm)) : null,
    aimRmsDeg:fixed.length ? rms(fixed.map(c => c.aimRmsDeg)) : null, tags:fixed.length ? mean(fixed.map(c => c.tags)) : 0};
}));

fs.mkdirSync(out, {recursive:true});
fs.writeFileSync(path.join(out, 'accuracy-grid.json'), JSON.stringify({setup:{gridStepM:STEP, samplesPerPoint:SAMPLES, cornerNoisePx:NOISE_PX,
  camera:'640x480, 70° HFOV, 0.30 m high, pitch estimated with a ±3° prior', pitchesDeg:PITCHES, hiveUp:{red:1, blue:-1}}, byAngle, grid:results}) + '\n');
fs.writeFileSync(path.join(out, 'accuracy-heatmaps.svg'), heatmaps());
fs.writeFileSync(path.join(out, 'accuracy-by-angle.svg'), angleChart());
console.log('Wrote examples/output/accuracy-grid.json, accuracy-heatmaps.svg and accuracy-by-angle.svg');

function rms(a) { return Math.sqrt(a.reduce((s, v) => s + v*v, 0)/a.length) }
function mean(a) { return a.reduce((s, v) => s + v, 0)/a.length }
function median(a) { const s = [...a].sort((p, q) => p - q); return s[Math.floor(s.length/2)] }

function loadPage() {
  const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8'), script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
  const defaults = {x:'-0.6', y:'1.8', z:'0.55', speed:'6.5', angle:'52', yaw:'-90', diameter:'0.071', drag:'0.5', threshold:'0.30', camHeight:'0.30',
    camPitch:'10', pitchError:'0', camFov:'70', pixelNoise:'1', frames:'1', visionMode:'geometry', camRes:'640x480', decimate:'1', team:'red', state:'1'};
  const ctx = new Proxy({}, {get:() => () => {}}), made = {};
  const element = id => made[id] ??= {value:defaults[id] ?? '0', checked:false, innerHTML:'', clientWidth:600, clientHeight:400, width:0, height:0, style:{},
    getContext:() => ctx, addEventListener() {}};
  const sandbox = {document:{getElementById:element}, window:{devicePixelRatio:1, addEventListener() {}}, requestAnimationFrame() {}, setTimeout, clearTimeout, Math, console};
  vm.createContext(sandbox);
  for (const file of ['localization.js', 'cad/hive-mesh.js']) vm.runInContext(fs.readFileSync(path.join(root, file), 'utf8'), sandbox);
  vm.runInContext(script + ';globalThis.exported={Loc,sightBlocked};', sandbox);
  return sandbox.exported;
}

// ---- Charts (static SVG, light theme; every cell and point carries a native tooltip) ----
function fieldPanel(cells, key, max, unit, ox, oy, size, title, levels = null) {
  const k = size/(2*HALF), X = x => ox + (x + HALF)*k, Y = y => oy + (HALF - y)*k, w = STEP*k;
  let s = `<text x="${ox}" y="${oy - 12}" ${font} font-size="13" font-weight="600" fill="${INK}">${esc(title)}</text>`;
  s += `<rect x="${ox}" y="${oy}" width="${size}" height="${size}" fill="${NOFIX}"/>`;
  for (const c of cells) {
    const v = c[key], tip = v == null ? `(${c.x}, ${c.y}) m: no tag in view` :
      `(${c.x}, ${c.y}) m · ${c.distM} m from target, ${c.offAxisDeg}° off axis · ${c.tags} tags · position ${c.posRmsCm} cm · aim ${c.aimRmsDeg}° · gate ${Math.round(c.gatePass*100)}%`;
    s += `<rect x="${(X(c.x) - w/2).toFixed(1)}" y="${(Y(c.y) - w/2).toFixed(1)}" width="${(w - .6).toFixed(1)}" height="${(w - .6).toFixed(1)}" rx="1" fill="${v == null ? NOFIX : levels ? levels[v - 1] : seq(v, max)}"><title>${esc(tip)}</title></rect>`;
  }
  // HIVE frame base, the four CELLs (upward ones filled), and the two targets.
  s += `<rect x="${X(-.628)}" y="${Y(.495)}" width="${1.256*k}" height="${.99*k}" fill="none" stroke="${INK2}" stroke-width="1.2" stroke-dasharray="4 3"/>`;
  for (const [hx, label] of [[-.324, 'R'], [.324, 'B']]) for (const sy of [1, -1]) {
    const isUp = (label === 'R' && sy === 1) || (label === 'B' && sy === -1);
    s += `<rect x="${X(hx - .254)}" y="${Y(sy*.41 + .15)}" width="${.508*k}" height="${.3*k}" rx="3" fill="${isUp ? INK : 'none'}" fill-opacity="${isUp ? .12 : 0}" stroke="${INK}" stroke-width="1"/>`;
    if (isUp) s += `<text x="${X(hx)}" y="${Y(sy*.41) + 4}" ${font} font-size="11" text-anchor="middle" fill="${INK}">${label} ↑</text>`;
  }
  s += `<rect x="${ox}" y="${oy}" width="${size}" height="${size}" fill="none" stroke="${INK2}" stroke-width="1"/>`;
  return s;
}

function colorBar(x, y, w, max, unit, label) {
  let s = `<text x="${x}" y="${y - 6}" ${font} font-size="12" fill="${INK2}">${esc(label)}</text>`;
  SEQ.forEach((c, i) => { s += `<rect x="${x + i*w/SEQ.length}" y="${y}" width="${w/SEQ.length + .5}" height="10" fill="${c}"/>` });
  for (let t = 0; t <= 4; t++) s += `<text x="${x + t*w/4}" y="${y + 24}" ${font} font-size="11" fill="${INK2}" text-anchor="middle">${+(max*t/4).toFixed(2)}${t === 4 ? '+' : ''}${t === 0 ? ' ' + unit : ''}</text>`;
  s += `<rect x="${x + w + 24}" y="${y}" width="14" height="10" fill="${NOFIX}" stroke="${GRID}"/><text x="${x + w + 44}" y="${y + 9}" ${font} font-size="11" fill="${INK2}">no tag in view · 看不到标签</text>`;
  return s;
}

function heatmaps() {
  const size = 330, gap = 60, ox = 40, top = 90, W = ox*2 + size*2 + gap, H = top + 3*size + 240;
  let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="${W}" height="${H}" fill="${SURFACE}"/>`;
  s += `<text x="${ox}" y="30" ${font} font-size="17" font-weight="700" fill="${INK}">Localization accuracy across the field · 全场定位精度</text>`;
  s += `<text x="${ox}" y="50" ${font} font-size="12" fill="${INK2}">Robot faces its target CELL (R ↑ / B ↑, the nearer HIVE in X) at each 0.1 m grid point · 1 px corner noise,</text>`;
  s += `<text x="${ox}" y="66" ${font} font-size="12" fill="${INK2}">${SAMPLES} samples per point, one frame, pitch estimated, CAD occlusion · top view, +Y up, dashed: HIVE frame base · 俯视图</text>`;
  PITCHES.forEach((p, i) => {
    s += fieldPanel(results[p], 'posRmsCm', 4, 'cm', ox + i*(size + gap), top + 10, size, `Position error RMS · 位置误差 — camera pitch ${p}°`);
    s += fieldPanel(results[p], 'aimRmsDeg', .8, '°', ox + i*(size + gap), top + size + 60, size, `Aim direction error RMS · 发射方向误差 — pitch ${p}°`);
  });
  PITCHES.forEach((p, i) => s += fieldPanel(results[p].map(c => ({...c, clusterLevel:c.tags ? c.clusters : null})), 'clusterLevel', 3, '', ox + i*(size + gap), top + 2*size + 110, size, `Tag clusters in view · 可见标签组数 — pitch ${p}°`, CLUSTERS));
  const by = top + 3*size + 150;
  s += colorBar(ox, by, 260, 4, 'cm', 'Position error RMS (cm) · 位置误差');
  s += colorBar(ox, by + 50, 260, .8, '°', 'Aim direction error RMS (°) · 发射方向误差');
  s += `<text x="${ox + 560}" y="${by - 6}" ${font} font-size="12" fill="${INK2}">Tag clusters in view · 可见标签组数</text>`;
  CLUSTERS.forEach((c, i) => { s += `<rect x="${ox + 560 + i*60}" y="${by}" width="14" height="10" fill="${c}"/><text x="${ox + 580 + i*60}" y="${by + 9}" ${font} font-size="11" fill="${INK2}">${i + 1}</text>` });
  return s + '</svg>';
}

function angleChart() {
  const W = 980, H = 500, panelW = 380, panelH = 280, top = 110, left = 70, gap = 110;
  let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="${W}" height="${H}" fill="${SURFACE}"/>`;
  s += `<text x="${left - 30}" y="30" ${font} font-size="17" font-weight="700" fill="${INK}">Error by viewing angle and distance · 误差随视角和距离的变化</text>`;
  s += `<text x="${left - 30}" y="50" ${font} font-size="12" fill="${INK2}">Camera pitch 25° · RMS over grid points in each 15° bin · 0° = straight in front of the target CELL's opening, ±90° = from the side.</text>`;
  s += `<text x="${left - 30}" y="${H - 14}" ${font} font-size="11" fill="${MUTED}">Angles below −30° are missing: from there the other HIVE is nearer in X and becomes the target. Values above the axis are clipped.</text>`;
  // Legend, top-left, in series order.
  bands.forEach(([d0, d1], i) => { const lx = left - 30 + i*170;
    s += `<line x1="${lx}" y1="74" x2="${lx + 22}" y2="74" stroke="${SERIES[i]}" stroke-width="2"/><circle cx="${lx + 11}" cy="74" r="4" fill="${SERIES[i]}" stroke="${SURFACE}" stroke-width="2"/>`;
    s += `<text x="${lx + 30}" y="78" ${font} font-size="12" fill="${INK}">${d0}–${d1} m from target</text>` });
  const panels = [['posRmsCm', 'Position error RMS (cm) · 位置误差', 5], ['aimRmsDeg', 'Aim direction error RMS (°) · 发射方向误差', 1]];
  panels.forEach(([key, title, max], pi) => {
    const x0 = left + pi*(panelW + gap), X = a => x0 + (a + 150)/300*panelW, Y = v => top + panelH - Math.min(v, max)/max*panelH;
    s += `<text x="${x0}" y="${top - 12}" ${font} font-size="13" font-weight="600" fill="${INK}">${esc(title)}</text>`;
    for (let t = 0; t <= 4; t++) { const v = max*t/4, y = Y(v);
      s += `<line x1="${x0}" y1="${y}" x2="${x0 + panelW}" y2="${y}" stroke="${GRID}" stroke-width="1"/><text x="${x0 - 8}" y="${y + 4}" ${font} font-size="11" fill="${INK2}" text-anchor="end">${+v.toFixed(2)}</text>` }
    for (const a of [-150, -90, 0, 90, 150]) s += `<text x="${X(a)}" y="${top + panelH + 18}" ${font} font-size="11" fill="${INK2}" text-anchor="middle">${a}°</text>`;
    for (const a of [-90, 90]) s += `<line x1="${X(a)}" y1="${top}" x2="${X(a)}" y2="${top + panelH}" stroke="${MUTED}" stroke-width="1" stroke-dasharray="3 3"/>`;
    s += `<text x="${X(0)}" y="${top + panelH + 36}" ${font} font-size="11" fill="${INK2}" text-anchor="middle">angle off the opening axis · 偏离开口正前方的角度</text>`;
    s += `<text x="${X(0)}" y="${top + panelH + 52}" ${font} font-size="11" fill="${MUTED}" text-anchor="middle">+ toward the field edge, − toward the other HIVE; beyond ±90° is behind the CELL</text>`;
    byAngle[25].forEach((series, i) => {
      const pts = series.filter(b => b[key] != null);
      // Break the line where a bin has no fix, so gaps stay visible.
      let d = '', prev = null;
      for (const b of series) { if (b[key] == null) { prev = null; continue } d += `${prev ? 'L' : 'M'}${X(b.angle).toFixed(1)},${Y(b[key]).toFixed(1)}`; prev = b }
      s += `<path d="${d}" fill="none" stroke="${SERIES[i]}" stroke-width="2" stroke-linejoin="round"/>`;
      for (const b of pts) s += `<circle cx="${X(b.angle).toFixed(1)}" cy="${Y(b[key]).toFixed(1)}" r="4" fill="${SERIES[i]}" stroke="${SURFACE}" stroke-width="2"><title>${b.angle}° · ${bands[i][0]}–${bands[i][1]} m · ${key === 'posRmsCm' ? b[key].toFixed(2) + ' cm' : b[key].toFixed(3) + '°'} · ${b.fixed}/${b.n} points see tags, ${b.tags.toFixed(1)} tags on average</title></circle>`;
    });
    s += `<line x1="${x0}" y1="${top + panelH}" x2="${x0 + panelW}" y2="${top + panelH}" stroke="${INK2}" stroke-width="1"/>`;
  });
  return s + '</svg>';
}
