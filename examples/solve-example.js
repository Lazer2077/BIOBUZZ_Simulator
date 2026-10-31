// Worked example: one robot camera frame, from rendering to the pose and the aim.
// Run from the repository root:  node examples/solve-example.js
// Writes examples/output/camera.png (the frame the detector saw, annotated), camera-zoom.png (the tags at 4×) and
// examples/output/result.json.
const fs = require('node:fs');
const path = require('node:path');
const zlib = require('node:zlib');
const L = require('../localization.js');
const C = require('../camera-render.js');

const IN = .0254, DEG = 180/Math.PI;
const out = path.join(__dirname, 'output');

// 1. Scenario. The red shooter stands on the +Y side and faces its upward CELL. The camera was calibrated at 10° pitch,
//    but the chassis is tilted 2° nose-up, so the real camera looks up at 12°.
const robot = {x:-0.6, y:1.8, headingDeg:-78.7};
const hiveUp = {red:1, blue:-1};                      // official start: red +Y CELL and blue −Y CELL up
const calibrated = {...L.defaultCamera(), imageWidth:640, imageHeight:480, hfovDeg:70, mountHeight:.30, pitchDeg:10};
const actual = {...calibrated, pitchDeg:12};
const effects = {noise:3, exposureMs:8, seed:7};      // sensor noise in gray levels; robot standing still
const detectorOptions = {decimate:1};                 // 3.25 in tags need full resolution at 640×480

// 2. Scene the camera sees besides the tags: CELL back, bottom and side panels and the frame legs and crossbar,
//    built from the same CELL frames the tag layout uses (opening 20 × 14 in, 12 in deep).
const W = 20*IN, H = 14*IN, D = 12*IN, shoulder = -H/2 + 7.61*IN, pivotZ = 43.95*IN, frameW = 49.46*IN, frameD = 38.95*IN;
const outline = [[-W/2,-H/2],[W/2,-H/2],[W/2,shoulder],[0,H/2],[-W/2,shoulder]];
const at = (o, a, ka, b, kb) => ({x:o.x + a.x*ka + b.x*kb, y:o.y + a.y*ka + b.y*kb, z:o.z + a.z*ka + b.z*kb});
const polygons = [];
for (const name of ['red','blue']) for (const s of [-1,1]) {
  const f = L.cellFrame(name, s, hiveUp[name]), X = {x:1, y:0, z:0}, albedo = name === 'red' ? .42 : .36;
  const front = outline.map(([u,v]) => at(f.origin, X, u, f.tangent, v)), back = front.map(p => at(p, f.normal, -D, X, 0));
  polygons.push({points:back, albedo});
  front.forEach((p,i) => { const j = (i+1) % front.length, inset = i === 0 ? L.BOTTOM_INSET : 0;
    polygons.push({points:[p, front[j], back[j], back[i]].map(q => at(q, f.tangent, inset, X, 0)), albedo:albedo + .08}) });
}
const pivot = {x:0, y:0, z:pivotZ}, leg = .5*IN;
const cylinders = [-1,1].flatMap(sx => [-1,1].map(sy => ({a:{x:sx*frameW/2, y:sy*frameD/2, z:0}, b:pivot, radius:leg, albedo:.75})));
cylinders.push({a:{x:-frameW/2, y:0, z:pivotZ}, b:{x:frameW/2, y:0, z:pivotZ}, radius:leg, albedo:.8});

require('../vendor/apriltag-wasm.js')().then(Module => {
  const detector = C.createDetector(Module);
  const tags = L.defaultTags(hiveUp);

  // 3. Render, detect, undistort, try the four HIVE poses, solve (x, y, heading, pitch), fuse 3 frames.
  const r = C.localizeFromImage({detector, robot, camera:actual, solverCamera:calibrated, pitchPriorDeg:3,
    tags, layouts:L.hiveLayouts(), polygons, cylinders, effects, detectorOptions, frames:3});
  const e = r.estimate;
  const gate = L.qualityGate(e, {camera:calibrated, tags});

  // 4. What the pose error does to the shot: aim at the opening centre of the upward red CELL.
  const target = L.cellFrame('red', hiveUp.red, hiveUp.red).origin;
  const bearing = p => Math.atan2(target.y - p.y, target.x - p.x)*DEG;
  const range = p => Math.hypot(target.x - p.x, target.y - p.y);
  const turn = bearing(e) - e.headingDeg;              // what the robot believes it must turn
  const yawErr = ((robot.headingDeg + turn - bearing(robot)) % 360 + 540) % 360 - 180;

  // The same frame solved the old way, with the pitch fixed at its calibration, for comparison.
  const fixed = L.estimate(r.detections, {camera:calibrated, tags});

  const result = {
    scenario:{robot, camera:{resolution:`${actual.imageWidth}x${actual.imageHeight}`, hfovDeg:actual.hfovDeg, heightM:actual.mountHeight,
      calibratedPitchDeg:calibrated.pitchDeg, actualPitchDeg:actual.pitchDeg}, effects, detectorOptions, hiveUp},
    timing:{renderMs:+r.timing.renderMs.toFixed(0), detectMsPerFrame:+r.timing.detectMs.toFixed(1)},
    detections:r.detections.map(d => ({id:d.id, decisionMargin:+d.decisionMargin.toFixed(1), hamming:d.hamming,
      corners:d.rawCorners.map(q => [+q.u.toFixed(2), +q.v.toFixed(2)])})),
    discarded:r.rejected.map(d => ({id:d.id, reason:d.reason})),
    estimate:{x:+e.x.toFixed(4), y:+e.y.toFixed(4), headingDeg:+e.headingDeg.toFixed(3), pitchDeg:+e.pitchDeg.toFixed(3),
      std:{xCm:+(e.std.x*100).toFixed(2), yCm:+(e.std.y*100).toFixed(2), headingDeg:+e.std.headingDeg.toFixed(3)},
      reprojectionRmsPx:+e.rmsPx.toFixed(3), tagsUsed:e.tagIds, outliers:e.outlierTagIds, framesFused:r.framesUsed, hiveUp:r.hiveUp},
    error:{positionCm:+(r.error.position*100).toFixed(2), headingDeg:+r.error.headingDeg.toFixed(3)},
    aim:{target:{x:+target.x.toFixed(3), y:+target.y.toFixed(3), z:+target.z.toFixed(3)}, robotTurnDeg:+turn.toFixed(2),
      launchDirectionErrorDeg:+yawErr.toFixed(3), sidewaysMissCm:+(range(robot)*Math.sin(yawErr/DEG)*100).toFixed(2),
      rangeErrorCm:+((range(e) - range(robot))*100).toFixed(2)},
    gate,
    withFixedPitch:{positionErrorCm:+(Math.hypot(fixed.x - robot.x, fixed.y - robot.y)*100).toFixed(1),
      headingErrorDeg:+(fixed.headingDeg - robot.headingDeg).toFixed(2)},
  };
  fs.mkdirSync(out, {recursive:true});
  // Pixel pairs on one line keep the corner lists readable.
  fs.writeFileSync(path.join(out, 'result.json'), JSON.stringify(result, null, 2).replace(/\[\s+(-?[\d.]+),\s+(-?[\d.]+)\s+\]/g, '[$1, $2]') + '\n');
  const image = annotate(r, e, calibrated, tags);
  fs.writeFileSync(path.join(out, 'camera.png'), png(image.rgb, image.w, image.h));
  fs.writeFileSync(path.join(out, 'camera-zoom.png'), zoom(image, r.detections, 4));
  printReport(result);
});

// The frame as the detector saw it, with the detections (green, raw corners and ID) and the tag corners reprojected
// from the estimated pose and pitch (magenta crosses), so the fit can be judged by eye.
function annotate(r, e, calibrated, tags) {
  const {width:w, height:h, gray} = r.frame, rgb = Buffer.alloc(w*h*3);
  for (let i = 0; i < w*h; i++) rgb[3*i] = rgb[3*i+1] = rgb[3*i+2] = gray[i];
  const put = (x, y, c) => { x = Math.round(x); y = Math.round(y); if (x >= 0 && y >= 0 && x < w && y < h) rgb.set(c, 3*(y*w + x)) };
  const line = (a, b, c) => { const n = Math.ceil(Math.hypot(b.u-a.u, b.v-a.v)*2) + 1; for (let k = 0; k <= n; k++) put(a.u + (b.u-a.u)*k/n, a.v + (b.v-a.v)*k/n, c) };
  const green = [97,229,160], magenta = [255,79,216], yellow = [255,224,102];
  for (const d of r.detections) {
    d.rawCorners.forEach((q,i) => line(q, d.rawCorners[(i+1)%4], green));
    const top = d.rawCorners.reduce((a,q) => q.v < a.v ? q : a);
    text(String(d.id), top.u - 6, top.v - 10, yellow, put);
  }
  const K = L.intrinsics(calibrated);
  const est = L.cameraPose({x:e.x, y:e.y, headingDeg:e.headingDeg}, {...calibrated, pitchDeg:e.pitchDeg});
  for (const d of r.detections) for (const P of L.tagCorners(tags.find(t => t.id === d.id))) {
    const q = L.project(P, est, K); if (!q) continue;
    const [xd, yd] = C.distort((q.u-K.cx)/K.fx, (q.v-K.cy)/K.fy, calibrated), u = xd*K.fx + K.cx, v = yd*K.fy + K.cy;
    for (let k = -3; k <= 3; k++) { put(u+k, v+k, magenta); put(u+k, v-k, magenta) }
  }
  return {rgb, w, h};
}

// Nearest-neighbour enlargement of the region around the detections, so single pixels and the fit are visible.
function zoom({rgb, w, h}, detections, k) {
  const pts = detections.flatMap(d => d.rawCorners), pad = 14;
  const x0 = Math.max(0, Math.floor(Math.min(...pts.map(q => q.u))) - pad), x1 = Math.min(w, Math.ceil(Math.max(...pts.map(q => q.u))) + pad);
  const y0 = Math.max(0, Math.floor(Math.min(...pts.map(q => q.v))) - pad - 12), y1 = Math.min(h, Math.ceil(Math.max(...pts.map(q => q.v))) + pad);
  const zw = (x1 - x0)*k, zh = (y1 - y0)*k, z = Buffer.alloc(zw*zh*3);
  for (let y = 0; y < zh; y++) for (let x = 0; x < zw; x++) rgb.copy(z, 3*(y*zw + x), 3*((y0 + Math.floor(y/k))*w + x0 + Math.floor(x/k)), 3*((y0 + Math.floor(y/k))*w + x0 + Math.floor(x/k)) + 3);
  return png(z, zw, zh);
}

// 3×5 pixel digits, drawn at 2× so tag IDs are readable in the PNG.
const DIGITS = ['111101101101111','010110010010111','111001111100111','111001111001111','101101111001001','111100111001111','111100111101111','111001001001001','111101111101111','111101111001111'];
function text(s, x, y, c, put) {
  [...s].forEach((ch, k) => { const g = DIGITS[+ch]; for (let j = 0; j < 5; j++) for (let i = 0; i < 3; i++) if (g[j*3+i] === '1')
    for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) put(x + k*8 + i*2 + a, y + j*2 + b, c) });
}

function png(rgb, w, h) {
  const crcTable = Array.from({length:256}, (_, n) => { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; return c >>> 0 });
  const crc = buf => { let c = ~0; for (const b of buf) c = crcTable[(c ^ b) & 255] ^ (c >>> 8); return ~c >>> 0 };
  const chunk = (type, data) => { const len = Buffer.alloc(4), body = Buffer.concat([Buffer.from(type), data]), sum = Buffer.alloc(4);
    len.writeUInt32BE(data.length); sum.writeUInt32BE(crc(body)); return Buffer.concat([len, body, sum]) };
  const raw = Buffer.alloc((w*3 + 1)*h); for (let y = 0; y < h; y++) rgb.copy(raw, y*(w*3 + 1) + 1, y*w*3, (y+1)*w*3);
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 2;
  return Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]);
}

function printReport(r) {
  const s = r.scenario, e = r.estimate;
  console.log(`Robot (true)      x=${s.robot.x} m  y=${s.robot.y} m  heading=${s.robot.headingDeg}°`);
  console.log(`Camera            ${s.camera.resolution}, HFOV ${s.camera.hfovDeg}°, height ${s.camera.heightM} m, pitch calibrated ${s.camera.calibratedPitchDeg}° / actual ${s.camera.actualPitchDeg}°`);
  console.log(`Detected tags     ${r.detections.map(d => `${d.id}(m=${d.decisionMargin})`).join(' ')}   render ${r.timing.renderMs} ms, detect ${r.timing.detectMsPerFrame} ms/frame`);
  if (r.discarded.length) console.log(`Discarded decodes ${r.discarded.map(d => `${d.id}:${d.reason}`).join(' ')}`);
  const side = v => v == null ? 'not seen' : v > 0 ? '+Y' : '−Y';
  console.log(`Upward CELLs      red ${side(e.hiveUp.red)}, blue ${side(e.hiveUp.blue)} (from the best-fitting layout)`);
  console.log(`Estimate          x=${e.x} m  y=${e.y} m  heading=${e.headingDeg}°  pitch=${e.pitchDeg}°`);
  console.log(`1σ                x ${e.std.xCm} cm, y ${e.std.yCm} cm, heading ${e.std.headingDeg}°;  reprojection RMS ${e.reprojectionRmsPx} px, ${e.framesFused} frames`);
  console.log(`Error             position ${r.error.positionCm} cm, heading ${r.error.headingDeg}°`);
  console.log(`Aim               turn ${r.aim.robotTurnDeg}°, launch direction error ${r.aim.launchDirectionErrorDeg}° (${r.aim.sidewaysMissCm} cm sideways), range error ${r.aim.rangeErrorCm} cm`);
  console.log(`Quality gate      ${r.gate.ok ? 'passed' : 'rejected: ' + r.gate.reasons.join(', ')} (motion blur ${r.gate.blurPx.toFixed(1)} px)`);
  console.log(`Fixed-pitch solve position error ${r.withFixedPitch.positionErrorCm} cm, heading ${r.withFixedPitch.headingErrorDeg}° (pitch held at the calibration)`);
  console.log(`Wrote examples/output/camera.png, camera-zoom.png and result.json`);
}
