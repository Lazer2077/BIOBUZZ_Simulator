const assert = require('node:assert/strict');
const L = require('./localization.js');

const poses = [{x:-.6,y:1.8,headingDeg:-90},{x:.6,y:-1.8,headingDeg:90},{x:1.2,y:.3,headingDeg:170},{x:0,y:1.5,headingDeg:-60}];
for (const robot of poses) {
  const exact = L.localize({robot, pixelNoise:0});
  assert.ok(exact.detections.length >= 3, 'every test pose must see several wall tags');
  assert.ok(exact.error.position < 1e-6 && Math.abs(exact.error.headingDeg) < 1e-5, 'noise-free corners must recover the exact pose: ' + JSON.stringify(exact.error));
}

const init = L.detect({robot:poses[0], pixelNoise:0});
const closedForm = L.estimate(init.detections, {maxIterations:0});
assert.ok(Math.hypot(closedForm.x-poses[0].x, closedForm.y-poses[0].y) < 1e-6, 'the closed-form edge-range initial pose must be exact without noise');

// The reported 1σ must match the actual error spread (normalized RMS near 1).
const z = [];
for (const robot of poses) for (let seed = 1; seed < 120; seed++) {
  const r = L.localize({robot, pixelNoise:.5, seed});
  z.push(r.error.x/r.estimate.std.x, r.error.y/r.estimate.std.y, r.error.headingDeg/r.estimate.std.headingDeg);
}
const nrms = Math.sqrt(z.reduce((s,v)=>s+v*v,0)/z.length);
assert.ok(nrms > .7 && nrms < 1.4, 'covariance must be calibrated, normalized RMS = ' + nrms.toFixed(2));

// Fusing frames must shrink the error.
const spread = frames => { let s = 0; for (let seed = 1; seed < 60; seed++) s += L.localize({robot:poses[2], pixelNoise:1, seed, frames}).error.position**2; return Math.sqrt(s/59) };
assert.ok(spread(8) < .6*spread(1), 'fusing 8 frames must reduce the position error');

// A tag with grossly wrong corners must be dropped as an outlier.
const noisy = L.detect({robot:poses[0], pixelNoise:.5, seed:3});
noisy.detections[1].corners = noisy.detections[1].corners.map(q => ({u:q.u+25, v:q.v-10}));
const robust = L.estimate(noisy.detections);
assert.deepEqual(robust.outlierTagIds, [noisy.detections[1].id]);
assert.ok(Math.hypot(robust.x-poses[0].x, robust.y-poses[0].y) < .03, 'the pose must stay accurate after outlier removal');

// Detector rejection reasons.
const tags = [{id:1,x:0,y:1.8,z:.3,yawDeg:-90,size:.165}];
const facing = {x:0,y:0,headingDeg:90}, camera = {pitchDeg:0};
assert.equal(L.detect({robot:facing, camera, tags}).detections.length, 1);
assert.equal(L.detect({robot:{...facing,headingDeg:-90}, camera, tags}).rejected[0].reason, 'behind');
assert.equal(L.detect({robot:{x:0,y:1.79,headingDeg:90}, camera, tags}).rejected[0].reason, 'fov');
assert.equal(L.detect({robot:{x:0,y:-3,headingDeg:90}, camera, tags}).rejected[0].reason, 'range');
assert.equal(L.detect({robot:{x:0,y:-2.6,headingDeg:90}, camera:{...camera,maxRange:9,minTagPixels:40}, tags}).rejected[0].reason, 'small');
assert.equal(L.detect({robot:facing, camera, tags:[{...tags[0],yawDeg:90}]}).rejected[0].reason, 'angle');
assert.equal(L.detect({robot:facing, camera, tags, occluded:()=>true}).rejected[0].reason, 'occluded');
assert.equal(L.estimate([]), null, 'no detections means no estimate');

// Filter: odometry prediction moves the state in the robot frame.
const f = L.createPoseFilter();
f.update({x:1, y:2, headingDeg:90, std:{x:.01,y:.01,headingDeg:.5}});
f.predict(.1, {dx:.5});
assert.ok(Math.abs(f.state.x-1) < 1e-9 && Math.abs(f.state.y-2.5) < 1e-9, 'forward odometry at heading 90° must move +Y');
// Frame tags: four, on the legs facing the shooting sides, and clear of the leg they are mounted on.
const frame = L.frameTags();
assert.deepEqual(frame.map(t => t.id), [13,14,15,16]);
assert.ok(frame.every(t => t.group === 'frame' && Math.abs(Math.abs(t.yawDeg) - 90) < 1e-9 && Math.sign(t.y) === Math.sign(t.yawDeg)), 'frame tags face away from the HIVE toward ±Y');
assert.equal(L.defaultTags().length, 16);

// Tag choice: 'auto' aims from the frame tags whenever one is seen, otherwise from everything.
const layout = L.defaultTags();
assert.deepEqual(L.selectTags(layout, 'auto', [7,8,13]).map(t => t.id), [13,14,15,16]);
assert.equal(L.selectTags(layout, 'auto', [7,8]).length, 16);
assert.deepEqual(L.selectTags(layout, 'wall', []).map(t => t.group), Array(12).fill('wall'));

// Wall tags misplaced relative to the HIVE: aiming from the frame tags is unaffected, walls alone are not.
const shooter = {x:-0.6, y:1.8, headingDeg:-78.67}, hive = {x:-25.5*.0254/2, y:.41};
const aimYawError = r => { const e = r.estimate, b = (p) => Math.atan2(hive.y-p.y, hive.x-p.x)*180/Math.PI;
  return Math.abs(((shooter.headingDeg + b(e) - e.headingDeg - b(shooter)) + 540) % 360 - 180) };
let wallWorst = 0, autoWorst = 0;
for (let seed = 1; seed <= 6; seed++) {
  const moved = L.perturbTags(layout, {sigma:.03, seed});
  assert.ok(moved.filter(t => t.group === 'frame').every((t,i) => t === frame[i] || (t.x === frame[i].x && t.y === frame[i].y)), 'frame tags are never perturbed');
  wallWorst = Math.max(wallWorst, aimYawError(L.localize({robot:shooter, tags:moved, layout, use:'wall', pixelNoise:.5, seed})));
  autoWorst = Math.max(autoWorst, aimYawError(L.localize({robot:shooter, tags:moved, layout, use:'auto', pixelNoise:.5, seed})));
}
assert.ok(autoWorst < .2, 'auto aim error must stay under 0.2° with 3 cm wall tag error: ' + autoWorst);
assert.ok(wallWorst > 5*autoWorst, `walls-only aim must be visibly worse (${wallWorst.toFixed(2)}° vs ${autoWorst.toFixed(2)}°)`);

// Quality gate: blur from the gyro rate, and a lone distant tag.
const gateCam = L.defaultCamera();
assert.equal(L.qualityGate({x:0,y:0,headingDeg:0,tagIds:[1,2]}, {camera:gateCam, turnRateDegS:10}).ok, true);
assert.deepEqual(L.qualityGate({x:0,y:0,headingDeg:0,tagIds:[1,2]}, {camera:gateCam, turnRateDegS:90}).reasons, ['blur']);
assert.deepEqual(L.qualityGate({x:0,y:0,headingDeg:0,tagIds:[8]}).reasons, ['singleFarTag']);
assert.equal(L.qualityGate({x:0,y:1.3,headingDeg:90,tagIds:[2]}).ok, true, 'a single tag 0.5 m away is trusted');

// Pixel pipeline: render the camera image and run the real AprilRobotics detector compiled to WebAssembly.
const C = require('./camera-render.js');
require('./vendor/apriltag-wasm.js')().then(Module => {
  const detector = C.createDetector(Module);
  assert.equal(detector.tagCount, 587, 'the detector must carry the full tag36h11 family');
  const p0 = detector.pattern(0);
  assert.equal(p0.width, 10);
  assert.ok([0,9].every(i => p0.data[i] === 255 && p0.data[i*10] === 255), 'tag images have a white outer ring');
  assert.ok([1,8].every(i => p0.data[10+i] === 0 && p0.data[80+i] === 0), 'tag images have a black border');

  const robot = {x:0, y:1.5, headingDeg:-60};
  const clean = C.localizeFromImage({detector, robot, effects:{noise:0}});
  assert.deepEqual(clean.detections.map(d => d.id).sort((a,b)=>a-b), [8,9,10,11,14], 'the detector must find the wall and frame tags in the rendered frame');
  const pose = L.cameraPose(robot, L.defaultCamera()), K = L.intrinsics(L.defaultCamera()), tags = L.defaultTags();
  for (const d of clean.detections) {
    const ideal = L.tagCorners(tags.find(t => t.id === d.id)).map(P => L.project(P, pose, K));
    d.corners.forEach((q,i) => assert.ok(Math.hypot(q.u-ideal[i].u, q.v-ideal[i].v) < .5, `detector corner ${i} of tag ${d.id} must match the projected corner in order`));
  }
  assert.ok(clean.error.position < .01 && Math.abs(clean.error.headingDeg) < .2, 'pixel pipeline pose error must be under 1 cm: ' + JSON.stringify(clean.error));

  const noisy = C.localizeFromImage({detector, robot, effects:{noise:4, seed:2}, frames:3});
  assert.ok(noisy.error.position < .02, 'sensor noise must keep the error within 2 cm');

  const distorted = C.localizeFromImage({detector, robot, camera:{k1:-.3}, effects:{noise:2}});
  assert.ok(distorted.detections.length >= 3 && distorted.error.position < .02, 'undistorting the corners with the known k1 must keep the pose accurate');
  const [xd, yd] = C.distort(.3, -.2, {k1:-.3, k2:.05}), [xu, yu] = C.undistort(xd, yd, {k1:-.3, k2:.05});
  assert.ok(Math.hypot(xu-.3, yu+.2) < 1e-9, 'undistort must invert distort');

  const blurred = C.localizeFromImage({detector, robot, effects:{turnRateDegS:150}});
  assert.ok(blurred.detections.every(d => d.id === 14), 'a 150°/s turn smears the distant wall tags beyond decoding; only the large nearby frame tag survives');
  assert.deepEqual(L.qualityGate(blurred.estimate, {turnRateDegS:150}).reasons, ['blur'], 'the gate must reject the blurred estimate');
  console.log('Localization checks passed (including the AprilTag image pipeline)');
});
