const assert = require('node:assert/strict');
const L = require('./localization.js');
const IN = .0254;

// Official layout (Manual §9.9): 16 tags of 3.25 in, IDs 30–45, four per CELL on its bottom face.
const tags = L.defaultTags();
assert.deepEqual(tags.map(t => t.id), Array.from({length:16}, (_,k) => 30+k));
assert.ok(tags.every(t => Math.abs(t.size - 3.25*IN) < 1e-12));
for (const [ids, hive, side] of [[[30,31,32,33],'red',-1],[[34,35,36,37],'red',1],[[38,39,40,41],'blue',1],[[42,43,44,45],'blue',-1]]) {
  const cluster = tags.filter(t => ids.includes(t.id));
  assert.ok(cluster.every(t => t.hive === hive && t.side === side), `tags ${ids} belong to the ${hive} ${side>0?'+Y':'−Y'} CELL`);
  const xs = cluster.map(t => t.x - (hive === 'red' ? -1 : 1)*25.5*IN/2).sort((a,b)=>a-b);
  [-6.5,-2.75,2.75,6.5].forEach((v,k) => assert.ok(Math.abs(xs[k] - v*IN) < 1e-9, 'Figure 9-15 tag spacing'));
}
// Figure 9-17 order along X: 33 32 31 30 | 45 44 43 42 on the far side, 34 35 36 37 | 38 39 40 41 on the audience side.
const xOf = id => tags.find(t => t.id === id).x;
for (const row of [[33,32,31,30,45,44,43,42],[34,35,36,37,38,39,40,41]]) row.slice(1).forEach((id,k) => assert.ok(xOf(id) > xOf(row[k]), `ID ${id} lies right of ${row[k]}`));
// The tags tip with their HIVE: upward CELL bottoms face down, toward the field centre.
const up = tags.filter(t => (t.hive === 'red' && t.side === 1) || (t.hive === 'blue' && t.side === -1));
assert.ok(up.every(t => t.normal.z < -.4 && Math.sign(t.normal.y) === -t.side), 'an upward CELL’s cluster faces down and toward the centre');
const tipped = L.defaultTags({red:-1, blue:-1});
assert.ok(Math.hypot(tipped[0].x-tags[0].x, tipped[0].y-tags[0].y, tipped[0].z-tags[0].z) > .1, 'tipping the red HIVE moves tag 30');
assert.ok(tipped.slice(8).every((t,k) => t.y === tags[8+k].y && t.z === tags[8+k].z), 'tipping red leaves the blue tags in place');

// Shooting poses aimed at the upward CELLs, which see a cluster on the other CELL of each HIVE.
const poses = [{x:-.6,y:1.8,headingDeg:-78.7},{x:.6,y:-1.8,headingDeg:101.3},{x:-1.2,y:1.5,headingDeg:-50.9},{x:-.3,y:1.0,headingDeg:-90}];
for (const robot of poses) {
  const exact = L.localize({robot, pixelNoise:0, layouts:L.hiveLayouts()});
  assert.ok(exact.detections.length >= 4, 'every test pose must see a cluster');
  assert.ok(exact.error.position < 1e-6 && Math.abs(exact.error.headingDeg) < 1e-5, 'noise-free corners must recover the exact pose: ' + JSON.stringify(exact.error));
  const seenHives = new Set(exact.detections.map(d => tags.find(t => t.id === d.id).hive));
  for (const [hive, truth] of Object.entries({red:1, blue:-1})) assert.equal(exact.hiveUp[hive], seenHives.has(hive) ? truth : null, `the ${hive} HIVE pose must be recovered when its tags are seen, else unknown`);
}

const init = L.detect({robot:poses[0], pixelNoise:0});
const fromOne = L.estimate(init.detections.slice(0,1), {maxIterations:0});
assert.ok(Math.hypot(fromOne.x-poses[0].x, fromOne.y-poses[0].y) < 1e-6, 'the heading-scan initial pose must be exact from a single tilted tag');

// The reported 1σ must match the actual error spread (normalized RMS near 1).
const z = [];
for (const robot of poses) for (let seed = 1; seed < 80; seed++) {
  const r = L.localize({robot, pixelNoise:.5, seed});
  z.push(r.error.x/r.estimate.std.x, r.error.y/r.estimate.std.y, r.error.headingDeg/r.estimate.std.headingDeg);
}
const nrms = Math.sqrt(z.reduce((s,v)=>s+v*v,0)/z.length);
assert.ok(nrms > .7 && nrms < 1.4, 'covariance must be calibrated, normalized RMS = ' + nrms.toFixed(2));

// Fusing frames must shrink the error.
const spread = frames => { let s = 0; for (let seed = 1; seed < 40; seed++) s += L.localize({robot:poses[2], pixelNoise:1, seed, frames}).error.position**2; return Math.sqrt(s/39) };
assert.ok(spread(8) < .6*spread(1), 'fusing 8 frames must reduce the position error');

// A tag with grossly wrong corners must be dropped as an outlier.
const noisy = L.detect({robot:poses[0], pixelNoise:.5, seed:3});
noisy.detections[1].corners = noisy.detections[1].corners.map(q => ({u:q.u+25, v:q.v-10}));
const robust = L.estimate(noisy.detections);
assert.deepEqual(robust.outlierTagIds, [noisy.detections[1].id]);
assert.ok(Math.hypot(robust.x-poses[0].x, robust.y-poses[0].y) < .03, 'the pose must stay accurate after outlier removal');

// Detector rejection reasons, on a vertical test tag.
const wall = [{id:1,x:0,y:1.8,z:.3,yawDeg:-90,size:.165}];
const facing = {x:0,y:0,headingDeg:90}, camera = {pitchDeg:0};
assert.equal(L.detect({robot:facing, camera, tags:wall}).detections.length, 1);
assert.equal(L.detect({robot:{...facing,headingDeg:-90}, camera, tags:wall}).rejected[0].reason, 'behind');
assert.equal(L.detect({robot:{x:0,y:1.79,headingDeg:90}, camera, tags:wall}).rejected[0].reason, 'fov');
assert.equal(L.detect({robot:{x:0,y:-3,headingDeg:90}, camera, tags:wall}).rejected[0].reason, 'range');
assert.equal(L.detect({robot:{x:0,y:-2.6,headingDeg:90}, camera:{...camera,maxRange:9,minTagPixels:40}, tags:wall}).rejected[0].reason, 'small');
assert.equal(L.detect({robot:facing, camera, tags:[{...wall[0],yawDeg:90}]}).rejected[0].reason, 'angle');
assert.equal(L.detect({robot:facing, camera, tags:wall, occluded:()=>true}).rejected[0].reason, 'occluded');
assert.equal(L.estimate([]), null, 'no detections means no estimate');

// Filter: odometry prediction moves the state in the robot frame.
const f = L.createPoseFilter();
f.update({x:1, y:2, headingDeg:90, std:{x:.01,y:.01,headingDeg:.5}});
f.predict(.1, {dx:.5});
assert.ok(Math.abs(f.state.x-1) < 1e-9 && Math.abs(f.state.y-2.5) < 1e-9, 'forward odometry at heading 90° must move +Y');

// Quality gate: blur from the gyro rate, and a lone distant tag.
assert.equal(L.qualityGate({x:0,y:0,headingDeg:0,tagIds:[30,31]}, {turnRateDegS:10}).ok, true);
assert.deepEqual(L.qualityGate({x:0,y:0,headingDeg:0,tagIds:[30,31]}, {turnRateDegS:90}).reasons, ['blur']);
assert.deepEqual(L.qualityGate({x:-.6,y:1.8,headingDeg:0,tagIds:[30]}).reasons, ['singleFarTag']);
assert.equal(L.qualityGate({x:-.3,y:-1.0,headingDeg:90,tagIds:[30]}).ok, true, 'a single tag within 1.5 m is trusted');

// Pixel pipeline: render the camera image and run the real AprilRobotics detector compiled to WebAssembly.
const C = require('./camera-render.js');
require('./vendor/apriltag-wasm.js')().then(Module => {
  const detector = C.createDetector(Module);
  assert.equal(detector.tagCount, 587, 'the detector must carry the full tag36h11 family');
  const p0 = detector.pattern(0);
  assert.equal(p0.width, 10);
  assert.ok([0,9].every(i => p0.data[i] === 255 && p0.data[i*10] === 255), 'tag images have a white outer ring');
  assert.ok([1,8].every(i => p0.data[10+i] === 0 && p0.data[80+i] === 0), 'tag images have a black border');

  const robot = poses[0], layouts = L.hiveLayouts();
  const clean = C.localizeFromImage({detector, robot, layouts, effects:{noise:0}, detectorOptions:{decimate:1}});
  assert.ok(clean.detections.length >= 4, 'the detector must find a cluster in the rendered frame: ' + clean.detections.map(d => d.id));
  const pose = L.cameraPose(robot, L.defaultCamera()), K = L.intrinsics(L.defaultCamera());
  for (const d of clean.detections) {
    const ideal = L.tagCorners(tags.find(t => t.id === d.id)).map(P => L.project(P, pose, K));
    d.corners.forEach((q,i) => assert.ok(Math.hypot(q.u-ideal[i].u, q.v-ideal[i].v) < .6, `detector corner ${i} of tag ${d.id} must match the projected corner in order`));
  }
  assert.ok(clean.error.position < .015 && Math.abs(clean.error.headingDeg) < .3, 'pixel pipeline pose error must be about 1 cm: ' + JSON.stringify(clean.error));
  assert.equal(clean.hiveUp.blue, -1, 'the blue cluster in view must reveal the blue HIVE pose');

  const coarse = C.localizeFromImage({detector, robot, layouts, effects:{noise:0}, detectorOptions:{decimate:2}});
  assert.ok(coarse.detections.length < clean.detections.length, '3.25 in tags at 2 m are too small for decimate 2 at 640×480');
  const hd = C.localizeFromImage({detector, robot, layouts, camera:{imageWidth:1280, imageHeight:800}, effects:{supersample:1}, detectorOptions:{decimate:2}});
  assert.ok(hd.detections.length >= 8 && hd.error.position < .015, '1280×800 with decimate 2 must see both clusters');

  const noisy = C.localizeFromImage({detector, robot, layouts, effects:{noise:4, seed:2}, detectorOptions:{decimate:1}, frames:3});
  assert.ok(noisy.error.position < .03, 'sensor noise must keep the error within 3 cm');

  const distorted = C.localizeFromImage({detector, robot, layouts, camera:{k1:-.2}, effects:{noise:2}, detectorOptions:{decimate:1}});
  assert.ok(distorted.detections.length >= 4 && distorted.error.position < .03, 'undistorting the corners with the known k1 must keep the pose accurate');
  const [xd, yd] = C.distort(.3, -.2, {k1:-.3, k2:.05}), [xu, yu] = C.undistort(xd, yd, {k1:-.3, k2:.05});
  assert.ok(Math.hypot(xu-.3, yu+.2) < 1e-9, 'undistort must invert distort');

  const blurred = C.localizeFromImage({detector, robot, layouts, effects:{turnRateDegS:150}, detectorOptions:{decimate:1}});
  assert.equal(blurred.detections.length, 0, 'a 150°/s turn over an 8 ms exposure smears the small CELL tags beyond decoding');
  console.log('Localization checks passed (including the AprilTag image pipeline)');
});
