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
console.log('Localization checks passed');
