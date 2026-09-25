# BIOBUZZ HIVE Shot and Tip Simulator

**English** | [中文](README.zh-CN.md)

Live version: <https://lazer2077.github.io/BIOBUZZ_Simulator/> (GitHub Pages; `.github/workflows/pages.yml` runs the tests and then deploys). To run locally, open `index.html` in a browser; nothing needs to be installed. The button in the top-right corner switches between Chinese and English, and the choice is remembered in the browser. You can also force a language with `?lang=en` or `?lang=zh`; otherwise the browser language is used.

Enter a launch point, speed and angles, and the page shows the 3D trajectory, a side view and the collision result. The target is whichever HIVE, red or blue, is closer in X to the launch point. **Solve angle** searches for a collision-free trajectory into the opening at the current launch speed; only **Launch one ball** adds a scoring ball to the HIVE.

## Model dimensions

- Units are metres and seconds. X runs between the red and blue HIVEs, Y between the two CELLs of one HIVE, and Z is up. The origin is the floor projection of the frame pivot.
- Frame 49.46 in wide, base 38.95 in deep, pivot 43.95 in high; red and blue HIVE centres 25.5 in apart.
- The 18.84 in in Figure 9-9 is the spacing between adjacent CELLs; the CELL thickness beside it is 12.04 in. The displayed CELLs use rib, clear side-panel and back-panel triangle meshes extracted directly from the official STEP file, keeping their real position and shape.
- The CELL opening is about 20 in wide, 14 in high and 12 in deep. The side edges below the apex are 7.61 in. From the official CAD rib bottom edge `(Y=0.5096, Z=1.3237) m` and apex `(Y=0.3054, Z=1.6769) m`, the opening's height direction is about 60° from horizontal; the inner proxy opening uses the clearance between them.
- POLLEN is about 2.8 in in diameter and 0.055 lb; NECTAR is about 3.6 in and 0.091 lb.

Sources: [FIRST official STEP field CAD](https://ftc-resources.firstinspires.org/ftc/field/field-cad-step), [FIRST 2026–27 Competition Manual §9.6, §9.8, §10.3, §10.5](https://ftc-resources.firstinspires.org/ftc/game/manual), [FIRST CELL assembly guide §6](https://ftc-resources.firstinspires.org/ftc/field/initialfieldguide), the [official animation](https://www.youtube.com/watch?v=sUH3z5a5S9I&t=200s) and [AndyMark ball specifications](https://andymark.com/products/biobuzz-scoring-elements). The CAD mesh is extracted by `cad/extract_mesh.py`. The 3D engine is a bundled copy of Three.js; its license is in `vendor/THREE-LICENSE.txt`.

## Physics and scoring logic

Free flight integrates gravity and quadratic drag `F = ρ Cd A |v|² / 2` in 5 ms steps, with air density 1.2 kg/m³ and frontal area `πd²/4`. The `drag` input in the page and the API is the dimensionless `Cd`, default 0.5; 0.3–1.0 is for sensitivity analysis on similar perforated balls and is not a measured range for NECTAR or POLLEN. The closed outer side and back panels use the official CAD triangles for sphere contact, and interior bounces also take the nearest contact point and normal from the CAD triangles. A bounding-volume tree speeds up queries; a pentagon proxy fills the unclosed edges of the CAD mesh and stops balls leaving through anything but the opening. Inside the CELL the ball is simulated in 0.5 ms steps until it bounces out of the opening, comes to rest or reaches a 3 s limit; spring-damper contact with tangential friction approximates ball compression and rebound. The trajectory shows the ball centre and orange dots show contact points. Retention depends on the ball's final position and speed inside the CELL, not on the number of contacts. The three starting NECTAR first settle under gravity, soft wall contact and ball-to-ball contact; a newly retained ball is settled again with them. The tests check the clearance of the starting balls from the CAD ribs, side panels and back panel. The edge proxy still has geometric error; check against a real field before relying on it on a robot.

At the start of an official match each upward CELL holds 3 NECTAR, placed against the back wall in a line along the side nearest that alliance's ALLIANCE AREA (manual §10.3.1: red toward −X, blue toward +X). The simulator shows these starting balls and treats them as the balance baseline; each later ball adds torque based on its AndyMark mass. When the added torque reaches the release threshold set in the page, the HIVE falls to its other stable pose following a simplified equation with moment of inertia, restoring force and damping; the other CELL then faces up and becomes the new target. After a tip, the balls in the original CELL are removed from the count.

**Calibration limits:** the FIRST manual does not give the tip torque, moment of inertia, damping, ball contact stiffness or NECTAR drag coefficient; all of these defaults are placeholders. Balls are modelled as lumped masses with compressible contact; shell deformation is not solved with finite elements and CELL flexibility is not modelled. Ball-to-ball contact is included while settling, but an incoming ball does not yet collide dynamically with balls already in the CELL, so retention predictions for a full CELL are still approximate. Measure real retention rates, tip timing and the number of balls needed on a physical field and adjust the parameters. See [NASA](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/) for the drag equation; drag on perforated balls is sensitive to hole size and porosity, see the [floorball ball experiments](https://odr.chalmers.se/items/dc68b2a3-48e5-4090-bb6b-023005930f18) and the [wind-tunnel study of perforated hollow spheres](https://www.cambridge.org/core/journals/journal-of-fluid-mechanics/article/drag-on-a-hollow-sphere-can-increase-with-porosity/96AC0BED872B41FDE7C447214028BA77).

## Control API

The page exposes a global `window.HiveControl`. Inputs are in metres, metres per second, degrees, newton-metres and seconds. The target is always the HIVE nearest the launch point in X. API calls also update the page controls so you can inspect the result.

```js
const solution = HiveControl.solve({
  x: -0.6, y: 1.8, z: 0.55, speed: 6.5,
  ballType: 'pollen', drag: 0.5
});
if (solution.status === 'hit') {
  console.log(solution.angleDeg, solution.yawDeg, solution.hitPoint);
  const outcome = HiveControl.fire();
  console.log(outcome.tipped, outcome.state.hives);
}
```

Methods:

- `getState()`: the upward CELL, ball counts and tip count for each HIVE, and the current nearest target.
- `predict({...})`: predicts one launch with the given `angleDeg` and `yawDeg`; status is `hit`, `collision` or `miss`, with `internalCollisions` and `retained`. Nothing is added to the ball count.
- `solve({...})`: searches for a collision-free angle for the given launch point, speed and ball type; returns `unreachable` if none exists.
- `fire({...})`: performs one launch with the current parameters; a hit adds the ball, and the HIVE advances to its next stable pose once the threshold is reached. The API computes this instantly; the page button animates it.
- `reset()`: restores the official starting orientation with 3 NECTAR per HIVE.

Collision `kind` values returned by the API (`封闭背板` closed back panel, `侧面` side panel, `开口边框` opening rim, `内壁 n` inner wall n) are fixed identifiers and do not change with the page language.

Run `node collision.test.js` to check collisions, nearest-HIVE selection, auto-aiming, re-aiming after a tip, the control API and the Chinese and English page text.

## Check against official sources (2026-27 Competition Manual TU02)

| Item | Official value | Simulator | Result |
|---|---|---|---|
| Frame width / base depth / pivot height | 49.46 / 38.95 / 43.95 in (§9.6.1) | Same | ✓ |
| Red–blue HIVE centre spacing | 25.5 in (Fig. 9-10) | 25.5 in | ✓ |
| Stable poses | Arm ±30° from horizontal, 60° per tip (Fig. 9-10) | 60° | ✓ |
| CELL opening | About 20 × 14 × 12 in (§9.6.2) | Same, plus official STEP mesh | ✓ |
| Opening bottom / top above tiles | 53.5 / 65.6 in (Fig. 9-10) | CAD rib outer edge 52.1 / 66.0 in | Within the "approximately" tolerance |
| POLLEN / NECTAR | 2.8 in, 0.055 lb / 3.6 in, 0.091 lb (§9.8, AndyMark) | Same | ✓ |
| Starting state | Red audience-side CELL up, blue far-side CELL up, 3 NECTAR each (Fig. 10-2) | Same | ✓ |
| Tip / retained-ball points | TIP 20 points; 2 points per ball left in an upward CELL at match end (Table 10-2) | Original CELL emptied after a tip | ✓ |

Fixed issues:

1. **Downward CELL proxy geometry was misplaced by about 19 cm.** In the official CAD each HIVE is mirror-symmetric about the vertical plane through its pivot (both CELLs sit on the same side of the arm), but the original code built the downward CELL by point symmetry about the pivot. As a result, after a tip the opening the solver aimed at, the opening-plane crossing test and the in-CELL coordinate frame were all about 19 cm (about 26° around the pivot) away from the real CAD CELL, and the downward CELL's opening rim caused false collisions in empty space. The downward CELL is now the same side's upward pose rotated 60° about the pivot; all four CELLs match the CAD within 1 cm in both stable poses, and `collision.test.js` has a regression check.
2. **Starting NECTAR placement.** They were centred; they now sit toward the alliance's own side as the manual specifies.

Still assumed and needing measurement (see "Calibration limits" above): the tip threshold (the default 0.30 N·m means about 4 POLLEN on top of the 3 NECTAR), drag coefficient, contact stiffness and damping, and tip dynamics. Also, the simulator picks the HIVE nearest the launch point in X, but in a match each alliance may only launch into the HIVE of its own colour (§10.5.1, G417), so keep the launch point on your alliance's HIVE side.
