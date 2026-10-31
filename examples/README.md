# Example: one camera frame to a pose and an aim

**English** | [中文](README.zh-CN.md)

```bash
node examples/solve-example.js
```

The script runs the whole pipeline once, outside the page:

1. **Scenario:** the red shooter at (−0.6, 1.8) m, heading −78.7°, faces its upward CELL. The 640×480, 70° camera is 0.30 m up. It was calibrated at 10° pitch, but the chassis is tilted 2° nose-up, so the real camera looks up at 12°. Sensor noise is σ = 3 gray levels.
2. **Render** the frame the camera sees: tiles, walls, the official CELL tags (Manual §9.9), the CELL panels and the HIVE frame.
3. **Detect** with the AprilRobotics detector compiled to WebAssembly (`decimate` 1), keeping only exact decodes with a decision margin of at least 10.
4. **Solve** for `(x, y, heading, pitch)`, with the pitch held to its 10° calibration by a ±3° prior. The solve is tried against all four HIVE-pose layouts, and three frames are fused.
5. **Check** the result with the quality gate, and compute what it means for the shot: the turn to the opening centre and the launch-direction error.

Outputs in [`output/`](output):

- [`camera.png`](output/camera.png): the frame as the detector saw it. Green: detected corners, with the tag ID in yellow. Magenta crosses: the tag corners reprojected from the estimated pose and pitch.
- [`camera-zoom.png`](output/camera-zoom.png): the same, 4× around the tags.
- [`result.json`](output/result.json): every detection with its corners, the estimate with 1σ, the error against the truth, the aim and the gate.
- [`console.txt`](output/console.txt): the printed report.

![Camera frame](output/camera.png)

![Tags at 4×](output/camera-zoom.png)

```
Robot (true)      x=-0.6 m  y=1.8 m  heading=-78.7°
Camera            640x480, HFOV 70°, height 0.3 m, pitch calibrated 10° / actual 12°
Detected tags     31(m=137.1) 32(m=133.3) 33(m=147.1) 34(m=164.8) 35(m=151.5) 36(m=157.6) 37(m=150.6)
Upward CELLs      red +Y, blue not seen (from the best-fitting layout)
Estimate          x=-0.5998 m  y=1.7998 m  heading=-78.708°  pitch=12.021°
1σ                x 0.18 cm, y 0.17 cm, heading 0.073°;  reprojection RMS 0.065 px, 3 frames
Error             position 0.03 cm, heading -0.008°
Aim               turn -0.08°, launch direction error 0.002° (0.01 cm sideways), range error -0.03 cm
Quality gate      passed (motion blur 0.0 px)
Fixed-pitch solve position error 12 cm, heading -0.44° (pitch held at the calibration)
```

How to read it:

- **What is in view:** seven tags decode, 34–37 under the target CELL and 31–33 under red's lower CELL. Tag 30 does not decode because a frame leg (1 in tube) passes in front of two of its corners: the sight lines to them pass 0.1 cm and 1.6 cm from the leg's axis, against a 1.27 cm tube radius.
- **HIVE pose:** the layout that fits best has red's +Y CELL up, which is the true pose. Blue is not in view, so its pose stays unknown.
- **Pitch:** the solver finds 12.02° although it was told 10°. The same detections solved with the pitch fixed at 10° land 12 cm off.
- **Simulated conditions:** the near-zero error reflects ideal lighting, a known calibration (apart from pitch) and a standing robot. Change the scenario at the top of the script to try other poses, tilts, noise, resolutions or motion blur (`effects.turnRateDegS`).

## Accuracy across the field

```bash
node examples/accuracy-grid.js              # about 2 minutes
node examples/accuracy-grid.js --charts-only # redraw from output/accuracy-grid.json
```

Setup:
- **Grid:** every 0.1 m point on the field (1201 points outside the HIVE footprint). At each point the robot faces its target, the upward CELL of the HIVE nearer in X.
- **Camera:** 640×480, 70°, 0.30 m high, at 10° or 25° pitch.
- **Noise and occlusion:** 1 px corner noise, 16 samples per point, one frame, pitch estimated with a ±3° prior. Occlusion uses the HIVE frame and the official CAD CELL skins.

Hover a cell or a point in the SVGs for its numbers. The raw data is in [`output/accuracy-grid.json`](output/accuracy-grid.json).

![Accuracy heatmaps](output/accuracy-heatmaps.svg)

![Error by angle and distance](output/accuracy-by-angle.svg)

| Camera pitch | Points with a tag in view | Position error RMS: median / 90th percentile | Aim direction error RMS: median / 90th percentile |
|---|---|---|---|
| 10° | 1168 / 1201 | 1.42 / 4.8 cm | 0.28° / 0.72° |
| 25° | 1183 / 1201 | 0.46 / 2.7 cm | 0.05° / 0.42° |

**Is the error angle-dependent?** Yes, but mostly indirectly. Angle and distance decide **how many tag clusters are in view**, and that is what sets the error:

| Clusters in view (pitch 25°) | Grid points | Position error median | Aim error median |
|---|---|---|---|
| 1 | 208 | 2.8 cm | 0.46° |
| 2 | 903 | 0.42 cm | 0.05° |
| 3 | 72 | 0.26 cm | 0.05° |

With two clusters in view the error is nearly flat in angle. At 1.2–1.8 m, from −10° to +110° off the opening axis, it stays at 0.42–0.56 cm and 0.05°. The error grows in three places:

- **Right in front of or under the target CELL** (less than about 0.7 m horizontally): the target's own cluster is above the frame, and only 2–3 tags of the far cluster remain. Aim error is 1.3–2°.
- **Behind the target CELL** (beyond about 115°) **and far away**: only one distant cluster is in view, at a steep angle. At 1.2–1.8 m the position error is 4.5–5.6 cm; beyond 1.8 m it is 13–20 cm, worst in the field corners (up to 31 cm with a single tag). Aim error is 0.3–0.6°.
- **With a 10° camera pitch:** a ring 0.8–1.6 m from each target, where the target's cluster has left the top of the frame. A 25° pitch removes most of it.

**What this means for shooting:** shoot from about 1.2–1.8 m, within roughly ±100° of the opening axis, with the camera at about 25°. Treat estimates from a single cluster with more caution: the quality gate passes them, but their error is 5–10 times larger.
