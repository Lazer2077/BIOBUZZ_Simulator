# BIOBUZZ HIVE 射击与翻转模拟器

用浏览器打开 `index.html`，无需安装依赖。输入发射点、初速和角度后，页面显示三维轨迹、侧视图及碰撞判定。目标会根据发射点 X 坐标选择较近的红色或蓝色 HIVE。“自动求解角度”在当前初速下寻找无碰撞的开口轨迹；点击“发射一球”才会把命中的球计入 HIVE。

## 模型尺寸

- 坐标单位为米、秒。X 沿红蓝 HIVE 排列方向，Y 沿同一 HIVE 的两个 CELL 排列方向，Z 向上。原点是框架转轴的地面投影。
- 框架宽 49.46 in、底座深 38.95 in、转轴高 43.95 in；红蓝 HIVE 中心相距 25.5 in。
- 图 9-9 中的 18.84 in 是相邻 CELL 之间的间距，旁边还标有 CELL 厚度 12.04 in。用于显示的 CELL 由官方 STEP 直接提取肋框、透明侧板和背板三角网格，保留其真实位置与造型。
- CELL 开口宽约 20 in、高约 14 in、深约 12 in。图中尖顶以下的侧边高度为 7.61 in。根据官方 CAD 肋框底边 `(Y=0.5096, Z=1.3237) m` 与尖顶 `(Y=0.3054, Z=1.6769) m`，开口高度方向相对水平面约 60°；内侧代理开口取两者之间的净空。
- POLLEN 直径约 2.8 in、质量 0.055 lb；NECTAR 直径约 3.6 in、质量 0.091 lb。

来源：[FIRST 官方 STEP 场地 CAD](https://ftc-resources.firstinspires.org/ftc/field/field-cad-step)、[FIRST 2026–27 Competition Manual §9.6、§9.8、§10.3、§10.5](https://ftc-resources.firstinspires.org/ftc/game/manual)、[FIRST CELL 装配指南 §6](https://ftc-resources.firstinspires.org/ftc/field/initialfieldguide)、[官方动画](https://www.youtube.com/watch?v=sUH3z5a5S9I&t=200s)和[AndyMark 球规格](https://andymark.com/products/biobuzz-scoring-elements)。CAD 网格的提取脚本是 `cad/extract_mesh.py`。三维引擎使用本地打包的 Three.js，许可证见 `vendor/THREE-LICENSE.txt`。

## 物理和判定

球的自由飞行按 5 ms 步长积分重力与平方阻力 `F = ρ Cd A |v|² / 2`，空气密度取 1.2 kg/m³，迎风面积取 `πd²/4`。界面 `drag` 和 API `drag` 输入的是无量纲 `Cd`，默认 0.5；0.3–1.0 仅供同类穿孔球的灵敏度分析，并非 NECTAR 或 POLLEN 的实测范围。外部封闭侧板和背板使用官方 CAD 三角面进行球体接触检测，内部反弹也从 CAD 三角面求最近接触点与法向量。碰撞树用于加速查询；五边形代理补足 CAD 网格未封闭的边缘，并限制球从非开口侧穿出。球在腔内以 0.5 ms 步长计算，直至弹出开口、静止或到达 3 秒上限；弹簧阻尼与切向摩擦近似球的压缩和回弹。轨迹画球心，橙点画接触点。留球由腔体内位置与最终速度决定，与碰撞次数无关。初始三颗 NECTAR 用重力、壁面软接触及球间接触先行沉降；新球留在腔体后也重新沉降。测试检查初始球与 CAD 肋框、侧板、背板的净空。边缘代理仍有几何误差，上机前应按实物校验。

官方比赛初始状态下，每个向上 CELL 有 3 颗 NECTAR。模拟器据此显示初始球数，并将它们视作平衡调校的基线；后续进入的球按 AndyMark 质量增加附加力矩。附加力矩达到界面中的释放阈值后，HIVE 以转动惯量、恢复力和阻尼的简化方程下坠到另一稳定姿态；随后新 CELL 朝上并重新瞄准。翻转后模型将原 CELL 中的球移出计数。

**标定限制：** FIRST 手册没有给出翻转力矩、转动惯量、阻尼、球体接触刚度或 NECTAR 阻力系数；这些默认参数都是演示值。球体用集中质量和可压缩接触近似，尚未使用有限元计算球壳变形，也未模拟篮筐柔性。沉降中的球间接触已计算，但飞入的新球尚未与已存球共同进行动态碰撞求解，因此满载 CELL 的留球预测仍有误差。实际留球率、翻转时间与所需球数应在实物场地上测量后调整。阻力方程见 [NASA](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/)；穿孔球阻力对孔径与孔隙率敏感，见 [floorball 球实验研究](https://odr.chalmers.se/items/dc68b2a3-48e5-4090-bb6b-023005930f18)和[穿孔空心球风洞研究](https://www.cambridge.org/core/journals/journal-of-fluid-mechanics/article/drag-on-a-hollow-sphere-can-increase-with-porosity/96AC0BED872B41FDE7C447214028BA77)。

## 控制算法接口

页面提供全局 `window.HiveControl`，输入为米、米每秒、角度、牛顿米和秒。目标 HIVE 永远按发射点 X 坐标选最近的一组。接口调用会同步更新页面控件，便于检查算法结果。

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

接口方法：

- `getState()`：返回红蓝 HIVE 的向上 CELL、球数、翻转次数和当前最近目标。
- `predict({...})`：使用给定 `angleDeg`、`yawDeg` 预测一次发射；状态为 `hit`、`collision` 或 `miss`，并返回 `internalCollisions` 与 `retained`，不计入球数。
- `solve({...})`：在给定发射点、初速和球种下寻找无碰撞的角度；无解时返回 `unreachable`。
- `fire({...})`：执行当前参数的一次发射；命中后增加球数，并在达到阈值时推进 HIVE 到下一稳定状态。此 API 即时完成计算，界面按钮则播放动画。
- `reset()`：恢复官方比赛初始方向和每组 3 颗 NECTAR。

运行 `node collision.test.js` 可验证碰撞、最近 HIVE 选择、自动命中、翻转后的再次瞄准以及控制接口。
