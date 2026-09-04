# SRS 7R 双机械臂工具

本项目包含与实机固件一致的 7R 双臂 IK、桌面 MuJoCo 检查工具，以及供后续界面使用的 Three.js + WebAssembly 核心库。

## Web 交互 UI 与核心库

`web/` 提供开箱即用的专业 Web 3D 机械臂控制台 UI（浅色高精度实验室风格）与可复用 TypeScript 核心库。

核心结构：

```text
Web 交互 UI（Light Studio 控制台）
  ├─ RoboArmController
  │   ├─ Joint / Cartesian / Delta 命令
  │   ├─ LaTeX f(t) 采样
  │   ├─ 历史修改与下游 IK 重算
  │   ├─ 播放插值数据
  │   └─ performance-action-sequence-v2 导入/导出
  ├─ ArmRenderer（Three.js 浅色实验室刚体与视角控制）
  └─ Web Worker
      └─ AdaptiveHybrid IK WASM
```

浏览器中没有 MuJoCo、动力学、Python 后端、数据库或 WebSocket。IK/FK 和关节链位置全部来自同一份 C++ 实机算法；TypeScript 不重复实现运动学。

### 构建与运行

在项目根目录构建 WASM：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\wasm\build_wasm.ps1
```

该脚本使用项目内 `.emscripten-env`，缓存写入 `.emscripten-cache`，输出以下可部署文件：

- `web/public/wasm/deploy_ik.js`
- `web/public/wasm/deploy_ik.wasm`

运行与构建 Web 控制台：

```powershell
Set-Location .\web
npm install
npm run dev     # 启动 Web UI 本地交互服务 (默认 http://localhost:5173)
npm run build   # 打包 Web 单页应用与 web/dist/roboarm-web-core.js 核心库
npm test        # 运行运动学与动作序列单元测试
```

生成的单页应用位于 `web/dist/index.html`，核心库位于 `web/dist/roboarm-web-core.js`。`dist`、`node_modules` 和所有本地缓存均不提交。

WASM 基址按页面 URL 解析：开发环境的 `./wasm/` 对应
`http://localhost:5173/wasm/`，部署到子目录时会自动对应同级的
`wasm/`。不要把它写成 `src/wasm/`；`src` 是源码模块目录，静态文件来自
`web/public/wasm/`。

### API 示例

```ts
import { ArmRenderer, RoboArmController } from './roboarm-web-core.js';

const controller = await RoboArmController.create('/wasm/');
const renderer = new ArmRenderer(canvasContainer, controller.ik);
controller.subscribe(state => renderer.update(state.current));

// 关节角输入单位为 deg。
controller.commandJoints({
  left:  [0, -30, 0, 30, 0, 0, 0],
  right: [0, -30, 0, 30, 0, 0, 0],
}, { durationMs: 2000 });

// 位姿为 X/Y/Z(mm) + Yaw/Pitch/Roll(deg)。
controller.commandCartesian({
  left:  [205, 249, 0, 30, 0, -90],
  right: [205, -249, 0, -30, 0, 90],
});

// Delta 位置为 mm，Delta 姿态为 deg，姿态增量在当前末端局部系右乘。
controller.commandDelta({
  left:  [10, 0, 0, 0, 0, 5],
  right: [10, 0, 0, 0, 0, -5],
});
```

函数轨迹使用轻量 JavaScript LaTeX 解析器 Cortex Compute Engine。关节与姿态公式输出 deg，位置公式输出 mm。当前状态作为轨迹起点，`keypointCount` 个采样点不包含重复起点；标准 JSON 只导出采样后的关键点，公式源码保留在运行时历史元数据中。

每个公式框只填写表达式右侧，例如 `275+25\cos(2\pi t)`；不要填写
`y(t)=`、中文分号等内容。乘法可以直接省略或使用 LaTeX `\cdot`。

```ts
await controller.addFunctionTrajectory({
  space: 'JointAngleSpace',
  tStart: 0,
  tEnd: 1,
  durationMs: 2000,
  keypointCount: 20,
  sources: {
    left:  ['0', '-30', '0', String.raw`30+10\sin(2\pi t)`, '0', '0', '0'],
    right: ['0', '-30', '0', String.raw`30+10\sin(2\pi t)`, '0', '0', '0'],
  },
});
```

历史点修改使用 `editHistory(index, edit)`。修改成功时，该点及所有后续 Cartesian 目标会以新的前序关节解重新计算；任何一步 IK 失败都会回滚整次修改。`export()` 和 `import()` 使用 `performance-action-sequence-v2`，规范单位为关节/姿态 rad、位置 mm、时间 ms。

## 桌面检查工具

在包含 MuJoCo 的 Conda 环境运行：

```powershell
conda activate mjlab_env
python mujoco_dual_arm_visualizer.py
```

桌面工具继续用于 MuJoCo XML 几何、关节配置和实机 IK 的对照检查。`deploy_arm_model.hpp` 是桌面原生桥和浏览器 WASM 共享的部署参数源。

原生代码可编译为 DLL 并通过 `deploy_ik.py` 的 `ctypes` 接口调用；运行该脚本也会执行 FK/IK、MuJoCo 关节限位和动作序列检查：

```powershell
conda run -n mjlab_env python -B deploy_ik.py
```
