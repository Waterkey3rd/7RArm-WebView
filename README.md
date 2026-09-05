# SRS 7R Dual-Arm Web Visualizer

这是从实机机械臂工程中独立出来的 Web 仓库。浏览器端使用 Three.js 显示双臂，
通过 WebAssembly 运行与实机一致的 7R IK/FK 和五次多项式速度规划；运行时不需要
Python、MuJoCo 或后端服务。

## 功能

- 关节角、笛卡尔位姿和增量运动控制
- LaTeX `f(t)` 轨迹、起点自动衔接及自定义模板
- 历史关键点查看、修改和下游 IK 重算
- 全部或连续区间动作轨迹 JSON 导入/导出
- 左右臂独立五次速度规划与速度/加速度限幅
- Three.js 简化刚体显示

## 目录

```text
web/                                      Vite + TypeScript 前端
web/public/wasm/                          可直接使用的 WASM 产物
wasm/                                     WASM C++ 接口与构建脚本
deploy_arm_model.hpp                      实机几何、安装位姿和关节限制
Arm/Lib/myikine7R/                        实机 7R IK/FK 头文件
Arm/Inc/Middleware/TrajectoryPlannerKernel.hpp
                                          五次速度规划共享内核
actionsequence.json                       动作序列导入回归测试样例
```

## 运行 Web

预编译 WASM 已提交，首次运行只需安装前端依赖：

```powershell
Set-Location .\web
npm ci
npm run dev
```

默认地址为 `http://localhost:5173`。

```powershell
npm test
npm run build
```

生产输出位于 `web/dist/`。该目录、`node_modules/` 和本地工具缓存不会提交。

## 重新构建 WASM

构建脚本要求项目根目录存在本地 Emscripten 环境：

```powershell
$env:CONDA_PKGS_DIRS = Join-Path $PWD '.conda-pkgs'
conda create --prefix .\.emscripten-env -c conda-forge emscripten
powershell -NoProfile -ExecutionPolicy Bypass -File .\wasm\build_wasm.ps1
```

生成文件为：

- `web/public/wasm/deploy_ik.js`
- `web/public/wasm/deploy_ik.wasm`

不要把 WASM URL 写成 `src/wasm/`。Vite 的静态文件来自 `web/public/wasm/`，
开发环境对应 `http://localhost:5173/wasm/`。

## 单位与轨迹

- UI 关节角和姿态：degree
- C++/WASM 关节角和姿态：radian
- 位置：mm
- 时间：ms

每个公式框只填写表达式右侧，例如 `275+25\cos(2\pi t)`。当前状态与公式起点
不一致时，系统先生成起点过渡，再从公式起点执行轨迹。

速度规划采用零起终速度、零起终加速度的五次曲线。左右臂分别按共享的关节速度和
加速度上限计算统一时长，先完成的一侧保持终点。当前默认限制是原始部署值的 2 倍。
