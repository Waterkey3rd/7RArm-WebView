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

## Docker Compose 部署

在本仓库根目录执行（需要 Docker 和 Docker Compose v2）：

```sh
docker compose up -d --build
```

启动后访问 `http://localhost:8080`；其他设备使用 `http://服务器IP:8080`，
并确保服务器防火墙允许该端口。

默认端口为 `8080`。可在根目录的 `.env` 文件中设置 `WEB_PORT=8090`，或在
PowerShell 中先执行 `$env:WEB_PORT = '8090'`，再启动 Compose。

```sh
docker compose logs -f web
docker compose down
```

Dockerfile 使用 Node 22 安装锁定依赖并执行生产构建，最终镜像由 Nginx 提供
静态页面、Worker 和 WASM。构建时需要联网下载基础镜像与 npm 依赖；无需在
宿主机安装 Node、Python 或 MuJoCo。

容器直接使用 `web/public/wasm/` 中已提交的产物，不重新编译 C++。修改 IK、
机械臂配置或速度规划内核后，先按下一节重新生成 WASM，然后重新执行
`docker compose up -d --build`。

模板仍保存在各访问者浏览器的 localStorage 中，轨迹 JSON 通过浏览器导入导出。
该容器没有服务器端用户数据存储，不需要挂载数据卷。

本次仅提供容器配置，未在 Docker 环境中验证。

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
