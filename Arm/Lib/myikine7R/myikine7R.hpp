#pragma once

// Deploy 7R 机械臂闭式逆解算，适用于当前项目的 3R-R-3R 构型：
//   球肩 q1-q3 + 单肘 q4 + 球腕 q5-q7。
//
// 这个文件是 IKandFK/myikine7R.m 的 MCU 友好 C++ 移植版。设计目标是：
//   1. 方便直接放入 STM32H723 工程；
//   2. 不依赖 Eigen，不使用动态内存；
//   3. 所有矩阵/向量都是固定 3 维，候选解使用固定容量数组；
//   4. 保留 MATLAB 版的主要能力：臂角/SEW 参数、关节限位、按当前角度选最近解。
//
// 坐标与单位约定：
//   - targetWorld.R 是目标工具坐标系相对世界坐标系的旋转矩阵；
//   - targetWorld.p 是目标工具坐标系原点在世界坐标系下的位置，单位 mm；
//   - qCurrent、psi、输出 qBest 均为弧度；
//   - cfg.installation 描述机器人基坐标系相对世界坐标系的安装位姿；
//   - cfg.geometry.toolAxisRotation 与 MATLAB 版保持一致，默认将原工具 z 轴映射为命令 x 轴。
//
// 基本用法：
//   roboarm7r::Config cfg = roboarm7r::defaultConfig();
//   roboarm7r::Pose target;
//   target.R = ...;             // 世界坐标系下的目标末端姿态
//   target.p = {x_mm, y_mm, z_mm};
//   roboarm7r::Vec7 qCurrent = {{0,0,0,0,0,0,0}};
//   roboarm7r::IkResult result;
//   bool ok = roboarm7r::solve(target, qCurrent, psi, true, cfg, result);
//   if (ok) { result.qBest.v[0] ... result.qBest.v[6]; }
//
// 如果不想手动给 psi，可以调用另一个重载：
//   bool ok = roboarm7r::solve(target, qCurrent, cfg, result);
// 它会从 qCurrent 的肩-肘-腕构型反推当前 SEW 角，通常可以保持当前肘部姿态。
//
// 性能建议：
//   - STM32H723 上建议保持默认 float；
//   - PC 端和 MATLAB 对比验证时，可在 include 之前写：
//       #define ROBOARM7R_SCALAR double
//       #include "myikine7R.hpp"
//   - 逆解内部会生成有限个闭式候选解，然后剔除越限解和误差不合格解；
//   - fitToJointLimits() 先选择限位内且最接近当前角度的 2*pi 表示，随后用
//     sum(weights[i] * (q[i]-qCurrent[i])^2) 选择真实命令距离最近的一组。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

#ifndef ROBOARM7R_SCALAR
#define ROBOARM7R_SCALAR float
#endif

namespace roboarm7r {

using real_t = ROBOARM7R_SCALAR;

static constexpr real_t kPi = std::numbers::pi_v<float>;
static constexpr real_t kTwoPi = static_cast<real_t>(2) * kPi;
// 闭式解候选数量的固定上限。正常 3R-R-3R 构型远小于这个值；
// 保留 96 是为了覆盖球肩/球腕奇异处分裂出的附加候选，同时避免动态分配。
static constexpr int kMaxCandidates = 96;

// 三维向量。为便于 MCU 调试，没有使用数组封装 x/y/z。
struct Vec3 {
    real_t x;
    real_t y;
    real_t z;
};

// 七个关节角，单位 rad。v[0] 对应 q1，v[6] 对应 q7。
struct Vec7 {
    real_t v[7];
};

// 3x3 旋转矩阵，按 m[row][col] 存储。
struct Mat3 {
    real_t m[3][3];
};

// 末端位姿：R 是旋转矩阵，p 是位置，位置单位 mm。
struct Pose {
    Mat3 R;
    Vec3 p;
};

enum class SewMode : uint8_t {
    Stereographic = 0,
    Conventional = 1,
};

enum class HybridStage : uint8_t {
    None = 0,
    FastAnalytic = 1,
    Manifold = 2,
    GeometricFallback = 3,
};

struct Geometry {
    // 基座到肩部的 z 向偏置，对应 MATLAB 中 d_L0。
    real_t baseOffsetMm = static_cast<real_t>(0.0);

    // 肩到肘的主臂长，对应 d_L1_1 + d_L1_2。
    real_t upperArmMm = static_cast<real_t>(158.50);

    // 肘部偏置，对应 d_L1_3。不要把它简单合并成一根等效斜边，
    // 因为它相对 q4 轴的方向会影响闭式逆解。
    real_t elbowOffsetMm = static_cast<real_t>(60.0);

    // 肘到腕的前臂长度，对应 d_L2_1。
    real_t forearmMm = static_cast<real_t>(249.38);

    // 法兰额外延长量 dF。
    real_t flangeExtensionMm = static_cast<real_t>(0.0);

    // 末端工具长度 dEE。
    real_t endEffectorMm = static_cast<real_t>(92.57);

    // 工具绕本地 z 轴的固定偏航角 thetaEE。
    real_t toolYawRad = static_cast<real_t>(0.0);

    // 工具坐标系重映射矩阵。默认值与 MATLAB 版一致：
    //   新 x = 原 z，新 y = -原 y，新 z = 原 x。
    Mat3 toolAxisRotation;
};

struct Installation {
    // 机器人基坐标系到世界坐标系的旋转 R_WR。
    // 世界目标会先通过 R_WR^T 转回机器人本地坐标系再求逆解。
    Mat3 rotationWorldFromRobot;

    // 机器人基坐标系原点在世界坐标系下的位置 p_WR，单位 mm。
    Vec3 positionWorldMm;
};

struct JointConfig {
    // 每个关节的命令角限位，单位 rad。注意这里是命令角，
    // 与 MATLAB RoboArm7R.m 中关节 2 的 MDH 固定 pi/2 偏置无关。
    real_t lowerRad[7];
    real_t upperRad[7];
};

struct SelectionConfig {
    // 候选解选择权重。权重越大，该关节越不愿意离开当前角度。
    // q 已由 fitToJointLimits() 映射为限位内的实际命令角，因此目标函数为：
    //   J = sum_i weights[i] * (q[i] - qCurrent[i])^2
    // 这里不能 wrapToPi，否则会把跨越 +/-pi 的大幅电机动作误判为短路径。
    real_t weights[7];
};

struct SewConfig {
    // 默认使用 stereographic SEW，避开传统 SEW 的双向奇异线。
    SewMode mode = SewMode::Stereographic;

    // stereographic SEW 的奇异方向，默认 -Z。
    Vec3 stereographicSingularityDirection;

    // stereographic SEW 的参考方向，默认 +Y。
    Vec3 stereographicReference;

    // conventional SEW 的参考方向，只有 mode=Conventional 时使用。
    Vec3 conventionalReference;
};

struct Tolerance {
    // FK 校验用的位置误差阈值，单位 mm。float/MCU 版本不宜设得过小。
    real_t positionMm = static_cast<real_t>(1.0e-3);

    // FK 校验用的姿态误差阈值，单位 rad。
    real_t orientationRad = static_cast<real_t>(1.0e-4);

    // 旋转矩阵正交性检查预留阈值。
    real_t rotationMatrix = static_cast<real_t>(1.0e-4);

    // 几何退化判断阈值，例如肩腕距离接近 0。
    real_t geometry = static_cast<real_t>(1.0e-6);

    // IK-Geo 子问题求解阈值。
    real_t subproblem = static_cast<real_t>(1.0e-6);

    // 球肩/球腕欧拉分解奇异判断阈值。
    real_t sphericalSingularity = static_cast<real_t>(1.0e-5);

    // 候选解去重阈值。
    real_t duplicateJoint = static_cast<real_t>(1.0e-5);

    // 关节限位判断阈值。
    real_t jointLimit = static_cast<real_t>(1.0e-6);
};

struct SolverConfig {
    // 子问题不可精确满足时会产生最近二乘解。实际位置控制通常应拒绝；
    // 调试或容错时可置 true，但必须结合 FK 误差判断使用。
    bool acceptLeastSquares = false;

    // 电机/关节命令空间加权 DLS。dlsJointWeights 是关节增量二次型
    // dq^T*Wq*dq 的对角元素；值越大，DLS 越不愿意使用对应关节。
    // 当前重点抑制球肩 q1-q3，尤其是中间轴 q2。
    int dlsMaxIterations = 32;
    real_t dlsDamping = static_cast<real_t>(5.0e-3);
    real_t dlsMaxIterationStepRad = static_cast<real_t>(10) * kPi /
                                    static_cast<real_t>(180);
    real_t dlsTrustRegionRad = static_cast<real_t>(30) * kPi /
                               static_cast<real_t>(180);
    // 将旋转误差/Jw 缩放成等效平移长度。100mm 表示 1rad 姿态误差
    // 在 DLS 任务度量中等效为 100mm 平移误差。
    real_t dlsOrientationScaleMm = static_cast<real_t>(100);

    // 纯 DLS 只处理相对 qCurrent 正运动学位置不超过该距离的局部目标。
    // 超过上限时直接拒绝，不截断目标、不插值，也不切换其他 IK。
    real_t dlsMaxTargetPositionDeltaMm = static_cast<real_t>(100);
    real_t dlsJointWeights[7] = {
        static_cast<real_t>(7),
        static_cast<real_t>(6),
        static_cast<real_t>(5),
        static_cast<real_t>(1),
        static_cast<real_t>(2),
        static_cast<real_t>(2),
        static_cast<real_t>(2),
    };

    // 球肩 Z/-Y/-X 轴序列在 q2=pi/2+k*pi 时退化。DLS 允许 q2 连续
    // 穿越奇异点，但只要本次迭代路径触碰奇异死区，就把 q1/q3 相对
    // qCurrent 的总变化限制在 dlsShoulderOuterMaxDeltaRad 内。
    real_t dlsShoulderDeadbandRad = static_cast<real_t>(3) * kPi /
                                    static_cast<real_t>(180);
    real_t dlsShoulderOuterMaxDeltaRad = static_cast<real_t>(0.2) * kPi /
                                         static_cast<real_t>(180);

    // 只有本次 DLS 确实触碰肩奇异死区时才允许使用这组放宽容差。
    // 未触碰死区的普通 DLS 仍严格使用 cfg.tolerance。
    real_t dlsShoulderRelaxedPositionMm = static_cast<real_t>(25);
    real_t dlsShoulderRelaxedOrientationRad = static_cast<real_t>(12) * kPi /
                                              static_cast<real_t>(180);

    // AdaptiveHybrid 的互斥路由阈值。只有平移和旋转变化同时不超过阈值时
    // 才调用加权 DLS，否则只调用原 Hybrid；同一周期不会先后运行两个求解器。
    // 2.5mm / 1.25deg 来自连续路径二维扫描：4995/4995 成功且腕翻转为 0。
    real_t adaptiveDlsPositionThresholdMm = static_cast<real_t>(90);
    real_t adaptiveDlsOrientationThresholdRad = static_cast<real_t>(30) * kPi /
                                                 static_cast<real_t>(180);
};

struct Config {
    // 所有可调参数集中在这里。实际上板时通常只需要改：
    //   geometry.*        机械尺寸
    //   installation.*    安装位姿
    //   joint.*           关节限位
    //   selection.weights 候选解偏好
    Geometry geometry;
    Installation installation;
    JointConfig joint;
    SelectionConfig selection;
    SewConfig sew;
    Tolerance tolerance;
    SolverConfig solver;
};

struct IkResult {
    // true 表示找到满足限位和 FK 误差阈值的精确候选解。
    bool success = false;

    // 闭式几何阶段生成的原始候选数量。
    int rawCandidateCount = 0;

    // 通过限位、最小二乘过滤、FK 误差过滤后的有效候选数量。
    int validCandidateCount = 0;

    // 被选中的有效候选索引。失败时为 -1。
    int selectedValidIndex = -1;

    // 本次求解使用的 SEW 角。
    real_t psi = static_cast<real_t>(0);

    // true 表示 psi 是从 qCurrent 自动推断的。
    bool psiWasInferredFromCurrent = false;

    // 最终选中的 7 轴角度，单位 rad。
    Vec7 qBest = {{0, 0, 0, 0, 0, 0, 0}};

    // 最优候选的选择代价。
    real_t bestCost = static_cast<real_t>(0);

    // 最优候选的 FK 位置/姿态误差。
    real_t bestPositionErrorMm = static_cast<real_t>(0);
    real_t bestOrientationErrorRad = static_cast<real_t>(0);

    // 流形法内部调用闭式固定-psi 求解器的次数；普通 solve() 保持为 0。
    int manifoldEvaluationCount = 0;

    // true 表示中心 psi 无解，流形法启用了分级可行域恢复搜索。
    bool manifoldUsedFeasibilityRecovery = false;

    // Hybrid 诊断：估算的固定-psi 闭式调用数、是否触发风险路径、
    // 最终采用的阶段和启发式风险代价。普通 solve()/流形入口保持默认值。
    int hybridEvaluationCount = 0;
    bool hybridRiskTriggered = false;
    HybridStage hybridStage = HybridStage::None;
    real_t hybridScore = static_cast<real_t>(0);

    // DLS 诊断。失败时 qBest 保持 qCurrent，调用方不会收到未收敛的中间量。
    int dlsIterationCount = 0;
    bool dlsTrustRegionHit = false;

    // 固定-psi 解析器最终选中的主分支编码，用于诊断解析支切换：
    // bit0=q4 距离分支，bit1=肩部 q1/q2 分支，bit2=腕部 q5/q6 分支。
    // -1 表示求解失败、未经过解析分支，或调用方没有请求该诊断。
    int selectedMajorBranch = -1;

    // AdaptiveHybrid 路由诊断：0=非自适应入口，1=DLS8，2=Hybrid。
    // delta 记录路由判定时使用的笛卡尔变化量，便于上板统计和重新标定阈值。
    uint8_t adaptiveRoute = 0;
    real_t adaptivePositionDeltaMm = static_cast<real_t>(0);
    real_t adaptiveOrientationDeltaRad = static_cast<real_t>(0);
};

namespace detail {

struct Robot {
    Vec3 H[7];
    Vec3 P[8];
    Mat3 RToolHome;
};

struct Candidate {
    Vec7 q;
    bool leastSquares;
    // 生成该候选时经过的 q4/肩/腕三层主分支，编码规则同
    // IkResult::selectedMajorBranch，仅用于诊断最终选择了哪条解析支。
    uint8_t majorBranch;
};

struct CandidateSet {
    Candidate c[kMaxCandidates];
    int n = 0;
};

struct AngleSet {
    real_t a[2];
    int n = 0;
    bool leastSquares = false;
};

struct Sp2Result {
    real_t theta1[2];
    real_t theta2[2];
    int n = 0;
    bool leastSquares = false;
};

inline real_t absr(real_t x) { return x < static_cast<real_t>(0) ? -x : x; }
inline real_t maxr(real_t a, real_t b) { return a > b ? a : b; }
inline real_t minr(real_t a, real_t b) { return a < b ? a : b; }
inline real_t clamp(real_t x, real_t lo, real_t hi) { return minr(hi, maxr(lo, x)); }

inline Vec3 vec(real_t x, real_t y, real_t z) { return Vec3{x, y, z}; }
inline Vec3 add(Vec3 a, Vec3 b) { return vec(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 sub(Vec3 a, Vec3 b) { return vec(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 scale(Vec3 a, real_t s) { return vec(a.x * s, a.y * s, a.z * s); }
inline real_t dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return vec(a.y * b.z - a.z * b.y,
               a.z * b.x - a.x * b.z,
               a.x * b.y - a.y * b.x);
}
inline real_t norm2(Vec3 a) { return dot(a, a); }
inline real_t norm(Vec3 a) { return std::sqrt(norm2(a)); }

inline Vec3 unit(Vec3 a, real_t tolerance, bool* ok = nullptr) {
    const real_t n = norm(a);
    const bool valid = n > tolerance;
    if (ok) {
        *ok = valid;
    }
    if (!valid) {
        return vec(0, 0, 0);
    }
    return scale(a, static_cast<real_t>(1) / n);
}

inline Mat3 eye() {
    return Mat3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}

inline Mat3 mat(real_t a00, real_t a01, real_t a02,
                real_t a10, real_t a11, real_t a12,
                real_t a20, real_t a21, real_t a22) {
    return Mat3{{{a00, a01, a02}, {a10, a11, a12}, {a20, a21, a22}}};
}

inline Mat3 transpose(Mat3 A) {
    return mat(A.m[0][0], A.m[1][0], A.m[2][0],
               A.m[0][1], A.m[1][1], A.m[2][1],
               A.m[0][2], A.m[1][2], A.m[2][2]);
}

inline Vec3 mul(Mat3 A, Vec3 v) {
    return vec(A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
               A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
               A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z);
}

inline Mat3 mul(Mat3 A, Mat3 B) {
    Mat3 C = {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            C.m[r][c] = A.m[r][0] * B.m[0][c] +
                        A.m[r][1] * B.m[1][c] +
                        A.m[r][2] * B.m[2][c];
        }
    }
    return C;
}

inline Mat3 rot(Vec3 axis, real_t theta) {
    bool ok = false;
    const Vec3 k = unit(axis, static_cast<real_t>(1.0e-12), &ok);
    if (!ok) {
        return eye();
    }
    const real_t s = std::sin(theta);
    const real_t c = std::cos(theta);
    const real_t v = static_cast<real_t>(1) - c;

    return mat(
        c + k.x * k.x * v,         k.x * k.y * v - k.z * s, k.x * k.z * v + k.y * s,
        k.y * k.x * v + k.z * s,   c + k.y * k.y * v,       k.y * k.z * v - k.x * s,
        k.z * k.x * v - k.y * s,   k.z * k.y * v + k.x * s, c + k.z * k.z * v);
}

inline real_t wrapToPi(real_t a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a < static_cast<real_t>(0)) {
        a += kTwoPi;
    }
    return a - kPi;
}

inline Vec7 wrapToPi(Vec7 q) {
    for (int i = 0; i < 7; ++i) {
        q.v[i] = wrapToPi(q.v[i]);
    }
    return q;
}

inline real_t rotationDistance(Mat3 R1, Mat3 R2) {
    const Mat3 Rt = mul(transpose(R1), R2);
    real_t c = (Rt.m[0][0] + Rt.m[1][1] + Rt.m[2][2] - static_cast<real_t>(1)) /
               static_cast<real_t>(2);
    c = clamp(c, static_cast<real_t>(-1), static_cast<real_t>(1));
    return std::acos(c);
}

inline bool addCandidate(CandidateSet& set, Vec7 q, bool leastSquares,
                         uint8_t majorBranch = 0) {
    if (set.n >= kMaxCandidates) {
        return false;
    }
    set.c[set.n].q = wrapToPi(q);
    set.c[set.n].leastSquares = leastSquares;
    set.c[set.n].majorBranch = majorBranch;
    ++set.n;
    return true;
}

inline Config makeDefaultConfig() {
    Config cfg;
    cfg.geometry.toolAxisRotation = mat(0, 0, 1,
                                        0, -1, 0,
                                        1, 0, 0);
    cfg.installation.rotationWorldFromRobot = eye();
    cfg.installation.positionWorldMm = vec(0, 0, 0);

    for (int i = 0; i < 7; ++i) {
        cfg.joint.lowerRad[i] = -kPi;
        cfg.joint.upperRad[i] = kPi;
        cfg.selection.weights[i] = static_cast<real_t>(1);
    }

    cfg.sew.mode = SewMode::Stereographic;
    cfg.sew.stereographicSingularityDirection = vec(0, 0, -1);
    cfg.sew.stereographicReference = vec(0, 1, 0);
    cfg.sew.conventionalReference = vec(1, 0, 0);
    return cfg;
}

inline Robot buildRobot(const Config& cfg) {
    const Vec3 ex = vec(1, 0, 0);
    const Vec3 ey = vec(0, 1, 0);
    const Vec3 ez = vec(0, 0, 1);

    Robot robot;
    robot.H[0] = ez;
    robot.H[1] = scale(ey, -1);
    robot.H[2] = scale(ex, -1);
    robot.H[3] = scale(ey, -1);
    robot.H[4] = ex;
    robot.H[5] = scale(ey, -1);
    robot.H[6] = ex;

    robot.P[0] = vec(0, 0, cfg.geometry.baseOffsetMm);
    robot.P[1] = vec(0, 0, 0);
    robot.P[2] = vec(0, 0, 0);
    robot.P[3] = vec(-cfg.geometry.upperArmMm, 0, 0);
    robot.P[4] = vec(cfg.geometry.forearmMm, 0, cfg.geometry.elbowOffsetMm);
    robot.P[5] = vec(0, 0, 0);
    robot.P[6] = vec(0, 0, 0);
    robot.P[7] = vec(cfg.geometry.flangeExtensionMm + cfg.geometry.endEffectorMm, 0, 0);

    const Mat3 RFlangeHome = mat(0, 0, 1,
                                 0, -1, 0,
                                 1, 0, 0);
    robot.RToolHome = mul(mul(RFlangeHome, rot(ez, cfg.geometry.toolYawRad)),
                          cfg.geometry.toolAxisRotation);
    return robot;
}

inline Pose worldTargetToRobot(const Pose& worldTarget, const Installation& installation) {
    const Mat3 Rt = transpose(installation.rotationWorldFromRobot);
    Pose robotTarget;
    robotTarget.R = mul(Rt, worldTarget.R);
    robotTarget.p = mul(Rt, sub(worldTarget.p, installation.positionWorldMm));
    return robotTarget;
}

inline Pose forwardKinematics(const Vec7& q, const Robot& robot) {
    Mat3 R = eye();
    Vec3 p = robot.P[0];
    for (int i = 0; i < 7; ++i) {
        R = mul(R, rot(robot.H[i], q.v[i]));
        p = add(p, mul(R, robot.P[i + 1]));
    }
    Pose T;
    T.R = mul(R, robot.RToolHome);
    T.p = p;
    return T;
}

inline void armPoints(const Vec7& q, const Robot& robot, Vec3& S, Vec3& E, Vec3& W) {
    S = robot.P[0];
    const Mat3 R03 = mul(mul(rot(robot.H[0], q.v[0]), rot(robot.H[1], q.v[1])),
                         rot(robot.H[2], q.v[2]));
    E = add(S, mul(R03, robot.P[3]));
    W = add(E, mul(mul(R03, rot(robot.H[3], q.v[3])), robot.P[4]));
}

inline AngleSet sp1(Vec3 p1, Vec3 p2, Vec3 k, real_t tolerance) {
    // 子问题 1：
    //   已知旋转轴 k 和两个向量 p1/p2，求 theta 使 R(k,theta)*p1 = p2。
    // 实现方式是把 p1/p2 投影到垂直于 k 的平面，在该平面内用 atan2 求圆周角。
    AngleSet out;
    bool ok = false;
    k = unit(k, tolerance, &ok);
    if (!ok) {
        out.n = 0;
        out.leastSquares = true;
        return out;
    }

    const Vec3 p1Parallel = scale(k, dot(k, p1));
    const Vec3 p2Parallel = scale(k, dot(k, p2));
    const Vec3 p1Perp = sub(p1, p1Parallel);
    const Vec3 p2Perp = sub(p2, p2Parallel);

    const real_t perpNormProduct = norm(p1Perp) * norm(p2Perp);
    if (perpNormProduct <= tolerance) {
        out.a[0] = 0;
        out.n = 1;
        out.leastSquares = norm(sub(p1, p2)) > tolerance;
        return out;
    }

    const real_t theta = std::atan2(dot(k, cross(p1Perp, p2Perp)), dot(p1Perp, p2Perp));
    const Vec3 residual = sub(mul(rot(k, theta), p1), p2);
    const real_t scaleVal = maxr(static_cast<real_t>(1), maxr(norm(p1), norm(p2)));
    out.a[0] = theta;
    out.n = 1;
    out.leastSquares = norm(residual) > tolerance * scaleVal;
    return out;
}

inline AngleSet sp4(Vec3 h, Vec3 p, Vec3 k, real_t d, real_t tolerance) {
    // 子问题 4：
    //   求 theta，使 h^T * R(k,theta) * p = d。
    // 展开后是 A*cos(theta) + B*sin(theta) = C，因此最多两个解。
    AngleSet out;
    bool ok = false;
    k = unit(k, tolerance, &ok);
    if (!ok) {
        out.n = 0;
        out.leastSquares = true;
        return out;
    }

    const Vec3 u = sub(p, scale(k, dot(k, p)));
    const real_t A = dot(h, u);
    const real_t B = dot(h, cross(k, p));
    const real_t C = d - dot(h, k) * dot(k, p);
    const real_t radius = std::hypot(A, B);

    if (radius <= tolerance) {
        out.a[0] = 0;
        out.n = 1;
        out.leastSquares = absr(C) > tolerance;
        return out;
    }

    const real_t phase = std::atan2(B, A);
    real_t normalizedC = C / radius;
    if (normalizedC > static_cast<real_t>(1) + tolerance) {
        out.a[0] = phase;
        out.n = 1;
        out.leastSquares = true;
    } else if (normalizedC < static_cast<real_t>(-1) - tolerance) {
        out.a[0] = wrapToPi(phase + kPi);
        out.n = 1;
        out.leastSquares = true;
    } else {
        normalizedC = clamp(normalizedC, static_cast<real_t>(-1), static_cast<real_t>(1));
        const real_t opening = std::acos(normalizedC);
        if (opening <= tolerance) {
            out.a[0] = phase;
            out.n = 1;
        } else {
            out.a[0] = wrapToPi(phase + opening);
            out.a[1] = wrapToPi(phase - opening);
            out.n = 2;
        }
        out.leastSquares = false;
    }
    return out;
}

inline Sp2Result sp2(Vec3 p1, Vec3 p2, Vec3 k1, Vec3 k2, real_t tolerance) {
    // 子问题 2：
    //   求 theta1/theta2，使 R(k1,theta1)*p1 = R(k2,theta2)*p2。
    // 这里复用两个 SP4 先分别求 theta1 和 theta2，再按 IK-Geo 的配对规则组合。
    Sp2Result out;
    const real_t p1Norm = norm(p1);
    const real_t p2Norm = norm(p2);
    if (p1Norm <= tolerance || p2Norm <= tolerance) {
        out.leastSquares = true;
        return out;
    }

    const Vec3 p1Unit = scale(p1, static_cast<real_t>(1) / p1Norm);
    const Vec3 p2Unit = scale(p2, static_cast<real_t>(1) / p2Norm);
    bool ok1 = false;
    bool ok2 = false;
    k1 = unit(k1, tolerance, &ok1);
    k2 = unit(k2, tolerance, &ok2);
    if (!ok1 || !ok2) {
        out.leastSquares = true;
        return out;
    }

    const AngleSet theta1All = sp4(k2, p1Unit, k1, dot(k2, p2Unit), tolerance);
    const AngleSet theta2All = sp4(k1, p2Unit, k2, dot(k1, p1Unit), tolerance);
    if (theta1All.n == 0 || theta2All.n == 0) {
        out.leastSquares = true;
        return out;
    }

    if (theta1All.n > 1 || theta2All.n > 1) {
        out.n = 2;
        out.theta1[0] = theta1All.a[0];
        out.theta1[1] = theta1All.a[theta1All.n - 1];
        out.theta2[0] = theta2All.a[theta2All.n - 1];
        out.theta2[1] = theta2All.a[0];
    } else {
        out.n = 1;
        out.theta1[0] = theta1All.a[0];
        out.theta2[0] = theta2All.a[0];
    }

    const real_t scaleVal = maxr(static_cast<real_t>(1), maxr(p1Norm, p2Norm));
    out.leastSquares = absr(p1Norm - p2Norm) > tolerance * scaleVal ||
                       theta1All.leastSquares || theta2All.leastSquares;

    for (int i = 0; i < out.n; ++i) {
        const Vec3 residual = sub(mul(rot(k1, out.theta1[i]), p1Unit),
                                  mul(rot(k2, out.theta2[i]), p2Unit));
        if (norm(residual) > static_cast<real_t>(20) * tolerance) {
            out.leastSquares = true;
        }
    }
    return out;
}

inline AngleSet sp3(Vec3 p1, Vec3 p2, Vec3 k, real_t d, real_t tolerance) {
    // 子问题 3：
    //   求 theta，使 ||R(k,theta)*p1 - p2|| = d。
    // 对当前机械臂，它用于根据肩腕距离求肘关节 q4。
    const real_t projection = static_cast<real_t>(0.5) * (dot(p1, p1) + dot(p2, p2) - d * d);
    return sp4(p2, p1, k, projection, tolerance);
}

inline bool sewInverse(Vec3 S, Vec3 W, real_t psi, const Config& cfg, Vec3& eCE, Vec3& nSEW) {
    // 根据肩点 S、腕点 W 和给定 SEW 角 psi，构造目标肘平面：
    //   eSW：肩到腕方向；
    //   nSEW：肩-肘-腕平面法向；
    //   eCE：在该平面中决定肘部处于哪一个半平面。
    const Vec3 pSW = sub(W, S);
    const real_t pSWNorm = norm(pSW);
    if (pSWNorm <= cfg.tolerance.geometry) {
        return false;
    }
    const Vec3 eSW = scale(pSW, static_cast<real_t>(1) / pSWNorm);

    if (cfg.sew.mode == SewMode::Stereographic) {
        bool okA = false;
        bool okB = false;
        const Vec3 singularDirection = unit(cfg.sew.stereographicSingularityDirection,
                                            cfg.tolerance.geometry, &okA);
        const Vec3 reference = unit(cfg.sew.stereographicReference, cfg.tolerance.geometry, &okB);
        if (!okA || !okB) {
            return false;
        }
        const Vec3 referenceAxis = cross(sub(eSW, singularDirection), reference);
        const Vec3 xAxis = cross(referenceAxis, pSW);
        const real_t xNorm = norm(xAxis);
        if (xNorm <= cfg.tolerance.geometry) {
            return false;
        }
        eCE = mul(rot(eSW, psi), scale(xAxis, static_cast<real_t>(1) / xNorm));
        nSEW = cross(eSW, eCE);
    } else {
        bool ok = false;
        const Vec3 reference = unit(cfg.sew.conventionalReference, cfg.tolerance.geometry, &ok);
        if (!ok) {
            return false;
        }
        Vec3 yAxis = cross(eSW, reference);
        const real_t yNorm = norm(yAxis);
        if (yNorm <= cfg.tolerance.geometry) {
            return false;
        }
        yAxis = scale(yAxis, static_cast<real_t>(1) / yNorm);
        nSEW = mul(rot(eSW, psi), yAxis);
        eCE = cross(nSEW, eSW);
    }

    eCE = scale(eCE, static_cast<real_t>(1) / norm(eCE));
    nSEW = scale(nSEW, static_cast<real_t>(1) / norm(nSEW));
    return true;
}

inline bool sewForward(Vec3 S, Vec3 E, Vec3 W, const Config& cfg, real_t& psi) {
    // 从当前构型的 S/E/W 三点反算 SEW 角。
    // 当用户不显式传入 psi 时，solve() 会调用它，从而尽量保持当前肘部姿态。
    const Vec3 pSE = sub(E, S);
    const Vec3 pSW = sub(W, S);
    if (norm(pSW) <= cfg.tolerance.geometry ||
        norm(cross(pSW, pSE)) <= cfg.tolerance.geometry) {
        return false;
    }

    const Vec3 eSW = scale(pSW, static_cast<real_t>(1) / norm(pSW));
    Vec3 nSEW = cross(pSW, pSE);
    nSEW = scale(nSEW, static_cast<real_t>(1) / norm(nSEW));

    if (cfg.sew.mode == SewMode::Stereographic) {
        bool okA = false;
        bool okB = false;
        const Vec3 singularDirection = unit(cfg.sew.stereographicSingularityDirection,
                                            cfg.tolerance.geometry, &okA);
        const Vec3 reference = unit(cfg.sew.stereographicReference, cfg.tolerance.geometry, &okB);
        if (!okA || !okB) {
            return false;
        }
        Vec3 nReference = cross(sub(eSW, singularDirection), reference);
        const real_t nRefNorm = norm(nReference);
        if (nRefNorm <= cfg.tolerance.geometry) {
            return false;
        }
        nReference = scale(nReference, static_cast<real_t>(1) / nRefNorm);
        psi = std::atan2(dot(nSEW, cross(eSW, nReference)), dot(nSEW, nReference));
    } else {
        bool ok = false;
        const Vec3 reference = unit(cfg.sew.conventionalReference, cfg.tolerance.geometry, &ok);
        if (!ok) {
            return false;
        }
        const Vec3 referencePerp = sub(reference, scale(eSW, dot(eSW, reference)));
        const Vec3 elbowPerp = sub(pSE, scale(eSW, dot(eSW, pSE)));
        if (norm(referencePerp) <= cfg.tolerance.geometry ||
            norm(elbowPerp) <= cfg.tolerance.geometry) {
            return false;
        }
        psi = std::atan2(dot(eSW, cross(referencePerp, elbowPerp)),
                         dot(referencePerp, elbowPerp));
    }
    psi = wrapToPi(psi);
    return true;
}

inline int singularOuterAngleCandidates(real_t qFirst, real_t qMiddle, real_t qLast,
                                        Vec3 hFirst, Vec3 hMiddle, Vec3 hLast,
                                        real_t qFirstCurrent, real_t qLastCurrent,
                                        real_t weightFirst, real_t weightLast,
                                        real_t tolerance,
                                        real_t out[2]) {
    // 球肩/球腕在中间轴角度接近奇异时，外侧两个关节不是唯一的。
    // 此时只有 qFirst +/- qLast 的组合可观测。这里返回：
    //   1. 原始代表解；
    //   2. 按 weights 和 qCurrent 分配后的最近解。
    const Vec3 transformedLastAxis = mul(rot(hMiddle, qMiddle), hLast);
    const real_t alignment = dot(hFirst, transformedLastAxis);
    if (absr(absr(alignment) - static_cast<real_t>(1)) > tolerance) {
        out[0] = qFirst;
        return 1;
    }

    real_t sign = alignment >= static_cast<real_t>(0) ? static_cast<real_t>(1)
                                                      : static_cast<real_t>(-1);
    const real_t observableAngle = qFirst + sign * qLast;
    const real_t currentObservableAngle = qFirstCurrent + sign * qLastCurrent;
    // 这里处理的是球关节奇异时唯一可观测的周期组合角，并非最终关节命令
    // 距离；候选落到限位内之后仍由 weightedCommandDistanceSq() 线性评分。
    const real_t observableDelta = wrapToPi(observableAngle - currentObservableAngle);

    const real_t totalWeight = weightFirst + weightLast;
    real_t qFirstOptimal = qFirstCurrent;
    if (totalWeight > tolerance) {
        qFirstOptimal = qFirstCurrent + weightLast / totalWeight * observableDelta;
    }

    out[0] = wrapToPi(qFirst);
    out[1] = wrapToPi(qFirstOptimal);
    return 2;
}

inline void ik3RR3R(Mat3 R07, Vec3 p0T, real_t psi, const Vec7& qCurrent,
                   const Robot& robot, const Config& cfg, CandidateSet& Q) {
    // 3R-R-3R 闭式逆解主流程，与 MATLAB localIK3RR3R 对应：
    //   1. 去掉工具长度，得到球腕中心 W；
    //   2. 用 SP3 根据肩腕距离求 q4；
    //   3. 用 SEW 平面构造肩部目标姿态 R03；
    //   4. 分解球肩得到 q1-q3；
    //   5. 根据 R47 分解球腕得到 q5-q7。
    const Vec3 W = sub(p0T, mul(R07, robot.P[7]));
    const Vec3 S = robot.P[0];
    const Vec3 pSW = sub(W, S);
    const real_t shoulderWristDistance = norm(pSW);
    if (shoulderWristDistance <= cfg.tolerance.geometry) {
        return;
    }

    const Vec3 eSW = scale(pSW, static_cast<real_t>(1) / shoulderWristDistance);
    Vec3 eCE;
    Vec3 nSEW;
    if (!sewInverse(S, W, psi, cfg, eCE, nSEW)) {
        return;
    }

    const AngleSet q4Set = sp3(robot.P[4], scale(robot.P[3], -1), robot.H[3],
                               shoulderWristDistance, cfg.tolerance.subproblem);
    for (int q4Index = 0; q4Index < q4Set.n; ++q4Index) {
        const real_t q4 = q4Set.a[q4Index];
        const Vec3 elbowVector = add(robot.P[3], mul(rot(robot.H[3], q4), robot.P[4]));

        const Sp2Result bc = sp2(pSW, elbowVector, scale(nSEW, -1), eSW,
                                 cfg.tolerance.subproblem);
        if (bc.n == 0) {
            continue;
        }

        const real_t thetaB = bc.theta1[0];
        const real_t thetaC = bc.theta2[0];
        const Mat3 middleShoulderRotation = mul(rot(nSEW, thetaB), rot(eSW, thetaC));

        const AngleSet thetaASet = sp4(nSEW, mul(middleShoulderRotation, robot.P[3]),
                                       eSW, 0, cfg.tolerance.subproblem);
        for (int thetaAIndex = 0; thetaAIndex < thetaASet.n; ++thetaAIndex) {
            const real_t thetaA = thetaASet.a[thetaAIndex];
            const Mat3 R03 = mul(rot(eSW, thetaA), middleShoulderRotation);

            if (dot(eCE, mul(R03, robot.P[3])) < -cfg.tolerance.geometry) {
                continue;
            }

            const Sp2Result q12 = sp2(robot.H[2], mul(R03, robot.H[2]),
                                      robot.H[1], scale(robot.H[0], -1),
                                      cfg.tolerance.subproblem);
            for (int shoulderIndex = 0; shoulderIndex < q12.n; ++shoulderIndex) {
                const real_t q1Representative = q12.theta2[shoulderIndex];
                const real_t q2 = q12.theta1[shoulderIndex];
                const Mat3 R02Representative = mul(rot(robot.H[0], q1Representative),
                                                   rot(robot.H[1], q2));
                const AngleSet q3RepresentativeSet =
                    sp1(robot.H[1],
                        mul(mul(transpose(R02Representative), R03), robot.H[1]),
                        robot.H[2], cfg.tolerance.subproblem);

                for (int q3RepIndex = 0; q3RepIndex < q3RepresentativeSet.n; ++q3RepIndex) {
                    const real_t q3Representative = q3RepresentativeSet.a[q3RepIndex];
                    real_t q1Candidates[2] = {};
                    const int q1Count = singularOuterAngleCandidates(
                        q1Representative, q2, q3Representative,
                        robot.H[0], robot.H[1], robot.H[2],
                        qCurrent.v[0], qCurrent.v[2],
                        cfg.selection.weights[0], cfg.selection.weights[2],
                        cfg.tolerance.sphericalSingularity, q1Candidates);

                    for (int q1CandidateIndex = 0; q1CandidateIndex < q1Count; ++q1CandidateIndex) {
                        const real_t q1 = q1Candidates[q1CandidateIndex];
                        const Mat3 R02 = mul(rot(robot.H[0], q1), rot(robot.H[1], q2));
                        const AngleSet q3Set =
                            sp1(robot.H[1], mul(mul(transpose(R02), R03), robot.H[1]),
                                robot.H[2], cfg.tolerance.subproblem);

                        for (int q3Index = 0; q3Index < q3Set.n; ++q3Index) {
                            const real_t q3 = q3Set.a[q3Index];
                            const Mat3 R34 = rot(robot.H[3], q4);
                            const Mat3 R47 = mul(transpose(mul(mul(R02, rot(robot.H[2], q3)), R34)),
                                                 R07);

                            const Sp2Result q56 = sp2(robot.H[6], mul(R47, robot.H[6]),
                                                      robot.H[5], scale(robot.H[4], -1),
                                                      cfg.tolerance.subproblem);
                            for (int wristIndex = 0; wristIndex < q56.n; ++wristIndex) {
                                const real_t q5Representative = q56.theta2[wristIndex];
                                const real_t q6 = q56.theta1[wristIndex];
                                const Mat3 R56Representative = mul(rot(robot.H[4], q5Representative),
                                                                   rot(robot.H[5], q6));
                                const AngleSet q7RepresentativeSet =
                                    sp1(robot.H[5],
                                        mul(mul(transpose(R56Representative), R47), robot.H[5]),
                                        robot.H[6], cfg.tolerance.subproblem);

                                for (int q7RepIndex = 0; q7RepIndex < q7RepresentativeSet.n;
                                     ++q7RepIndex) {
                                    const real_t q7Representative =
                                        q7RepresentativeSet.a[q7RepIndex];
                                    real_t q5Candidates[2] = {};
                                    const int q5Count = singularOuterAngleCandidates(
                                        q5Representative, q6, q7Representative,
                                        robot.H[4], robot.H[5], robot.H[6],
                                        qCurrent.v[4], qCurrent.v[6],
                                        cfg.selection.weights[4], cfg.selection.weights[6],
                                        cfg.tolerance.sphericalSingularity, q5Candidates);

                                    for (int q5CandidateIndex = 0; q5CandidateIndex < q5Count;
                                         ++q5CandidateIndex) {
                                        const real_t q5 = q5Candidates[q5CandidateIndex];
                                        const Mat3 R56 = mul(rot(robot.H[4], q5),
                                                            rot(robot.H[5], q6));
                                        const AngleSet q7Set =
                                            sp1(robot.H[5],
                                                mul(mul(transpose(R56), R47), robot.H[5]),
                                                robot.H[6], cfg.tolerance.subproblem);

                                        for (int q7Index = 0; q7Index < q7Set.n; ++q7Index) {
                                            Vec7 q = {{q1, q2, q3, q4, q5, q6,
                                                       q7Set.a[q7Index]}};
                                            const bool isLS = q4Set.leastSquares ||
                                                              bc.leastSquares ||
                                                              thetaASet.leastSquares ||
                                                              q12.leastSquares ||
                                                              q3RepresentativeSet.leastSquares ||
                                                              q3Set.leastSquares ||
                                                              q56.leastSquares ||
                                                              q7RepresentativeSet.leastSquares ||
                                                              q7Set.leastSquares;
                                            // 只编码三层会成倍扩展候选数的主分支；
                                            // 奇异外角内部展开仍由该主分支内的最近解逻辑处理。
                                            const uint8_t majorBranch = static_cast<uint8_t>(
                                                (q4Index & 1) | ((shoulderIndex & 1) << 1) |
                                                ((wristIndex & 1) << 2));
                                            addCandidate(Q, q, isLS, majorBranch);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

inline void deduplicate(CandidateSet& set, real_t tolerance) {
    // 候选解去重：闭式推导的不同路径有时会得到 2*pi 等价解。
    // 比较时使用 wrapToPi(qA-qB)，避免因为多转一圈导致重复保留。
    CandidateSet unique;
    for (int i = 0; i < set.n; ++i) {
        int duplicate = -1;
        for (int j = 0; j < unique.n; ++j) {
            real_t maxDelta = 0;
            for (int k = 0; k < 7; ++k) {
                maxDelta = maxr(maxDelta, absr(wrapToPi(set.c[i].q.v[k] - unique.c[j].q.v[k])));
            }
            if (maxDelta <= tolerance) {
                duplicate = j;
                break;
            }
        }

        if (duplicate < 0) {
            unique.c[unique.n++] = set.c[i];
        } else {
            unique.c[duplicate].leastSquares =
                unique.c[duplicate].leastSquares && set.c[i].leastSquares;
        }
    }
    set = unique;
}

inline bool fitToJointLimits(const Vec7& q, const Vec7& qCurrent, const Config& cfg, Vec7& out) {
    // 将一个 wrap 到 [-pi,pi) 的候选解调整到实际关节限位内。
    // 如果 q、q+2pi、q-2pi 中有多个等价角都在限位内，则选离 qCurrent 最近的那个。
    for (int i = 0; i < 7; ++i) {
        const real_t lo = cfg.joint.lowerRad[i];
        const real_t hi = cfg.joint.upperRad[i];
        const real_t tol = cfg.tolerance.jointLimit;
        const int minTurn = static_cast<int>(std::ceil((lo - q.v[i] - tol) / kTwoPi));
        const int maxTurn = static_cast<int>(std::floor((hi - q.v[i] + tol) / kTwoPi));
        if (minTurn > maxTurn) {
            return false;
        }

        const real_t currentTurn = std::round((qCurrent.v[i] - q.v[i]) / kTwoPi);
        const int centerTurn = static_cast<int>(
            clamp(currentTurn, static_cast<real_t>(minTurn), static_cast<real_t>(maxTurn)));

        real_t best = q.v[i] + kTwoPi * static_cast<real_t>(centerTurn);
        real_t bestAbs = absr(best - qCurrent.v[i]);
        const int firstTurn = centerTurn - 1 < minTurn ? minTurn : centerTurn - 1;
        const int lastTurn = centerTurn + 1 > maxTurn ? maxTurn : centerTurn + 1;
        for (int turn = firstTurn; turn <= lastTurn; ++turn) {
            const real_t candidate = q.v[i] + kTwoPi * static_cast<real_t>(turn);
            const real_t d = absr(candidate - qCurrent.v[i]);
            if (d < bestAbs) {
                best = candidate;
                bestAbs = d;
            }
        }

        out.v[i] = clamp(best, lo, hi);
    }
    return true;
}

inline real_t weightedCommandDistanceSq(const Vec7& q,
                                        const Vec7& qCurrent,
                                        const Config& cfg) {
    // q 是 fitToJointLimits() 选出的实际关节命令表示。有限位关节不能默认
    // 跨越 +/-pi 边界，因此这里必须计算线性命令空间距离，不能 wrapToPi。
    real_t cost = 0;
    for (int i = 0; i < 7; ++i) {
        const real_t d = q.v[i] - qCurrent.v[i];
        cost += cfg.selection.weights[i] * d * d;
    }
    return cost;
}

inline bool validateConfig(const Config& cfg) {
    for (int i = 0; i < 7; ++i) {
        if (!(cfg.joint.lowerRad[i] < cfg.joint.upperRad[i])) {
            return false;
        }
        if (cfg.selection.weights[i] < static_cast<real_t>(0)) {
            return false;
        }
    }
    return cfg.geometry.upperArmMm > static_cast<real_t>(0) &&
           cfg.geometry.forearmMm > static_cast<real_t>(0) &&
           cfg.geometry.endEffectorMm >= static_cast<real_t>(0) &&
           cfg.geometry.flangeExtensionMm >= static_cast<real_t>(0);
}

}  // namespace detail

inline Config defaultConfig() {
    return detail::makeDefaultConfig();
}

inline Pose forwardKinematics(const Vec7& q, const Config& cfg = defaultConfig()) {
    return detail::forwardKinematics(q, detail::buildRobot(cfg));
}

namespace detail {

// 固定臂角解析求解的内部实现。CandidateFilter 只决定一个已经完成限位映射
// 的候选是否可用；基础 solve() 使用全接受策略，其他算法可在自身文件中
// 注入连续性约束，而不把策略语义耦合进闭式 IK。
template <typename CandidateFilter>
inline bool solveWithResolvedPsiFiltered(const Pose& targetWorld,
                                         const Vec7& qCurrent,
                                         real_t psi,
                                         bool psiWasInferredFromCurrent,
                                         const Config& cfg,
                                         IkResult& result,
                                         const CandidateFilter& candidateAccepted) {
    result = IkResult{};
    result.psiWasInferredFromCurrent = psiWasInferredFromCurrent;

    if (!detail::validateConfig(cfg)) {
        return false;
    }

    const detail::Robot robot = detail::buildRobot(cfg);
    const Pose target = detail::worldTargetToRobot(targetWorld, cfg.installation);

    psi = detail::wrapToPi(psi);
    result.psi = psi;

    const Mat3 R07 = detail::mul(target.R, detail::transpose(robot.RToolHome));

    // 完整生成解析候选，再统一执行去重、限位和 FK 验证。
    detail::CandidateSet raw;
    detail::ik3RR3R(R07, target.p, psi, qCurrent, robot, cfg, raw);
    detail::deduplicate(raw, cfg.tolerance.duplicateJoint);
    result.rawCandidateCount = raw.n;

    bool haveBest = false;
    real_t bestCost = static_cast<real_t>(0);
    Vec7 bestQ = {{0, 0, 0, 0, 0, 0, 0}};
    real_t bestPosErr = static_cast<real_t>(0);
    real_t bestOriErr = static_cast<real_t>(0);

    for (int i = 0; i < raw.n; ++i) {
        Vec7 qLimited;
        if (!detail::fitToJointLimits(raw.c[i].q, qCurrent, cfg, qLimited)) {
            continue;
        }
        if (!candidateAccepted(qLimited)) {
            continue;
        }
        if (!cfg.solver.acceptLeastSquares && raw.c[i].leastSquares) {
            continue;
        }

        const Pose check = detail::forwardKinematics(qLimited, robot);
        const real_t posErr = detail::norm(detail::sub(check.p, target.p));
        const real_t oriErr = detail::rotationDistance(check.R, target.R);
        if (posErr > cfg.tolerance.positionMm || oriErr > cfg.tolerance.orientationRad) {
            continue;
        }

        const real_t cost = detail::weightedCommandDistanceSq(qLimited, qCurrent, cfg);
        if (!haveBest || cost < bestCost) {
            haveBest = true;
            bestCost = cost;
            bestQ = qLimited;
            bestPosErr = posErr;
            bestOriErr = oriErr;
            result.selectedValidIndex = result.validCandidateCount;
            // 与 qBest 同步保存分支，调用方只能在 success=true 时使用它延续状态。
            result.selectedMajorBranch = static_cast<int>(raw.c[i].majorBranch);
        }
        ++result.validCandidateCount;
    }

    if (!haveBest) {
        result.success = false;
        return false;
    }

    result.success = true;
    result.qBest = bestQ;
    result.bestCost = bestCost;
    result.bestPositionErrorMm = bestPosErr;
    result.bestOrientationErrorRad = bestOriErr;
    return true;
}

// 基础固定-psi 入口完整接受所有通过限位和 FK 校验的解析候选。
inline bool solveWithResolvedPsi(const Pose& targetWorld,
                                 const Vec7& qCurrent,
                                 real_t psi,
                                 bool psiWasInferredFromCurrent,
                                 const Config& cfg,
                                 IkResult& result) {
    const auto acceptAll = [](const Vec7&) { return true; };
    return solveWithResolvedPsiFiltered(
        targetWorld, qCurrent, psi, psiWasInferredFromCurrent, cfg, result, acceptAll);
}

inline bool inferCurrentPsi(const Vec7& qCurrent, const Config& cfg, real_t& psi) {
    if (!validateConfig(cfg)) {
        return false;
    }

    const Robot robot = buildRobot(cfg);
    Vec3 S;
    Vec3 E;
    Vec3 W;
    armPoints(qCurrent, robot, S, E, W);
    return sewForward(S, E, W, cfg, psi);
}

}  // namespace detail

// 逆解主入口。
// psiIsGiven=true：使用传入的 psi；
// psiIsGiven=false：从 qCurrent 自动推断当前 SEW 角。若当前 psi 无解，
// 再按 +/-5deg、+/-10deg、+/-20deg、+/-40deg 做局部臂角搜索。
inline bool solve(const Pose& targetWorld,
                  const Vec7& qCurrent,
                  real_t psi,
                  bool psiIsGiven,
                  const Config& cfg,
                  IkResult& result) {
    if (psiIsGiven) {
        return detail::solveWithResolvedPsi(targetWorld, qCurrent, psi, false, cfg, result);
    }

    if (!detail::inferCurrentPsi(qCurrent, cfg, psi)) {
        result = IkResult{};
        result.psiWasInferredFromCurrent = true;
        return false;
    }

    IkResult baseResult;
    if (detail::solveWithResolvedPsi(targetWorld, qCurrent, psi, true, cfg, baseResult)) {
        result = baseResult;
        return true;
    }

    static constexpr real_t kDegToRad = kPi / static_cast<real_t>(180);
    static constexpr real_t kPsiSearchOffsets[] = {
        static_cast<real_t>(5) * kDegToRad,
        static_cast<real_t>(-5) * kDegToRad,
        static_cast<real_t>(10) * kDegToRad,
        static_cast<real_t>(-10) * kDegToRad,
        static_cast<real_t>(20) * kDegToRad,
        static_cast<real_t>(-20) * kDegToRad,
        static_cast<real_t>(40) * kDegToRad,
        static_cast<real_t>(-40) * kDegToRad,
    };

    bool haveBest = false;
    IkResult bestResult;
    for (real_t const offset : kPsiSearchOffsets) {
        IkResult candidateResult;
        if (!detail::solveWithResolvedPsi(targetWorld, qCurrent, psi + offset, true, cfg, candidateResult)) {
            continue;
        }
        if (!haveBest || candidateResult.bestCost < bestResult.bestCost) {
            haveBest = true;
            bestResult = candidateResult;
        }
    }

    result = haveBest ? bestResult : baseResult;
    return haveBest;
}

inline bool solve(const Pose& targetWorld,
                  const Vec7& qCurrent,
                  const Config& cfg,
                  IkResult& result) {
    return solve(targetWorld, qCurrent, static_cast<real_t>(0), false, cfg, result);
}

}  // namespace roboarm7r
