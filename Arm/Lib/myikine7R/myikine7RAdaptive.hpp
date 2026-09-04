#pragma once
// AdaptiveHybrid：按实际末端到目标的笛卡尔变化量，在 DLS8 与原 Hybrid
// 之间做互斥选择。互斥是时间预算的关键：一个周期只运行一个求解器，
// 不采用“DLS 失败后同周期再跑 Hybrid”的串行回退。
#include "myikine7RDls.hpp"
#include "myikine7RHybrid.hpp"

namespace roboarm7r {
namespace detail {

inline bool validateAdaptiveConfig(const Config& cfg) {
    return validateConfig(cfg) &&
           cfg.solver.adaptiveDlsPositionThresholdMm >= static_cast<real_t>(0) &&
           cfg.solver.adaptiveDlsOrientationThresholdRad >= static_cast<real_t>(0);
}

}  // namespace detail

// 固件主入口。currentWorld 由 ArmMiddleware 每周期已有的 FK 状态提供，
// 路由判定不需要额外做一次 FK。
inline bool solveAdaptiveHybrid(const Pose& targetWorld,
                                const Pose& currentWorld,
                                const Vec7& qCurrent,
                                const Config& cfg,
                                IkResult& result) {
    if (!detail::validateAdaptiveConfig(cfg)) {
        result = IkResult{};
        result.qBest = qCurrent;
        return false;
    }

    // 两个位姿先统一转换到机器人基座系，避免安装旋转/平移影响阈值含义。
    const Pose current = detail::worldTargetToRobot(currentWorld, cfg.installation);
    const Pose target = detail::worldTargetToRobot(targetWorld, cfg.installation);
    const real_t positionDelta = detail::norm(detail::sub(target.p, current.p));
    const real_t orientationDelta = detail::rotationDistance(current.R, target.R);

    // 必须同时满足平移和旋转阈值才属于局部变化。任一方向较大都交给
    // 具有全局解析/流形恢复能力的 Hybrid，避免 DLS8 超出 20deg 信赖域。
    const bool useDls =
        positionDelta <= cfg.solver.adaptiveDlsPositionThresholdMm &&
        orientationDelta <= cfg.solver.adaptiveDlsOrientationThresholdRad;

    // 注意：这里只做一次互斥分发。选中的求解器失败时直接返回 false，
    // 上层保持上一目标，从而维持 max(T_DLS8, T_Hybrid) 的结构预算。
    const bool ok = useDls
                        ? solveDls(targetWorld, qCurrent, cfg, result)
                        : solveHybrid(targetWorld, qCurrent, cfg, result);

    // 无论成功失败都保留路由依据，便于 SystemView/实机日志定位误路由。
    result.adaptiveRoute = useDls ? static_cast<uint8_t>(1)
                                  : static_cast<uint8_t>(2);
    result.adaptivePositionDeltaMm = positionDelta;
    result.adaptiveOrientationDeltaRad = orientationDelta;
    return ok;
}

// 兼容入口：旧调用方只有 qCurrent，没有缓存的当前笛卡尔位姿。
// 此重载会额外执行一次 FK；固件 ArmMiddleware 正常路径不走这里。
inline bool solveAdaptiveHybrid(const Pose& targetWorld,
                                const Vec7& qCurrent,
                                const Config& cfg,
                                IkResult& result) {
    if (!detail::validateAdaptiveConfig(cfg)) {
        result = IkResult{};
        result.qBest = qCurrent;
        return false;
    }

    const detail::Robot robot = detail::buildRobot(cfg);
    const Pose current = detail::forwardKinematics(qCurrent, robot);

    // forwardKinematics() 返回机器人基座系位姿；转换到世界系后复用主入口，
    // 确保两个重载的阈值计算与安装配置完全一致。
    Pose currentWorld{};
    currentWorld.R = detail::mul(cfg.installation.rotationWorldFromRobot, current.R);
    currentWorld.p = detail::add(
        detail::mul(cfg.installation.rotationWorldFromRobot, current.p),
        cfg.installation.positionWorldMm);
    return solveAdaptiveHybrid(targetWorld, currentWorld, qCurrent, cfg, result);
}

}  // namespace roboarm7r
