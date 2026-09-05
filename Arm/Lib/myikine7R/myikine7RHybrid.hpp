#pragma once

// 7R 机械臂有界混合 IK。
//
// 基础数据结构、FK 与固定-psi 闭式候选法由 myikine7R.hpp 提供；本文件
// 负责 SEW 流形/腕部命令曲线投影、Hybrid 风险判定、评分与调度逻辑。
// 两个头文件使用同一个 roboarm7r 命名空间，调用接口保持为：
//   roboarm7r::solveHybrid(target, qCurrent, cfg, result)

#include "myikine7R.hpp"

namespace roboarm7r {
namespace detail {

// 将当前物理肘部方向投影到新目标的肩-腕交圆，再在目标自己的 SEW 图中
// 计算 psi。该连续化只服务于 Hybrid 的腕部命令曲线投影。
inline bool inferTransferredPsi(const Pose& targetWorld,
                                const Vec7& qCurrent,
                                const Config& cfg,
                                real_t& psi) {
    if (!validateConfig(cfg)) {
        return false;
    }
    const Robot robot = buildRobot(cfg);
    const Pose target = worldTargetToRobot(targetWorld, cfg.installation);
    const Mat3 R07 = mul(target.R, transpose(robot.RToolHome));
    const Vec3 targetW = sub(target.p, mul(R07, robot.P[7]));

    Vec3 S;
    Vec3 currentE;
    Vec3 currentW;
    armPoints(qCurrent, robot, S, currentE, currentW);
    (void)currentW;

    const Vec3 pSW = sub(targetW, S);
    const real_t distanceSW = norm(pSW);
    const real_t upperLength = norm(robot.P[3]);
    const real_t forearmLength = norm(robot.P[4]);
    if (distanceSW <= cfg.tolerance.geometry ||
        distanceSW > upperLength + forearmLength + cfg.tolerance.geometry ||
        distanceSW < absr(upperLength - forearmLength) - cfg.tolerance.geometry) {
        return false;
    }

    const Vec3 eSW = scale(pSW, static_cast<real_t>(1) / distanceSW);
    const real_t circleAxial =
        (upperLength * upperLength - forearmLength * forearmLength +
         distanceSW * distanceSW) /
        (static_cast<real_t>(2) * distanceSW);
    real_t circleRadiusSq = upperLength * upperLength - circleAxial * circleAxial;
    if (circleRadiusSq < -cfg.tolerance.geometry) {
        return false;
    }
    circleRadiusSq = maxr(circleRadiusSq, static_cast<real_t>(0));
    const real_t circleRadius = std::sqrt(circleRadiusSq);
    const Vec3 circleCenter = add(S, scale(eSW, circleAxial));
    Vec3 radial = sub(currentE, circleCenter);
    radial = sub(radial, scale(eSW, dot(radial, eSW)));
    const real_t radialNorm = norm(radial);
    if (radialNorm <= cfg.tolerance.geometry ||
        circleRadius <= cfg.tolerance.geometry) {
        return false;
    }
    const Vec3 targetE = add(circleCenter, scale(radial, circleRadius / radialNorm));
    return sewForward(S, targetE, targetW, cfg, psi);
}

}  // namespace detail

// 流形投影思路（带可行域恢复的最优解析迭代）逆解入口。
// 正常路径只在 psi_0 及其左右微扰处估计梯度/Hessian；中心点无解时，
// 才分级搜索 +/-5/10/20/40deg 恢复可行域。
inline bool solveManifoldProjection(const Pose& targetWorld,
                                    const Vec7& qCurrent,
                                    const Config& cfg,
                                    IkResult& finalResult) {
    real_t psi0 = static_cast<real_t>(0);
    if (!detail::inferCurrentPsi(qCurrent, cfg, psi0)) {
        const bool ok = solve(targetWorld, qCurrent, cfg, finalResult);
        finalResult.manifoldUsedFeasibilityRecovery = true;
        return ok;
    }

    const real_t deltaPsi = static_cast<real_t>(0.01);
    static constexpr real_t kDegToRad = kPi / static_cast<real_t>(180);
    static constexpr real_t kRecoveryOffsets[] = {
        static_cast<real_t>(5) * kDegToRad,
        static_cast<real_t>(-5) * kDegToRad,
        static_cast<real_t>(10) * kDegToRad,
        static_cast<real_t>(-10) * kDegToRad,
        static_cast<real_t>(20) * kDegToRad,
        static_cast<real_t>(-20) * kDegToRad,
        static_cast<real_t>(40) * kDegToRad,
        static_cast<real_t>(-40) * kDegToRad,
    };
    int evaluationCount = 0;

    auto evaluate = [&](real_t psi, IkResult& result) {
        ++evaluationCount;
        return detail::solveWithResolvedPsi(
            targetWorld, qCurrent, psi, true, cfg, result);
    };

    bool haveBest = false;
    IkResult bestResult;
    real_t bestCost = static_cast<real_t>(0);
    auto consider = [&](bool ok, const IkResult& candidate) {
        if (!ok) {
            return;
        }
        const real_t cost =
            detail::weightedCommandDistanceSq(candidate.qBest, qCurrent, cfg);
        if (!haveBest || cost < bestCost) {
            haveBest = true;
            bestCost = cost;
            bestResult = candidate;
        }
    };
    auto runExpandedSearch = [&]() {
        for (real_t const offset : kRecoveryOffsets) {
            IkResult recovered;
            const bool okRecovered = evaluate(psi0 + offset, recovered);
            consider(okRecovered, recovered);
        }
    };

    IkResult resCenter, resPlus, resMinus;
    const bool okCenter = evaluate(psi0, resCenter);
    const bool okPlus = evaluate(psi0 + deltaPsi, resPlus);
    const bool okMinus = evaluate(psi0 - deltaPsi, resMinus);
    consider(okCenter, resCenter);
    consider(okPlus, resPlus);
    consider(okMinus, resMinus);

    const bool usedRecovery = !okCenter;
    real_t localPsi = psi0;
    IkResult localCenter = resCenter;
    bool okLocalCenter = okCenter;
    IkResult localPlus = resPlus;
    IkResult localMinus = resMinus;
    bool okLocalPlus = okPlus;
    bool okLocalMinus = okMinus;

    if (!okCenter) {
        runExpandedSearch();
        if (!haveBest) {
            finalResult = resCenter;
            finalResult.manifoldEvaluationCount = evaluationCount;
            finalResult.manifoldUsedFeasibilityRecovery = true;
            return false;
        }

        localPsi = bestResult.psi;
        localCenter = bestResult;
        okLocalCenter = true;
        okLocalPlus = evaluate(localPsi + deltaPsi, localPlus);
        okLocalMinus = evaluate(localPsi - deltaPsi, localMinus);
        consider(okLocalPlus, localPlus);
        consider(okLocalMinus, localMinus);
    }

    if (okLocalCenter && okLocalPlus && okLocalMinus) {
        const real_t costCenter =
            detail::weightedCommandDistanceSq(localCenter.qBest, qCurrent, cfg);
        const real_t costPlus =
            detail::weightedCommandDistanceSq(localPlus.qBest, qCurrent, cfg);
        const real_t costMinus =
            detail::weightedCommandDistanceSq(localMinus.qBest, qCurrent, cfg);
        const real_t gradient =
            (costPlus - costMinus) / (static_cast<real_t>(2) * deltaPsi);
        const real_t hessian =
            (costPlus - static_cast<real_t>(2) * costCenter + costMinus) /
            (deltaPsi * deltaPsi);

        real_t step = static_cast<real_t>(0);
        const real_t hessianEpsilon = static_cast<real_t>(1e-5);
        if (hessian > hessianEpsilon) {
            step = detail::clamp(-gradient / hessian,
                                 static_cast<real_t>(-0.1),
                                 static_cast<real_t>(0.1));
        } else {
            step = detail::clamp(-gradient * static_cast<real_t>(0.05),
                                 static_cast<real_t>(-0.05),
                                 static_cast<real_t>(0.05));
        }

        if (detail::absr(step) > static_cast<real_t>(1e-5)) {
            IkResult resOptimal;
            const bool okOptimal = evaluate(localPsi + step, resOptimal);
            consider(okOptimal, resOptimal);
        }
    }

    if (!haveBest) {
        finalResult = resCenter;
        finalResult.manifoldEvaluationCount = evaluationCount;
        finalResult.manifoldUsedFeasibilityRecovery = usedRecovery;
        return false;
    }

    finalResult = bestResult;
    finalResult.manifoldEvaluationCount = evaluationCount;
    finalResult.manifoldUsedFeasibilityRecovery = usedRecovery;
    return true;
}

// 目标特定的电机命令曲线前向割线投影。固定末端位姿后，精确 IK 解
// q(psi) 是一维曲线；这里用中心点和单侧微扰近似切向并执行一次投影。
// 腕部外轴候选仍受 90deg 单步保护，避免用解析等价支换取表面上的成功。
inline bool solveMotorCurveProjection(const Pose& targetWorld,
                                      const Vec7& qCurrent,
                                      const Config& cfg,
                                      IkResult& finalResult) {
    real_t psi0 = static_cast<real_t>(0);
    if (!detail::inferTransferredPsi(targetWorld, qCurrent, cfg, psi0) &&
        !detail::inferCurrentPsi(qCurrent, cfg, psi0)) {
        const bool ok = solve(targetWorld, qCurrent, cfg, finalResult);
        finalResult.manifoldUsedFeasibilityRecovery = true;
        return ok;
    }

    const real_t deltaPsi = static_cast<real_t>(0.01);
    const real_t maxWristOuterStep = kPi / static_cast<real_t>(2);
    int evaluationCount = 0;
    auto evaluate = [&](real_t psi, IkResult& result) {
        ++evaluationCount;
        const auto wristOuterStepAccepted = [&](const Vec7& candidate) {
            return detail::absr(candidate.v[4] - qCurrent.v[4]) <= maxWristOuterStep &&
                   detail::absr(candidate.v[6] - qCurrent.v[6]) <= maxWristOuterStep;
        };
        return detail::solveWithResolvedPsiFiltered(
            targetWorld, qCurrent, psi, true, cfg, result, wristOuterStepAccepted);
    };

    bool haveBest = false;
    real_t bestCost = static_cast<real_t>(0);
    IkResult bestResult;
    auto consider = [&](bool ok, const IkResult& candidate) {
        if (!ok) {
            return;
        }
        const real_t cost =
            detail::weightedCommandDistanceSq(candidate.qBest, qCurrent, cfg);
        if (!haveBest || cost < bestCost) {
            haveBest = true;
            bestCost = cost;
            bestResult = candidate;
        }
    };

    IkResult center;
    IkResult plus;
    const bool okCenter = evaluate(psi0, center);
    const bool okPlus = evaluate(psi0 + deltaPsi, plus);
    consider(okCenter, center);
    consider(okPlus, plus);

    if (!okCenter) {
        static constexpr real_t kDegToRad = kPi / static_cast<real_t>(180);
        IkResult recoveredMinus;
        consider(evaluate(psi0 - deltaPsi, recoveredMinus), recoveredMinus);
        if (!haveBest) {
            auto searchRadius = [&](real_t radiusDeg) {
                IkResult recoveredPlus;
                IkResult recoveredMinusRadius;
                consider(evaluate(psi0 + radiusDeg * kDegToRad, recoveredPlus),
                         recoveredPlus);
                consider(evaluate(psi0 - radiusDeg * kDegToRad, recoveredMinusRadius),
                         recoveredMinusRadius);
            };
            for (int radiusDeg = 1; radiusDeg <= 10 && !haveBest; ++radiusDeg) {
                searchRadius(static_cast<real_t>(radiusDeg));
            }
            for (int radiusDeg = 15; radiusDeg <= 180 && !haveBest; radiusDeg += 5) {
                searchRadius(static_cast<real_t>(radiusDeg));
            }
        }
        finalResult = haveBest ? bestResult : center;
        finalResult.manifoldEvaluationCount = evaluationCount;
        finalResult.manifoldUsedFeasibilityRecovery = true;
        return haveBest;
    }

    if (okPlus) {
        real_t numerator = static_cast<real_t>(0);
        real_t denominator = static_cast<real_t>(0);
        for (int i = 0; i < 7; ++i) {
            const real_t tangent = (plus.qBest.v[i] - center.qBest.v[i]) / deltaPsi;
            const real_t residual = center.qBest.v[i] - qCurrent.v[i];
            const real_t weight = cfg.selection.weights[i];
            numerator += weight * residual * tangent;
            denominator += weight * tangent * tangent;
        }

        const real_t tangentEpsilon = static_cast<real_t>(1e-8);
        if (denominator > tangentEpsilon) {
            const real_t step = detail::clamp(
                -numerator / denominator,
                static_cast<real_t>(-0.1),
                static_cast<real_t>(0.1));
            if (detail::absr(step) > static_cast<real_t>(1e-5) &&
                detail::absr(step - deltaPsi) > static_cast<real_t>(1e-5)) {
                IkResult trial;
                consider(evaluate(psi0 + step, trial), trial);
            }
        }
    }

    finalResult = bestResult;
    finalResult.manifoldEvaluationCount = evaluationCount;
    finalResult.manifoldUsedFeasibilityRecovery = false;
    return haveBest;
}

// Hybrid 使用的有界启发代价。第一项是真实关节命令变化；其余项只在
// 关节限位或解析奇异附近逐渐生效，用来在两个精确 IK 解之间做选择。
inline real_t calculateHybridScore(const Vec7& q,
                                   const Vec7& qCurrent,
                                   const Config& cfg) {
    real_t score = detail::weightedCommandDistanceSq(q, qCurrent, cfg);

    // 限位最外侧 8% 使用软惩罚。它不改变解，只影响候选/流形结果二选一。
    const real_t softLimitRatio = static_cast<real_t>(0.08);
    const real_t limitPenalty = static_cast<real_t>(0.25);
    for (int i = 0; i < 7; ++i) {
        const real_t lo = cfg.joint.lowerRad[i];
        const real_t hi = cfg.joint.upperRad[i];
        const real_t span = hi - lo;
        const real_t margin = detail::minr(q.v[i] - lo, hi - q.v[i]) / span;
        if (margin < softLimitRatio) {
            const real_t x = detail::clamp(
                (softLimitRatio - margin) / softLimitRatio,
                static_cast<real_t>(0), static_cast<real_t>(1));
            score += cfg.selection.weights[i] * limitPenalty * x * x;
        }
    }

    // 当前轴序列中：q2 是肩部中间角，q4 是肘角，q6 是腕部中间角。
    // 这些廉价指标避免在 MCU 上每周期计算完整 Jacobian SVD。
    const real_t shoulderMetric = detail::absr(std::cos(q.v[1]));
    const real_t elbowMetric = detail::absr(std::sin(q.v[3]));
    const real_t wristMetric = detail::absr(std::sin(q.v[5]));
    auto singularPenalty = [&](real_t metric, real_t threshold, real_t weight) {
        if (metric >= threshold) {
            return static_cast<real_t>(0);
        }
        const real_t x = (threshold - metric) / threshold;
        return weight * x * x;
    };
    score += singularPenalty(shoulderMetric, static_cast<real_t>(0.10), static_cast<real_t>(0.10));
    score += singularPenalty(elbowMetric, static_cast<real_t>(0.10), static_cast<real_t>(0.20));
    score += singularPenalty(wristMetric, static_cast<real_t>(0.15), static_cast<real_t>(0.30));

    // 腕部接近奇异时 q5/q7 强耦合；对两侧外角同时大幅变化额外惩罚。
    if (wristMetric < static_cast<real_t>(0.20)) {
        const real_t wristMotion = detail::absr(q.v[4] - qCurrent.v[4]) +
                                   detail::absr(q.v[6] - qCurrent.v[6]);
        const real_t freeMotion = static_cast<real_t>(20) * kPi / static_cast<real_t>(180);
        if (wristMotion > freeMotion) {
            const real_t excess = wristMotion - freeMotion;
            score += static_cast<real_t>(0.50) * excess * excess;
        }
    }
    return score;
}

inline bool hybridNeedsRobustPath(const Vec7& q,
                                  const Vec7& qCurrent,
                                  const Config& cfg) {
    const real_t largeStep = kPi / static_cast<real_t>(2);  // 任一关节超过 90deg
    for (int i = 0; i < 7; ++i) {
        if (detail::absr(q.v[i] - qCurrent.v[i]) > largeStep) {
            return true;
        }
        const real_t lo = cfg.joint.lowerRad[i];
        const real_t hi = cfg.joint.upperRad[i];
        const real_t normalizedMargin =
            detail::minr(q.v[i] - lo, hi - q.v[i]) / (hi - lo);
        if (normalizedMargin < static_cast<real_t>(0.04)) {
            return true;
        }
    }
    return detail::absr(std::cos(q.v[1])) < static_cast<real_t>(0.08) ||
           detail::absr(std::sin(q.v[3])) < static_cast<real_t>(0.08) ||
           detail::absr(std::sin(q.v[5])) < static_cast<real_t>(0.12);
}

// 有界混合 IK：
//   1. 先运行候选法；中心 psi 可行时通常只需一次闭式候选快解；
//   2. 快解远离限位/奇异且单步不大时立即返回；
//   3. 只有风险路径才运行流形投影，并在两个精确解之间按 Hybrid score 选择。
// 正常路径仅一次固定-psi 闭式调用；风险路径的额外调用数由流形诊断给出。
inline bool solveHybrid(const Pose& targetWorld,
                        const Vec7& qCurrent,
                        const Config& cfg,
                        IkResult& finalResult) {
    real_t psi0 = static_cast<real_t>(0);
    if (!detail::inferCurrentPsi(qCurrent, cfg, psi0)) {
        const bool ok = solveManifoldProjection(targetWorld, qCurrent, cfg, finalResult);
        finalResult.hybridRiskTriggered = true;
        finalResult.hybridStage = ok ? HybridStage::Manifold : HybridStage::GeometricFallback;
        finalResult.hybridEvaluationCount = finalResult.manifoldEvaluationCount;
        if (ok) {
            finalResult.hybridScore = calculateHybridScore(finalResult.qBest, qCurrent, cfg);
        }
        return ok;
    }

    IkResult fastResult;
    const bool okFast = solve(targetWorld, qCurrent, cfg, fastResult);
    const bool fastUsedCenter = okFast &&
        detail::absr(detail::wrapToPi(fastResult.psi - psi0)) < static_cast<real_t>(1e-5);
    int evaluationCount = fastUsedCenter ? 1 : 9;
    bool forceWristManifold = false;

    // solve() 已经完成 9 点恢复时不再重复执行流形恢复。这样 Hybrid 的
    // 固定-psi 闭式调用硬上限保持为 9；恢复仍失败则让上层保持当前目标。
    if (!okFast) {
        finalResult = fastResult;
        finalResult.hybridEvaluationCount = evaluationCount;
        finalResult.hybridRiskTriggered = true;
        finalResult.hybridStage = HybridStage::GeometricFallback;
        return false;
    }
    if (!fastUsedCenter) {
        finalResult = fastResult;
        finalResult.hybridEvaluationCount = evaluationCount;
        finalResult.hybridRiskTriggered = true;
        finalResult.hybridStage = HybridStage::GeometricFallback;
        finalResult.hybridScore = calculateHybridScore(finalResult.qBest, qCurrent, cfg);
        return true;
    }

    {
        const real_t wristMetric = detail::absr(std::sin(fastResult.qBest.v[5]));
        real_t maxCommandStep = static_cast<real_t>(0);
        for (int i = 0; i < 7; ++i) {
            maxCommandStep = detail::maxr(
                maxCommandStep,
                detail::absr(fastResult.qBest.v[i] - qCurrent.v[i]));
        }
        const real_t wristOuterStep =
            detail::absr(fastResult.qBest.v[4] - qCurrent.v[4]) +
            detail::absr(fastResult.qBest.v[6] - qCurrent.v[6]);
        const real_t wrist5Margin = detail::minr(
            fastResult.qBest.v[4] - cfg.joint.lowerRad[4],
            cfg.joint.upperRad[4] - fastResult.qBest.v[4]) /
            (cfg.joint.upperRad[4] - cfg.joint.lowerRad[4]);
        const real_t wrist7Margin = detail::minr(
            fastResult.qBest.v[6] - cfg.joint.lowerRad[6],
            cfg.joint.upperRad[6] - fastResult.qBest.v[6]) /
            (cfg.joint.upperRad[6] - cfg.joint.lowerRad[6]);
        forceWristManifold = wristMetric < static_cast<real_t>(0.50) &&
                             (wrist5Margin < static_cast<real_t>(0.08) ||
                              wrist7Margin < static_cast<real_t>(0.08));

        // 精确球腕奇异附近优先保留候选法已有的外角连续化结果。
        // 只有它本身已经发生大翻腕或腕外角靠近限位时才进入流形路径。
        if (!forceWristManifold && wristMetric < static_cast<real_t>(0.20) &&
            maxCommandStep < kPi / static_cast<real_t>(2) &&
            wristOuterStep < kPi / static_cast<real_t>(2) &&
            wrist5Margin > static_cast<real_t>(0.08) &&
            wrist7Margin > static_cast<real_t>(0.08)) {
            finalResult = fastResult;
            finalResult.hybridEvaluationCount = evaluationCount;
            finalResult.hybridRiskTriggered = true;
            finalResult.hybridStage = HybridStage::FastAnalytic;
            finalResult.hybridScore = calculateHybridScore(finalResult.qBest, qCurrent, cfg);
            return true;
        }
    }

    if (!forceWristManifold &&
        !hybridNeedsRobustPath(fastResult.qBest, qCurrent, cfg)) {
        finalResult = fastResult;
        finalResult.hybridEvaluationCount = evaluationCount;
        finalResult.hybridRiskTriggered = false;
        finalResult.hybridStage = HybridStage::FastAnalytic;
        finalResult.hybridScore = calculateHybridScore(finalResult.qBest, qCurrent, cfg);
        return true;
    }

    IkResult manifoldResult;
    const bool okManifold = solveManifoldProjection(targetWorld, qCurrent, cfg, manifoldResult);
    evaluationCount += manifoldResult.manifoldEvaluationCount;

    const bool haveAny = okFast || okManifold;
    if (!haveAny) {
        finalResult = manifoldResult;
        finalResult.hybridEvaluationCount = evaluationCount;
        finalResult.hybridRiskTriggered = true;
        finalResult.hybridStage = HybridStage::GeometricFallback;
        return false;
    }

    const real_t fastScore = okFast
                                 ? calculateHybridScore(fastResult.qBest, qCurrent, cfg)
                                 : static_cast<real_t>(0);
    const real_t manifoldScore = okManifold
                                     ? calculateHybridScore(manifoldResult.qBest, qCurrent, cfg)
                                     : static_cast<real_t>(0);
    finalResult = okFast ? fastResult : manifoldResult;
    real_t selectedScore = okFast ? fastScore : manifoldScore;
    HybridStage selectedStage = okFast ? HybridStage::FastAnalytic
                                       : HybridStage::Manifold;
    if (okManifold && (forceWristManifold || !okFast || manifoldScore < selectedScore)) {
        finalResult = manifoldResult;
        selectedScore = manifoldScore;
        selectedStage = HybridStage::Manifold;
    }
    finalResult.hybridEvaluationCount = evaluationCount;
    finalResult.hybridRiskTriggered = true;
    finalResult.hybridStage = selectedStage;
    finalResult.hybridScore = selectedScore;
    return true;
}

}  // namespace roboarm7r
