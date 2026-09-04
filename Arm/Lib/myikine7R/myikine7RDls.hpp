#pragma once

// 7R 机械臂固定预算电机命令空间 DLS。
//
// 与 SEW 流形法不同，本解算器从当前真实反馈对应的 qCurrent 直接做局部
// 位姿投影，不先把冗余自由度降维成臂角，因此不会因为 SEW 坐标图换支而
// 主动选择远端腕解析支。实现约束：无动态内存、解析几何 Jacobian、固定
// 6x6 带主元线性求解，适合在 MCU 上做 cycle 实测。

#include "myikine7R.hpp"

namespace roboarm7r {
namespace detail {

inline Vec3 matrixColumn(Mat3 const& matrix, int column) {
    return vec(matrix.m[0][column], matrix.m[1][column], matrix.m[2][column]);
}

inline Vec3 dlsOrientationError(Mat3 const& actual, Mat3 const& target) {
    Vec3 error = vec(0, 0, 0);
    for (int column = 0; column < 3; ++column) {
        error = add(error, cross(matrixColumn(actual, column),
                                 matrixColumn(target, column)));
    }
    return scale(error, static_cast<real_t>(0.5));
}

// 同时计算 FK 和世界/机器人基坐标系下的 6x7 几何 Jacobian。
// 平移行使用 m，旋转行乘 orientationScaleM，从而与误差向量量纲一致。
inline Pose dlsForwardAndJacobian(Vec7 const& q,
                                  Robot const& robot,
                                  real_t orientationScaleM,
                                  real_t jacobian[6][7]) {
    Mat3 rotation = eye();
    Vec3 position = robot.P[0];
    Vec3 axes[7]{};
    Vec3 origins[7]{};
    for (int joint = 0; joint < 7; ++joint) {
        axes[joint] = mul(rotation, robot.H[joint]);
        origins[joint] = position;
        rotation = mul(rotation, rot(robot.H[joint], q.v[joint]));
        position = add(position, mul(rotation, robot.P[joint + 1]));
    }

    for (int joint = 0; joint < 7; ++joint) {
        const Vec3 linearMm = cross(axes[joint], sub(position, origins[joint]));
        jacobian[0][joint] = linearMm.x * static_cast<real_t>(1.0e-3);
        jacobian[1][joint] = linearMm.y * static_cast<real_t>(1.0e-3);
        jacobian[2][joint] = linearMm.z * static_cast<real_t>(1.0e-3);
        jacobian[3][joint] = axes[joint].x * orientationScaleM;
        jacobian[4][joint] = axes[joint].y * orientationScaleM;
        jacobian[5][joint] = axes[joint].z * orientationScaleM;
    }

    Pose pose{};
    pose.R = mul(rotation, robot.RToolHome);
    pose.p = position;
    return pose;
}

inline bool solveLinear6(real_t matrix[6][6], real_t rhs[6], real_t solution[6]) {
    const real_t pivotTolerance = static_cast<real_t>(1.0e-12);
    for (int column = 0; column < 6; ++column) {
        int pivotRow = column;
        real_t pivotAbs = absr(matrix[column][column]);
        for (int row = column + 1; row < 6; ++row) {
            const real_t candidateAbs = absr(matrix[row][column]);
            if (candidateAbs > pivotAbs) {
                pivotAbs = candidateAbs;
                pivotRow = row;
            }
        }
        if (pivotAbs <= pivotTolerance) {
            return false;
        }
        if (pivotRow != column) {
            for (int k = column; k < 6; ++k) {
                const real_t temporary = matrix[column][k];
                matrix[column][k] = matrix[pivotRow][k];
                matrix[pivotRow][k] = temporary;
            }
            const real_t temporary = rhs[column];
            rhs[column] = rhs[pivotRow];
            rhs[pivotRow] = temporary;
        }

        const real_t pivot = matrix[column][column];
        for (int row = column + 1; row < 6; ++row) {
            const real_t factor = matrix[row][column] / pivot;
            matrix[row][column] = static_cast<real_t>(0);
            for (int k = column + 1; k < 6; ++k) {
                matrix[row][k] -= factor * matrix[column][k];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    for (int row = 5; row >= 0; --row) {
        real_t value = rhs[row];
        for (int column = row + 1; column < 6; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return true;
}

inline bool validateDlsJointWeights(const real_t weights[7]) {
    for (int joint = 0; joint < 7; ++joint) {
        if (!std::isfinite(weights[joint]) ||
            weights[joint] <= static_cast<real_t>(0)) {
            return false;
        }
    }
    return true;
}

inline bool validateDlsShoulderConstraint(const SolverConfig& solver) {
    return std::isfinite(solver.dlsShoulderDeadbandRad) &&
           solver.dlsShoulderDeadbandRad > static_cast<real_t>(0) &&
           solver.dlsShoulderDeadbandRad < kPi / static_cast<real_t>(2) &&
           std::isfinite(solver.dlsShoulderOuterMaxDeltaRad) &&
           solver.dlsShoulderOuterMaxDeltaRad > static_cast<real_t>(0) &&
           std::isfinite(solver.dlsShoulderRelaxedPositionMm) &&
           solver.dlsShoulderRelaxedPositionMm > static_cast<real_t>(0) &&
           std::isfinite(solver.dlsShoulderRelaxedOrientationRad) &&
           solver.dlsShoulderRelaxedOrientationRad > static_cast<real_t>(0);
}

// 球肩轴序列 Z/-Y/-X 在 q2=pi/2+k*pi 处第一、第三轴共线。
// 这里直接对物理关节角判断，不使用论文中的全局构型分支符号。
inline bool isInShoulderSingularityDeadband(real_t q2,
                                            real_t halfWidth) {
    const real_t halfPi = kPi / static_cast<real_t>(2);
    const real_t center = halfPi +
        std::round((q2 - halfPi) / kPi) * kPi;
    return absr(q2 - center) <= halfWidth;
}

// 检查真实关节命令线段是否触碰任一周期肩奇异死区。不能只检查端点，
// 否则未来把单步上限调得大于死区宽度时可能一步跨过整个死区。
inline bool shoulderStepTouchesSingularityDeadband(real_t q2Begin,
                                                   real_t q2End,
                                                   real_t halfWidth) {
    const real_t halfPi = kPi / static_cast<real_t>(2);
    const real_t lower = minr(q2Begin, q2End);
    const real_t upper = maxr(q2Begin, q2End);
    const real_t firstIndex =
        std::ceil((lower - halfWidth - halfPi) / kPi);
    const real_t firstCenter = halfPi + firstIndex * kPi;
    return firstCenter - halfWidth <= upper;
}

inline real_t dlsTaskMetric(real_t positionErrorMm,
                            real_t orientationErrorRad,
                            real_t orientationScaleM) {
    const real_t positionErrorM =
        positionErrorMm * static_cast<real_t>(1.0e-3);
    const real_t scaledOrientation =
        orientationErrorRad * orientationScaleM;
    return positionErrorM * positionErrorM +
           scaledOrientation * scaledOrientation;
}

// 求解加权 DLS 的单次未裁剪关节步长：
//
//   min ||J*dq-error||^2 + lambda^2 * dq^T*Wq*dq
//
//   dq = Wq^-1*J^T*(J*Wq^-1*J^T + lambda^2*I)^-1*error
//
// Wq 是正对角矩阵。权重越大，对应关节在冗余方向上的运动越小。
inline bool solveWeightedDlsStep(const real_t jacobian[6][7],
                                 const real_t error[6],
                                 real_t dampingSq,
                                 const real_t weights[7],
                                 real_t jointStep[7],
                                 const bool enabledJoints[7] = nullptr) {
    if (!std::isfinite(dampingSq) ||
        dampingSq <= static_cast<real_t>(0) ||
        !validateDlsJointWeights(weights)) {
        return false;
    }

    real_t inverseWeights[7]{};
    for (int joint = 0; joint < 7; ++joint) {
        if (enabledJoints == nullptr || enabledJoints[joint]) {
            inverseWeights[joint] =
                static_cast<real_t>(1) / weights[joint];
        }
    }

    real_t normal[6][6]{};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            real_t value = static_cast<real_t>(0);
            for (int joint = 0; joint < 7; ++joint) {
                value += jacobian[row][joint] *
                         inverseWeights[joint] *
                         jacobian[column][joint];
            }
            normal[row][column] = value;
        }
        normal[row][row] += dampingSq;
    }

    real_t rhs[6]{};
    for (int row = 0; row < 6; ++row) {
        rhs[row] = error[row];
    }
    real_t dualStep[6]{};
    if (!solveLinear6(normal, rhs, dualStep)) {
        return false;
    }

    for (int joint = 0; joint < 7; ++joint) {
        real_t value = static_cast<real_t>(0);
        for (int row = 0; row < 6; ++row) {
            value += jacobian[row][joint] * dualStep[row];
        }
        jointStep[joint] = inverseWeights[joint] * value;
    }
    return true;
}

}  // namespace detail

inline bool solveDls(const Pose& targetWorld,
                     const Vec7& qCurrent,
                     const Config& cfg,
                     IkResult& result) {
    result = IkResult{};
    result.qBest = qCurrent;
    if (!detail::validateConfig(cfg) || cfg.solver.dlsMaxIterations < 1 ||
        cfg.solver.dlsDamping <= static_cast<real_t>(0) ||
        cfg.solver.dlsMaxIterationStepRad <= static_cast<real_t>(0) ||
        cfg.solver.dlsTrustRegionRad <= static_cast<real_t>(0) ||
        cfg.solver.dlsOrientationScaleMm <= static_cast<real_t>(0) ||
        !std::isfinite(cfg.solver.dlsMaxTargetPositionDeltaMm) ||
        cfg.solver.dlsMaxTargetPositionDeltaMm <=
            static_cast<real_t>(0) ||
        !detail::validateDlsJointWeights(cfg.solver.dlsJointWeights) ||
        !detail::validateDlsShoulderConstraint(cfg.solver)) {
        return false;
    }

    const detail::Robot robot = detail::buildRobot(cfg);
    const Pose target = detail::worldTargetToRobot(targetWorld, cfg.installation);
    const Pose current = detail::forwardKinematics(qCurrent, robot);
    const real_t initialPositionErrorMm =
        detail::norm(detail::sub(target.p, current.p));
    const real_t initialOrientationErrorRad =
        detail::rotationDistance(current.R, target.R);
    result.bestPositionErrorMm = initialPositionErrorMm;
    result.bestOrientationErrorRad = initialOrientationErrorRad;
    if (!std::isfinite(initialPositionErrorMm) ||
        !std::isfinite(initialOrientationErrorRad) ||
        initialPositionErrorMm >
            cfg.solver.dlsMaxTargetPositionDeltaMm) {
        // 输入拒绝发生在第一次迭代之前。保留真实反馈关节角，同时把
        // 初始残差写入既有诊断字段，供上层区分超距/非有限输入。
        return false;
    }

    const real_t orientationScaleM =
        cfg.solver.dlsOrientationScaleMm * static_cast<real_t>(1.0e-3);
    const real_t dampingSq = cfg.solver.dlsDamping * cfg.solver.dlsDamping;
    Vec7 q = qCurrent;
    Vec7 trustLower{};
    Vec7 trustUpper{};
    for (int joint = 0; joint < 7; ++joint) {
        trustLower.v[joint] = detail::maxr(
            cfg.joint.lowerRad[joint],
            qCurrent.v[joint] - cfg.solver.dlsTrustRegionRad);
        trustUpper.v[joint] = detail::minr(
            cfg.joint.upperRad[joint],
            qCurrent.v[joint] + cfg.solver.dlsTrustRegionRad);
    }

    const real_t shoulderOuterLower[2] = {
        detail::maxr(trustLower.v[0],
                     qCurrent.v[0] -
                         cfg.solver.dlsShoulderOuterMaxDeltaRad),
        detail::maxr(trustLower.v[2],
                     qCurrent.v[2] -
                         cfg.solver.dlsShoulderOuterMaxDeltaRad),
    };
    const real_t shoulderOuterUpper[2] = {
        detail::minr(trustUpper.v[0],
                     qCurrent.v[0] +
                         cfg.solver.dlsShoulderOuterMaxDeltaRad),
        detail::minr(trustUpper.v[2],
                     qCurrent.v[2] +
                         cfg.solver.dlsShoulderOuterMaxDeltaRad),
    };
    bool shoulderZoneTouched = detail::isInShoulderSingularityDeadband(
        qCurrent.v[1], cfg.solver.dlsShoulderDeadbandRad);

    Vec7 bestShoulderConstrainedQ = qCurrent;
    real_t initialTaskMetric = static_cast<real_t>(0);
    real_t bestShoulderConstrainedPositionErrorMm =
        static_cast<real_t>(0);
    real_t bestShoulderConstrainedOrientationErrorRad =
        static_cast<real_t>(0);
    real_t bestShoulderConstrainedMetric =
        static_cast<real_t>(0);
    bool bestShoulderConstrainedValid = false;

    bool trustRegionHit = false;
    for (int iteration = 0; iteration < cfg.solver.dlsMaxIterations; ++iteration) {
        result.dlsIterationCount = iteration + 1;
        real_t jacobian[6][7]{};
        const Pose actual = detail::dlsForwardAndJacobian(
            q, robot, orientationScaleM, jacobian);
        const Vec3 positionErrorMm = detail::sub(target.p, actual.p);
        const Vec3 orientationError = detail::dlsOrientationError(actual.R, target.R);
        const real_t positionErrorNormMm =
            detail::norm(positionErrorMm);

        if (iteration == 0 || shoulderZoneTouched) {
            const real_t orientationErrorRad =
                detail::rotationDistance(actual.R, target.R);
            const real_t metric = detail::dlsTaskMetric(
                positionErrorNormMm, orientationErrorRad,
                orientationScaleM);
            if (iteration == 0) {
                initialTaskMetric = metric;
            }
            if (shoulderZoneTouched &&
                (!bestShoulderConstrainedValid ||
                 metric < bestShoulderConstrainedMetric)) {
                bestShoulderConstrainedQ = q;
                bestShoulderConstrainedPositionErrorMm =
                    positionErrorNormMm;
                bestShoulderConstrainedOrientationErrorRad =
                    orientationErrorRad;
                bestShoulderConstrainedMetric = metric;
                bestShoulderConstrainedValid = true;
            }
        }

        // 与 Python 基线一致的提前停止条件：0.01mm / 5e-5rad 小角误差。
        if (positionErrorNormMm < static_cast<real_t>(1.0e-2) &&
            detail::norm(orientationError) < static_cast<real_t>(5.0e-5)) {
            break;
        }

        real_t error[6] = {
            positionErrorMm.x * static_cast<real_t>(1.0e-3),
            positionErrorMm.y * static_cast<real_t>(1.0e-3),
            positionErrorMm.z * static_cast<real_t>(1.0e-3),
            orientationError.x * orientationScaleM,
            orientationError.y * orientationScaleM,
            orientationError.z * orientationScaleM,
        };
        real_t jointSteps[7]{};
        if (!detail::solveWeightedDlsStep(
                jacobian, error, dampingSq,
                cfg.solver.dlsJointWeights, jointSteps)) {
            result.dlsTrustRegionHit = trustRegionHit;
            return false;
        }

        const real_t q2Step = detail::clamp(
            jointSteps[1],
            -cfg.solver.dlsMaxIterationStepRad,
            cfg.solver.dlsMaxIterationStepRad);
        const real_t q2Candidate = detail::clamp(
            q.v[1] + q2Step, trustLower.v[1], trustUpper.v[1]);
        shoulderZoneTouched =
            shoulderZoneTouched ||
            detail::shoulderStepTouchesSingularityDeadband(
                q.v[1], q2Candidate,
                cfg.solver.dlsShoulderDeadbandRad);

        // q1/q3 达到死区专用总变化边界且步长仍向外时，把对应列从本次
        // 有效 Jacobian 中移除并重算，使其他可用关节接管剩余任务误差。
        if (shoulderZoneTouched) {
            bool enabledJoints[7] = {
                true, true, true, true, true, true, true,
            };
            bool mustResolve = false;
            const int outerJoints[2] = {0, 2};
            for (int outer = 0; outer < 2; ++outer) {
                const int joint = outerJoints[outer];
                const real_t boundaryTolerance =
                    static_cast<real_t>(1.0e-7);
                const bool pushingBelowLower =
                    q.v[joint] <=
                        shoulderOuterLower[outer] + boundaryTolerance &&
                    jointSteps[joint] < static_cast<real_t>(0);
                const bool pushingAboveUpper =
                    q.v[joint] >=
                        shoulderOuterUpper[outer] - boundaryTolerance &&
                    jointSteps[joint] > static_cast<real_t>(0);
                if (pushingBelowLower || pushingAboveUpper) {
                    enabledJoints[joint] = false;
                    mustResolve = true;
                }
            }
            if (mustResolve) {
                trustRegionHit = true;
                if (!detail::solveWeightedDlsStep(
                        jacobian, error, dampingSq,
                        cfg.solver.dlsJointWeights, jointSteps,
                        enabledJoints)) {
                    result.dlsTrustRegionHit = trustRegionHit;
                    return false;
                }
            }
        }

        for (int joint = 0; joint < 7; ++joint) {
            const real_t jointStep = detail::clamp(
                jointSteps[joint],
                -cfg.solver.dlsMaxIterationStepRad,
                cfg.solver.dlsMaxIterationStepRad);
            const real_t unconstrained = q.v[joint] + jointStep;
            real_t constrained = detail::clamp(
                unconstrained, trustLower.v[joint], trustUpper.v[joint]);
            if (shoulderZoneTouched && joint == 0) {
                constrained = detail::clamp(
                    constrained,
                    shoulderOuterLower[0], shoulderOuterUpper[0]);
            } else if (shoulderZoneTouched && joint == 2) {
                constrained = detail::clamp(
                    constrained,
                    shoulderOuterLower[1], shoulderOuterUpper[1]);
            }
            trustRegionHit = trustRegionHit ||
                             detail::absr(constrained - unconstrained) >
                                 static_cast<real_t>(1.0e-7);
            q.v[joint] = constrained;
        }
    }

    const Pose finalPose = detail::forwardKinematics(q, robot);
    const real_t positionErrorMm = detail::norm(detail::sub(finalPose.p, target.p));
    const real_t orientationErrorRad = detail::rotationDistance(finalPose.R, target.R);

    if (shoulderZoneTouched) {
        const real_t finalMetric = detail::dlsTaskMetric(
            positionErrorMm, orientationErrorRad, orientationScaleM);
        if (!bestShoulderConstrainedValid ||
            finalMetric < bestShoulderConstrainedMetric) {
            bestShoulderConstrainedQ = q;
            bestShoulderConstrainedPositionErrorMm = positionErrorMm;
            bestShoulderConstrainedOrientationErrorRad =
                orientationErrorRad;
            bestShoulderConstrainedMetric = finalMetric;
            bestShoulderConstrainedValid = true;
        }
    }

    result.dlsTrustRegionHit = trustRegionHit;
    result.bestPositionErrorMm = positionErrorMm;
    result.bestOrientationErrorRad = orientationErrorRad;

    const bool exactSolution =
        positionErrorMm <= cfg.tolerance.positionMm &&
        orientationErrorRad <= cfg.tolerance.orientationRad;
    const bool relaxedShoulderSolution =
        !exactSolution &&
        shoulderZoneTouched &&
        bestShoulderConstrainedValid &&
        bestShoulderConstrainedMetric < initialTaskMetric &&
        bestShoulderConstrainedPositionErrorMm <=
            cfg.solver.dlsShoulderRelaxedPositionMm &&
        bestShoulderConstrainedOrientationErrorRad <=
            cfg.solver.dlsShoulderRelaxedOrientationRad;
    if (!exactSolution && !relaxedShoulderSolution) {
        // 未收敛中间量不作为电机目标输出。
        result.qBest = qCurrent;
        return false;
    }

    result.success = true;
    if (relaxedShoulderSolution) {
        result.qBest = bestShoulderConstrainedQ;
        result.bestPositionErrorMm =
            bestShoulderConstrainedPositionErrorMm;
        result.bestOrientationErrorRad =
            bestShoulderConstrainedOrientationErrorRad;
    } else {
        result.qBest = q;
    }
    result.bestCost =
        detail::weightedCommandDistanceSq(result.qBest, qCurrent, cfg);
    result.validCandidateCount = 1;
    result.selectedValidIndex = 0;
    return true;
}

}  // namespace roboarm7r
