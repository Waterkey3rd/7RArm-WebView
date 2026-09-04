// C ABI used by the MuJoCo visualizer.  It executes the exact header-only IK
// implementation deployed by Arm/Inc/Middleware/KinematicsSolver.hpp.
#define ROBOARM7R_SCALAR float
#include "deploy_arm_model.hpp"

#include <cstdint>

#if defined(_WIN32)
#define DEPLOY_IK_EXPORT extern "C" __declspec(dllexport)
#else
#define DEPLOY_IK_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace {

roboarm7r::Config deployedConfig(int side) {
    return deploy_arm_model::makeConfig(deploy_arm_model::toSide(side));
}

roboarm7r::Pose readPose(const double* rotation, const double* positionMm) {
    roboarm7r::Pose pose{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            pose.R.m[row][column] = static_cast<float>(rotation[3 * row + column]);
        }
    }
    pose.p = roboarm7r::Vec3{
        static_cast<float>(positionMm[0]),
        static_cast<float>(positionMm[1]),
        static_cast<float>(positionMm[2]),
    };
    return pose;
}

roboarm7r::Vec7 readJoints(const double* q) {
    roboarm7r::Vec7 joints{};
    for (int i = 0; i < 7; ++i) {
        joints.v[i] = static_cast<float>(q[i]);
    }
    return joints;
}

}  // namespace

// side: 0=left, 1=right. diagnostics:
// success, position error mm, orientation error rad, adaptive route,
// position delta mm, orientation delta rad, DLS iterations, hybrid stage.
DEPLOY_IK_EXPORT int deploy_ik_solve(int side, const double* rotation,
                                     const double* positionMm,
                                     const double* qCurrent, double* qOut,
                                     double* diagnostics) {
    if ((side != 0 && side != 1) || !rotation || !positionMm || !qCurrent ||
        !qOut || !diagnostics) {
        return 0;
    }
    const roboarm7r::Config cfg = deployedConfig(side);
    roboarm7r::IkResult result{};
    const bool ok = roboarm7r::solveAdaptiveHybrid(
        readPose(rotation, positionMm), readJoints(qCurrent), cfg, result);
    for (int i = 0; i < 7; ++i) {
        qOut[i] = static_cast<double>(result.qBest.v[i]);
    }
    diagnostics[0] = result.success ? 1.0 : 0.0;
    diagnostics[1] = result.bestPositionErrorMm;
    diagnostics[2] = result.bestOrientationErrorRad;
    diagnostics[3] = result.adaptiveRoute;
    diagnostics[4] = result.adaptivePositionDeltaMm;
    diagnostics[5] = result.adaptiveOrientationDeltaRad;
    diagnostics[6] = result.dlsIterationCount;
    diagnostics[7] = static_cast<uint8_t>(result.hybridStage);
    return ok ? 1 : 0;
}

DEPLOY_IK_EXPORT int deploy_ik_scalar_bytes() {
    return static_cast<int>(sizeof(roboarm7r::real_t));
}
