#define ROBOARM7R_SCALAR float
#include "../deploy_arm_model.hpp"
#include "../Arm/Inc/Middleware/TrajectoryPlannerKernel.hpp"

#include <cstdint>

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define WEB_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define WEB_EXPORT extern "C"
#endif

namespace {

using deploy_arm_model::Side;
using roboarm7r::Config;
using roboarm7r::Mat3;
using roboarm7r::Pose;
using roboarm7r::Vec3;
using roboarm7r::Vec7;

Pose readPose(const double* rotation, const double* positionMm) {
    Pose pose{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            pose.R.m[r][c] = static_cast<float>(rotation[3 * r + c]);
        }
    }
    pose.p = Vec3{static_cast<float>(positionMm[0]), static_cast<float>(positionMm[1]),
                  static_cast<float>(positionMm[2])};
    return pose;
}

Vec7 readJoints(const double* q) {
    Vec7 result{};
    for (int i = 0; i < 7; ++i) result.v[i] = static_cast<float>(q[i]);
    return result;
}

Vec3 toWorldVector(Mat3 installation, Vec3 local) {
    return roboarm7r::detail::mul(installation, local);
}

Vec3 toWorldPoint(const Config& cfg, Vec3 local) {
    return roboarm7r::detail::add(
        toWorldVector(cfg.installation.rotationWorldFromRobot, local),
        cfg.installation.positionWorldMm);
}

void writeVec(Vec3 value, double* output) {
    output[0] = value.x;
    output[1] = value.y;
    output[2] = value.z;
}

}  // namespace

WEB_EXPORT int deploy_ik_solve(int side, const double* rotation,
                               const double* positionMm, const double* qCurrent,
                               double* qOut, double* diagnostics) {
    if (!deploy_arm_model::validSide(side) || !rotation || !positionMm ||
        !qCurrent || !qOut || !diagnostics) return 0;
    const Config cfg = deploy_arm_model::makeConfig(deploy_arm_model::toSide(side));
    roboarm7r::IkResult result{};
    const bool ok = roboarm7r::solveAdaptiveHybrid(
        readPose(rotation, positionMm), readJoints(qCurrent), cfg, result);
    for (int i = 0; i < 7; ++i) qOut[i] = result.qBest.v[i];
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

WEB_EXPORT int deploy_fk(int side, const double* q, double* rotation,
                         double* positionMm) {
    if (!deploy_arm_model::validSide(side) || !q || !rotation || !positionMm) return 0;
    const Config cfg = deploy_arm_model::makeConfig(deploy_arm_model::toSide(side));
    const Pose local = roboarm7r::forwardKinematics(readJoints(q), cfg);
    const Mat3 worldRotation = roboarm7r::detail::mul(
        cfg.installation.rotationWorldFromRobot, local.R);
    const Vec3 worldPosition = toWorldPoint(cfg, local.p);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) rotation[3 * r + c] = worldRotation.m[r][c];
    }
    writeVec(worldPosition, positionMm);
    return 1;
}

// positionsMm: seven joint origins followed by tool tip (8 x xyz).
// axesWorld: seven unit joint axes in world coordinates (7 x xyz).
WEB_EXPORT int deploy_fk_chain(int side, const double* qInput,
                               double* positionsMm, double* axesWorld) {
    if (!deploy_arm_model::validSide(side) || !qInput || !positionsMm || !axesWorld) return 0;
    const Config cfg = deploy_arm_model::makeConfig(deploy_arm_model::toSide(side));
    const auto robot = roboarm7r::detail::buildRobot(cfg);
    const Vec7 q = readJoints(qInput);
    Mat3 rotation = roboarm7r::detail::eye();
    Vec3 position = robot.P[0];
    for (int i = 0; i < 7; ++i) {
        writeVec(toWorldPoint(cfg, position), positionsMm + 3 * i);
        writeVec(toWorldVector(cfg.installation.rotationWorldFromRobot,
                               roboarm7r::detail::mul(rotation, robot.H[i])),
                 axesWorld + 3 * i);
        rotation = roboarm7r::detail::mul(rotation,
                                          roboarm7r::detail::rot(robot.H[i], q.v[i]));
        position = roboarm7r::detail::add(
            position, roboarm7r::detail::mul(rotation, robot.P[i + 1]));
    }
    writeVec(toWorldPoint(cfg, position), positionsMm + 21);
    return 1;
}

WEB_EXPORT int deploy_get_joint_limits(int side, double* lower, double* upper) {
    if (!deploy_arm_model::validSide(side) || !lower || !upper) return 0;
    const Config cfg = deploy_arm_model::makeConfig(deploy_arm_model::toSide(side));
    for (int i = 0; i < 7; ++i) {
        lower[i] = cfg.joint.lowerRad[i];
        upper[i] = cfg.joint.upperRad[i];
    }
    return 1;
}

WEB_EXPORT int deploy_model_version() { return 1; }

// Browser visualizer version of Arm/Inc/Middleware/TrajectoryPlanner.hpp.
// Each arm has its own seven-joint synchronized duration, like the two
// firmware TrajectoryPlanner instances. Inputs/outputs use rad and ms.
WEB_EXPORT double deploy_trajectory_duration(const double* startRad,
                                              const double* targetRad,
                                              double requestedDurationMs) {
    if (!startRad || !targetRad || !std::isfinite(requestedDurationMs)) return 0.0;
    float start[7]{};
    float target[7]{};
    for (int i = 0; i < 7; ++i) {
        start[i] = static_cast<float>(startRad[i]);
        target[i] = static_cast<float>(targetRad[i]);
    }
    constexpr float speedMax[7] = {2.0f, 0.8f, 0.8f, 2.0f, 2.0f, 2.0f, 2.0f};
    constexpr float accelerationMax[7] = {4.0f, 2.0f, 2.0f, 4.0f, 4.0f, 4.0f, 4.0f};
    const float durationSec = trajectory_planner_kernel::synchronizedDurationSeconds(
        start, target, 7, static_cast<float>(requestedDurationMs) * 0.001f,
        speedMax, accelerationMax, 0.5f, 8.0f, 0.00001f);
    return static_cast<double>(durationSec) * 1000.0;
}

WEB_EXPORT int deploy_trajectory_sample(const double* startRad,
                                        const double* targetRad,
                                        double durationMs, double elapsedMs,
                                        double* positionRad,
                                        double* velocityRadPerSec) {
    if (!startRad || !targetRad || !positionRad || !velocityRadPerSec
        || !std::isfinite(durationMs) || !std::isfinite(elapsedMs)) return 0;
    const float durationSec = static_cast<float>(durationMs) * 0.001f;
    const float elapsedSec = static_cast<float>(elapsedMs) * 0.001f;
    const float normalizedTime = durationSec > trajectory_planner_kernel::kEpsilon
        ? elapsedSec / durationSec : 1.0f;
    const float normalizedVelocity = trajectory_planner_kernel::normalizedVelocity(normalizedTime);
    for (int i = 0; i < 7; ++i) {
        const float start = static_cast<float>(startRad[i]);
        const float target = static_cast<float>(targetRad[i]);
        positionRad[i] = trajectory_planner_kernel::samplePosition(
            start, target, elapsedSec, durationSec);
        velocityRadPerSec[i] = durationSec > trajectory_planner_kernel::kEpsilon
            ? (target - start) / durationSec * normalizedVelocity : 0.0;
    }
    return 1;
}
