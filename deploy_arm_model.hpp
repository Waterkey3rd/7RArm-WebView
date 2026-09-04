#pragma once

// Single source of truth shared by the desktop DLL and browser WASM bridge.
#include "Arm/Lib/myikine7R/myikine7RAdaptive.hpp"

namespace deploy_arm_model {

enum class Side : int { Left = 0, Right = 1 };

inline roboarm7r::Config makeConfig(Side side) {
    using namespace roboarm7r;
    Config cfg = defaultConfig();
    constexpr real_t deg = kPi / static_cast<real_t>(180);

    cfg.geometry.baseOffsetMm = 0.0f;
    cfg.geometry.upperArmMm = 158.50f;
    cfg.geometry.elbowOffsetMm = 60.0f;
    cfg.geometry.forearmMm = 249.38f;
    cfg.geometry.flangeExtensionMm = 0.0f;
    cfg.geometry.endEffectorMm = 92.57f;
    cfg.geometry.toolYawRad = 0.0f;
    cfg.geometry.toolAxisRotation = Mat3{{
        {0.0f, 0.0f, 1.0f},
        {0.0f, -1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    }};

    if (side == Side::Left) {
        cfg.installation.rotationWorldFromRobot = Mat3{{
            {1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            {0.0f, -1.0f, 0.0f},
        }};
        cfg.installation.positionWorldMm = Vec3{0.0f, 110.0f, 0.0f};
    } else {
        cfg.installation.rotationWorldFromRobot = Mat3{{
            {1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, -1.0f},
            {0.0f, 1.0f, 0.0f},
        }};
        cfg.installation.positionWorldMm = Vec3{0.0f, -110.0f, 0.0f};
    }

    constexpr real_t lowerLeft[7] = {-150, -179, -179, 0, -179, -120, -179};
    constexpr real_t upperLeft[7] = {150, 10, 179, 200, 179, 120, 269};
    constexpr real_t lowerRight[7] = {-150, -179, -179, 0, -179, -120, -269};
    constexpr real_t upperRight[7] = {150, 10, 179, 200, 179, 120, 179};
    for (int i = 0; i < 7; ++i) {
        cfg.joint.lowerRad[i] = (side == Side::Left ? lowerLeft[i] : lowerRight[i]) * deg;
        cfg.joint.upperRad[i] = (side == Side::Left ? upperLeft[i] : upperRight[i]) * deg;
    }

    cfg.tolerance.positionMm = 25.0f;
    cfg.tolerance.orientationRad = 12.0f * deg;
    cfg.solver.adaptiveDlsPositionThresholdMm = 90.0f;
    cfg.solver.adaptiveDlsOrientationThresholdRad = 30.0f * deg;
    return cfg;
}

inline bool validSide(int side) { return side == 0 || side == 1; }
inline Side toSide(int side) { return side == 0 ? Side::Left : Side::Right; }

}  // namespace deploy_arm_model
