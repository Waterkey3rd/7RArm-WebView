#ifndef TRAJECTORY_PLANNER_KERNEL_HPP
#define TRAJECTORY_PLANNER_KERNEL_HPP

#include <cmath>

// Platform-independent math shared by the firmware TrajectoryPlanner and the
// browser WASM visualizer. Values are SI: rad, seconds, rad/s and rad/s^2.
namespace trajectory_planner_kernel {

inline constexpr float kPeakVelocityFactor = 1.875f;
inline constexpr float kPeakAccelerationFactor = 5.773502691896258f;  // 10/sqrt(3)
inline constexpr float kEpsilon = 1.0e-6f;

inline float clampRequestedDuration(float requestedSec, float minDurationSec,
                                    float maxDurationSec, float guardSec) {
    float duration = requestedSec;
    if (duration > maxDurationSec) duration = maxDurationSec;
    else if (duration < minDurationSec) duration = minDurationSec;
    if (!std::isfinite(duration) || duration < guardSec) duration = guardSec;
    return duration;
}

inline float synchronizedDurationSeconds(
    const float* startRad, const float* targetRad, int jointCount,
    float requestedSec, const float* speedMaxRadPerSec,
    const float* accelerationMaxRadPerSec2, float minDurationSec,
    float maxDurationSec, float guardSec) {
    float requiredSec = clampRequestedDuration(
        requestedSec, minDurationSec, maxDurationSec, guardSec);
    for (int i = 0; i < jointCount; ++i) {
        const float delta = std::fabs(targetRad[i] - startRad[i]);
        if (!std::isfinite(delta) || delta <= kEpsilon) continue;
        const float vmax = std::fabs(speedMaxRadPerSec[i]);
        if (std::isfinite(vmax) && vmax > kEpsilon) {
            const float limited = kPeakVelocityFactor * delta / vmax;
            if (limited > requiredSec) requiredSec = limited;
        }
        const float amax = std::fabs(accelerationMaxRadPerSec2[i]);
        if (std::isfinite(amax) && amax > kEpsilon) {
            const float limited = std::sqrt(kPeakAccelerationFactor * delta / amax);
            if (limited > requiredSec) requiredSec = limited;
        }
    }
    if (!std::isfinite(requiredSec) || requiredSec < guardSec) return guardSec;
    return requiredSec;
}

inline float normalizedPosition(float normalizedTime) {
    if (!std::isfinite(normalizedTime) || normalizedTime <= 0.0f) return 0.0f;
    if (normalizedTime >= 1.0f) return 1.0f;
    const float s2 = normalizedTime * normalizedTime;
    const float s3 = s2 * normalizedTime;
    return s3 * (10.0f + normalizedTime * (-15.0f + 6.0f * normalizedTime));
}

inline float normalizedVelocity(float normalizedTime) {
    if (!std::isfinite(normalizedTime) || normalizedTime <= 0.0f || normalizedTime >= 1.0f) return 0.0f;
    const float oneMinus = 1.0f - normalizedTime;
    return 30.0f * normalizedTime * normalizedTime * oneMinus * oneMinus;
}

inline float samplePosition(float startRad, float targetRad, float elapsedSec,
                            float durationSec) {
    if (!std::isfinite(durationSec) || durationSec <= kEpsilon) return targetRad;
    return startRad + (targetRad - startRad) * normalizedPosition(elapsedSec / durationSec);
}

}  // namespace trajectory_planner_kernel

#endif
