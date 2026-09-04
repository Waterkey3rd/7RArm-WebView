#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include "ArmSpaces.hpp"
#include "TrajectoryPlannerKernel.hpp"
#include "sheriffos.h"
#include <cmath>

/**
 * @brief 机械臂关节速度规划器（五次多项式 + 多关节统一时间轴）
 *
 * 核心思想：
 * 1) setTrajectory() 时先根据上层请求 duration、各关节位移、速度上限、加速度上限，
 *    求出一条所有关节共享的轨迹时长 T。
 * 2) 每个关节都在同一个 T 上生成五次多项式：
 *
 *      q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 *
 *    当前接口没有传入关节起始速度/加速度，因此采用：
 *      q(0)=q0, dq(0)=0, ddq(0)=0
 *      q(T)=qf, dq(T)=0, ddq(T)=0
 *
 * 3) compute() 按实际经过时间采样 dq(t)，因此所有运动关节天然同步结束。
 * 4) 不在 compute() 中再次对速度做硬裁剪。若 T 太短，setTrajectory() 会主动拉长 T，
 *    这样不会因为逐关节限速而破坏五次轨迹和多关节同步。
 *
 * 对上层保持原有调用接口不变：
 * - setTrajectory(ref, start, duration)
 * - updateCurrentMotorAngles(current)
 * - compute(outSpeeds)
 * - getState()/isRunning()/reset()
 */
class TrajectoryPlanner {
   public:
	static constexpr int kJointCount = kArmJointCount;

	enum class PlannerState : uint8_t {
		Idle = 0,
		Running,
		Arrived,
	};

	/**
	 * @brief 兼容旧版接口保留的参数结构。
	 *
	 * 五次多项式实现不再使用 v0/x0/c，但保留该结构及构造函数签名，
	 * 避免现有上层初始化代码需要修改。
	 */
	struct JointSmoothParam {
		units::angular_velocity::radians_per_second_t v0{0.2_rad_per_s};
		units::angle::radian_t x0{0.15_rad};
		float c{0.0f};
	};

	struct PlannerConfig {
		// 各关节最大速度，单位 rad/s。
		units::angular_velocity::radians_per_second_t speedMax[kJointCount] = {
			2.0_rad_per_s,
			0.8_rad_per_s,
			0.8_rad_per_s,
			2.0_rad_per_s,
			2.0_rad_per_s,
			2.0_rad_per_s,
			2.0_rad_per_s,
		};

		// 上层请求 duration 的软上下界。
		// 注意：若为了满足速度/加速度约束需要更长时间，最终统一 T 可以超过 maxDuration。
		units::time::millisecond_t minDuration{500_ms};
		units::time::millisecond_t maxDuration{8000_ms};

		// -----------------------------------------------------------------
		// 以下字段为兼容旧版配置保留。五次多项式算法不再使用这些分段/log 参数。
		// -----------------------------------------------------------------
		float startBlendRatio{0.80f};
		float endBlendRatio{0.20f};
		units::angle::radian_t tinyTotalDiffThreshold{0.1_rad};
		units::angle::radian_t nearTargetThreshold{0.06_rad};

		// 到位仍使用实际关节反馈判断，保持旧版状态机语义。
		units::angle::radian_t arrivedThreshold{0.05_rad};

		units::time::millisecond_t durationGuard{0.01_ms};
		float logGuard{1.0e-4f};

		// 新增：各关节最大加速度，单位 rad/s^2。
		// 使用 float 是为了避免额外依赖特定 units 库中的角加速度类型/字面量名称。
		// 该字段追加在 PlannerConfig 尾部，尽量兼容已有聚合初始化代码。
		float accelerationMax[kJointCount] = {
			4.0f,
			2.0f,
			2.0f,
			4.0f,
			4.0f,
			4.0f,
			4.0f,
		};
	};

	TrajectoryPlanner();
	TrajectoryPlanner(PlannerConfig const& config, JointSmoothParam const (&jointParams)[kJointCount]);
	~TrajectoryPlanner() = default;

	/**
	 * @brief 设置一条新的同步轨迹
	 * @param refMotorSpace   目标电机角空间
	 * @param startMotorSpace 起始电机角空间
	 * @param duration        上层期望的轨迹时长；若过短导致违反约束，内部会自动增大统一 T
	 */
	void setTrajectory(MotorAngleSpace const& refMotorSpace, MotorAngleSpace const& startMotorSpace,
					   units::time::millisecond_t duration);

	void updateCurrentMotorAngles(MotorAngleSpace const& currentMotorSpace);

	/**
	 * @brief 按当前轨迹时间采样速度 dq(t)
	 * @param outSmoothSpeeds 输出数组（长度为 6）
	 */
	void compute(units::angular_velocity::radians_per_second_t outSmoothSpeeds[kJointCount]);

	PlannerState getState() const;
	bool isRunning() const;
	void reset();

   private:
	/**
	 * @brief 单关节五次多项式系数。
	 *
	 * 系数统一使用 SI 数值：角度 rad，时间 s。
	 * 因而 a1~a5 的量纲依次为 rad/s ... rad/s^5。
	 */
	struct QuinticCoefficients {
		float a0{0.0f};
		float a1{0.0f};
		float a2{0.0f};
		float a3{0.0f};
		float a4{0.0f};
		float a5{0.0f};
	};

	PlannerConfig config_{};
	JointSmoothParam jointParams_[kJointCount]{}; // 仅用于兼容旧构造接口
	QuinticCoefficients coeffs_[kJointCount]{};

	units::angle::radian_t currentMotorAngles_[kJointCount]{};
	units::angle::radian_t refMotorAngles_[kJointCount]{};
	units::angle::radian_t startMotorAngles_[kJointCount]{};

	units::angular_velocity::radians_per_second_t refSmoothSpeeds_[kJointCount]{};
	units::angular_velocity::radians_per_second_t outputSmoothSpeeds_[kJointCount]{};

	units::time::millisecond_t startTime_{0_ms};
	units::time::millisecond_t currentTime_{0_ms};
	units::time::millisecond_t refDuration_{0_ms};

	PlannerState state_{PlannerState::Idle};

	// 根据请求 duration 和所有关节约束计算统一轨迹时间 T。
	units::time::millisecond_t calculateSynchronizedDuration_(units::time::millisecond_t requestedDuration) const;

	// 根据 q0/qf 和统一 T 生成零起终速度、零起终加速度的五次多项式。
	void buildQuintic_(int jointIndex, float durationSec);

	// 采样单关节五次多项式的速度 dq(t)。
	units::angular_velocity::radians_per_second_t sampleVelocity_(int jointIndex, float timeSec) const;

	// 判断所有关节实际反馈是否都进入到位阈值。
	bool allJointsWithinArrivedThreshold_() const;

	// 只有统一时间轴走完后，才根据实际反馈切换到 Arrived。
	void updateArrivedState_();

	void clearOutputs_();
};

inline TrajectoryPlanner::TrajectoryPlanner() {
	for (int i = 0; i < kJointCount; ++i) {
		jointParams_[i] = JointSmoothParam{};
	}
}

inline TrajectoryPlanner::TrajectoryPlanner(PlannerConfig const& config,
											JointSmoothParam const (&jointParams)[kJointCount])
	: config_(config) {
	for (int i = 0; i < kJointCount; ++i) {
		jointParams_[i] = jointParams[i];
	}
}

inline void TrajectoryPlanner::setTrajectory(MotorAngleSpace const& refMotorSpace,
											 MotorAngleSpace const& startMotorSpace,
											 units::time::millisecond_t duration) {
	// Step 1: 冻结本次轨迹的起点和终点。
	for (int i = 0; i < kJointCount; ++i) {
		refMotorAngles_[i] = refMotorSpace.motorAngles[i];
		startMotorAngles_[i] = startMotorSpace.motorAngles[i];
		currentMotorAngles_[i] = startMotorSpace.motorAngles[i];
	}

	// Step 2: 由所有关节共同决定统一运动时间 T。
	// duration 是上层期望时间；若无法满足速度/加速度约束，这里自动拉长。
	refDuration_ = calculateSynchronizedDuration_(duration);

	float durationSec = refDuration_.to<float>() * 0.001f;
	if (!std::isfinite(durationSec) || durationSec <= 0.0f) {
		durationSec = config_.durationGuard.to<float>() * 0.001f;
	}

	// Step 3: 所有关节使用相同 T，各自计算五次多项式系数。
	for (int i = 0; i < kJointCount; ++i) {
		buildQuintic_(i, durationSec);

		// 保留该缓存字段，表示本次轨迹的峰值速度方向/量级，便于兼容旧调试观察。
		float const delta = refMotorAngles_[i].to<float>() - startMotorAngles_[i].to<float>();
		float const peakVelocity = 1.875f * delta / durationSec;
		refSmoothSpeeds_[i] = units::angular_velocity::radians_per_second_t(peakVelocity);
	}

	startTime_ = os::getTime();
	currentTime_ = startTime_;
	clearOutputs_();

	// 如果起点本来就在所有目标的到位阈值内，则无需等待完整 T。
	// 否则一旦启动，就必须沿统一时间轴运行到 T，避免某个反馈提前进入阈值导致整条轨迹提前结束。
	state_ = allJointsWithinArrivedThreshold_() ? PlannerState::Arrived : PlannerState::Running;
}

inline void TrajectoryPlanner::updateCurrentMotorAngles(MotorAngleSpace const& currentMotorSpace) {
	for (int i = 0; i < kJointCount; ++i) {
		currentMotorAngles_[i] = currentMotorSpace.motorAngles[i];
	}
}

inline void TrajectoryPlanner::compute(units::angular_velocity::radians_per_second_t outSmoothSpeeds[kJointCount]) {
	if (state_ == PlannerState::Idle || state_ == PlannerState::Arrived) {
		clearOutputs_();
		for (int i = 0; i < kJointCount; ++i) {
			outSmoothSpeeds[i] = outputSmoothSpeeds_[i];
		}
		return;
	}

	currentTime_ = os::getTime();

	float const durationSec = refDuration_.to<float>() * 0.001f;
	float elapsedSec = (currentTime_ - startTime_).to<float>() * 0.001f;

	if (!std::isfinite(elapsedSec) || elapsedSec < 0.0f) {
		elapsedSec = 0.0f;
	}

	// 到统一 T 后，理论上所有关节 dq(T)=0。
	// 若实际反馈尚未进入 arrivedThreshold，则保持 Running，但速度前馈为 0；
	// 这样仍保留旧版“实际反馈到位才 Arrived”的状态语义。
	if (elapsedSec >= durationSec) {
		clearOutputs_();
	} else {
		for (int i = 0; i < kJointCount; ++i) {
			outputSmoothSpeeds_[i] = sampleVelocity_(i, elapsedSec);
		}
	}

	updateArrivedState_();

	for (int i = 0; i < kJointCount; ++i) {
		outSmoothSpeeds[i] = outputSmoothSpeeds_[i];
	}
}

inline TrajectoryPlanner::PlannerState TrajectoryPlanner::getState() const {
	return state_;
}

inline bool TrajectoryPlanner::isRunning() const {
	return state_ == PlannerState::Running;
}

inline void TrajectoryPlanner::reset() {
	refDuration_ = 0_ms;
	startTime_ = 0_ms;
	currentTime_ = 0_ms;
	state_ = PlannerState::Idle;

	for (int i = 0; i < kJointCount; ++i) {
		coeffs_[i] = QuinticCoefficients{};
		refSmoothSpeeds_[i] = 0.0_rad_per_s;
	}
	clearOutputs_();
}

inline units::time::millisecond_t TrajectoryPlanner::calculateSynchronizedDuration_(
	units::time::millisecond_t requestedDuration) const {
	// 对零起终速度/加速度的标准五次时间缩放：
	//   max|dq|  = 1.875 * |Delta q| / T
	//   max|ddq| = (10/sqrt(3)) * |Delta q| / T^2
	// 因此每个关节至少需要：
	//   T_v >= 1.875 * |Delta q| / v_max
	//   T_a >= sqrt((10/sqrt(3)) * |Delta q| / a_max)
	float start[kJointCount]{};
	float target[kJointCount]{};
	float speedMax[kJointCount]{};
	for (int i = 0; i < kJointCount; ++i) {
		start[i] = startMotorAngles_[i].to<float>();
		target[i] = refMotorAngles_[i].to<float>();
		speedMax[i] = config_.speedMax[i].to<float>();
	}
	float requiredSec = trajectory_planner_kernel::synchronizedDurationSeconds(
		start, target, kJointCount, requestedDuration.to<float>() * 0.001f, speedMax,
		config_.accelerationMax, config_.minDuration.to<float>() * 0.001f,
		config_.maxDuration.to<float>() * 0.001f,
		config_.durationGuard.to<float>() * 0.001f);

	// 这里故意不再次 clamp 到 maxDuration：
	// maxDuration 只是上层请求时间的软上限；运动学约束优先级更高。
	float const guardSec = config_.durationGuard.to<float>() * 0.001f;
	if (!std::isfinite(requiredSec) || requiredSec < guardSec) {
		requiredSec = guardSec;
	}

	return units::time::millisecond_t(requiredSec * 1000.0f);
}

inline void TrajectoryPlanner::buildQuintic_(int jointIndex, float durationSec) {
	constexpr float kEpsilon = 1.0e-6f;

	float const q0 = startMotorAngles_[jointIndex].to<float>();
	float const qf = refMotorAngles_[jointIndex].to<float>();
	float const delta = qf - q0;

	QuinticCoefficients& c = coeffs_[jointIndex];
	c = QuinticCoefficients{};
	c.a0 = q0;

	if (!std::isfinite(durationSec) || durationSec <= kEpsilon || std::fabs(delta) <= kEpsilon) {
		return;
	}

	float const t2 = durationSec * durationSec;
	float const t3 = t2 * durationSec;
	float const t4 = t3 * durationSec;
	float const t5 = t4 * durationSec;

	// 边界条件：
	// q(0)=q0, dq(0)=0, ddq(0)=0
	// q(T)=qf, dq(T)=0, ddq(T)=0
	c.a1 = 0.0f;
	c.a2 = 0.0f;
	c.a3 = 10.0f * delta / t3;
	c.a4 = -15.0f * delta / t4;
	c.a5 = 6.0f * delta / t5;
}

inline units::angular_velocity::radians_per_second_t TrajectoryPlanner::sampleVelocity_(int jointIndex,
																	 float timeSec) const {
	float const durationSec = refDuration_.to<float>() * 0.001f;
	float const delta = refMotorAngles_[jointIndex].to<float>() - startMotorAngles_[jointIndex].to<float>();
	float const speed = (!std::isfinite(durationSec) || durationSec <= trajectory_planner_kernel::kEpsilon)
		? 0.0f
		: delta / durationSec * trajectory_planner_kernel::normalizedVelocity(timeSec / durationSec);

	if (!std::isfinite(speed)) {
		return 0.0_rad_per_s;
	}
	return units::angular_velocity::radians_per_second_t(speed);
}

inline bool TrajectoryPlanner::allJointsWithinArrivedThreshold_() const {
	for (int i = 0; i < kJointCount; ++i) {
		auto const absAngleDiff = units::angle::radian_t(
			std::fabs((refMotorAngles_[i] - currentMotorAngles_[i]).to<float>()));
		if (absAngleDiff > config_.arrivedThreshold) {
			return false;
		}
	}
	return true;
}

inline void TrajectoryPlanner::updateArrivedState_() {
	if (state_ == PlannerState::Idle) {
		return;
	}

	float const elapsedMs = (currentTime_ - startTime_).to<float>();
	float const durationMs = refDuration_.to<float>();
	bool const trajectoryTimeFinished = std::isfinite(elapsedMs) && (elapsedMs >= durationMs);

	// 统一时间 T 结束之前，即使反馈已经靠近目标，也不能提前结束轨迹，
	// 否则所有关节共享同一时间轴的意义会被破坏。
	if (!trajectoryTimeFinished) {
		state_ = PlannerState::Running;
		return;
	}

	state_ = allJointsWithinArrivedThreshold_() ? PlannerState::Arrived : PlannerState::Running;
	if (state_ == PlannerState::Arrived) {
		clearOutputs_();
	}
}

inline void TrajectoryPlanner::clearOutputs_() {
	for (int i = 0; i < kJointCount; ++i) {
		outputSmoothSpeeds_[i] = 0.0_rad_per_s;
	}
}

#endif
