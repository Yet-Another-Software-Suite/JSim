// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file double_differential.hpp
 * @brief Kinematic mapper for a two-motor double-differential turret mechanism.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

/** @addtogroup mechanisms @{ */

namespace frcsim {

/**
 * @brief Kinematic mapping utility for a two-motor double differential turret
 * mechanism.
 *
 * The configured 2x2 matrix maps motor shaft states to yaw/pitch joint states.
 */
class DoubleDifferentialMechanism {
 public:
  /** @brief Optional soft limits applied to joint coordinates. */
  struct Limits {
    /// @brief Lower yaw limit in radians.
    double min_yaw_rad{-std::numeric_limits<double>::infinity()};
    /// @brief Upper yaw limit in radians.
    double max_yaw_rad{std::numeric_limits<double>::infinity()};
    /// @brief Lower pitch limit in radians.
    double min_pitch_rad{-std::numeric_limits<double>::infinity()};
    /// @brief Upper pitch limit in radians.
    double max_pitch_rad{std::numeric_limits<double>::infinity()};
  };

  /**
   * @brief Mapping coefficients and inversion thresholds.
   *
   * The 2×2 matrix maps [motor_a, motor_b] to [yaw, pitch]:
   * - yaw   = m00 * a + m01 * b
   * - pitch = m10 * a + m11 * b
   */
  struct Config {
    /// @brief Row 0, col 0 of the kinematic mapping matrix.
    double m00{0.5};
    /// @brief Row 0, col 1 of the kinematic mapping matrix.
    double m01{0.5};
    /// @brief Row 1, col 0 of the kinematic mapping matrix.
    double m10{0.5};
    /// @brief Row 1, col 1 of the kinematic mapping matrix.
    double m11{-0.5};
    /// @brief Multiplicative scale applied to the yaw output.
    double yaw_scale{1.0};
    /// @brief Multiplicative scale applied to the pitch output.
    double pitch_scale{1.0};
    /// @brief Joint soft limits applied after matrix mapping.
    Limits limits{};
    /// @brief Determinant threshold below which the matrix is treated as
    /// singular.
    double singularity_epsilon{1e-9};
  };

  /** @brief Input state for the two drive motors. */
  struct MotorState {
    /// @brief Motor A shaft position in radians.
    double motor_a_position_rad{0.0};
    /// @brief Motor B shaft position in radians.
    double motor_b_position_rad{0.0};
    /// @brief Motor A shaft velocity in rad/s.
    double motor_a_velocity_radps{0.0};
    /// @brief Motor B shaft velocity in rad/s.
    double motor_b_velocity_radps{0.0};
  };

  /** @brief Output state for mechanism yaw/pitch joints. */
  struct JointState {
    /// @brief Turret yaw angle in radians.
    double yaw_rad{0.0};
    /// @brief Hood pitch angle in radians.
    double pitch_rad{0.0};
    /// @brief Turret yaw rate in rad/s.
    double yaw_velocity_radps{0.0};
    /// @brief Hood pitch rate in rad/s.
    double pitch_velocity_radps{0.0};
  };

  /** @brief Result from inverse kinematics solve. */
  struct InverseResult {
    /// @brief Computed motor state (valid only when valid == true).
    MotorState motor_state{};
    /// @brief False when the mapping matrix is singular within configured
    /// epsilon.
    bool valid{true};
  };

  DoubleDifferentialMechanism() = default;

  explicit DoubleDifferentialMechanism(const Config& config) : config_(config) {}

  /** @brief Returns current mapping configuration. @return Immutable Config
   * reference. */
  const Config& config() const { return config_; }
  /** @brief Replaces mapping configuration. @param config New mapping
   * configuration. */
  void setConfig(const Config& config) { config_ = config; }

  /**
   * @brief Computes joint state from motor state via forward mapping.
   * @param motor_state Motor positions/velocities in mechanism motor space.
   * @return Joint state with configured limits applied.
   */
  JointState forward(const MotorState& motor_state) const {
    JointState out{};
    out.yaw_rad = config_.yaw_scale * (config_.m00 * motor_state.motor_a_position_rad + config_.m01 * motor_state.motor_b_position_rad);
    out.pitch_rad = config_.pitch_scale * (config_.m10 * motor_state.motor_a_position_rad + config_.m11 * motor_state.motor_b_position_rad);

    out.yaw_velocity_radps =
        config_.yaw_scale * (config_.m00 * motor_state.motor_a_velocity_radps + config_.m01 * motor_state.motor_b_velocity_radps);
    out.pitch_velocity_radps =
        config_.pitch_scale * (config_.m10 * motor_state.motor_a_velocity_radps + config_.m11 * motor_state.motor_b_velocity_radps);

    applyLimits(out);
    return out;
  }

  /**
   * @brief Solves inverse mapping from desired joint state to motor state.
   * @param desired_joint_state Requested joint angles/rates.
   * @return InverseResult with valid=false when configuration is singular.
   */
  InverseResult inverse(const JointState& desired_joint_state) const {
    InverseResult result{};

    const double a11 = config_.yaw_scale * config_.m00;
    const double a12 = config_.yaw_scale * config_.m01;
    const double a21 = config_.pitch_scale * config_.m10;
    const double a22 = config_.pitch_scale * config_.m11;

    const double det = a11 * a22 - a12 * a21;
    if (std::abs(det) <= config_.singularity_epsilon) {
      result.valid = false;
      return result;
    }

    const double inv11 = a22 / det;
    const double inv12 = -a12 / det;
    const double inv21 = -a21 / det;
    const double inv22 = a11 / det;

    const JointState clamped = clampedToLimits(desired_joint_state);

    result.motor_state.motor_a_position_rad = inv11 * clamped.yaw_rad + inv12 * clamped.pitch_rad;
    result.motor_state.motor_b_position_rad = inv21 * clamped.yaw_rad + inv22 * clamped.pitch_rad;

    result.motor_state.motor_a_velocity_radps = inv11 * clamped.yaw_velocity_radps + inv12 * clamped.pitch_velocity_radps;
    result.motor_state.motor_b_velocity_radps = inv21 * clamped.yaw_velocity_radps + inv22 * clamped.pitch_velocity_radps;

    return result;
  }

  /**
   * @brief Returns a copy of state clamped to configured joint limits.
   * @param state Input joint state.
   * @return Clamped joint state.
   */
  JointState clampedToLimits(const JointState& state) const {
    JointState out = state;
    applyLimits(out);
    return out;
  }

 private:
  void applyLimits(JointState& state) const {
    state.yaw_rad = std::clamp(state.yaw_rad, config_.limits.min_yaw_rad, config_.limits.max_yaw_rad);
    state.pitch_rad = std::clamp(state.pitch_rad, config_.limits.min_pitch_rad, config_.limits.max_pitch_rad);
  }

  Config config_{};
};

}  // namespace frcsim

/** @} */
