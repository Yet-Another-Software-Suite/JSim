// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file revolute_joint.hpp
 * @brief Hinge joint that permits one rotational degree of freedom between two
 * bodies.
 */

#pragma once

#include "frcsim/joints/joint_base.hpp"

/** @addtogroup joints @{ */

namespace frcsim {

/**
 * @brief Hinge joint that permits rotation about one axis while constraining
 * relative translation.
 *
 * Physics principle: constrained rigid-body dynamics with one rotational
 * degree of freedom. Limit and motor behavior are represented as angular
 * inequality/equality constraints solved by impulses.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Hinge
 */
class RevoluteJoint : public JointBase {
 public:
  /**
   * @brief Constructs a revolute joint between two bodies.
   * @param body_a First (parent) rigid body.
   * @param body_b Second (child) rigid body.
   * @param axis_local Hinge axis in body A's local frame; normalized
   * internally.
   */
  RevoluteJoint(RigidBody* body_a, RigidBody* body_b, const Vector3& axis_local)
      : JointBase(JointType::kRevolute, body_a, body_b), axis_local_(axis_local.normalized()) {}

  /// @brief Returns the hinge axis in body A's local frame (normalized).
  const Vector3& axisLocal() const { return axis_local_; }

  /**
   * @brief Installs angular limits.
   * @param min_angle_rad Lower limit in radians.
   * @param max_angle_rad Upper limit in radians.
   */
  void setLimits(double min_angle_rad, double max_angle_rad) {
    has_limits_ = true;
    min_angle_rad_ = min_angle_rad;
    max_angle_rad_ = max_angle_rad;
  }
  /// @brief Removes previously set angular limits.
  void clearLimits() { has_limits_ = false; }
  /// @brief Returns true when angular limits are active.
  bool hasLimits() const { return has_limits_; }
  /// @brief Returns the lower angular limit in radians.
  double minAngle() const { return min_angle_rad_; }
  /// @brief Returns the upper angular limit in radians.
  double maxAngle() const { return max_angle_rad_; }

  /**
   * @brief Configures motor target for this revolute joint.
   * @param target_velocity_radps Desired angular velocity in rad/s.
   * @param max_torque_nm Maximum motor torque in N·m.
   */
  void setMotorTarget(double target_velocity_radps, double max_torque_nm) {
    has_motor_ = true;
    motor_target_velocity_radps_ = target_velocity_radps;
    motor_max_torque_nm_ = max_torque_nm;
  }
  /// @brief Returns true when a motor target has been configured.
  bool hasMotor() const { return has_motor_; }
  /// @brief Returns the motor target angular velocity in rad/s.
  double motorTargetVelocity() const { return motor_target_velocity_radps_; }
  /// @brief Returns the motor maximum torque in N·m.
  double motorMaxTorque() const { return motor_max_torque_nm_; }

  void solveConstraint(double dt_s, int iterations) override;
  double constraintError() const override;

 private:
  Vector3 axis_local_{Vector3::unitZ()};

  bool has_limits_{false};
  double min_angle_rad_{0.0};
  double max_angle_rad_{0.0};

  bool has_motor_{false};
  double motor_target_velocity_radps_{0.0};
  double motor_max_torque_nm_{0.0};
};

}  // namespace frcsim

/** @} */
