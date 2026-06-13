// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file prismatic_joint.hpp
 * @brief Slider joint that permits one translational degree of freedom between two bodies.
 */

#pragma once

#include "frcsim/joints/joint_base.hpp"

/** @addtogroup joints @{ */

namespace frcsim {

/**
 * @brief Slider joint that permits translation along one axis and constrains
 * all other relative motion.
 *
 * Physics principle: constrained rigid-body dynamics with one translational
 * degree of freedom. Optional limits and motor targets are enforced with
 * impulse constraints.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Slider-crank_linkage
 */
class PrismaticJoint : public JointBase {
 public:
  /**
   * @brief Constructs a prismatic joint between two bodies.
   * @param body_a First (parent) rigid body.
   * @param body_b Second (child) rigid body.
   * @param axis_local Slide axis in body A's local frame; normalized internally.
   */
  PrismaticJoint(RigidBody* body_a, RigidBody* body_b,
                 const Vector3& axis_local)
      : JointBase(JointType::kPrismatic, body_a, body_b),
        axis_local_(axis_local.normalized()) {}

  /// @brief Returns the slide axis in body A's local frame (normalized).
  const Vector3& axisLocal() const { return axis_local_; }

  /**
   * @brief Installs translational limits.
   * @param min_displacement_m Lower limit in meters.
   * @param max_displacement_m Upper limit in meters.
   */
  void setLimits(double min_displacement_m, double max_displacement_m) {
    has_limits_ = true;
    min_displacement_m_ = min_displacement_m;
    max_displacement_m_ = max_displacement_m;
  }
  /// @brief Returns true when translational limits are active.
  bool hasLimits() const { return has_limits_; }
  /// @brief Returns the lower displacement limit in meters.
  double minDisplacement() const { return min_displacement_m_; }
  /// @brief Returns the upper displacement limit in meters.
  double maxDisplacement() const { return max_displacement_m_; }

  /**
   * @brief Configures motor target for this prismatic joint.
   * @param target_velocity_mps Desired linear velocity in m/s.
   * @param max_force_n Maximum motor force in newtons.
   */
  void setMotorTarget(double target_velocity_mps, double max_force_n) {
    has_motor_ = true;
    motor_target_velocity_mps_ = target_velocity_mps;
    motor_max_force_n_ = max_force_n;
  }
  /// @brief Returns true when a motor target has been configured.
  bool hasMotor() const { return has_motor_; }
  /// @brief Returns the motor target linear velocity in m/s.
  double motorTargetVelocity() const { return motor_target_velocity_mps_; }
  /// @brief Returns the motor maximum force in newtons.
  double motorMaxForce() const { return motor_max_force_n_; }

  void solveConstraint(double dt_s, int iterations) override;
  double constraintError() const override;

 private:
  Vector3 axis_local_{Vector3::unitX()};

  bool has_limits_{false};
  double min_displacement_m_{0.0};
  double max_displacement_m_{0.0};

  bool has_motor_{false};
  double motor_target_velocity_mps_{0.0};
  double motor_max_force_n_{0.0};
};

}  // namespace frcsim

/** @} */
