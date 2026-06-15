// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file fixed_joint.hpp
 * @brief Rigid weld joint that constrains all relative translation and
 * rotation.
 */

#pragma once

#include "frcsim/joints/joint_base.hpp"

/** @addtogroup joints @{ */

namespace frcsim {

/**
 * @brief Rigidly locks two bodies together (no relative translation or
 * rotation).
 *
 * Physics principle: holonomic rigid constraint. The solver applies impulses
 * to keep the relative transform constant, modeling an infinitely stiff weld
 * in the idealized limit.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Holonomic_constraints
 */
class FixedJoint : public JointBase {
 public:
  /**
   * @brief Constructs a fixed (weld) joint between two bodies.
   * @param body_a First rigid body.
   * @param body_b Second rigid body; welded relative to body_a at construction
   * time.
   */
  FixedJoint(RigidBody* body_a, RigidBody* body_b) : JointBase(JointType::kFixed, body_a, body_b) {}

  void solveConstraint(double dt_s, int iterations) override;
  double constraintError() const override;
};

}  // namespace frcsim

/** @} */
