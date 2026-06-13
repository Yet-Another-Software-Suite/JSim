// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file joint_base.hpp
 * @brief Abstract base class for all bilateral rigid-body joint constraints.
 */

#pragma once

#include "frcsim/math/vector.hpp"

/** @defgroup joints Joint Constraints */
/** @addtogroup joints @{ */

namespace frcsim {

class RigidBody;

/** @brief Enumerates the concrete joint sub-types supported by JSim. */
enum class JointType {
  kRevolute,   ///< Hinge: one rotational DOF about a fixed axis.
  kPrismatic,  ///< Slider: one translational DOF along a fixed axis.
  kFixed,      ///< Rigid weld: no relative motion.
  kBall,       ///< Ball joint: unconstrained rotation about a shared anchor.
  kDistance,   ///< Distance constraint: bodies remain at a fixed separation.
};

/**
 * @brief Base class for bilateral kinematic constraints between two rigid
 * bodies.
 *
 * Joints are enforced as constraint equations solved each step by an iterative
 * impulse method (sequential impulse / projected Gauss-Seidel style). Each
 * derived joint defines which relative degrees of freedom are permitted and
 * which are constrained.
 *
 * References:
 * https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf
 * https://en.wikipedia.org/wiki/Constraint_(classical_mechanics)
 */
class JointBase {
 public:
  virtual ~JointBase() = default;

  /// @brief Returns the joint subtype.
  JointType type() const { return type_; }
  /// @brief Returns mutable pointer to the first constrained body.
  RigidBody* bodyA() { return body_a_; }
  /// @brief Returns mutable pointer to the second constrained body.
  RigidBody* bodyB() { return body_b_; }
  /// @brief Returns immutable pointer to the first constrained body.
  const RigidBody* bodyA() const { return body_a_; }
  /// @brief Returns immutable pointer to the second constrained body.
  const RigidBody* bodyB() const { return body_b_; }

  /// @brief Returns anchor point in body A's local frame (meters).
  const Vector3& anchorA() const { return anchor_a_; }
  /// @brief Returns anchor point in body B's local frame (meters).
  const Vector3& anchorB() const { return anchor_b_; }
  /// @brief Sets anchor point in body A's local frame. @param anchor Local anchor in meters.
  void setAnchorA(const Vector3& anchor) { anchor_a_ = anchor; }
  /// @brief Sets anchor point in body B's local frame. @param anchor Local anchor in meters.
  void setAnchorB(const Vector3& anchor) { anchor_b_ = anchor; }

  /// @brief Returns true when this joint participates in constraint solving.
  bool isEnabled() const { return is_enabled_; }
  /// @brief Enables or disables constraint solving for this joint. @param enabled New state.
  void setEnabled(bool enabled) { is_enabled_ = enabled; }

  /// @brief Returns impulse magnitude threshold above which the joint breaks (N).
  double breakForceThreshold() const { return break_force_threshold_; }
  /**
   * @brief Sets the impulse magnitude above which the joint permanently breaks.
   * @param force_n Break threshold in newtons.
   */
  void setBreakForceThreshold(double force_n) {
    break_force_threshold_ = force_n;
  }

  /// @brief Returns true after the joint has been broken by excess force.
  bool isBroken() const { return is_broken_; }
  /// @brief Clears the broken flag, re-enabling the joint.
  void resetBroken() { is_broken_ = false; }

  /**
   * @brief Solves this constraint for a single iteration step.
   * @param dt_s Current simulation timestep in seconds.
   * @param iterations Number of solver iterations already run this frame.
   */
  virtual void solveConstraint(double dt_s, int iterations) = 0;

  /**
   * @brief Applies cached impulse warm-start from the previous frame.
   * @note Default no-op; override to improve convergence.
   */
  virtual void warmStart() {}

  /**
   * @brief Returns a scalar measure of how far the constraint is violated.
   * @return Constraint error in meters (positional) or radians (angular).
   */
  virtual double constraintError() const = 0;

 protected:
  explicit JointBase(JointType type, RigidBody* body_a, RigidBody* body_b)
      : type_(type), body_a_(body_a), body_b_(body_b) {}

  JointType type_;
  RigidBody* body_a_{nullptr};
  RigidBody* body_b_{nullptr};
  Vector3 anchor_a_{};
  Vector3 anchor_b_{};
  bool is_enabled_{true};
  bool is_broken_{false};
  double break_force_threshold_{1e6};
};

}  // namespace frcsim

/** @} */
