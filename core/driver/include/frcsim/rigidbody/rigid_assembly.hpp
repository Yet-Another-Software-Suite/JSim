// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file rigid_assembly.hpp
 * @brief Multi-body articulated system composed of rigid links and joints.
 */

#pragma once

#include <memory>
#include <vector>

#include "frcsim/joints/joint_base.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

/** @addtogroup bodies @{ */

namespace frcsim {

class RevoluteJoint;
class PrismaticJoint;
class FixedJoint;

/**
 * @brief Multi-body articulated system made from rigid links and joints.
 *
 * Physics principle: an assembly forms a constrained rigid-body graph where
 * each joint removes degrees of freedom and the solver enforces those
 * constraints through impulses each step.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Multibody_system
 */
class RigidAssembly {
 public:
  RigidAssembly() = default;

  /**
   * @brief Creates and adds a new rigid body to this assembly.
   * @param mass_kg Body mass in kilograms.
   * @return Pointer to the newly created body owned by this assembly.
   */
  RigidBody* addBody(double mass_kg);

  /// @brief Mutable access to the assembly body list.
  std::vector<RigidBody>& bodies() { return bodies_; }
  /// @brief Immutable access to the assembly body list.
  const std::vector<RigidBody>& bodies() const { return bodies_; }

  /**
   * @brief Creates a revolute (hinge) joint between two bodies in this assembly.
   * @param body_a_idx Index of the first body.
   * @param body_b_idx Index of the second body.
   * @param axis_local Rotation axis in the local frame of body A.
   * @return Pointer to the new joint owned by this assembly.
   */
  RevoluteJoint* addRevoluteJoint(size_t body_a_idx, size_t body_b_idx,
                                  const Vector3& axis_local);
  /**
   * @brief Creates a prismatic (slider) joint between two bodies.
   * @param body_a_idx Index of the first body.
   * @param body_b_idx Index of the second body.
   * @param axis_local Slide axis in the local frame of body A.
   * @return Pointer to the new joint owned by this assembly.
   */
  PrismaticJoint* addPrismaticJoint(size_t body_a_idx, size_t body_b_idx,
                                    const Vector3& axis_local);
  /**
   * @brief Creates a fixed (weld) joint between two bodies.
   * @param body_a_idx Index of the first body.
   * @param body_b_idx Index of the second body.
   * @return Pointer to the new joint owned by this assembly.
   */
  FixedJoint* addFixedJoint(size_t body_a_idx, size_t body_b_idx);

  /// @brief Mutable access to the assembly joint list.
  std::vector<std::shared_ptr<JointBase>>& joints() { return joints_; }
  /// @brief Immutable access to the assembly joint list.
  const std::vector<std::shared_ptr<JointBase>>& joints() const {
    return joints_;
  }

  /**
   * @brief Marks one body as the root (reference origin) of the assembly.
   * @param idx Body index to designate as root.
   * @note This is an organizational hint with no direct physics effect.
   */
  void setRootBodyIndex(size_t idx);
  /// @brief Returns the index of the designated root body.
  size_t rootBodyIndex() const { return root_body_idx_; }

  /**
   * @brief Computes the total mass of all bodies in this assembly.
   * @return Sum of body masses in kilograms.
   */
  double totalMass() const;

  /**
   * @brief Computes the assembly center of mass in world space.
   * @return Mass-weighted centroid position in meters.
   */
  Vector3 centerOfMass() const;

  /**
   * @brief Enables or disables all joint constraints in this assembly.
   * @param enable True to enable constraint solving; false to skip it.
   */
  void enableConstraints(bool enable);
  /**
   * @brief Runs the iterative constraint solver for all joints.
   * @param dt_s Timestep in seconds used for Baumgarte stabilization.
   * @param iterations Number of sequential-impulse solver iterations.
   * @note Must be called each step after force integration.
   */
  void solveConstraints(double dt_s, int iterations);

 private:
  std::vector<RigidBody> bodies_;
  std::vector<std::shared_ptr<JointBase>> joints_;
  size_t root_body_idx_{0};
};

}  // namespace frcsim

/** @} */
