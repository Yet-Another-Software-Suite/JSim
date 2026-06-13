// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file body_flags.hpp
 * @brief Per-body boolean feature flags controlling physics simulation
 * behavior.
 */

#pragma once

/** @defgroup bodies Rigid Bodies */
/** @addtogroup bodies @{ */

namespace frcsim {

/**
 * @brief Per-body feature flags controlling which physics sub-systems affect a
 * body.
 *
 * Each flag can be toggled independently to customize behavior on a per-body
 * basis without changing world-level configuration.
 */
struct BodyFlags {
  /// @brief Apply gravity to this body each integration step.
  bool enable_gravity{true};

  /// @brief Apply friction and tangential contact forces for this body.
  bool enable_friction{true};

  /// @brief Enable collision detection and contact response for this body.
  bool enable_collisions{true};

  /// @brief Enable kinematic constraint solving (joints, etc.) for this body.
  bool enable_kinematic_constraints{true};

  /// @brief Enable deformation/warping/bending for deformable body variants.
  bool enable_deformation{false};

  /**
   * @brief When true, the body is kinematic and not integrated by the physics
   * solver.
   *
   * Kinematic bodies ignore accumulated forces and are moved by external code
   * only.
   * @note Setting this flag prevents @ref RigidBody::integrate from updating
   * state.
   */
  bool is_kinematic{false};
};

}  // namespace frcsim

/** @} */
