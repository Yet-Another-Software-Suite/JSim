// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file force_generator.hpp
 * @brief Abstract base class for physics force laws applied by the world
 * integrator.
 */

#pragma once

/** @defgroup forces Force Generators */
/** @addtogroup forces @{ */

namespace frcsim {

class RigidBody;

/**
 * @brief Interface for force laws used by the rigid-body integrator.
 *
 * Implementations encode a physics principle that maps the current body state
 * (and optionally time step) to an applied force in newtons. The world update
 * then uses Newton's second law, F = m a, to convert those forces into
 * accelerations.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion
 */
class ForceGenerator {
 public:
  virtual ~ForceGenerator() = default;

  /**
   * @brief Applies a force to body for a single simulation step.
   * @param body Rigid body to receive the applied force.
   * @param dt_s Current simulation timestep in seconds.
   */
  virtual void apply(RigidBody& body, double dt_s) const = 0;
};

}  // namespace frcsim

/** @} */
