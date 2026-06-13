// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file integrators.hpp
 * @brief Time-integration helpers for translational and rotational rigid-body dynamics.
 */

#pragma once

#include "vector.hpp"
#include "quaternion.hpp"

/** @addtogroup math @{ */

namespace frcsim {

/**
 * @brief Time-integration helpers for translational and rotational rigid-body
 * dynamics.
 *
 * This utility exposes explicit Euler, semi-implicit (symplectic) Euler, and
 * midpoint RK2 variants for linear motion, plus quaternion-based angular
 * integration using q_dot = 0.5 * omega_quat * q.
 *
 * References:
 * https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
 * https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
 * https://en.wikipedia.org/wiki/Quaternion#Using_quaternions_as_rotations
 */
struct Integrator {
  /**
   * @brief Semi-implicit (symplectic) Euler integration for linear motion.
   *
   * Updates velocity first, then uses the updated velocity for position.
   * This is stable and standard for real-time physics engines.
   *
   * @param position Position vector modified in place.
   * @param velocity Linear velocity modified in place.
   * @param acceleration Acceleration vector for this step.
   * @param dt Timestep duration in seconds.
   */
  static inline void integrateLinear(Vector3& position, Vector3& velocity,
                                     const Vector3& acceleration,
                                     double dt) noexcept {
    velocity += acceleration * dt;
    position += velocity * dt;
  }

  /**
   * @brief Explicit (position-first) Euler integration for linear motion.
   *
   * Updates position with current velocity before updating velocity.
   * Useful for debugging or as a predictor step in multi-stage integrators.
   *
   * @param position Position vector modified in place.
   * @param velocity Linear velocity modified in place.
   * @param acceleration Acceleration vector for this step.
   * @param dt Timestep duration in seconds.
   */
  static inline void integrateLinearExplicit(Vector3& position,
                                             Vector3& velocity,
                                             const Vector3& acceleration,
                                             double dt) noexcept {
    position += velocity * dt;
    velocity += acceleration * dt;
  }

  /**
   * @brief Integrates orientation using the quaternion kinematic equation.
   *
   * Uses q_dot = 0.5 * omega_quat * q where omega_quat = (0, wx, wy, wz).
   * The quaternion is re-normalized after each step to prevent drift.
   *
   * @param orientation Orientation quaternion modified in place.
   * @param angularVelocity World-space angular velocity in radians per second.
   * @param dt Timestep duration in seconds.
   */
  static inline void integrateAngular(Quaternion& orientation,
                                      const Vector3& angularVelocity,
                                      double dt) noexcept {
    Quaternion omegaQuat(0.0, angularVelocity.x, angularVelocity.y,
                         angularVelocity.z);
    Quaternion dq = omegaQuat * orientation * 0.5;
    orientation = orientation + dq * dt;
    orientation.normalizeIfNeeded();
  }

  /**
   * @brief Integrates angular velocity with a constant angular acceleration.
   *
   * @param angularVelocity Angular velocity modified in place (rad/s).
   * @param angularAcceleration Angular acceleration for this step (rad/s^2).
   * @param dt Timestep duration in seconds.
   */
  static inline void integrateAngularVelocity(
      Vector3& angularVelocity, const Vector3& angularAcceleration,
      double dt) noexcept {
    angularVelocity += angularAcceleration * dt;
  }

  /**
   * @brief Second-order midpoint (RK2) integration for linear motion.
   *
   * Evaluates velocity at the midpoint of the interval to obtain second-order
   * accuracy. Useful for projectiles or high-speed mechanisms.
   *
   * @param position Position vector modified in place.
   * @param velocity Linear velocity modified in place.
   * @param acceleration Acceleration assumed constant over the step.
   * @param dt Timestep duration in seconds.
   */
  static inline void integrateLinearRK2(Vector3& position, Vector3& velocity,
                                        const Vector3& acceleration,
                                        double dt) noexcept {
    Vector3 halfVelocity = velocity + acceleration * (0.5 * dt);

    position += halfVelocity * dt;

    velocity += acceleration * dt;
  }
};

}  // namespace frcsim

/** @} */
