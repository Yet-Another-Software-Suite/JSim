// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file physics_config.hpp
 * @brief Top-level simulation configuration aggregating integration, aerodynamics, and contact settings.
 */

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/math/quaternion.hpp"

/** @addtogroup world @{ */

namespace frcsim {

/**
 * @brief Configuration parameters for the physics world and simulation.
 */
struct PhysicsConfig {
  /// @brief Fixed timestep for simulation updates in seconds.
  double fixed_dt_s{0.01};

  /// @brief Gravity acceleration vector in m/s^2 (default: Earth -Z).
  Vector3 gravity_mps2{0.0, 0.0, -9.81};

  /// @brief Global linear velocity damping coefficient applied each step (1/s).
  double linear_damping_per_s{0.0};
  /// @brief Global angular velocity damping coefficient applied each step (1/s).
  double angular_damping_per_s{0.0};

  /** @brief Selects the numerical integration algorithm for all bodies. */
  enum class IntegrationMethod {
    kSemiImplicitEuler, ///< Velocity updated first, then position — symplectic, energy-conserving.
    kExplicitEuler,     ///< Forward Euler — simple but energy-drifting.
    kRK2,               ///< Midpoint/Runge-Kutta 2 — second-order accurate.
  } integration_method{IntegrationMethod::kSemiImplicitEuler};

  /// @brief Enables narrow-phase collision detection between bodies.
  bool enable_collision_detection{true};

  /// @brief Enables joint constraint solving for rigid assemblies.
  bool enable_joint_constraints{true};

  /// @brief Enables aerodynamic drag and Magnus lift forces on bodies.
  bool enable_aerodynamics{false};

  /// @brief Default dimensionless drag coefficient (Cd) when per-body value is unset.
  double default_drag_coefficient{0.47};

  /// @brief Default frontal reference area for drag in m^2 when per-body geometry is unset.
  double default_drag_reference_area_m2{0.01};

  /// @brief Air density used in aerodynamic force calculations in kg/m^3.
  double air_density_kgpm3{1.225};

  /// @brief Scalar coefficient for Magnus spin-induced lift.
  double magnus_coefficient{1.0e-4};

  /// @brief World Z height of the flat ground plane used for ball contact in meters.
  double ground_height_m{0.0};

  /// @brief Exponential rolling friction decay rate on the ground plane in 1/s.
  double rolling_friction_per_s{1.5};

  /// @brief Minimum downward impact speed that triggers a bounce in m/s.
  double min_bounce_speed_mps{0.06};

  /// @brief Baumgarte position stabilization factor; fraction of error corrected per step.
  double baumgarte_beta{0.2};

  /// @brief Allowed penetration before Baumgarte stabilization activates, in meters.
  double baumgarte_slop_m{0.005};
};

}  // namespace frcsim

/** @} */