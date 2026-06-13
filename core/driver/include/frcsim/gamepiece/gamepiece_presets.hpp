// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file gamepiece_presets.hpp
 * @brief Factory functions providing standard ball and field configurations.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece_sim.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim::BallGamepiecePresets {

/** @brief Returns BallProperties tuned for the FRC Infinite Recharge fuel cell. */
inline BallPhysicsSim3D::BallProperties fuelProperties() {
  BallPhysicsSim3D::BallProperties properties{};
  properties.mass_kg = 0.216;
  properties.radius_m = 0.075;
  properties.drag_coefficient = 0.35;
  properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
  properties.restitution = 0.30;
  return properties;
}

/** @brief Returns physics Config tuned for the FRC Infinite Recharge fuel cell. */
inline BallPhysicsSim3D::Config fuelConfig() {
  BallPhysicsSim3D::Config config = evergreenBallConfig();
  config.rolling_friction_per_s = 2.0;
  config.min_bounce_speed_mps = 0.03;
  return config;
}

/** @brief Returns a generic "evergreen" ball BallProperties suitable for any season. */
inline BallPhysicsSim3D::BallProperties evergreenBallProperties() {
  BallPhysicsSim3D::BallProperties properties{};
  properties.mass_kg = 0.24;
  properties.radius_m = 0.09;
  properties.drag_coefficient = 0.50;
  properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
  properties.restitution = 0.48;
  return properties;
}

/** @brief Returns a generic "evergreen" physics Config with sensible defaults. */
inline BallPhysicsSim3D::Config evergreenBallConfig() {
  BallPhysicsSim3D::Config config{};
  config.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
  config.air_density_kgpm3 = 1.225;
  config.magnus_coefficient = 1.0e-4;
  config.ground_height_m = 0.0;
  config.rolling_friction_per_s = 1.5;
  config.min_bounce_speed_mps = 0.06;
  return config;
}

/** @brief Returns a standard field config sized for a generic 16.54 x 8.21 m FRC field. */
inline BallGamepieceSim::FieldConfig evergreenFieldConfig() {
  BallGamepieceSim::FieldConfig config{};
  config.min_corner_m = Vector3(0.0, 0.0, 0.0);
  config.max_corner_m = Vector3(16.54, 8.21, 3.0);
  config.wall_restitution = 0.25;
  config.wall_friction = 0.2;
  return config;
}

}  // namespace frcsim::BallGamepiecePresets

/** @} */
