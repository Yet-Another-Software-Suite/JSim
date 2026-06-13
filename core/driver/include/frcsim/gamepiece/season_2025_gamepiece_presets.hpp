// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file season_2025_gamepiece_presets.hpp
 * @brief Factory functions for 2025 season ball properties and field configuration.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece_presets.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim::BallGamepiecePresets {

/** @brief Returns field config tuned for the 2025 FRC game. */
inline BallGamepieceSim::FieldConfig season2025FieldConfig() {
  BallGamepieceSim::FieldConfig config = evergreenFieldConfig();
  config.net_boundary_user_id = 2025;
  config.wall_restitution = 0.24;
  config.wall_friction = 0.22;
  return config;
}

/** @brief Returns BallProperties for the 2025 season game piece. */
inline BallPhysicsSim3D::BallProperties season2025BallProperties() {
  BallPhysicsSim3D::BallProperties properties{};
  properties.mass_kg = 0.20;
  properties.radius_m = 0.075;
  properties.drag_coefficient = 0.45;
  properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
  properties.restitution = 0.46;
  return properties;
}

/** @brief Returns physics Config for the 2025 season game piece. */
inline BallPhysicsSim3D::Config season2025BallConfig() {
  BallPhysicsSim3D::Config config = evergreenBallConfig();
  config.magnus_coefficient = 1.1e-4;
  config.rolling_friction_per_s = 1.6;
  return config;
}

/**
 * @brief Applies the 2025 field config to a simulator, clearing previous obstacles and goals.
 * @param sim Simulator to configure.
 */
inline void configureSeason2025Field(BallGamepieceSim& sim) {
  sim.setFieldConfig(season2025FieldConfig());
  sim.fieldElements().clear();
  sim.goals().clear();
}

}  // namespace frcsim::BallGamepiecePresets

/** @} */
