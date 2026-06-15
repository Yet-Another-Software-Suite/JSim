// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file season_2026_gamepiece_presets.hpp
 * @brief Factory functions for 2026 season ball properties, field config, and
 * goal zones.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece_presets.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim::BallGamepiecePresets {

/** @brief Returns field config tuned for the 2026 FRC game, including net
 * boundary user-id. */
inline BallGamepieceSim::FieldConfig season2026FieldConfig() {
  BallGamepieceSim::FieldConfig config = evergreenFieldConfig();
  config.net_boundary_user_id = 2026;
  config.wall_restitution = 0.22;
  config.wall_friction = 0.25;
  return config;
}

/** @brief Returns BallProperties for the 2026 season game piece (coral disc,
 * r=0.075 m). */
inline BallPhysicsSim3D::BallProperties season2026BallProperties() {
  BallPhysicsSim3D::BallProperties properties{};
  properties.mass_kg = 0.216;
  properties.radius_m = 0.15 * 0.5;
  properties.drag_coefficient = 0.58;
  properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
  properties.restitution = 0.52;
  return properties;
}

/** @brief Returns physics Config for the 2026 season game piece. */
inline BallPhysicsSim3D::Config season2026BallConfig() {
  BallPhysicsSim3D::Config config = evergreenBallConfig();
  config.magnus_coefficient = 1.2e-4;
  config.rolling_friction_per_s = 1.8;
  return config;
}

/**
 * @brief Applies the 2026 field layout to a simulator, including the hub net
 * boundary and goal zone.
 * @param sim Simulator to configure.
 */
inline void configureSeason2026Field(BallGamepieceSim& sim) {
  sim.setFieldConfig(season2026FieldConfig());
  sim.fieldElements().clear();
  sim.goals().clear();

  EnvironmentalBoundary net{};
  net.type = BoundaryType::kBox;
  net.user_id = 2026;
  net.position_m = Vector3(7.0, 4.1, 1.7);
  net.half_extents_m = Vector3(0.4, 0.4, 0.5);
  net.is_active = true;
  sim.addFieldElement(net);

  BallGamepieceSim::GoalZone hub_goal{};
  hub_goal.shape = BallGamepieceSim::GoalZone::Shape::kBox;
  hub_goal.center_m = net.position_m;
  hub_goal.half_extents_m = Vector3(0.35, 0.35, 0.45);
  hub_goal.accepted_type = BallGamepieceSim::GamePieceType::kBall;
  hub_goal.require_positive_vertical_velocity = false;
  sim.addGoalZone(hub_goal);
}

}  // namespace frcsim::BallGamepiecePresets

/** @} */
