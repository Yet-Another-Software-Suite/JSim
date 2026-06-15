// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file season_2025_gamepiece.hpp
 * @brief Season-2025 convenience gamepiece class pre-configured with 2025
 * presets.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece.hpp"
#include "frcsim/gamepiece/season_2025_gamepiece_presets.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim {

/**
 * @brief Season-2025 convenience gamepiece wrapper.
 *
 * Creates `Gamepiece` instances pre-configured with the 2025 presets.
 */
class Season2025Gamepiece : public Gamepiece {
 public:
  Season2025Gamepiece() : Gamepiece(BallGamepiecePresets::season2025BallConfig(), BallGamepiecePresets::season2025BallProperties()) {
    setTypeName("Ball2025");
  }

  /**
   * @brief Constructs with explicit config and properties, still sets type
   * name.
   * @param cfg Physics environment config.
   * @param props Ball physical properties.
   */
  Season2025Gamepiece(const Gamepiece::Config& cfg, const Gamepiece::Properties& props) : Gamepiece(cfg, props) { setTypeName("Ball2025"); }

  /** @brief Returns default 2025 ball properties. */
  static Gamepiece::Properties defaultProperties() { return BallGamepiecePresets::season2025BallProperties(); }

  /** @brief Returns default 2025 physics config. */
  static Gamepiece::Config defaultConfig() { return BallGamepiecePresets::season2025BallConfig(); }
};

}  // namespace frcsim

/** @} */
