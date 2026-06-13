// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file season_2026_gamepiece.hpp
 * @brief Season-2026 convenience gamepiece class pre-configured with 2026 presets.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece.hpp"
#include "frcsim/gamepiece/season_2026_gamepiece_presets.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim {

/**
 * @brief Season-2026 convenience gamepiece wrapper.
 *
 * Creates `Gamepiece` instances pre-configured with the 2026 presets.
 */
class Season2026Gamepiece : public Gamepiece {
 public:
  Season2026Gamepiece()
      : Gamepiece(BallGamepiecePresets::season2026BallConfig(),
                  BallGamepiecePresets::season2026BallProperties()) {
    setTypeName("Ball2026");
  }

  /**
   * @brief Constructs with explicit config and properties, still sets type name.
   * @param cfg Physics environment config.
   * @param props Ball physical properties.
   */
  Season2026Gamepiece(const Gamepiece::Config& cfg,
                      const Gamepiece::Properties& props)
      : Gamepiece(cfg, props) {
    setTypeName("Ball2026");
  }

  /** @brief Returns default 2026 ball properties. */
  static Gamepiece::Properties defaultProperties() {
    return BallGamepiecePresets::season2026BallProperties();
  }

  /** @brief Returns default 2026 physics config. */
  static Gamepiece::Config defaultConfig() {
    return BallGamepiecePresets::season2026BallConfig();
  }
};

}  // namespace frcsim

/** @} */
