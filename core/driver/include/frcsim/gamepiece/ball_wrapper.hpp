// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file ball_wrapper.hpp
 * @brief Backwards-compatible ball type alias wrapping the generic Gamepiece.
 */

#pragma once

#include "frcsim/gamepiece/gamepiece.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim {

/**
 * @brief Thin, backwards-friendly ball wrapper around the generic `Gamepiece`.
 *
 * This type exists to make call sites that previously relied on `BallPhysicsSim3D`
 * or `Ball` compile with minimal changes while encouraging new code to use
 * the generic `Gamepiece` abstraction.
 */
class Ball : public Gamepiece {
 public:
  using Gamepiece::Gamepiece;

  Ball() = default;

  Ball(const Gamepiece::Config& cfg, const Gamepiece::Properties& props)
      : Gamepiece(cfg, props) {}

  /**
   * @brief Convenience factory returning a ball pre-configured with evergreen defaults.
   * @return Ball instance using default Config and Properties.
   */
  static Ball defaultEvergreen() {
    Gamepiece::Properties p = Gamepiece::Properties();
    Gamepiece::Config c = Gamepiece::Config();
    return Ball(c, p);
  }
};

}  // namespace frcsim

/** @} */
