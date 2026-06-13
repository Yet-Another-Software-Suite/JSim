// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file gamepiece_sim.hpp
 * @brief Convenience alias selecting the active gamepiece simulator implementation.
 */

#pragma once

#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"

/** @addtogroup gamepieces @{ */

namespace frcsim {

/// @brief Canonical type alias for the active gamepiece simulator.
using GamepieceSim = BallGamepieceSim;

}  // namespace frcsim

/** @} */
