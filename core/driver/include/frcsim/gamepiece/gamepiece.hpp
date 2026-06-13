// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file gamepiece.hpp
 * @brief Generic runtime gamepiece abstraction extending BallPhysicsSim3D.
 */

#pragma once

#include "frcsim/gamepiece/ball_physics.hpp"
#include <algorithm>
#include <cctype>
#include <string>

/** @addtogroup gamepieces @{ */

namespace frcsim {

/**
 * @brief Generic runtime gamepiece abstraction.
 *
 * For now this type is a thin specialization of BallPhysicsSim3D to provide
 * a single top-level gamepiece abstraction that can be extended later for
 * other hitbox families (boxes, custom CAD meshes, etc.).
 */
class Gamepiece : public BallPhysicsSim3D {
 public:
  using Config = BallPhysicsSim3D::Config;
  using Properties = BallPhysicsSim3D::BallProperties;

  using BallPhysicsSim3D::BallPhysicsSim3D;

  /// @brief High-level lifecycle state of a gamepiece.
  enum class State {
    /// @brief Gamepiece is in free flight (full physics).
    kAirborne,
    /// @brief Gamepiece is resting on the ground plane.
    kGrounded,
    /// @brief Gamepiece is attached to a carrier robot.
    kHeld,
  };

  Gamepiece() = default;
  Gamepiece(const BallPhysicsSim3D::Config& cfg,
            const BallPhysicsSim3D::BallProperties& props)
      : BallPhysicsSim3D(cfg, props) {}

  /**
   * @brief Overrides the current lifecycle state.
   * @param s New state to assign.
   */
  void setGamepieceState(State s) { state_ = s; }
  /// @brief Returns the current lifecycle state.
  State getGamepieceState() const { return state_; }

  /**
   * @brief Attempt to pick up the gamepiece into a carrier.
   * Delegates to BallPhysicsSim3D::requestPickup and flips state to kHeld
   * on success.
   */
  bool pick(const PickupRequest& req) {
    const bool ok = requestPickup(req);
    if (ok) {
      state_ = State::kHeld;
    }
    return ok;
  }

  /**
   * @brief Place the gamepiece at world position and mark as grounded.
   */
  void place(const Vector3& pos) {
    BallPhysicsSim3D::BallState s = BallPhysicsSim3D::state();
    s.position_m = pos;
    s.velocity_mps = Vector3::zero();
    s.spin_radps = Vector3::zero();
    s.held = false;
    BallPhysicsSim3D::setState(s);
    state_ = State::kGrounded;
  }

  /**
   * @brief Outtake (launch) the gamepiece into free flight and mark as airborne.
   */
  void outtake(const Vector3& muzzle_position_m,
             const Vector3& muzzle_velocity_mps,
             const Vector3& muzzle_spin_radps = Vector3::zero()) {
    BallPhysicsSim3D::shoot(muzzle_position_m, muzzle_velocity_mps,
                            muzzle_spin_radps);
    state_ = State::kAirborne;
  }

  /**
   * @brief Step wrapper that applies low-cost behavior for grounded/held.
   */
  void step(double dt_s) {
    if (state_ == State::kHeld) {
      // Held pieces are updated by carrier pose; avoid integrating.
      return;
    }

    if (state_ == State::kGrounded) {
      // Lightweight ground update: snap to ground and decay planar speed.
      BallPhysicsSim3D::BallState s = BallPhysicsSim3D::state();
      const double floor_z = BallPhysicsSim3D::config().ground_height_m +
                             BallPhysicsSim3D::ballProperties().radius_m;
      if (s.position_m.z < floor_z) {
        s.position_m.z = floor_z;
      }

      const double planar_speed = s.velocity_mps.planarSpeed();
      if (planar_speed > 1e-9) {
        const double decay =
            std::max(0.0, 1.0 - BallPhysicsSim3D::config().rolling_friction_per_s * dt_s);
        s.velocity_mps.x *= decay;
        s.velocity_mps.y *= decay;
        if (s.velocity_mps.planarSpeed() <= 1e-3) {
          s.velocity_mps.x = 0.0;
          s.velocity_mps.y = 0.0;
        }
      } else {
        s.velocity_mps.x = 0.0;
        s.velocity_mps.y = 0.0;
      }

      s.velocity_mps.z = 0.0;
      s.spin_radps = Vector3::zero();
      BallPhysicsSim3D::setState(s);
      return;
    }

    // Default: airborne -> perform full ball physics step.
    BallPhysicsSim3D::step(dt_s);
    // If ball touches ground during step, transition to grounded state.
    const double floor_z = BallPhysicsSim3D::config().ground_height_m +
                           BallPhysicsSim3D::ballProperties().radius_m;
    if (BallPhysicsSim3D::state().position_m.z <= floor_z + 1e-6) {
      state_ = State::kGrounded;
    }
  }

  /**
   * @brief Sets a human-readable type name for this gamepiece.
   * @param name Type name string (e.g. "generic_sphere", "fuel_2026").
   */
  void setTypeName(const std::string& name) { type_name_ = name; }

  /**
   * @brief Returns the registered type name.
   * @return Type name string, which may be empty when unset.
   */
  const std::string& typeName() const { return type_name_; }

  /**
   * @brief Returns true when this gamepiece should follow the ball physics path.
   *
   * Ball path is used when the type name is empty, contains "ball" or "sphere",
   * or starts with "generic_".
   */
  bool isBall() const {
    if (type_name_.empty()) return true;
    std::string lower = type_name_;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c){ return std::tolower(c); });
    if (lower.find("ball") != std::string::npos) return true;
    if (lower.find("sphere") != std::string::npos) return true;
    if (lower.rfind("generic_", 0) == 0) return true;
    return false;
  }

 private:
  State state_{State::kGrounded};
  std::string type_name_{};
};

}  // namespace frcsim

/** @} */
