// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file driversource.cpp
 * @brief C ABI wrappers around the frcsim physics engine for language-binding
 * use.
 *
 * All worlds are stored in a process-wide handle map protected by a single
 * mutex. Functions return 0 on success and -1 on error to keep the interface
 * simple for FFI consumers (Java, Python, etc.).
 */

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "driverheader.h"
#include "frcsim/physics_world.hpp"
#include "frcsim/rigidbody/material.hpp"

namespace {

/// Mutex protecting global world map and related state.
std::mutex g_world_mutex;

/// Map from handle -> owned PhysicsWorld instance.
std::unordered_map<std::uint64_t, std::unique_ptr<frcsim::PhysicsWorld>> g_worlds;

/// Monotonic handle allocator for new worlds.
std::uint64_t g_next_handle = 1;

/// Mapping of world handle -> (gamepiece index -> type tag)
static std::unordered_map<std::uint64_t, std::unordered_map<int, int>> g_gamepiece_types;

/**
 * @brief Convert a small integer kind tag to a human readable type name.
 *
 * @param type Integer type tag.
 * @return Corresponding string name, or "unknown" if not recognised.
 */
std::string typeNameForKind(int type) {
  switch (type) {
    case 0:
      return "Ball";
    case 1:
      return "Fuel";
    case 2:
      return "Coral";
    case 3:
      return "custom1";
    case 4:
      return "custom2";
    case 5:
      return "custom3";
    case 6:
      return "custom4";
    default:
      return "unknown";
  }
}

/**
 * @brief Check whether a world handle refers to a live world.
 *
 * @param handle World handle to check.
 * @return true if the world exists, false otherwise.
 */
bool worldExists(std::uint64_t handle) { return g_worlds.find(handle) != g_worlds.end(); }

/**
 * @brief Lookup a world pointer for a handle.
 *
 * @param handle World handle.
 * @return Pointer to the world, or nullptr if not found.
 */
frcsim::PhysicsWorld* lookupWorld(std::uint64_t handle) {
  if (!worldExists(handle)) {
    return nullptr;
  }

  return g_worlds.at(handle).get();
}

/**
 * @brief Alias for lookupWorld (keeps historical naming).
 *
 * @param handle World handle.
 * @return Pointer to the world, or nullptr if not found.
 */
frcsim::PhysicsWorld* getWorld(std::uint64_t handle) { return lookupWorld(handle); }

/**
 * @brief Safely get a body pointer from a world by index.
 *
 * Returns nullptr for invalid indices or null world pointers.
 *
 * @param world PhysicsWorld pointer.
 * @param body_index Index of the requested body.
 * @return Pointer to the body or nullptr.
 */
frcsim::RigidBody* getBody(frcsim::PhysicsWorld* world, int body_index) {
  if (!world || body_index < 0) {
    return nullptr;
  }
  auto& bodies = world->bodies();
  const std::size_t idx = static_cast<std::size_t>(body_index);
  if (idx >= bodies.size()) {
    return nullptr;
  }
  return &bodies[idx];
}

}  // namespace

extern "C" {
/**
 * @brief C-compatible API exported for language bindings.
 *
 * The functions below use plain C types and simple return codes where
 * appropriate (0 for success, -1 for error) to make interop straightforward.
 */

/**
 * @brief No-op helper kept for binary compatibility.
 */
void c_doThing(void) {}

/**
 * @brief Create a new physics world and return its opaque handle.
 *
 * @param fixed_dt_s Fixed simulation timestep (seconds). If <= 0, a
 * reasonable default is used.
 * @param enable_gravity Non-zero to enable default gravity.
 * @return Opaque world handle (non-zero) on success.
 */
uint64_t c_rsCreateWorld(double fixed_dt_s, int enable_gravity) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = (fixed_dt_s > 0.0) ? fixed_dt_s : 0.01;
  config.enable_gravity = (enable_gravity != 0);

  const std::uint64_t handle = g_next_handle++;
  g_worlds.emplace(handle, std::make_unique<frcsim::PhysicsWorld>(config));
  return handle;
}

/**
 * @brief Destroy a previously created world and release resources.
 *
 * Calling with an invalid handle is a no-op.
 *
 * @param world_handle Handle returned from c_rsCreateWorld.
 */
void c_rsDestroyWorld(uint64_t world_handle) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  g_worlds.erase(world_handle);
}

/**
 * @brief Create a rigid body in the given world.
 *
 * @param world_handle Target world handle.
 * @param mass_kg Mass in kilograms for the body.
 * @return Index of the created body within the world, or -1 on error.
 */
int c_rsCreateBody(uint64_t world_handle, double mass_kg) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  world->createBody(mass_kg);
  return static_cast<int>(world->bodies().size() - 1);
}

/**
 * @brief Backwards-compatible wrapper for legacy 'ball' API.
 *
 * Some language bindings and export definitions still reference
 * `c_rsCreateBall`. Provide a thin wrapper that forwards to the
 * new `c_rsCreateGamepiece` implementation.
 */
// Legacy `c_rsCreateBall` compatibility wrapper removed; use
// `c_rsCreateGamepiece` instead.

/**
 * @brief Create a spherical gamepiece in the world.

 * @param world_handle Target world handle.
 * @param radius_m Radius in meters.
 * @param mass_kg Mass in kilograms.
 * @param restitution Coefficient of restitution [0..1].
 * @return Index of the created gamepiece, or -1 on error.
 */
int c_rsCreateGamepiece(uint64_t world_handle, double radius_m, double mass_kg, double restitution) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  frcsim::BallPhysicsSim3D::BallProperties props;
  props.radius_m = std::max(0.0, radius_m);
  if (props.radius_m <= 0.0) {
    props.radius_m = 0.12;
  }

  props.mass_kg = (mass_kg > 0.0) ? mass_kg : 0.27;
  props.restitution = std::clamp(restitution, 0.0, 1.0);
  props.reference_area_m2 = 3.14159265358979323846 * props.radius_m * props.radius_m;

  world->createBall(frcsim::BallPhysicsSim3D::Config(), props);
  return static_cast<int>(world->gamepieces().size() - 1);
}

/**
 * @brief Create a gamepiece and associate a small integer type tag.
 *
 * This helper stores the type tag in an internal map and attempts to set
 * a readable type name on the gamepiece when available.
 *
 * @param world_handle World handle.
 * @param type Integer type tag.
 * @param radius_m Radius in meters.
 * @param mass_kg Mass in kg.
 * @param restitution Restitution [0..1].
 * @return Index of the created gamepiece, or -1 on error.
 */
int c_rsCreateGamepieceWithType(uint64_t world_handle, int type, double radius_m, double mass_kg, double restitution) {
  const int idx = c_rsCreateGamepiece(world_handle, radius_m, mass_kg, restitution);
  if (idx < 0) {
    return idx;
  }

  // NOTE: c_rsCreateGamepiece already acquires and releases g_world_mutex, so
  // we take a fresh lock here to update the type map and set the name.
  std::lock_guard<std::mutex> lock(g_world_mutex);
  g_gamepiece_types[world_handle][idx] = type;
  const std::string type_name = typeNameForKind(type);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (world) {
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx < world->gamepieces().size()) {
      try {
        world->gamepieces()[uidx].setTypeName(type_name);
      } catch (...) {
      }
    }
  }
  return idx;
}

/**
 * @brief Create a gamepiece and set a textual type name.
 *
 * @param world_handle World handle.
 * @param type_name Null-terminated type name (best-effort; may be ignored).
 * @param radius_m Radius in meters.
 * @param mass_kg Mass in kg.
 * @param restitution Restitution [0..1].
 * @return Index of the created gamepiece, or -1 on error.
 */
int c_rsCreateGamepieceWithTypeName(uint64_t world_handle, const char* type_name, double radius_m, double mass_kg, double restitution) {
  // Create the gamepiece using existing spherical defaults today.
  int idx = c_rsCreateGamepiece(world_handle, radius_m, mass_kg, restitution);
  if (idx < 0) {
    return idx;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (world) {
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx < world->gamepieces().size()) {
      try {
        if (type_name) {
          world->gamepieces()[uidx].setTypeName(type_name);
        }
      } catch (...) {
        // best-effort only
      }
    }
  }
  return idx;
}

/**
 * @brief Retrieve the type name for a gamepiece.
 *
 * Returns a pointer to thread-local storage which remains valid until the
 * next call on the same thread.
 *
 * @param world_handle World handle.
 * @param gamepiece_index Index of the gamepiece.
 * @return Null-terminated name or nullptr if unavailable.
 */
const char* c_rsGetGamepieceTypeName(uint64_t world_handle, int gamepiece_index) {
  // NOTE: A thread-local string is used so the returned pointer stays valid
  // until the next call on the same thread, satisfying C interop callers that
  // immediately copy the string (e.g., JNI NewStringUTF).
  thread_local std::string type_name_storage;

  std::lock_guard<std::mutex> lock(g_world_mutex);
  if (!worldExists(world_handle) || gamepiece_index < 0) return nullptr;
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (world) {
    auto& gamepieces = world->gamepieces();
    const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
    if (idx < gamepieces.size()) {
      const std::string& direct_name = gamepieces[idx].typeName();
      if (!direct_name.empty()) {
        type_name_storage = direct_name;
        return type_name_storage.c_str();
      }
    }
  }
  auto it = g_gamepiece_types.find(world_handle);
  if (it == g_gamepiece_types.end()) return nullptr;
  auto it2 = it->second.find(gamepiece_index);
  if (it2 == it->second.end()) return nullptr;

  type_name_storage = typeNameForKind(it2->second);
  return type_name_storage.empty() ? nullptr : type_name_storage.c_str();
}

/**
 * @brief Attempt to pick up a gamepiece with an intake position.
 *
 * @param world_handle World handle.
 * @param gamepiece_index Index of the gamepiece to pick.
 * @param intake_x Intake x position (m).
 * @param intake_y Intake y position (m).
 * @param intake_z Intake z position (m).
 * @param capture_radius Capture radius (m).
 * @param carry_offset_x Carry offset x component (m).
 * @param carry_offset_y Carry offset y component (m).
 * @param carry_offset_z Carry offset z component (m).
 * @return 0 on success, -1 on failure.
 */
int c_rsPickGamepiece(uint64_t world_handle, int gamepiece_index, double intake_x, double intake_y, double intake_z, double capture_radius,
                      double carry_offset_x, double carry_offset_y, double carry_offset_z) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  auto& gps = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gps.size()) {
    return -1;
  }

  frcsim::BallPhysicsSim3D::PickupRequest req;
  req.intake_position_m = frcsim::Vector3{intake_x, intake_y, intake_z};
  req.capture_radius_m = std::max(0.0, capture_radius);
  req.carry_offset_m = frcsim::Vector3{carry_offset_x, carry_offset_y, carry_offset_z};

  const bool ok = gps[idx].pick(req);
  return ok ? 0 : -1;
}

/**
 * @brief Place a gamepiece at the given world-space position.
 *
 * Convenience wrapper around c_rsSetGamepiecePosition.
 */
int c_rsPlaceGamepiece(uint64_t world_handle, int gamepiece_index, double x_m, double y_m, double z_m) {
  return c_rsSetGamepiecePosition(world_handle, gamepiece_index, x_m, y_m, z_m);
}

/**
 * @brief Eject (outtake) a held gamepiece with given position and velocity.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsOuttakeGamepiece(uint64_t world_handle, int gamepiece_index, double px, double py, double pz, double vx, double vy, double vz) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  auto& gps = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gps.size()) {
    return -1;
  }

  gps[idx].outtake(frcsim::Vector3{px, py, pz}, frcsim::Vector3{vx, vy, vz});
  return 0;
}

/**
 * @brief Set the position of a rigid body by index.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyPosition(uint64_t world_handle, int body_index, double x_m, double y_m, double z_m) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->setPosition(frcsim::Vector3{x_m, y_m, z_m});
  return 0;
}

/**
 * @brief Set a body's linear velocity.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyLinearVelocity(uint64_t world_handle, int body_index, double vx_mps, double vy_mps, double vz_mps) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->setLinearVelocity(frcsim::Vector3{vx_mps, vy_mps, vz_mps});
  return 0;
}

/**
 * @brief Set a rigid body's orientation.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyOrientation(uint64_t world_handle, int body_index, double qw, double qx, double qy, double qz) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->setOrientation(frcsim::Quaternion(qw, qx, qy, qz).normalized());
  return 0;
}

/**
 * @brief Enable or disable gravity for a specific body.
 *
 * @param enabled Non-zero to enable gravity.
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyGravityEnabled(uint64_t world_handle, int body_index, int enabled) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  body->flags().enable_gravity = (enabled != 0);
  return 0;
}

/**
 * @brief Read a rigid body's orientation quaternion.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsGetBodyOrientation(uint64_t world_handle, int body_index, double* out_qw, double* out_qx, double* out_qy, double* out_qz) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }
  const auto& q = body->orientation();
  if (out_qw) *out_qw = q.w;
  if (out_qx) *out_qx = q.x;
  if (out_qy) *out_qy = q.y;
  if (out_qz) *out_qz = q.z;
  return 0;
}

/**
 * @brief Set material properties for a body.
 *
 * Values are clamped to reasonable ranges where applicable.
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyMaterial(uint64_t world_handle, int body_index, double restitution, double friction_kinetic, double friction_static,
                        double collision_damping) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::Material material;
  material.coefficient_of_restitution = std::clamp(restitution, 0.0, 1.0);
  material.coefficient_of_friction_kinetic = std::max(0.0, friction_kinetic);
  material.coefficient_of_friction_static = std::max(0.0, friction_static);
  material.collision_damping = std::clamp(collision_damping, 0.0, 1.0);

  body->setMaterial(material);
  return 0;
}

/**
 * @brief Assign a material identifier to a body for interaction lookup.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyMaterialId(uint64_t world_handle, int body_index, int32_t material_id) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  body->setMaterialId(material_id);
  return 0;
}

/**
 * @brief Configure collision layer/mask bits for a body.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyCollisionFilter(uint64_t world_handle, int body_index, uint32_t collision_layer_bits, uint32_t collision_mask_bits) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  body->setCollisionLayer(collision_layer_bits);
  body->setCollisionMask(collision_mask_bits);
  return 0;
}

/**
 * @brief Define a spherical aerodynamic proxy for the body.
 *
 * Also updates the world's default drag coefficient when provided.
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyAerodynamicSphere(uint64_t world_handle, int body_index, double radius_m, double drag_coefficient) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::RigidBody::AerodynamicGeometry geometry;
  geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kSphere;
  geometry.radius_m = std::max(0.0, radius_m);
  body->setAerodynamicGeometry(geometry);

  world->config().default_drag_coefficient = std::max(0.0, drag_coefficient);
  return 0;
}

/**
 * @brief Define a box-shaped aerodynamic proxy for the body.
 * @return 0 on success, -1 on error.
 */
int c_rsSetBodyAerodynamicBox(uint64_t world_handle, int body_index, double x_m, double y_m, double z_m, double drag_coefficient) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  frcsim::RigidBody::AerodynamicGeometry geometry;
  geometry.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kBox;
  geometry.box_dimensions_m = frcsim::Vector3(std::max(0.0, x_m), std::max(0.0, y_m), std::max(0.0, z_m));
  body->setAerodynamicGeometry(geometry);

  world->config().default_drag_coefficient = std::max(0.0, drag_coefficient);
  return 0;
}

/**
 * @brief Directly set a gamepiece's position.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetGamepiecePosition(uint64_t world_handle, int gamepiece_index, double x_m, double y_m, double z_m) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  auto& gamepieces = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gamepieces.size()) {
    return -1;
  }

  auto state = gamepieces[idx].state();
  state.position_m = frcsim::Vector3{x_m, y_m, z_m};
  gamepieces[idx].setState(state);
  return 0;
}

/**
 * @brief Set a gamepiece's linear velocity.
 * @return 0 on success, -1 on error.
 */
int c_rsSetGamepieceLinearVelocity(uint64_t world_handle, int gamepiece_index, double vx_mps, double vy_mps, double vz_mps) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  auto& gamepieces = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gamepieces.size()) {
    return -1;
  }

  auto state = gamepieces[idx].state();
  state.velocity_mps = frcsim::Vector3{vx_mps, vy_mps, vz_mps};
  gamepieces[idx].setState(state);
  return 0;
}

/**
 * @brief Configure global aerodynamics settings for a world.
 * @return 0 on success, -1 on error.
 */
int c_rsSetWorldAerodynamics(uint64_t world_handle, int enabled, double air_density_kgpm3, double linear_drag_coefficient_n_per_mps,
                             double magnus_coefficient, double default_drag_coefficient, double default_drag_reference_area_m2) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  auto& cfg = world->config();
  cfg.enable_aerodynamics = (enabled != 0);
  cfg.air_density_kgpm3 = std::max(0.0, air_density_kgpm3);
  cfg.linear_drag_coefficient_n_per_mps = std::max(0.0, linear_drag_coefficient_n_per_mps);
  cfg.magnus_coefficient = magnus_coefficient;
  cfg.default_drag_coefficient = std::max(0.0, default_drag_coefficient);
  cfg.default_drag_reference_area_m2 = std::max(0.0, default_drag_reference_area_m2);

  return 0;
}

/**
 * @brief Define interaction properties between two material ids.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetMaterialInteraction(uint64_t world_handle, int32_t material_a_id, int32_t material_b_id, double restitution, double friction,
                               int enabled) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  frcsim::PhysicsWorld::MaterialInteraction interaction;
  interaction.material_a_id = material_a_id;
  interaction.material_b_id = material_b_id;
  interaction.restitution = std::clamp(restitution, 0.0, 1.0);
  interaction.friction = std::max(0.0, friction);
  interaction.enabled = (enabled != 0);
  world->setMaterialInteraction(interaction);
  return 0;
}

/**
 * @brief Advance the world's simulation by a number of fixed steps.
 *
 * @param steps Number of internal fixed steps to run (clamped to >= 1).
 * @return 0 on success, -1 on error.
 */
int c_rsStepWorld(uint64_t world_handle, int steps) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const int safe_steps = std::max(steps, 1);
  for (int i = 0; i < safe_steps; ++i) {
    world->step();
  }
  return 0;
}

/**
 * @brief Set the world's gravity vector (m/s^2) and enable gravity.
 *
 * @return 0 on success, -1 on error.
 */
int c_rsSetWorldGravity(uint64_t world_handle, double gx_mps2, double gy_mps2, double gz_mps2) {
  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  world->config().gravity_mps2 = frcsim::Vector3{gx_mps2, gy_mps2, gz_mps2};
  world->config().enable_gravity = true;
  return 0;
}

/**
 * @brief Read a gamepiece's position into output pointers.
 *
 * All output pointers must be non-null.
 * @return 0 on success, -1 on error.
 */
int c_rsGetGamepiecePosition(uint64_t world_handle, int gamepiece_index, double* x_m, double* y_m, double* z_m) {
  if (!x_m || !y_m || !z_m) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  const auto& gamepieces = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gamepieces.size()) {
    return -1;
  }

  const frcsim::Vector3 position = gamepieces[idx].state().position_m;
  *x_m = position.x;
  *y_m = position.y;
  *z_m = position.z;
  return 0;
}

/**
 * @brief Read a gamepiece's linear velocity into output pointers.
 * @return 0 on success, -1 on error.
 */
int c_rsGetGamepieceLinearVelocity(uint64_t world_handle, int gamepiece_index, double* vx_mps, double* vy_mps, double* vz_mps) {
  if (!vx_mps || !vy_mps || !vz_mps) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world || gamepiece_index < 0) {
    return -1;
  }

  const auto& gamepieces = world->gamepieces();
  const std::size_t idx = static_cast<std::size_t>(gamepiece_index);
  if (idx >= gamepieces.size()) {
    return -1;
  }

  const frcsim::Vector3 velocity = gamepieces[idx].state().velocity_mps;
  *vx_mps = velocity.x;
  *vy_mps = velocity.y;
  *vz_mps = velocity.z;
  return 0;
}

/**
 * @brief Read a rigid body's position into output pointers.
 * @return 0 on success, -1 on error.
 */
int c_rsGetBodyPosition(uint64_t world_handle, int body_index, double* x_m, double* y_m, double* z_m) {
  if (!x_m || !y_m || !z_m) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  const frcsim::Vector3 p = body->position();
  *x_m = p.x;
  *y_m = p.y;
  *z_m = p.z;
  return 0;
}

/**
 * @brief Read a rigid body's linear velocity into output pointers.
 * @return 0 on success, -1 on error.
 */
int c_rsGetBodyLinearVelocity(uint64_t world_handle, int body_index, double* vx_mps, double* vy_mps, double* vz_mps) {
  if (!vx_mps || !vy_mps || !vz_mps) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  frcsim::RigidBody* body = getBody(world, body_index);
  if (!body) {
    return -1;
  }

  const frcsim::Vector3 v = body->linearVelocity();
  *vx_mps = v.x;
  *vy_mps = v.y;
  *vz_mps = v.z;
  return 0;
}

/**
 * @brief Fill an array with pose (x,y,z, qw,qx,qy,qz) for up to max_bodies.
 *
 * The caller supplies a buffer of size at least 7*max_bodies doubles.
 * @return Number of bodies written, or -1 on error.
 */
int c_rsGetBodyPose7Array(uint64_t world_handle, double* out_pose7, int max_bodies) {
  if (!out_pose7 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto p = body.position();
    const auto q = body.orientation();
    const int base = i * 7;
    out_pose7[base + 0] = p.x;
    out_pose7[base + 1] = p.y;
    out_pose7[base + 2] = p.z;
    out_pose7[base + 3] = q.w;
    out_pose7[base + 4] = q.x;
    out_pose7[base + 5] = q.y;
    out_pose7[base + 6] = q.z;
  }
  return count;
}

/**
 * @brief Fill an array with linear and angular velocity (vx,vy,vz,wx,wy,wz).
 * @return Number of bodies written, or -1 on error.
 */
int c_rsGetBodyVelocity6Array(uint64_t world_handle, double* out_velocity6, int max_bodies) {
  if (!out_velocity6 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto v = body.linearVelocity();
    const auto w = body.angularVelocity();
    const int base = i * 6;
    out_velocity6[base + 0] = v.x;
    out_velocity6[base + 1] = v.y;
    out_velocity6[base + 2] = v.z;
    out_velocity6[base + 3] = w.x;
    out_velocity6[base + 4] = w.y;
    out_velocity6[base + 5] = w.z;
  }
  return count;
}

/**
 * @brief Fill an array with a compact 13-element state per body
 * (pos(3), quat(4), linvel(3), angvel(3)).
 * @return Number of bodies written, or -1 on error.
 */
int c_rsGetBodyState13Array(uint64_t world_handle, double* out_state13, int max_bodies) {
  if (!out_state13 || max_bodies < 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(g_world_mutex);
  frcsim::PhysicsWorld* world = getWorld(world_handle);
  if (!world) {
    return -1;
  }

  const auto& bodies = world->bodies();
  const int count = std::min(max_bodies, static_cast<int>(bodies.size()));
  for (int i = 0; i < count; ++i) {
    const auto& body = bodies[static_cast<std::size_t>(i)];
    const auto p = body.position();
    const auto q = body.orientation();
    const auto v = body.linearVelocity();
    const auto w = body.angularVelocity();

    const int base = i * 13;
    out_state13[base + 0] = p.x;
    out_state13[base + 1] = p.y;
    out_state13[base + 2] = p.z;
    out_state13[base + 3] = q.w;
    out_state13[base + 4] = q.x;
    out_state13[base + 5] = q.y;
    out_state13[base + 6] = q.z;
    out_state13[base + 7] = v.x;
    out_state13[base + 8] = v.y;
    out_state13[base + 9] = v.z;
    out_state13[base + 10] = w.x;
    out_state13[base + 11] = w.y;
    out_state13[base + 12] = w.z;
  }
  return count;
}
}  // extern "C"
