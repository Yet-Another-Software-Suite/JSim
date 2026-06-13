/**
 * @file api.h
 * @brief Minimal C++ extern "C" API for creating worlds, bodies, and gamepieces via pointer handles.
 *
 * This header exposes a thin C++ ABI using raw pointers to JSim types.
 * It is suitable for lightweight embedding where the caller controls object lifetime.
 *
 * @note Prefer driverheader.h for a fully C-compatible opaque-handle API.
 */
// Minimal C API wrapper for basic world/ball/body operations.
#pragma once

#include "frcsim/physics_world.hpp"

extern "C" {

using PhysicsWorld_t = frcsim::PhysicsWorld;
using RigidBody_t = frcsim::RigidBody;
using Gamepiece_t = frcsim::Gamepiece;

/** @brief Allocates and returns a new PhysicsWorld using default configuration. */
PhysicsWorld_t* frcsim_create_world();

/**
 * @brief Destroys a PhysicsWorld and releases all associated memory.
 * @param w World to destroy; must not be used after this call.
 */
void frcsim_destroy_world(PhysicsWorld_t* w);

/**
 * @brief Creates a rigid body in the given world.
 * @param w Target world.
 * @param mass_kg Body mass in kilograms.
 * @return Pointer to the new RigidBody owned by the world.
 */
RigidBody_t* frcsim_create_body(PhysicsWorld_t* w, double mass_kg);

/**
 * @brief Creates a generic gamepiece in the given world.
 *
 * For now this materializes a ball-based gamepiece instance; non-ball types
 * may be added in the future.
 *
 * @param w Target world.
 * @param config Ball physics environment config; may be nullptr for defaults.
 * @param props Ball physical properties; may be nullptr for defaults.
 * @return Pointer to the new Gamepiece owned by the world.
 */
Gamepiece_t* frcsim_create_gamepiece(PhysicsWorld_t* w,
                                    const frcsim::Gamepiece::Config* config,
                                    const frcsim::Gamepiece::Properties* props);

/**
 * @brief Advances the world by dt_s seconds.
 * @param w Target world.
 * @param dt_s Simulation timestep in seconds.
 */
void frcsim_step_world(PhysicsWorld_t* w, double dt_s);

/**
 * @brief Sets box collision geometry on a body.
 * @param body Target body.
 * @param dim_x Full extent along local X in meters.
 * @param dim_y Full extent along local Y in meters.
 * @param dim_z Full extent along local Z in meters.
 */
void frcsim_set_body_box_geometry(RigidBody_t* body, double dim_x, double dim_y,
                                  double dim_z);

/**
 * @brief Sets sphere collision geometry on a body.
 * @param body Target body.
 * @param radius Sphere radius in meters.
 */
void frcsim_set_body_sphere_geometry(RigidBody_t* body, double radius);

/**
 * @brief Sets the world-space position of a body.
 * @param body Target body.
 * @param x World X position in meters.
 * @param y World Y position in meters.
 * @param z World Z position in meters.
 */
void frcsim_set_body_position(RigidBody_t* body, double x, double y, double z);

/**
 * @brief Reads position and velocity from a gamepiece.
 * @param gamepiece Gamepiece to query.
 * @param px Output X position in meters.
 * @param py Output Y position in meters.
 * @param pz Output Z position in meters.
 * @param vx Output X velocity in m/s.
 * @param vy Output Y velocity in m/s.
 * @param vz Output Z velocity in m/s.
 */
void frcsim_get_gamepiece_state(Gamepiece_t* gamepiece, double* px, double* py, double* pz,
                                double* vx, double* vy, double* vz);

/**
 * @brief Launches a gamepiece from a muzzle pose into free flight.
 * @param gamepiece Gamepiece to launch.
 * @param px Muzzle world X position in meters.
 * @param py Muzzle world Y position in meters.
 * @param pz Muzzle world Z position in meters.
 * @param vx Launch velocity X in m/s.
 * @param vy Launch velocity Y in m/s.
 * @param vz Launch velocity Z in m/s.
 */
void frcsim_gamepiece_outtake(Gamepiece_t* gamepiece, double px, double py, double pz,
                              double vx, double vy, double vz);

}
