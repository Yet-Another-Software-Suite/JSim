/**
 * @file api.cpp
 * @brief Implements the high-level frcsim C API declared in frcsim/api.h.
 *
 * Thin wrappers around PhysicsWorld and Gamepiece that expose a stable
 * pointer-based C interface. Callers own the PhysicsWorld_t lifetime and are
 * responsible for calling frcsim_destroy_world.
 */

#include "frcsim/api.h"

#include "frcsim/physics_world.hpp"

extern "C" {

PhysicsWorld_t* frcsim_create_world() {
  return new PhysicsWorld_t;
}

void frcsim_destroy_world(PhysicsWorld_t* w) {
  delete w;
}

RigidBody_t* frcsim_create_body(PhysicsWorld_t* w, double mass_kg) {
  if (!w) return nullptr;
  return &w->createBody(mass_kg);
}

Gamepiece_t* frcsim_create_gamepiece(PhysicsWorld_t* w,
                                    const frcsim::Gamepiece::Config* config,
                                    const frcsim::Gamepiece::Properties* props) {
  if (!w) return nullptr;
  // NOTE: Either or both pointers may be null; substitute defaults so callers
  // can omit parameters they don't care about without a separate overload.
  if (config && props) {
    return static_cast<Gamepiece_t*>(&w->createGamepiece(*config, *props));
  }
  if (config) {
    return static_cast<Gamepiece_t*>(&w->createGamepiece(*config, frcsim::Gamepiece::Properties()));
  }
  if (props) {
    return static_cast<Gamepiece_t*>(&w->createGamepiece(frcsim::Gamepiece::Config(), *props));
  }
  return static_cast<Gamepiece_t*>(&w->createGamepiece(frcsim::Gamepiece::Config(), frcsim::Gamepiece::Properties()));
}

void frcsim_step_world(PhysicsWorld_t* w, double dt_s) {
  if (!w) return;
  // Use configured fixed_dt_s if dt_s <= 0
  if (dt_s > 0.0) {
    // NOTE: The timestep is patched directly on the config each call rather
    // than passed as a parameter so that the single-step PhysicsWorld::step()
    // interface stays timestep-agnostic internally.
    auto cfg = w->config();
    cfg.fixed_dt_s = dt_s;
    w->config() = cfg;
  }
  w->step();
}

void frcsim_set_body_box_geometry(RigidBody_t* body, double dim_x, double dim_y, double dim_z) {
  if (!body) return;
  frcsim::RigidBody::AerodynamicGeometry g;
  g.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kBox;
  g.box_dimensions_m = frcsim::Vector3(dim_x, dim_y, dim_z);
  body->setAerodynamicGeometry(g);
}

void frcsim_set_body_sphere_geometry(RigidBody_t* body, double radius) {
  if (!body) return;
  frcsim::RigidBody::AerodynamicGeometry g;
  g.shape = frcsim::RigidBody::AerodynamicGeometry::Shape::kSphere;
  g.radius_m = radius;
  body->setAerodynamicGeometry(g);
}

void frcsim_set_body_position(RigidBody_t* body, double x, double y, double z) {
  if (!body) return;
  body->setPosition(x, y, z);
}

void frcsim_get_gamepiece_state(Gamepiece_t* gamepiece, double* px, double* py, double* pz,
                                double* vx, double* vy, double* vz) {
  if (!gamepiece) return;
  const auto& state = gamepiece->state();
  if (px) *px = state.position_m.x;
  if (py) *py = state.position_m.y;
  if (pz) *pz = state.position_m.z;
  if (vx) *vx = state.velocity_mps.x;
  if (vy) *vy = state.velocity_mps.y;
  if (vz) *vz = state.velocity_mps.z;
}

void frcsim_gamepiece_outtake(Gamepiece_t* gamepiece, double px, double py, double pz,
                              double vx, double vy, double vz) {
  if (!gamepiece) return;
  frcsim::Vector3 pos(px, py, pz);
  frcsim::Vector3 vel(vx, vy, vz);
  gamepiece->outtake(pos, vel);
}

}
