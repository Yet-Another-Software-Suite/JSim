// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file physics_world.cpp
 * @brief PhysicsWorld step loop, collision resolution, and object factory methods.
 *
 * The step() method is the heart of the simulation: it applies aerodynamic
 * forces, integrates bodies, solves assembly joint constraints, advances
 * gamepiece physics, and finally resolves all pairwise collisions (gamepiece-
 * body and gamepiece-gamepiece). Order matters — gamepiece physics runs after
 * body integration so carried pieces follow their carrier correctly.
 */

#include "frcsim/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "frcsim/aerodynamics/drag_model.hpp"

namespace frcsim {

RigidBody& PhysicsWorld::createBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  if (!config_.enable_gravity) {
    bodies_.back().flags().enable_gravity = false;
  }
  return bodies_.back();
}

RigidAssembly& PhysicsWorld::createAssembly() {
  assemblies_.emplace_back();
  return assemblies_.back();
}

BallPhysicsSim3D& PhysicsWorld::createBall(
    const Gamepiece::Config& config,
    const Gamepiece::Properties& properties) {
  // Backwards-compatible wrapper: create a Gamepiece and return base reference.
  Gamepiece& gamepiece = createGamepiece(config, properties);
  return static_cast<BallPhysicsSim3D&>(gamepiece);
}

EnvironmentalBoundary& PhysicsWorld::addBoundary() {
  boundaries_.emplace_back();
  return boundaries_.back();
}

Gamepiece& PhysicsWorld::createGamepiece(
    const Gamepiece::Config& config,
    const Gamepiece::Properties& properties) {
  gamepieces_.emplace_back(config, properties);
  return gamepieces_.back();
}

void PhysicsWorld::addGlobalForceGenerator(
    const std::shared_ptr<ForceGenerator>& generator) {
  if (generator) {
    global_force_generators_.push_back(generator);
  }
}

void PhysicsWorld::setMaterialInteraction(
    const MaterialInteraction& interaction) {
  // NOTE: IDs are canonicalised (min,max) so that (A,B) and (B,A) resolve to
  // the same entry; this makes lookup symmetric without a second hash bucket.
  const std::int32_t a = std::min(interaction.material_a_id,
                                  interaction.material_b_id);
  const std::int32_t b = std::max(interaction.material_a_id,
                                  interaction.material_b_id);
  for (auto& existing : material_interactions_) {
    const std::int32_t ea = std::min(existing.material_a_id,
                                     existing.material_b_id);
    const std::int32_t eb = std::max(existing.material_a_id,
                                     existing.material_b_id);
    if (ea == a && eb == b) {
      existing = interaction;
      return;
    }
  }
  material_interactions_.push_back(interaction);
}

void PhysicsWorld::clearMaterialInteractions() {
  material_interactions_.clear();
}

bool PhysicsWorld::shouldInteract(std::uint32_t layer_a, std::uint32_t mask_a,
                                  std::uint32_t layer_b,
                                  std::uint32_t mask_b) const {
  return ((layer_a & mask_b) != 0u) && ((layer_b & mask_a) != 0u);
}

const PhysicsWorld::MaterialInteraction* PhysicsWorld::findMaterialInteraction(
    std::int32_t material_a_id, std::int32_t material_b_id) const {
  const std::int32_t a = std::min(material_a_id, material_b_id);
  const std::int32_t b = std::max(material_a_id, material_b_id);
  for (const auto& interaction : material_interactions_) {
    if (!interaction.enabled) {
      continue;
    }
    const std::int32_t ia =
        std::min(interaction.material_a_id, interaction.material_b_id);
    const std::int32_t ib =
        std::max(interaction.material_a_id, interaction.material_b_id);
    if (ia == a && ib == b) {
      return &interaction;
    }
  }
  return nullptr;
}

void PhysicsWorld::step() {
  const double dt_s = config_.fixed_dt_s;

  constexpr double kPi = 3.14159265358979323846;
  // Returns a sphere-equivalent collision radius for a body based on its
  // aerodynamic geometry. Used for broadphase contact testing against
  // boundaries and gamepieces; a single radius is cheaper than shape-specific
  // GJK and sufficient for FRC game accuracy requirements.
  auto body_collision_radius_m = [&](const RigidBody& body) {
    const auto* geom = body.aerodynamicGeometry();
    if (geom) {
      switch (geom->shape) {
        case RigidBody::AerodynamicGeometry::Shape::kSphere:
          return std::max(0.0, geom->radius_m);
        case RigidBody::AerodynamicGeometry::Shape::kBox: {
          const Vector3 half = geom->box_dimensions_m * 0.5;
          return std::max(0.0, half.norm());
        }
        case RigidBody::AerodynamicGeometry::Shape::kCylinder: {
          const double radius_m = std::max(0.0, geom->radius_m);
          const double half_len_m = std::max(0.0, geom->cylinder_length_m) * 0.5;
          return std::sqrt(radius_m * radius_m + half_len_m * half_len_m);
        }
        case RigidBody::AerodynamicGeometry::Shape::kCustom:
        default:
          if (geom->reference_area_m2 > 0.0) {
            return std::sqrt(geom->reference_area_m2 / kPi);
          }
          break;
      }
    }

    const double fallback_area_m2 =
        std::max(0.0, config_.default_drag_reference_area_m2);
    if (fallback_area_m2 > 0.0) {
      return std::sqrt(fallback_area_m2 / kPi);
    }

    return 0.1;
  };

  auto resolve_boundary_contact = [&](RigidBody& body) {
    if (!config_.enable_collision_detection || body.flags().is_kinematic) {
      return;
    }

    const double body_radius_m = body_collision_radius_m(body);

    const Material* body_material = body.material();
    const double body_restitution =
        body_material ? body_material->coefficient_of_restitution : 0.4;
    const double body_mu_kinetic =
        body_material ? body_material->coefficient_of_friction_kinetic : 0.6;

    for (const auto& boundary : boundaries_) {
      if (!boundary.is_active) {
        continue;
      }

      if (!shouldInteract(body.collisionLayer(), body.collisionMask(),
                          boundary.collision_layer_bits,
                          boundary.collision_mask_bits)) {
        continue;
      }

      Vector3 contact_normal_world = boundary.orientation.rotate(boundary.normal());
      if (contact_normal_world.isZero()) {
        contact_normal_world = Vector3::unitZ();
      } else {
        contact_normal_world = contact_normal_world.normalized();
      }

      bool has_contact = false;
      double penetration_m = 0.0;

      if (boundary.type == BoundaryType::kPlane || boundary.type == BoundaryType::kWall) {
        const Vector3 rel = body.position() - boundary.position_m;
        const double signed_distance_m = rel.dot(contact_normal_world);
        if (signed_distance_m < body_radius_m) {
          has_contact = true;
          penetration_m = body_radius_m - signed_distance_m;
        }
      } else if (boundary.type == BoundaryType::kBox) {
        const Vector3 rel_world = body.position() - boundary.position_m;
        const Vector3 rel_local = boundary.orientation.inverse().rotate(rel_world);
        const Vector3 half = boundary.half_extents_m;

        const bool inside_xy =
            std::abs(rel_local.x) <= std::max(0.0, half.x) &&
            std::abs(rel_local.y) <= std::max(0.0, half.y);
        const double top_local_z = std::max(0.0, half.z);
        if (inside_xy && rel_local.z < top_local_z + body_radius_m) {
          has_contact = true;
          penetration_m = (top_local_z + body_radius_m) - rel_local.z;
          contact_normal_world = boundary.orientation.rotate(Vector3::unitZ()).normalized();
        }
      } else if (boundary.type == BoundaryType::kCylinder) {
        const Vector3 rel_world = body.position() - boundary.position_m;
        const Vector3 rel_local = boundary.orientation.inverse().rotate(rel_world);
        const double radial_m = std::sqrt(rel_local.x * rel_local.x + rel_local.y * rel_local.y);
        const double radius_m = std::max(0.0, boundary.radius_m);
        const double top_local_z = std::max(0.0, boundary.half_extents_m.z);

        if (radial_m <= radius_m && rel_local.z < top_local_z + body_radius_m) {
          has_contact = true;
          penetration_m = (top_local_z + body_radius_m) - rel_local.z;
          contact_normal_world = boundary.orientation.rotate(Vector3::unitZ()).normalized();
        }
      }

      if (!has_contact || penetration_m <= 0.0) {
        continue;
      }

      body.setPosition(body.position() + contact_normal_world * penetration_m);

      const Vector3 velocity = body.linearVelocity();
      const double vn = velocity.dot(contact_normal_world);
      if (vn >= 0.0) {
        continue;
      }

        double restitution = std::clamp(
          0.5 * (boundary.restitution + body_restitution), 0.0, 1.0);
        double friction =
          std::max(0.0, 0.5 * (boundary.friction_coefficient + body_mu_kinetic));

        // NOTE: Per-material-pair overrides win unconditionally; if a pair
        // entry exists it replaces the averaged body+boundary values entirely
        // rather than blending again, so the table provides exact control.
        if (const auto* pair =
            findMaterialInteraction(body.materialId(), boundary.material_id)) {
        restitution = std::clamp(pair->restitution, 0.0, 1.0);
        friction = std::max(0.0, pair->friction);
        }

      const Vector3 v_normal = contact_normal_world * vn;
      const Vector3 v_tangent = velocity - v_normal;
      const Vector3 v_normal_after = contact_normal_world * (-vn * restitution);

      Vector3 v_tangent_after = v_tangent;
      const double vt_mag = v_tangent.norm();
      if (vt_mag > 1e-9) {
        const double friction_impulse_ratio =
            std::clamp(friction * (1.0 + restitution) * std::abs(vn) / vt_mag,
                       0.0, 1.0);
        v_tangent_after = v_tangent * (1.0 - friction_impulse_ratio);
      }

      body.setLinearVelocity(v_normal_after + v_tangent_after);
    }
  };

  auto step_body = [&](RigidBody& body) {
    if (config_.enable_aerodynamics && !body.flags().is_kinematic) {
      DragModel drag_model(config_.default_drag_coefficient,
                           config_.default_drag_reference_area_m2,
                           config_.air_density_kgpm3,
                           config_.linear_drag_coefficient_n_per_mps);
      body.applyForce(drag_model.computeForce(body));

      const Vector3 magnus =
          Vector3::magnusForce(body.linearVelocity(), body.angularVelocity(),
                               config_.magnus_coefficient);
      body.applyForce(magnus);
    }

    for (const auto& generator : global_force_generators_) {
      generator->apply(body, dt_s);
    }

    body.integrate(dt_s, config_.integration_method, config_.gravity_mps2,
                   config_.linear_damping_per_s, config_.angular_damping_per_s);

    resolve_boundary_contact(body);
  };

  for (auto& body : bodies_) {
    step_body(body);
  }

  for (auto& assembly : assemblies_) {
    for (auto& body : assembly.bodies()) {
      step_body(body);
    }
    if (config_.enable_joint_constraints) {
      assembly.solveConstraints(dt_s, 4);
    }
  }

  for (auto& gamepiece : gamepieces_) {
    gamepiece.step(dt_s);
  }

  // Resolve gamepiece <-> rigid-body collisions after the gamepiece has advanced its own physics.
  // NOTE: Collisions are resolved after gamepiece.step() so that a carried
  // gamepiece has already been moved to its carrier's position before the
  // collision check runs — preventing spurious contacts with the carrying body.
  for (auto& gamepiece : gamepieces_) {
    // Only consider airborne gamepieces for rigid-body collisions.
    if (gamepiece.getGamepieceState() != Gamepiece::State::kAirborne) {
      continue;
    }
    BallPhysicsSim3D::BallState s = gamepiece.state();

    const BallPhysicsSim3D::BallProperties& bp = gamepiece.ballProperties();
    const double gamepiece_r = std::max(0.0, bp.radius_m);
    const double gamepiece_m = std::max(1e-9, bp.mass_kg);
    const double inv_gamepiece_m = 1.0 / gamepiece_m;

    auto apply_impulse = [&](RigidBody& body, const Vector3& contact_normal) {
      Vector3 normal = contact_normal;
      if (normal.isZero()) {
        normal = Vector3::unitZ();
      } else {
        normal = normal.normalized();
      }

      const Vector3 rel_vel = s.velocity_mps - body.linearVelocity();
      const double vn = rel_vel.dot(normal);
      if (vn >= 0.0) {
        return;
      }

      const Material* body_mat = body.material();
      const double body_restitution =
          body_mat ? body_mat->coefficient_of_restitution : 0.4;
      const double e = std::clamp(0.5 * (bp.restitution + body_restitution), 0.0, 1.0);
      const double inv_body_m = body.flags().is_kinematic ? 0.0 : body.inverseMass();
      const double j = -(1.0 + e) * vn / (inv_gamepiece_m + inv_body_m);
      const Vector3 impulse = normal * j;

      s.velocity_mps += impulse * inv_gamepiece_m;
      if (!body.flags().is_kinematic) {
        body.setLinearVelocity(body.linearVelocity() - impulse * inv_body_m);
      }
    };

    if (!config_.enable_collision_detection) {
      gamepiece.setState(s);
      continue;
    }

    auto resolve_gamepiece_body_collision = [&](RigidBody& body) {
      const double body_r = body_collision_radius_m(body);
      const Vector3 rel = s.position_m - body.position();
      const double dist = rel.norm();
      const double contact_dist = gamepiece_r + body_r;
      if (dist <= contact_dist) {
        const Vector3 normal = dist > 1e-9 ? rel / dist : Vector3::unitZ();
        s.position_m += normal * (contact_dist - dist);
        apply_impulse(body, normal);
      }
    };

    for (auto& body : bodies_) {
      resolve_gamepiece_body_collision(body);
    }

    for (auto& assembly : assemblies_) {
      for (auto& body : assembly.bodies()) {
        resolve_gamepiece_body_collision(body);
      }
    }

    gamepiece.setState(s);
  }

  // Resolve gamepiece <-> gamepiece collisions (pairwise) so gamepieces interact with each
  // other. We only consider airborne gamepieces here to match rigid-body
  // collision handling.
  // NOTE: O(n^2) pairwise loop is acceptable because FRC matches typically
  // have fewer than ~20 gamepieces on the field simultaneously.
  for (std::size_t i = 0; i < gamepieces_.size(); ++i) {
    for (std::size_t j = i + 1; j < gamepieces_.size(); ++j) {
      auto& a = gamepieces_[i];
      auto& b = gamepieces_[j];

      if (a.getGamepieceState() != Gamepiece::State::kAirborne ||
          b.getGamepieceState() != Gamepiece::State::kAirborne) {
        continue;
      }

      auto sa = a.state();
      auto sb = b.state();

      const auto& ap = a.ballProperties();
      const auto& bp = b.ballProperties();
      const double ra = std::max(0.0, ap.radius_m);
      const double rb = std::max(0.0, bp.radius_m);
      const double ma = std::max(1e-9, ap.mass_kg);
      const double mb = std::max(1e-9, bp.mass_kg);
      const double inv_ma = 1.0 / ma;
      const double inv_mb = 1.0 / mb;

      Vector3 rel = sa.position_m - sb.position_m;
      const double dist = rel.norm();
      const double contact_dist = ra + rb;
      if (dist <= contact_dist) {
        Vector3 normal = dist > 1e-9 ? rel / dist : Vector3::unitZ();

        // positional correction: separate proportionally to mass
        const double penetration = contact_dist - dist;
        const double total_m = ma + mb;
        if (total_m > 0.0) {
          sa.position_m += normal * (penetration * (mb / total_m));
          sb.position_m -= normal * (penetration * (ma / total_m));
        }

        const Vector3 rel_vel = sa.velocity_mps - sb.velocity_mps;
        const double vn = rel_vel.dot(normal);
        if (vn < 0.0) {
          const double ea = ap.restitution;
          const double eb = bp.restitution;
          const double e = std::clamp(0.5 * (ea + eb), 0.0, 1.0);
          const double j = -(1.0 + e) * vn / (inv_ma + inv_mb);
          const Vector3 impulse = normal * j;

          sa.velocity_mps += impulse * inv_ma;
          sb.velocity_mps -= impulse * inv_mb;
        }

        a.setState(sa);
        b.setState(sb);
      }
    }
  }

  ++step_count_;
  accumulated_sim_time_s_ += dt_s;
}

}  // namespace frcsim
