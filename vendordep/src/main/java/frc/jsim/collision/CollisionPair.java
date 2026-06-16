package frc.jsim.collision;

import frc.jsim.dynamics.RigidBody;

/** A candidate pair of bodies returned by broadphase for narrowphase evaluation. */
public final class CollisionPair {
    public final RigidBody a;
    public final RigidBody b;

    public CollisionPair(RigidBody a, RigidBody b) {
        this.a = a;
        this.b = b;
    }
}
