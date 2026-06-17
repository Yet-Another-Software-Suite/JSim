package frc.jsim.collision;

import frc.jsim.dynamics.RigidBody;

/** A candidate pair of bodies returned by broadphase for narrowphase evaluation. */
public final class CollisionPair {
    /** First body in the pair. */
    public final RigidBody a;
    /** Second body in the pair. */
    public final RigidBody b;

    /**
     * @param a first body
     * @param b second body
     */
    public CollisionPair(RigidBody a, RigidBody b) {
        this.a = a;
        this.b = b;
    }
}
