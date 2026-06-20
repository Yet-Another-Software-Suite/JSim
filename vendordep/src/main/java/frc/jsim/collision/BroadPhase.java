package frc.jsim.collision;

import java.util.ArrayList;
import java.util.List;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;

/**
 * O(n²) AABB sweep broadphase.
 *
 * <p>FRC simulations rarely exceed ~50 bodies, so the quadratic cost is irrelevant.
 * Bodies with {@link RigidBodyFlags#NO_COLLISION} are skipped. Two static bodies
 * never collide with each other.
 */
public final class BroadPhase {
    private BroadPhase() {}

    /**
     * Return all body pairs whose cached AABBs overlap.
     * The supplied list is cleared and repopulated each call.
     *
     * @param bodies all bodies in the world
     * @param out    cleared and populated with overlapping pairs
     */
    public static void findPairs(List<RigidBody> bodies, List<CollisionPair> out) {
        out.clear();
        int n = bodies.size();
        for (int i = 0; i < n; i++) {
            RigidBody a = bodies.get(i);
            if (RigidBodyFlags.isSet(a.flags, RigidBodyFlags.NO_COLLISION)) continue;
            if (a.collider == null) continue;

            for (int j = i + 1; j < n; j++) {
                RigidBody b = bodies.get(j);
                if (RigidBodyFlags.isSet(b.flags, RigidBodyFlags.NO_COLLISION)) continue;
                if (b.collider == null) continue;
                // Two static bodies never interact.
                if (a.isStatic() && b.isStatic()) continue;

                if (aabbOverlap(a.aabb, b.aabb)) {
                    out.add(new CollisionPair(a, b));
                }
            }
        }
    }

    private static boolean aabbOverlap(double[] a, double[] b) {
        return a[0] <= b[3] && a[3] >= b[0]
            && a[1] <= b[4] && a[4] >= b[1]
            && a[2] <= b[5] && a[5] >= b[2];
    }
}
