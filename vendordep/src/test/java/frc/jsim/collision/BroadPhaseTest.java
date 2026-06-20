package frc.jsim.collision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.material.Material;
import org.junit.jupiter.api.Test;

class BroadPhaseTest {

    private static RigidBody sphere(double px, double py, double pz,
                                     double radius, boolean isStatic) {
        int flags = isStatic ? RigidBodyFlags.STATIC : RigidBodyFlags.DYNAMIC;
        RigidBody b = new RigidBody("s", flags, Material.DEFAULT);
        b.posX = px; b.posY = py; b.posZ = pz;
        b.collider = new SphereCollider(radius);
        if (isStatic) b.setStatic();
        else b.setMassProperties(1.0, 1, 1, 1);
        b.collider.computeAABB(b.getPose(), b.aabb);
        return b;
    }

    private static List<CollisionPair> findPairs(List<RigidBody> bodies) {
        List<CollisionPair> out = new ArrayList<>();
        BroadPhase.findPairs(bodies, out);
        return out;
    }

    @Test
    void overlappingSpheres_produceOnePair() {
        RigidBody a = sphere(0, 0, 0, 1.0, false);
        RigidBody b = sphere(1.0, 0, 0, 1.0, false); // overlapping by 1 m
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertEquals(1, pairs.size());
    }

    @Test
    void separateSpheres_noPair() {
        RigidBody a = sphere(0, 0, 0, 0.5, false);
        RigidBody b = sphere(10, 0, 0, 0.5, false);
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertTrue(pairs.isEmpty());
    }

    @Test
    void twoStaticBodies_noPair() {
        RigidBody a = sphere(0, 0, 0, 1.0, true);
        RigidBody b = sphere(0, 0, 0, 1.0, true); // same position — still no pair
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertTrue(pairs.isEmpty(), "Two static bodies must never be paired");
    }

    @Test
    void dynamicAndStaticOverlapping_producesPair() {
        RigidBody dyn = sphere(0, 0, 0, 1.0, false);
        RigidBody stat = sphere(0, 0, 0, 1.0, true);
        List<CollisionPair> pairs = findPairs(List.of(dyn, stat));
        assertEquals(1, pairs.size());
    }

    @Test
    void noCollisionBody_isSkipped() {
        RigidBody a = new RigidBody("a", RigidBodyFlags.NO_COLLISION, Material.DEFAULT);
        a.posX = 0; a.posY = 0; a.posZ = 0;
        a.collider = new SphereCollider(1.0);
        a.setMassProperties(1.0, 1, 1, 1);
        a.collider.computeAABB(a.getPose(), a.aabb);

        RigidBody b = sphere(0, 0, 0, 1.0, false);
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertTrue(pairs.isEmpty(), "NO_COLLISION body should be skipped in broadphase");
    }

    @Test
    void nullColliderBody_isSkipped() {
        RigidBody a = new RigidBody("a", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        a.posX = 0; a.posY = 0; a.posZ = 0;
        a.setMassProperties(1.0, 1, 1, 1);
        // collider is null

        RigidBody b = sphere(0, 0, 0, 1.0, false);
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertTrue(pairs.isEmpty(), "Body with null collider should be skipped");
    }

    @Test
    void singleBody_noPairs() {
        RigidBody a = sphere(0, 0, 0, 1.0, false);
        assertTrue(findPairs(List.of(a)).isEmpty());
    }

    @Test
    void emptyList_noPairs() {
        assertTrue(findPairs(List.of()).isEmpty());
    }

    @Test
    void threeBodies_onlyOneOverlappingPair() {
        // A at 0, B at 1.5 (overlap with A), C at 10 (no overlap)
        RigidBody a = sphere(0, 0, 0, 1.0, false);
        RigidBody b = sphere(1.5, 0, 0, 1.0, false);
        RigidBody c = sphere(10, 0, 0, 1.0, false);
        List<CollisionPair> pairs = findPairs(List.of(a, b, c));
        assertEquals(1, pairs.size());
    }

    @Test
    void outputListIsCleared_beforeRepopulation() {
        RigidBody a = sphere(0, 0, 0, 1.0, false);
        RigidBody b = sphere(0, 0, 0, 1.0, false);
        List<CollisionPair> out = new ArrayList<>();
        BroadPhase.findPairs(List.of(a, b), out);
        assertEquals(1, out.size());
        BroadPhase.findPairs(List.of(), out); // second call — no bodies
        assertEquals(0, out.size(), "Output list should be cleared between calls");
    }

    @Test
    void pairContainsCorrectBodies() {
        RigidBody a = sphere(0, 0, 0, 1.0, false);
        RigidBody b = sphere(0, 0, 0, 1.0, false);
        List<CollisionPair> pairs = findPairs(List.of(a, b));
        assertEquals(1, pairs.size());
        CollisionPair p = pairs.get(0);
        assertTrue((p.a == a && p.b == b) || (p.a == b && p.b == a),
            "Pair must reference the two input bodies");
    }
}
