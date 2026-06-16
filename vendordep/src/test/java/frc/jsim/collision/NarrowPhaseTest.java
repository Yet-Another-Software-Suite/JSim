package frc.jsim.collision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.ArrayList;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.material.Material;
import org.junit.jupiter.api.Test;

class NarrowPhaseTest {

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    private static RigidBody sphere(double px, double py, double pz, double radius) {
        RigidBody b = new RigidBody("s", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        b.posX = px; b.posY = py; b.posZ = pz;
        b.collider = new SphereCollider(radius);
        double I = 0.4 * 1.0 * radius * radius;
        b.setMassProperties(1.0, I, I, I);
        return b;
    }

    private static RigidBody box(double px, double py, double pz,
                                  double hx, double hy, double hz) {
        RigidBody b = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        b.posX = px; b.posY = py; b.posZ = pz;
        b.collider = new BoxCollider(hx, hy, hz);
        b.setMassProperties(1.0, 1, 1, 1);
        return b;
    }

    private static RigidBody staticFloor() {
        RigidBody b = new RigidBody("floor", RigidBodyFlags.STATIC, Material.CARPET);
        b.setStatic();
        b.collider = new PlaneCollider(0, 0, 1, 0);
        return b;
    }

    private static List<ContactPoint> contacts(RigidBody a, RigidBody b) {
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(a, b));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        return out;
    }

    // ------------------------------------------------------------------
    // Sphere vs Sphere
    // ------------------------------------------------------------------

    @Test
    void sphereSphere_overlapping_generatesContact() {
        // Two unit spheres whose centres are 1.5 apart (overlap of 0.5)
        RigidBody a = sphere(0, 0, 0, 1.0);
        RigidBody b = sphere(1.5, 0, 0, 1.0);
        List<ContactPoint> out = contacts(a, b);
        assertEquals(1, out.size(), "Should produce exactly one contact");
        ContactPoint cp = out.get(0);
        assertEquals(0.5, cp.penetration, 1e-10);
    }

    @Test
    void sphereSphere_touching_noContact() {
        // Spheres exactly touching (distance == sum of radii) should NOT generate contact
        RigidBody a = sphere(0, 0, 0, 1.0);
        RigidBody b = sphere(2.0, 0, 0, 1.0);
        assertEquals(0, contacts(a, b).size());
    }

    @Test
    void sphereSphere_separated_noContact() {
        RigidBody a = sphere(0, 0, 0, 0.5);
        RigidBody b = sphere(5, 0, 0, 0.5);
        assertEquals(0, contacts(a, b).size());
    }

    @Test
    void sphereSphere_normalPointsFromBToA() {
        RigidBody a = sphere(1, 0, 0, 1.0);  // A is to the right
        RigidBody b = sphere(-1, 0, 0, 1.0); // B is to the left
        List<ContactPoint> out = contacts(a, b);
        assertEquals(1, out.size());
        ContactPoint cp = out.get(0);
        // Normal should point from B toward A, i.e., (+1, 0, 0)
        assertEquals(1.0, cp.normalX, 1e-10);
        assertEquals(0.0, cp.normalY, 1e-10);
        assertEquals(0.0, cp.normalZ, 1e-10);
    }

    @Test
    void sphereSphere_contactBodiesAssignedCorrectly() {
        RigidBody a = sphere(0, 0, 0, 1.0);
        RigidBody b = sphere(1.5, 0, 0, 1.0);
        ContactPoint cp = contacts(a, b).get(0);
        assertSame(a, cp.bodyA);
        assertSame(b, cp.bodyB);
    }

    @Test
    void sphereSphere_combinedMaterialIsSet() {
        RigidBody a = sphere(0, 0, 0, 1.0);
        RigidBody b = sphere(1.5, 0, 0, 1.0);
        ContactPoint cp = contacts(a, b).get(0);
        assertTrue(cp.combinedFriction >= 0, "friction must be non-negative");
        assertTrue(cp.combinedRestitution >= 0 && cp.combinedRestitution <= 1);
    }

    // ------------------------------------------------------------------
    // Sphere vs Plane
    // ------------------------------------------------------------------

    @Test
    void spherePlane_above_noContact() {
        RigidBody s = sphere(0, 0, 2.0, 0.5); // centre 2m above floor, radius 0.5
        RigidBody floor = staticFloor();
        // Dispatch: sphere is A, plane is B
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(s, floor));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(0, out.size());
    }

    @Test
    void spherePlane_penetrating_contact() {
        RigidBody s = sphere(0, 0, 0.3, 0.5); // centre 0.3m above floor, radius 0.5 → pen=0.2
        RigidBody floor = staticFloor();
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(s, floor));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(1, out.size());
        assertEquals(0.2, out.get(0).penetration, 1e-9);
    }

    @Test
    void spherePlane_normalPointsUp() {
        RigidBody s = sphere(0, 0, 0.3, 0.5);
        RigidBody floor = staticFloor();
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(s, floor));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(1, out.size());
        // Normal should point upward from floor toward sphere
        assertEquals(0.0, out.get(0).normalX, 1e-9);
        assertEquals(0.0, out.get(0).normalY, 1e-9);
        assertEquals(1.0, out.get(0).normalZ, 1e-9);
    }

    // ------------------------------------------------------------------
    // Box vs Plane
    // ------------------------------------------------------------------

    @Test
    void boxPlane_cornerBelowFloor_contact() {
        // Box at z=0.4, half-height 0.5 → bottom corners at z=-0.1 (penetrating)
        RigidBody b = box(0, 0, 0.4, 0.3, 0.3, 0.5);
        RigidBody floor = staticFloor();
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(b, floor));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(1, out.size());
        assertTrue(out.get(0).penetration > 0, "box corner should be penetrating floor");
    }

    @Test
    void boxPlane_fullyAbove_noContact() {
        // Box bottom at z=1 (half-height 0.5, centre at z=1.5)
        RigidBody b = box(0, 0, 1.5, 0.3, 0.3, 0.5);
        RigidBody floor = staticFloor();
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(b, floor));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(0, out.size());
    }

    // ------------------------------------------------------------------
    // Box vs Sphere
    // ------------------------------------------------------------------

    @Test
    void boxSphere_overlapping_contact() {
        RigidBody b = box(0, 0, 0, 1.0, 1.0, 1.0);  // unit cube centred at origin
        RigidBody s = sphere(1.5, 0, 0, 0.8);         // sphere centre at x=1.5, radius 0.8 → overlap 0.3
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(b, s));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(1, out.size());
        assertTrue(out.get(0).penetration > 0);
    }

    @Test
    void boxSphere_separated_noContact() {
        RigidBody b = box(0, 0, 0, 0.5, 0.5, 0.5);
        RigidBody s = sphere(5, 0, 0, 0.5);
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(b, s));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(0, out.size());
    }

    // ------------------------------------------------------------------
    // Box vs Box
    // ------------------------------------------------------------------

    @Test
    void boxBox_overlapping_contact() {
        RigidBody a = box(0, 0, 0, 1.0, 1.0, 1.0);
        RigidBody b = box(1.5, 0, 0, 1.0, 1.0, 1.0); // overlap 0.5 in X
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(a, b));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(1, out.size());
        assertTrue(out.get(0).penetration > 0);
    }

    @Test
    void boxBox_separated_noContact() {
        RigidBody a = box(0, 0, 0, 0.5, 0.5, 0.5);
        RigidBody b = box(5, 0, 0, 0.5, 0.5, 0.5);
        List<CollisionPair> pairs = new ArrayList<>();
        pairs.add(new CollisionPair(a, b));
        List<ContactPoint> out = new ArrayList<>();
        NarrowPhase.generateContacts(pairs, out);
        assertEquals(0, out.size());
    }
}
