package jsim.forces;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import jsim.core.SimConstants;
import jsim.dynamics.RigidBody;
import jsim.dynamics.RigidBodyFlags;
import jsim.material.Material;
import org.junit.jupiter.api.Test;

class GravityForceTest {

    private static RigidBody dynamic(double mass) {
        RigidBody b = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        b.setMassProperties(mass, 1, 1, 1);
        return b;
    }

    @Test
    void appliesDefaultGravityForce() {
        RigidBody b = dynamic(2.0);
        new GravityForce(List.of(b)).apply(0.02);
        assertEquals(2.0 * SimConstants.GRAVITY_Z, b.forceZ, 1e-10);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
    }

    @Test
    void skipsStaticBody() {
        RigidBody b = new RigidBody("floor", RigidBodyFlags.STATIC, Material.DEFAULT);
        b.setStatic();
        new GravityForce(List.of(b)).apply(0.02);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void skipsNoGravityBody() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.NO_GRAVITY, Material.DEFAULT);
        b.setMassProperties(1.0, 1, 1, 1);
        new GravityForce(List.of(b)).apply(0.02);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void skipsZeroInvMassBody() {
        // invMass is 0 by default if setMassProperties is never called
        RigidBody b = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        new GravityForce(List.of(b)).apply(0.02);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void customGravityVector() {
        RigidBody b = dynamic(3.0);
        new GravityForce(List.of(b), 1.0, 2.0, 3.0).apply(0.02);
        assertEquals(3.0, b.forceX, 1e-10);
        assertEquals(6.0, b.forceY, 1e-10);
        assertEquals(9.0, b.forceZ, 1e-10);
    }

    @Test
    void multipleBodies_onlyDynamicAffected() {
        RigidBody dyn = dynamic(5.0);
        RigidBody stat = new RigidBody("floor", RigidBodyFlags.STATIC, Material.DEFAULT);
        stat.setStatic();
        List<RigidBody> bodies = new ArrayList<>();
        bodies.add(dyn);
        bodies.add(stat);
        new GravityForce(bodies).apply(0.02);
        assertEquals(5.0 * SimConstants.GRAVITY_Z, dyn.forceZ, 1e-10);
        assertEquals(0.0, stat.forceZ, 1e-12);
    }

    @Test
    void forceMagnitudeScalesWithMass() {
        RigidBody b1 = dynamic(1.0);
        RigidBody b2 = dynamic(4.0);
        List<RigidBody> bodies = new ArrayList<>();
        bodies.add(b1);
        bodies.add(b2);
        new GravityForce(bodies).apply(0.02);
        assertEquals(4.0, b2.forceZ / b1.forceZ, 1e-9, "Force should scale linearly with mass");
    }
}
