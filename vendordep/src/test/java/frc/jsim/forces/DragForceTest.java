package frc.jsim.forces;

import static org.junit.jupiter.api.Assertions.*;

import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.material.Material;
import org.junit.jupiter.api.Test;

class DragForceTest {

    private static RigidBody dynamic() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        b.setMassProperties(1.0, 1, 1, 1);
        return b;
    }

    @Test
    void linearDragOpposesFowardVelocity() {
        RigidBody b = dynamic();
        b.velX = 4.0;
        new DragForce(b, 0.5, 0.0).apply(0.02);
        assertEquals(-2.0, b.forceX, 1e-10); // -0.5 * 4.0
        assertEquals(0.0, b.forceY, 1e-12);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void angularDragOpposesRotation() {
        RigidBody b = dynamic();
        b.omZ = 3.0;
        new DragForce(b, 0.0, 2.0).apply(0.02);
        assertEquals(0.0, b.torqueX, 1e-12);
        assertEquals(0.0, b.torqueY, 1e-12);
        assertEquals(-6.0, b.torqueZ, 1e-10); // -2.0 * 3.0
    }

    @Test
    void zeroVelocity_noDragForce() {
        RigidBody b = dynamic();
        new DragForce(b, 5.0, 5.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
        assertEquals(0.0, b.forceZ, 1e-12);
        assertEquals(0.0, b.torqueX, 1e-12);
    }

    @Test
    void skipsStaticBody() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.STATIC, Material.DEFAULT);
        b.setStatic();
        b.velX = 10.0;
        new DragForce(b, 1.0, 1.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
    }

    @Test
    void bothLinearAndAngularDragApplied() {
        RigidBody b = dynamic();
        b.velX = 2.0; b.velY = 3.0; b.velZ = 1.0;
        b.omX = 1.0; b.omY = 0.5; b.omZ = 2.0;
        double ld = 0.3, ad = 0.7;
        new DragForce(b, ld, ad).apply(0.02);
        assertEquals(-ld * 2.0, b.forceX, 1e-10);
        assertEquals(-ld * 3.0, b.forceY, 1e-10);
        assertEquals(-ld * 1.0, b.forceZ, 1e-10);
        assertEquals(-ad * 1.0, b.torqueX, 1e-10);
        assertEquals(-ad * 0.5, b.torqueY, 1e-10);
        assertEquals(-ad * 2.0, b.torqueZ, 1e-10);
    }

    @Test
    void negativVelocity_dragIsPositive() {
        RigidBody b = dynamic();
        b.velX = -5.0;
        new DragForce(b, 1.0, 0.0).apply(0.02);
        assertEquals(5.0, b.forceX, 1e-10); // force opposes motion
    }
}
