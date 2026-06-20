package frc.jsim.forces;

import static org.junit.jupiter.api.Assertions.*;

import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.material.Material;
import org.junit.jupiter.api.Test;

/**
 * Tests for MagnusForce: F = k * (ω × v).
 */
class MagnusForceTest {

    private static RigidBody dynamic() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        b.setMassProperties(1.0, 1, 1, 1);
        return b;
    }

    @Test
    void magnusForceMatchesCrossProduct() {
        // ω = (0, 0, 1), v = (1, 0, 0) → ω × v = (0*0−1*0, 1*1−0*0, 0*0−0*1) = (0, 1, 0)
        RigidBody b = dynamic();
        b.omZ = 1.0;
        b.velX = 1.0;
        new MagnusForce(b, 2.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-10);
        assertEquals(2.0, b.forceY, 1e-10); // k * 1.0
        assertEquals(0.0, b.forceZ, 1e-10);
    }

    @Test
    void zeroOmega_noForce() {
        RigidBody b = dynamic();
        b.velX = 5.0;
        new MagnusForce(b, 1.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void zeroVelocity_noForce() {
        RigidBody b = dynamic();
        b.omX = 5.0;
        new MagnusForce(b, 1.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void skipsStaticBody() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.STATIC, Material.DEFAULT);
        b.setStatic();
        b.velX = 2.0;
        b.omZ = 1.0;
        new MagnusForce(b, 1.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
    }

    @Test
    void coefficientScalesForce() {
        // ω = (0, 0, 1), v = (1, 0, 0) → F = k * (0, 1, 0)
        RigidBody b = dynamic();
        b.omZ = 1.0;
        b.velX = 1.0;
        new MagnusForce(b, 3.5).apply(0.02);
        assertEquals(3.5, b.forceY, 1e-10);
    }

    @Test
    void parallelOmegaAndVelocity_noForce() {
        // ω ∥ v → ω × v = 0
        RigidBody b = dynamic();
        b.omX = 2.0;
        b.velX = 3.0;
        new MagnusForce(b, 1.0).apply(0.02);
        assertEquals(0.0, b.forceX, 1e-12);
        assertEquals(0.0, b.forceY, 1e-12);
        assertEquals(0.0, b.forceZ, 1e-12);
    }

    @Test
    void magnusForcePerpendicularToVelocity() {
        // ω = (0, 0, 1), v = (1, 0, 0) → F perpendicular to v (in Y direction)
        RigidBody b = dynamic();
        b.omZ = 1.0;
        b.velX = 1.0;
        new MagnusForce(b, 1.0).apply(0.02);
        // F · v should be zero (F perpendicular to v)
        double fDotV = b.forceX * b.velX + b.forceY * 0 + b.forceZ * 0;
        assertEquals(0.0, fDotV, 1e-12);
    }
}
