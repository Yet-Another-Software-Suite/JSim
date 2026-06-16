package frc.jsim.solver;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import frc.jsim.collision.ContactPoint;
import frc.jsim.collision.SphereCollider;
import frc.jsim.collision.PlaneCollider;
import frc.jsim.dynamics.RigidBody;
import frc.jsim.dynamics.RigidBodyFlags;
import frc.jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Isolated unit tests for the impulse solver.
 * Bodies and contact points are built manually — no full world pipeline.
 *
 * <p>Key physics contract: for a pair of bodies with invMass > 0 and contact normal n,
 * the PGS solver must satisfy:
 * <ul>
 *   <li>Linear momentum is conserved: m_A*v_A + m_B*v_B = const.
 *   <li>Normal relative velocity post-solve ≥ 0 (non-penetrating).
 *   <li>With restitution e, post-solve vRelN ≈ e * |pre_vRelN|.
 * </ul>
 */
class ImpulseSolverTest {

    private ImpulseSolver solver;

    @BeforeEach
    void setUp() {
        solver = new ImpulseSolver(20); // 20 iterations for convergence
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /** Create a dynamic sphere body with the given state and mass. */
    private static RigidBody dynSphere(String name, double mass, double radius,
                                        double px, double py, double pz,
                                        double vx, double vy, double vz) {
        RigidBody b = new RigidBody(name, RigidBodyFlags.DYNAMIC, new Material(0.0, 0.0));
        b.posX = px; b.posY = py; b.posZ = pz;
        b.velX = vx; b.velY = vy; b.velZ = vz;
        double I = 0.4 * mass * radius * radius;
        b.setMassProperties(mass, I, I, I);
        b.collider = new SphereCollider(radius);
        b.refreshWorldInertia();
        return b;
    }

    /** Create a static floor plane at Z=0. */
    private static RigidBody staticFloor() {
        RigidBody b = new RigidBody("floor", RigidBodyFlags.STATIC, new Material(0.0, 0.0));
        b.setStatic();
        b.collider = new PlaneCollider(0, 0, 1, 0);
        return b;
    }

    /** Build a contact point between two bodies manually. */
    private static ContactPoint contact(
            RigidBody a, RigidBody b,
            double cx, double cy, double cz,
            double nx, double ny, double nz,
            double pen, double friction, double restitution) {
        ContactPoint cp = new ContactPoint();
        cp.set(a, b, cx, cy, cz, nx, ny, nz, pen);
        cp.combinedFriction = friction;
        cp.combinedRestitution = restitution;
        return cp;
    }

    private static double totalMomentumX(RigidBody... bodies) {
        double p = 0;
        for (RigidBody b : bodies) {
            if (b.invMass > 0) p += b.velX / b.invMass;
        }
        return p;
    }

    // ------------------------------------------------------------------
    // Elastic head-on collision (e=1)
    // ------------------------------------------------------------------

    @Test
    void elasticHeadOnCollision_velocitiesExchanged() {
        // Equal masses: A moving right, B moving left
        RigidBody a = dynSphere("A", 1.0, 0.1, -0.5, 0, 0, 1.0, 0, 0);
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0, -1.0, 0, 0);
        // Contact at origin, normal points from B to A = (-1,0,0)
        // rA and rB are parallel to n → no angular terms for this geometry
        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 1.0);
        List<ContactPoint> contacts = new ArrayList<>();
        contacts.add(cp);

        solver.solve(contacts, 0.02);

        // Velocities should be exchanged
        assertEquals(-1.0, a.velX, 1e-9, "A should now move left");
        assertEquals( 1.0, b.velX, 1e-9, "B should now move right");
        // Y, Z unchanged
        assertEquals(0.0, a.velY, 1e-9);
        assertEquals(0.0, b.velY, 1e-9);
    }

    @Test
    void elasticCollision_linearMomentumConserved() {
        RigidBody a = dynSphere("A", 2.0, 0.1, -0.5, 0, 0, 3.0, 0, 0);
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0, -1.0, 0, 0);
        double p0 = totalMomentumX(a, b); // 2*3 + 1*(-1) = 5
        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 1.0);
        solver.solve(List.of(cp), 0.02);
        assertEquals(p0, totalMomentumX(a, b), 1e-9, "Linear momentum must be conserved");
    }

    @Test
    void elasticCollision_kineticEnergyConserved() {
        RigidBody a = dynSphere("A", 1.0, 0.1, -0.5, 0, 0, 2.0, 0, 0);
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0, -2.0, 0, 0);
        double ke0 = 0.5 * (2.0 * 2.0 + 2.0 * 2.0); // 4.0 J

        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 1.0);
        solver.solve(List.of(cp), 0.02);

        double ke1 = 0.5 * (a.velX * a.velX + b.velX * b.velX);
        assertEquals(ke0, ke1, 1e-8, "Kinetic energy conserved for e=1");
    }

    // ------------------------------------------------------------------
    // Perfectly inelastic collision (e=0)
    // ------------------------------------------------------------------

    @Test
    void inelasticCollision_equalMasses_commonVelocityAlongNormal() {
        RigidBody a = dynSphere("A", 1.0, 0.1, -0.5, 0, 0, 2.0, 0, 0);
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0, -2.0, 0, 0);

        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 0.0);
        solver.solve(List.of(cp), 0.02);

        // Post-collision: relative normal velocity should be ~0 (no bounce)
        double vRelN = (a.velX - b.velX) * (-1.0); // n=(-1,0,0)
        assertEquals(0.0, vRelN, 1e-9, "Relative normal velocity should be zero (e=0)");
        // Momentum conserved (= 0 here)
        assertEquals(0.0, totalMomentumX(a, b), 1e-9);
    }

    @Test
    void inelasticCollision_momentumConserved() {
        RigidBody a = dynSphere("A", 3.0, 0.1, -0.5, 0, 0, 4.0, 0, 0);
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0, 0.0, 0, 0);
        double p0 = totalMomentumX(a, b); // 3*4 + 1*0 = 12

        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 0.0);
        solver.solve(List.of(cp), 0.02);

        assertEquals(p0, totalMomentumX(a, b), 1e-9);
    }

    // ------------------------------------------------------------------
    // Partial restitution (0 < e < 1)
    // ------------------------------------------------------------------

    @Test
    void partialRestitution_postVelocityMatchesTarget() {
        double e = 0.5;
        // Single sphere hitting a static floor at -2 m/s (downward)
        RigidBody ball = dynSphere("Ball", 1.0, 0.1, 0, 0, 0.1, 0, 0, -2.0);
        RigidBody floor = staticFloor();

        // Contact: contact at origin, normal pointing up (+Z), penetration = 0
        ContactPoint cp = contact(ball, floor, 0, 0, 0, 0, 0, 1, 0.0, 0.0, e);
        solver.solve(List.of(cp), 0.02);

        // Post-bounce: velocity should be +e*|incoming| = +1.0 m/s in Z
        assertEquals(1.0, ball.velZ, 1e-9, "Ball Z velocity post-bounce");
        // X and Y should be unaffected (zero friction)
        assertEquals(0.0, ball.velX, 1e-9);
        assertEquals(0.0, ball.velY, 1e-9);
    }

    @Test
    void partialRestitution_differentE_scalesCorrectly() {
        double e = 0.75;
        double incoming = -3.0;
        RigidBody ball = dynSphere("Ball", 1.0, 0.1, 0, 0, 0.1, 0, 0, incoming);
        RigidBody floor = staticFloor();

        ContactPoint cp = contact(ball, floor, 0, 0, 0, 0, 0, 1, 0.0, 0.0, e);
        solver.solve(List.of(cp), 0.02);

        assertEquals(-e * incoming, ball.velZ, 1e-9); // expected: +2.25
    }

    // ------------------------------------------------------------------
    // Normal impulse is non-negative
    // ------------------------------------------------------------------

    @Test
    void separatingContact_noImpulseApplied() {
        // Bodies already moving apart: no impulse should be applied
        RigidBody a = dynSphere("A", 1.0, 0.1, -0.5, 0, 0, -1.0, 0, 0); // moving left
        RigidBody b = dynSphere("B", 1.0, 0.1,  0.5, 0, 0,  1.0, 0, 0); // moving right
        double vAx0 = a.velX, vBx0 = b.velX;

        ContactPoint cp = contact(a, b, 0, 0, 0, -1, 0, 0, 0.0, 0.0, 1.0);
        solver.solve(List.of(cp), 0.02);

        // Velocities unchanged — bodies are separating, no impulse
        assertEquals(vAx0, a.velX, 1e-12);
        assertEquals(vBx0, b.velX, 1e-12);
    }

    // ------------------------------------------------------------------
    // Static body
    // ------------------------------------------------------------------

    @Test
    void staticBody_neverMoves() {
        RigidBody ball = dynSphere("Ball", 1.0, 0.1, 0, 0, 0.5, 0, 0, -2.0);
        RigidBody floor = staticFloor();

        ContactPoint cp = contact(ball, floor, 0, 0, 0, 0, 0, 1, 0.1, 0.0, 0.5);
        solver.solve(List.of(cp), 0.02);

        // Floor must not move
        assertEquals(0.0, floor.velX, 1e-12);
        assertEquals(0.0, floor.velY, 1e-12);
        assertEquals(0.0, floor.velZ, 1e-12);
    }

    // ------------------------------------------------------------------
    // Friction
    // ------------------------------------------------------------------

    @Test
    void friction_deceleratesSliding() {
        // Ball sliding along floor in X-direction with friction
        double mu = 0.5;
        RigidBody ball = dynSphere("Ball", 1.0, 0.1, 0, 0, 0.1, 2.0, 0, 0);
        RigidBody floor = staticFloor();

        // Contact pushing up (normal = Z), ball sliding in X
        // Gravity is simulated by a downward resting force → normal impulse pre-computed
        // We simulate a resting contact by giving a non-zero accumulated normal impulse
        ContactPoint cp = contact(ball, floor, 0, 0, 0, 0, 0, 1, 0.0, mu, 0.0);
        // Inject a non-zero accumulated normal impulse to activate friction clamping
        cp.accumulatedNormalImpulse = 10.0; // simulates weight pressing down

        double vx0 = ball.velX;
        solver.solve(List.of(cp), 0.02);

        // Friction should have reduced sliding velocity
        assertTrue(Math.abs(ball.velX) < Math.abs(vx0),
            "Friction must reduce sliding speed: " + ball.velX);
    }
}
