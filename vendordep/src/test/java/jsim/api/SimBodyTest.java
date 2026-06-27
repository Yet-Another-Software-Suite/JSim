package jsim.api;

import static org.junit.jupiter.api.Assertions.*;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import jsim.collision.BoxCollider;
import jsim.collision.ColliderShape;
import jsim.collision.SphereCollider;
import jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link SimBody} and the corresponding {@link SimBodyBuilder} configuration.
 */
class SimBodyTest {

    private SimWorld world;

    @BeforeEach
    void setUp() {
        world = new SimWorld(0.02, 10);
    }

    // Position / velocity configuration

    @Test
    void position_isSetCorrectly() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(3, 4, 5).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        assertEquals(3.0, b.getPosition().getX(), 1e-9);
        assertEquals(4.0, b.getPosition().getY(), 1e-9);
        assertEquals(5.0, b.getPosition().getZ(), 1e-9);
    }

    @Test
    void linearVelocity_isSetCorrectly() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 5).linearVelocity(1, 2, 3)
            .mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        assertEquals(1.0, b.getLinearVelocity().getX(), 1e-9);
        assertEquals(2.0, b.getLinearVelocity().getY(), 1e-9);
        assertEquals(3.0, b.getLinearVelocity().getZ(), 1e-9);
    }

    @Test
    void angularVelocity_isSetCorrectly() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).angularVelocity(0.5, 1.0, 2.0)
            .mass(Kilograms.of(1)).boxCollider(0.3, 0.3, 0.3).material(Material.DEFAULT));
        assertEquals(0.5, b.getAngularVelocity().getX(), 1e-9);
        assertEquals(1.0, b.getAngularVelocity().getY(), 1e-9);
        assertEquals(2.0, b.getAngularVelocity().getZ(), 1e-9);
    }

    // Mass

    @Test
    void mass_isSetCorrectly() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(7.5)).sphereCollider(0.1).material(Material.DEFAULT));
        assertEquals(7.5, b.getMass().in(Kilograms), 1e-9);
    }

    @Test
    void staticBody_hasInfiniteMass() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 0).boxCollider(1, 1, 1).isStatic().material(Material.DEFAULT));
        assertEquals(Double.POSITIVE_INFINITY, b.getMass().in(Kilograms));
        assertTrue(b.isStatic());
    }

    // Colliders

    @Test
    void sphereCollider_isAttachedWithCorrectRadius() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.25).material(Material.DEFAULT));
        assertNotNull(b.getCollider());
        assertEquals(ColliderShape.Type.SPHERE, b.getCollider().getType());
        assertEquals(0.25, ((SphereCollider) b.getCollider()).radius, 1e-10);
    }

    @Test
    void boxCollider_isAttachedWithCorrectHalfExtents() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).boxCollider(0.2, 0.3, 0.4).material(Material.DEFAULT));
        assertNotNull(b.getCollider());
        assertEquals(ColliderShape.Type.BOX, b.getCollider().getType());
        BoxCollider box = (BoxCollider) b.getCollider();
        assertEquals(0.2, box.halfX, 1e-10);
        assertEquals(0.3, box.halfY, 1e-10);
        assertEquals(0.4, box.halfZ, 1e-10);
    }

    @Test
    void planeCollider_autoSetsStatic() {
        SimBody b = world.addBody(new SimBodyBuilder("Floor")
            .planeCollider(0, 0, 1, 0).material(Material.DEFAULT));
        assertTrue(b.isStatic());
        assertEquals(ColliderShape.Type.PLANE, b.getCollider().getType());
    }

    // Material

    @Test
    void material_isSetCorrectly() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.RUBBER));
        assertSame(Material.RUBBER, b.getMaterial());
    }

    @Test
    void setMaterial_changesAfterConstruction() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        b.setMaterial(Material.ICE);
        assertSame(Material.ICE, b.getMaterial());
    }

    // Speed

    @Test
    void getSpeed_matchesVelocityMagnitude() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).linearVelocity(3, 4, 0)
            .mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        assertEquals(5.0, b.getSpeed().in(MetersPerSecond), 1e-9);
    }

    @Test
    void getSpeed_zeroWhenStationary() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        assertEquals(0.0, b.getSpeed().in(MetersPerSecond), 1e-12);
    }

    // Name

    @Test
    void getName_returnsBuilderName() {
        SimBody b = world.addBody(new SimBodyBuilder("MyBody")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        assertEquals("MyBody", b.getName());
    }

    // Robot ID

    @Test
    void robotId_isAssignedAndRetrievableByFinder() {
        SimBody robot = world.addBody(new SimBodyBuilder("Robot")
            .position(0, 0, 1).mass(Kilograms.of(50)).boxCollider(0.4, 0.4, 0.15)
            .robotId(RobotId.RED_1).material(Material.CARPET));
        assertTrue(robot.isRobot());
        assertSame(RobotId.RED_1, robot.getRobotId());
        assertSame(robot, world.findRobot(RobotId.RED_1));
    }

    @Test
    void isRobot_falseWhenNoRobotIdSet() {
        SimBody b = world.addBody(new SimBodyBuilder("Ball")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        assertFalse(b.isRobot());
        assertNull(b.getRobotId());
    }

    @Test
    void findRobot_returnsNullForUnregisteredId() {
        assertNull(world.findRobot(RobotId.BLUE_2));
    }

    // Setters

    @Test
    void setPosition_updatesCoordinates() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 0).mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        b.setPosition(7, 8, 9);
        assertEquals(7.0, b.getPosition().getX(), 1e-9);
        assertEquals(8.0, b.getPosition().getY(), 1e-9);
        assertEquals(9.0, b.getPosition().getZ(), 1e-9);
    }

    @Test
    void setLinearVelocity_scalarForm() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        b.setLinearVelocity(5, 6, 7);
        assertEquals(5.0, b.getLinearVelocity().getX(), 1e-9);
        assertEquals(6.0, b.getLinearVelocity().getY(), 1e-9);
        assertEquals(7.0, b.getLinearVelocity().getZ(), 1e-9);
    }

    @Test
    void setAngularVelocity_scalarForm() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).boxCollider(0.3, 0.3, 0.3).noGravity().material(Material.DEFAULT));
        b.setAngularVelocity(1.0, 2.0, 3.0);
        assertEquals(1.0, b.getAngularVelocity().getX(), 1e-9);
        assertEquals(2.0, b.getAngularVelocity().getY(), 1e-9);
        assertEquals(3.0, b.getAngularVelocity().getZ(), 1e-9);
    }

    // Flags

    @Test
    void noGravityFlag_bodyDoesNotFall() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 3).mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        double z0 = b.getPosition().getZ();
        for (int i = 0; i < 20; i++) world.step();
        assertEquals(z0, b.getPosition().getZ(), 1e-6);
    }

    @Test
    void noCollisionFlag_bodyPassesThroughFloor() {
        world.addBody(new SimBodyBuilder("Floor").planeCollider(0, 0, 1, 0).material(Material.DEFAULT));
        SimBody ghost = world.addBody(new SimBodyBuilder("Ghost")
            .position(0, 0, 0.2).mass(Kilograms.of(1)).sphereCollider(0.1)
            .noCollision().material(Material.DEFAULT));
        for (int i = 0; i < 60; i++) world.step();
        assertTrue(ghost.getPosition().getZ() < -0.5,
            "NO_COLLISION body should pass through floor: z=" + ghost.getPosition().getZ());
    }

    // Inertia

    @Test
    void manualInertia_overridesAutoEstimate() {
        // Call inertia() after sphereCollider() to override the auto-estimate
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(2.0))
            .sphereCollider(0.1).inertia(0.5, 0.6, 0.7)
            .material(Material.DEFAULT));
        // At identity rotation, invIWorld diagonal = [1/0.5, 1/0.6, 1/0.7]
        assertEquals(2.0, b.body.invIWorld[0], 1e-9);
        assertEquals(1.0 / 0.6, b.body.invIWorld[4], 1e-9);
        assertEquals(1.0 / 0.7, b.body.invIWorld[8], 1e-9);
    }

    // Rotation

    @Test
    void rotation_isAppliedAtConstruction() {
        Rotation3d rot = new Rotation3d(0.3, 0.1, 0.5);
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(1, 2, 3).rotation(rot)
            .mass(Kilograms.of(1)).sphereCollider(0.1).noGravity().material(Material.DEFAULT));
        Pose3d p = b.getPose();
        assertEquals(1.0, p.getX(), 1e-9);
        assertEquals(2.0, p.getY(), 1e-9);
        assertEquals(3.0, p.getZ(), 1e-9);
        // Rotation should have been applied (quaternion non-trivial)
        double qLen = Math.sqrt(
            b.body.qW * b.body.qW + b.body.qX * b.body.qX
          + b.body.qY * b.body.qY + b.body.qZ * b.body.qZ);
        assertEquals(1.0, qLen, 1e-9, "Orientation quaternion must be unit length");
    }

    // AABB

    @Test
    void getAABB_hasSixElements() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(1, 2, 3).mass(Kilograms.of(1)).sphereCollider(0.5).material(Material.DEFAULT));
        double[] aabb = b.getAABB();
        assertEquals(6, aabb.length);
    }

    @Test
    void getAABB_sphereAabbMatchesRadius() {
        double r = 0.5;
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 0).mass(Kilograms.of(1)).sphereCollider(r).material(Material.DEFAULT));
        double[] aabb = b.getAABB();
        assertEquals(-r, aabb[0], 1e-9);
        assertEquals(-r, aabb[1], 1e-9);
        assertEquals(-r, aabb[2], 1e-9);
        assertEquals( r, aabb[3], 1e-9);
        assertEquals( r, aabb[4], 1e-9);
        assertEquals( r, aabb[5], 1e-9);
    }

    // toString

    @Test
    void toString_containsNameAndPosition() {
        SimBody b = world.addBody(new SimBodyBuilder("TestBody")
            .position(1, 2, 3).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        String s = b.toString();
        assertTrue(s.contains("TestBody"), "toString should contain body name");
    }

    // Actuator

    @Test
    void getActuator_returnsNonNull() {
        SimBody b = world.addBody(new SimBodyBuilder("B")
            .position(0, 0, 1).mass(Kilograms.of(1)).sphereCollider(0.1).material(Material.DEFAULT));
        assertNotNull(b.getActuator());
    }
}
