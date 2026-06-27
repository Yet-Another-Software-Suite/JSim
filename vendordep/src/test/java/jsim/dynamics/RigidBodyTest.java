package jsim.dynamics;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.material.Material;
import org.junit.jupiter.api.Test;

class RigidBodyTest {

    private static RigidBody create() {
        return new RigidBody("test", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
    }

    // Initial state

    @Test
    void initialQuaternion_isIdentity() {
        RigidBody b = create();
        assertEquals(1.0, b.qW, 1e-12);
        assertEquals(0.0, b.qX, 1e-12);
        assertEquals(0.0, b.qY, 1e-12);
        assertEquals(0.0, b.qZ, 1e-12);
    }

    @Test
    void initialPosition_isZero() {
        RigidBody b = create();
        assertEquals(0.0, b.posX);
        assertEquals(0.0, b.posY);
        assertEquals(0.0, b.posZ);
    }

    @Test
    void initialVelocity_isZero() {
        RigidBody b = create();
        assertEquals(0.0, b.velX);
        assertEquals(0.0, b.velY);
        assertEquals(0.0, b.velZ);
        assertEquals(0.0, b.omX);
        assertEquals(0.0, b.omY);
        assertEquals(0.0, b.omZ);
    }

    // Force / torque accumulation

    @Test
    void applyForce_accumulates() {
        RigidBody b = create();
        b.applyForce(1, 2, 3);
        b.applyForce(4, 5, 6);
        assertEquals(5.0, b.forceX, 1e-10);
        assertEquals(7.0, b.forceY, 1e-10);
        assertEquals(9.0, b.forceZ, 1e-10);
    }

    @Test
    void applyTorque_accumulates() {
        RigidBody b = create();
        b.applyTorque(1, 0, 0);
        b.applyTorque(0, 2, 3);
        assertEquals(1.0, b.torqueX, 1e-10);
        assertEquals(2.0, b.torqueY, 1e-10);
        assertEquals(3.0, b.torqueZ, 1e-10);
    }

    @Test
    void clearAccumulators_zerosForceAndTorque() {
        RigidBody b = create();
        b.applyForce(10, 20, 30);
        b.applyTorque(1, 2, 3);
        b.clearAccumulators();
        assertEquals(0.0, b.forceX);
        assertEquals(0.0, b.forceY);
        assertEquals(0.0, b.forceZ);
        assertEquals(0.0, b.torqueX);
        assertEquals(0.0, b.torqueY);
        assertEquals(0.0, b.torqueZ);
    }

    // Mass properties

    @Test
    void setMassProperties_computesCorrectInvMass() {
        RigidBody b = create();
        b.setMassProperties(4.0, 1, 1, 1);
        assertEquals(0.25, b.invMass, 1e-12);
    }

    @Test
    void setMassProperties_zeroMass_zeroInvMass() {
        RigidBody b = create();
        b.setMassProperties(0.0, 1, 1, 1);
        assertEquals(0.0, b.invMass, 1e-12);
    }

    @Test
    void setStatic_zerosInvMassAndSetsFlag() {
        RigidBody b = create();
        b.setMassProperties(5.0, 1, 1, 1);
        b.setStatic();
        assertEquals(0.0, b.invMass, 1e-12);
        assertTrue(b.isStatic());
    }

    @Test
    void setStatic_zerosInvIWorld() {
        RigidBody b = create();
        b.setMassProperties(1.0, 1, 1, 1);
        b.setStatic();
        for (double v : b.invIWorld) assertEquals(0.0, v, 1e-12);
    }

    // Quaternion

    @Test
    void normalizeQuaternion_normalizesArbitraryQuat() {
        RigidBody b = create();
        b.qW = 2; b.qX = 2; b.qY = 2; b.qZ = 2;
        b.normalizeQuaternion();
        double len = Math.sqrt(b.qW * b.qW + b.qX * b.qX + b.qY * b.qY + b.qZ * b.qZ);
        assertEquals(1.0, len, 1e-10);
    }

    @Test
    void normalizeQuaternion_zeroLength_resetsToIdentity() {
        RigidBody b = create();
        b.qW = 0; b.qX = 0; b.qY = 0; b.qZ = 0;
        b.normalizeQuaternion();
        assertEquals(1.0, b.qW, 1e-12);
        assertEquals(0.0, b.qX, 1e-12);
        assertEquals(0.0, b.qY, 1e-12);
        assertEquals(0.0, b.qZ, 1e-12);
    }

    // Force at point

    @Test
    void applyForceAtPoint_generatesLinearAndTorque() {
        // Force (0, 0, 1) at point (1, 0, 0) with CoM at origin
        // r = (1, 0, 0), f = (0, 0, 1)
        // r × f = (0*1−0*0, 0*0−1*1, 1*0−0*0) = (0, −1, 0)
        RigidBody b = create();
        b.posX = 0; b.posY = 0; b.posZ = 0;
        b.applyForceAtPoint(0, 0, 1, 1, 0, 0);
        assertEquals(0.0, b.forceX, 1e-10);
        assertEquals(0.0, b.forceY, 1e-10);
        assertEquals(1.0, b.forceZ, 1e-10);
        assertEquals(0.0, b.torqueX, 1e-10);
        assertEquals(-1.0, b.torqueY, 1e-10);
        assertEquals(0.0, b.torqueZ, 1e-10);
    }

    @Test
    void applyForceAtCoM_noTorqueGenerated() {
        RigidBody b = create();
        b.posX = 3; b.posY = 4; b.posZ = 5;
        // Apply force at CoM position itself
        b.applyForceAtPoint(10, 0, 0, 3, 4, 5);
        assertEquals(10.0, b.forceX, 1e-10);
        assertEquals(0.0, b.torqueX, 1e-10);
        assertEquals(0.0, b.torqueY, 1e-10);
        assertEquals(0.0, b.torqueZ, 1e-10);
    }

    // Pose

    @Test
    void getPose_returnsCorrectPosition() {
        RigidBody b = create();
        b.posX = 1; b.posY = 2; b.posZ = 3;
        Pose3d p = b.getPose();
        assertEquals(1.0, p.getX(), 1e-10);
        assertEquals(2.0, p.getY(), 1e-10);
        assertEquals(3.0, p.getZ(), 1e-10);
    }

    @Test
    void setPose_updatesPositionAndOrientation() {
        RigidBody b = create();
        b.setMassProperties(1.0, 1, 1, 1);
        Pose3d target = new Pose3d(new Translation3d(4, 5, 6), new Rotation3d(0.1, 0.2, 0.3));
        b.setPose(target);
        assertEquals(4.0, b.posX, 1e-9);
        assertEquals(5.0, b.posY, 1e-9);
        assertEquals(6.0, b.posZ, 1e-9);
        // Quaternion must remain unit length after setPose
        double len = Math.sqrt(b.qW * b.qW + b.qX * b.qX + b.qY * b.qY + b.qZ * b.qZ);
        assertEquals(1.0, len, 1e-9);
    }

    // Velocity

    @Test
    void setLinearVelocity_updatesVelFields() {
        RigidBody b = create();
        b.setLinearVelocity(new Translation3d(1, 2, 3));
        assertEquals(1.0, b.velX, 1e-10);
        assertEquals(2.0, b.velY, 1e-10);
        assertEquals(3.0, b.velZ, 1e-10);
    }

    @Test
    void setAngularVelocity_updatesOmFields() {
        RigidBody b = create();
        b.setAngularVelocity(new Translation3d(0.5, 1.0, 2.0));
        assertEquals(0.5, b.omX, 1e-10);
        assertEquals(1.0, b.omY, 1e-10);
        assertEquals(2.0, b.omZ, 1e-10);
    }

    // invIWorld

    @Test
    void refreshWorldInertia_populatesDiagonalAtIdentity() {
        RigidBody b = create();
        b.setMassProperties(1.0, 2.0, 3.0, 4.0);
        b.refreshWorldInertia();
        // At identity rotation, off-diagonals should be ~0
        assertEquals(0.5, b.invIWorld[0], 1e-9);    // 1/2.0
        assertEquals(1.0 / 3.0, b.invIWorld[4], 1e-9);
        assertEquals(0.25, b.invIWorld[8], 1e-9);   // 1/4.0
    }

    @Test
    void refreshWorldInertia_zerosForStaticBody() {
        RigidBody b = create();
        b.setMassProperties(1.0, 1, 1, 1);
        b.setStatic();
        b.refreshWorldInertia();
        for (double v : b.invIWorld) assertEquals(0.0, v, 1e-12);
    }

    @Test
    void refreshWorldInertia_zerosForFixedRotation() {
        RigidBody b = new RigidBody("b", RigidBodyFlags.FIXED_ROTATION, Material.DEFAULT);
        b.setMassProperties(1.0, 1, 1, 1);
        b.refreshWorldInertia();
        for (double v : b.invIWorld) assertEquals(0.0, v, 1e-12);
    }

    // ID

    @Test
    void initialId_isMinusOne() {
        RigidBody b = create();
        assertEquals(-1, b.getId());
    }
}
