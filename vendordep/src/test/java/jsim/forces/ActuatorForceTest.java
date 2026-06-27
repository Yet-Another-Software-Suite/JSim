package jsim.forces;

import static org.junit.jupiter.api.Assertions.*;

import jsim.dynamics.RigidBody;
import jsim.dynamics.RigidBodyFlags;
import jsim.material.Material;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ActuatorForceTest {

    private RigidBody body;
    private ActuatorForce actuator;

    @BeforeEach
    void setUp() {
        body = new RigidBody("b", RigidBodyFlags.DYNAMIC, Material.DEFAULT);
        body.setMassProperties(1.0, 1, 1, 1);
        body.posX = 0; body.posY = 0; body.posZ = 0;
        actuator = new ActuatorForce(body);
    }

    @Test
    void setForce_appliesForceAtCoM() {
        actuator.setForce(10, 5, -3);
        actuator.apply(0.02);
        assertEquals(10.0, body.forceX, 1e-10);
        assertEquals(5.0, body.forceY, 1e-10);
        assertEquals(-3.0, body.forceZ, 1e-10);
        // No torque from CoM force
        assertEquals(0.0, body.torqueX, 1e-12);
        assertEquals(0.0, body.torqueY, 1e-12);
        assertEquals(0.0, body.torqueZ, 1e-12);
    }

    @Test
    void setTorque_appliesTorque() {
        actuator.setTorque(0, 0, 7);
        actuator.apply(0.02);
        assertEquals(0.0, body.torqueX, 1e-10);
        assertEquals(0.0, body.torqueY, 1e-10);
        assertEquals(7.0, body.torqueZ, 1e-10);
    }

    @Test
    void zero_clearsAllForcesAndTorques() {
        actuator.setForce(10, 10, 10);
        actuator.setTorque(5, 5, 5);
        actuator.zero();
        actuator.apply(0.02);
        assertEquals(0.0, body.forceX, 1e-12);
        assertEquals(0.0, body.forceY, 1e-12);
        assertEquals(0.0, body.forceZ, 1e-12);
        assertEquals(0.0, body.torqueX, 1e-12);
        assertEquals(0.0, body.torqueY, 1e-12);
        assertEquals(0.0, body.torqueZ, 1e-12);
    }

    @Test
    void skipsStaticBody() {
        RigidBody stat = new RigidBody("b", RigidBodyFlags.STATIC, Material.DEFAULT);
        stat.setStatic();
        ActuatorForce act = new ActuatorForce(stat);
        act.setForce(100, 100, 100);
        act.apply(0.02);
        assertEquals(0.0, stat.forceX, 1e-12);
        assertEquals(0.0, stat.forceY, 1e-12);
        assertEquals(0.0, stat.forceZ, 1e-12);
    }

    @Test
    void setForceAtPoint_generatesLinearAndTorque() {
        // Force (0, 0, 1) at point (1, 0, 0); CoM at origin
        // r × f = (1,0,0) × (0,0,1) = (0*1−0*0, 0*0−1*1, 1*0−0*0) = (0, −1, 0)
        actuator.setForceAtPoint(0, 0, 1.0, 1.0, 0, 0);
        actuator.apply(0.02);
        assertEquals(0.0, body.forceX, 1e-10);
        assertEquals(0.0, body.forceY, 1e-10);
        assertEquals(1.0, body.forceZ, 1e-10);
        assertEquals(0.0, body.torqueX, 1e-10);
        assertEquals(-1.0, body.torqueY, 1e-10);
        assertEquals(0.0, body.torqueZ, 1e-10);
    }

    @Test
    void setForce_afterSetForceAtPoint_clearsApplicationPoint() {
        // setForce() should revert to CoM application (no torque from offset)
        actuator.setForceAtPoint(0, 0, 1.0, 1.0, 0, 0);
        actuator.setForce(5, 0, 0);
        actuator.apply(0.02);
        assertEquals(5.0, body.forceX, 1e-10);
        assertEquals(0.0, body.torqueX, 1e-12);
        assertEquals(0.0, body.torqueY, 1e-12);
        assertEquals(0.0, body.torqueZ, 1e-12);
    }

    @Test
    void defaultState_noForceApplied() {
        actuator.apply(0.02);
        assertEquals(0.0, body.forceX, 1e-12);
        assertEquals(0.0, body.forceY, 1e-12);
        assertEquals(0.0, body.forceZ, 1e-12);
        assertEquals(0.0, body.torqueX, 1e-12);
    }

    @Test
    void forcePersistsAcrossTicks() {
        actuator.setForce(1, 0, 0);
        actuator.apply(0.02);
        body.clearAccumulators();
        actuator.apply(0.02); // second tick without calling setForce again
        assertEquals(1.0, body.forceX, 1e-10);
    }
}
