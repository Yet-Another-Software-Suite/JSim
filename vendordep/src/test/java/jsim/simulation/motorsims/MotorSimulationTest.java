package jsim.simulation.motorsims;

import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

class MotorSimulationTest {
    @Test
    void motorAcceleratesFromVoltageCommand() {
        MotorSimulation motor = new MotorSimulation(SimMotorConfigs.defaultConfig());
        motor.setCommandedVoltage(12.0);
        for (int i = 0; i < 50; i++) {
            motor.step(0.02, 0.0);
        }
        assertTrue(motor.getState().getShaftVelocityRadPerSecond() > 1.0);
        assertTrue(motor.getState().getSupplyCurrentAmps() >= 0.0);
    }

    @Test
    void batteryDropsWithLoad() {
        SimulatedBattery battery = new SimulatedBattery();
        battery.registerCurrentDraw(() -> 80.0);
        double loadedVoltage = battery.updateAndGetVoltage();
        assertTrue(loadedVoltage < 12.6);
        assertTrue(loadedVoltage >= 6.0);
    }
}
