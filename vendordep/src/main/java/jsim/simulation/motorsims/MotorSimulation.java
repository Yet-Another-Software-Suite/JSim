package jsim.simulation.motorsims;

/**
 * Minimal DC motor model with inertia and current draw.
 */
public final class MotorSimulation {
    private final SimMotorConfigs config;
    private final SimMotorState state = new SimMotorState();

    public MotorSimulation(SimMotorConfigs config) {
        this.config = config;
    }

    public void setCommandedVoltage(double volts) {
        state.setCommandedVoltage(volts);
    }

    /**
     * Advance motor state by a fixed timestep.
     *
     * @param dtSeconds simulation timestep in seconds
     * @param loadTorqueNm external resisting torque (N*m)
     */
    public void step(double dtSeconds, double loadTorqueNm) {
        double omega = state.getShaftVelocityRadPerSecond();
        double backEmf = omega / config.kvRadPerSecondPerVolt();
        double current = (state.getCommandedVoltage() - backEmf) / config.resistanceOhms();
        double motorTorque = current * config.ktNewtonMeterPerAmp();
        double acceleration = (motorTorque - loadTorqueNm) / config.rotorInertiaKgM2();
        double newOmega = omega + acceleration * dtSeconds;
        state.setShaftVelocityRadPerSecond(newOmega);
        state.setShaftPositionRad(state.getShaftPositionRad() + newOmega * dtSeconds);
        state.setSupplyCurrentAmps(Math.max(0.0, Math.abs(current)));
    }

    public SimMotorState getState() {
        return state;
    }
}
