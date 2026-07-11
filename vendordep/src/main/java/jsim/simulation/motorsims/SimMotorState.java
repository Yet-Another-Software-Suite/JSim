package jsim.simulation.motorsims;

/** Mutable state for a simulated motor/controller pair. */
public final class SimMotorState {
    private double commandedVoltage;
    private double shaftPositionRad;
    private double shaftVelocityRadPerSecond;
    private double supplyCurrentAmps;

    public double getCommandedVoltage() {
        return commandedVoltage;
    }

    public void setCommandedVoltage(double commandedVoltage) {
        this.commandedVoltage = commandedVoltage;
    }

    public double getShaftPositionRad() {
        return shaftPositionRad;
    }

    public void setShaftPositionRad(double shaftPositionRad) {
        this.shaftPositionRad = shaftPositionRad;
    }

    public double getShaftVelocityRadPerSecond() {
        return shaftVelocityRadPerSecond;
    }

    public void setShaftVelocityRadPerSecond(double shaftVelocityRadPerSecond) {
        this.shaftVelocityRadPerSecond = shaftVelocityRadPerSecond;
    }

    public double getSupplyCurrentAmps() {
        return supplyCurrentAmps;
    }

    public void setSupplyCurrentAmps(double supplyCurrentAmps) {
        this.supplyCurrentAmps = supplyCurrentAmps;
    }
}
