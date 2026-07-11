package jsim.simulation.motorsims;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Aggregates current draw and computes loaded battery voltage.
 */
public final class SimulatedBattery {
    private final double nominalVoltage;
    private final double internalResistanceOhms;
    private final List<Supplier<Double>> currentDrawSuppliers = new ArrayList<>();
    private double latestVoltage;

    public SimulatedBattery() {
        this(12.6, 0.020);
    }

    public SimulatedBattery(double nominalVoltage, double internalResistanceOhms) {
        this.nominalVoltage = nominalVoltage;
        this.internalResistanceOhms = internalResistanceOhms;
        this.latestVoltage = nominalVoltage;
    }

    public void registerCurrentDraw(Supplier<Double> currentSupplier) {
        currentDrawSuppliers.add(currentSupplier);
    }

    public double updateAndGetVoltage() {
        double totalCurrent = 0.0;
        for (Supplier<Double> supplier : currentDrawSuppliers) {
            totalCurrent += Math.max(0.0, supplier.get());
        }
        latestVoltage = Math.max(6.0, nominalVoltage - totalCurrent * internalResistanceOhms);
        return latestVoltage;
    }

    /** WPILib integration hook for desktop simulation. */
    public void publishToRoboRio() {
        RoboRioSim.setVInVoltage(updateAndGetVoltage());
    }

    public double getLatestVoltage() {
        return latestVoltage;
    }
}
