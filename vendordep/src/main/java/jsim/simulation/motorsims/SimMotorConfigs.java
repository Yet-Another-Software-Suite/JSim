package jsim.simulation.motorsims;

/** Immutable parameters for a single DC motor simulation. */
public record SimMotorConfigs(
        double kvRadPerSecondPerVolt,
        double ktNewtonMeterPerAmp,
        double resistanceOhms,
        double rotorInertiaKgM2) {

    /** Default profile tuned for simple mechanism simulation. */
    public static SimMotorConfigs defaultConfig() {
        return new SimMotorConfigs(50.0, 0.02, 0.09, 0.0002);
    }
}
