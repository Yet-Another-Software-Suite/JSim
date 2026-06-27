package jsim.api;

/**
 * Identifies a robot by its FRC alliance and driver-station number (1–3).
 *
 * <p>Use this with {@link SimBodyBuilder#robotId(RobotId)} when creating a robot body,
 * and with {@link SimWorld#findRobot(RobotId)} to retrieve it by station later.
 *
 * <p>Example:
 * <pre>{@code
 * SimBody robot = world.addBody(new SimBodyBuilder("Red1Robot")
 *     .robotId(RobotId.RED_1)
 *     .mass(54).boxCollider(0.45, 0.4, 0.15)
 *     .material(Material.CARPET));
 *
 * Elsewhere in simulationPeriodic():
 * SimBody red1 = world.findRobot(RobotId.RED_1);
 * }</pre>
 */
public enum RobotId {
    /** Red alliance, driver station 1. */
    RED_1(Alliance.RED, 1),
    /** Red alliance, driver station 2. */
    RED_2(Alliance.RED, 2),
    /** Red alliance, driver station 3. */
    RED_3(Alliance.RED, 3),
    /** Blue alliance, driver station 1. */
    BLUE_1(Alliance.BLUE, 1),
    /** Blue alliance, driver station 2. */
    BLUE_2(Alliance.BLUE, 2),
    /** Blue alliance, driver station 3. */
    BLUE_3(Alliance.BLUE, 3);

    /** FRC alliance color. */
    public enum Alliance {
        /** Red alliance. */
        RED,
        /** Blue alliance. */
        BLUE
    }

    /** Which alliance this robot belongs to. */
    public final Alliance alliance;

    /** Driver-station number (1, 2, or 3). */
    public final int station;

    RobotId(Alliance alliance, int station) {
        this.alliance = alliance;
        this.station = station;
    }

    /**
     * Look up a {@code RobotId} from an alliance and station number.
     *
     * @param alliance RED or BLUE
     * @param station  1, 2, or 3
     * @return the matching {@code RobotId}
     * @throws IllegalArgumentException if station is not 1–3
     */
    public static RobotId of(Alliance alliance, int station) {
        for (RobotId id : values()) {
            if (id.alliance == alliance && id.station == station) return id;
        }
        throw new IllegalArgumentException("Invalid station: " + station + " (must be 1–3)");
    }

    @Override
    public String toString() {
        return alliance.name().charAt(0) + alliance.name().substring(1).toLowerCase()
            + station;
    }
}
