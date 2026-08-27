package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import jsim.physics.layers.PhysicsLayer;
import jsim.physics.layers.gamepieces.Fuel2026;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.World;

import java.util.ArrayList;
import java.util.List;

/**
 * Physics layer that spawns and manages 360 Fuel sphere game pieces in the dyn4j simulation world.
 */
public class FuelLayer implements PhysicsLayer {

  public static final Mass FUEL_MASS = Grams.of(0.216);
  public static final Distance FUEL_DIAMETER = Meters.of(0.15);

  private final List<Fuel2026> fuelPieces = new ArrayList<>();
  private final World<Body> world;
  private final Translation2d spawnCorner;
  private final int rows;
  private final int cols;

  private boolean initialized = false;

  /**
   * Constructs a FuelLayer with a default 20x18 grid (360 pieces).
   *
   * @param world Target dyn4j physics world instance shared with robot collisions.
   * @param spawnCorner X, Y coordinates for the bottom-left corner of the spawn grid.
   */
  public FuelLayer(World<Body> world, Translation2d spawnCorner) {
    this(world, spawnCorner, 20, 18);
  }

  /**
   * Constructs a FuelLayer with a custom grid dimension.
   *
   * @param world Target dyn4j physics world.
   * @param spawnCorner Bottom-left corner of the spawn rectangular area.
   * @param rows Number of rows in the grid.
   * @param cols Number of columns in the grid.
   */
  public FuelLayer(World<Body> world, Translation2d spawnCorner, int rows, int cols) {
    this.world = world;
    this.spawnCorner = spawnCorner;
    this.rows = rows;
    this.cols = cols;
  }

  @Override
  public ChassisSpeeds process(Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions) {
    if (!initialized) {
      spawnFuelGrid();
      initialized = true;
    }
    return inputSpeeds;
  }

  /**
   * Spawns a rectangular grid of 360 Fuel spheres into the dyn4j world.
   */
  private void spawnFuelGrid() {
    double radiusMeters = FUEL_DIAMETER.in(Meters) / 2.0;
    double massKg = FUEL_MASS.in(edu.wpi.first.units.Units.Kilograms);
    double spacing = FUEL_DIAMETER.in(Meters) + 0.01; // 1cm gap to prevent overlapping on spawn

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        double x = spawnCorner.getX() + (c * spacing);
        double y = spawnCorner.getY() + (r * spacing);

        Body fuelBody = new Body();
        BodyFixture fixture = fuelBody.addFixture(Geometry.createCircle(radiusMeters));
        fixture.setFriction(0.4);
        fixture.setRestitution(0.3); // Carpet bounce resilience

        // Compute circular density based on 2D area (Mass / Area)
        double area = Math.PI * radiusMeters * radiusMeters;
        fixture.setDensity(massKg / area);

        fuelBody.setMass(MassType.NORMAL);
        fuelBody.getTransform().setTranslation(x, y);

        world.addBody(fuelBody);
        fuelPieces.add(new Fuel2026(fuelBody));
      }
    }
  }

  /**
   * Returns all spawned fuel piece instances for intake checks or UI visualization.
   */
  public List<Fuel2026> getFuelPieces() {
    return fuelPieces;
  }
}