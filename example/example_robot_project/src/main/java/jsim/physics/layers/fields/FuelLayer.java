package jsim.physics.layers.fields;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
  private final StructArrayPublisher<Pose3d> posePublisher;

  private boolean initialized = false;

  /**
   * Constructs a FuelLayer with a default 20x18 grid (360 pieces).
   *
   * @param world Target dyn4j physics world instance shared with robot collisions.
   * @param spawnCorner X, Y coordinates for the bottom-left corner of the spawn grid.
   */
  public FuelLayer(World<Body> world, Translation2d spawnCorner) {
    this(world, spawnCorner, "fuel_test", 20, 18);
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
    this(world, spawnCorner, "fuel_test", rows, cols);
  }

  /**
   * Constructs a FuelLayer with an explicit NetworkTables subtable name used for publishing live
   * fuel poses.
   */
  public FuelLayer(World<Body> world, Translation2d spawnCorner, String ntSubTableName, int rows, int cols) {
    this.world = world;
    this.spawnCorner = spawnCorner;
    this.rows = rows;
    this.cols = cols;
    this.posePublisher = NetworkTableInstance.getDefault()
        .getTable("Mechanisms")
        .getSubTable(ntSubTableName)
        .getStructArrayTopic("poses", Pose3d.struct)
        .publish();
  }

  @Override
  public ChassisSpeeds process(Pose2d currentPose, ChassisSpeeds inputSpeeds, Translation2d robotDimensions, double dtSeconds) {
    if (!initialized) {
      spawnFuelGrid();
      initialized = true;
    }
    updatePublishedPose3dList();
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

  /**
   * Creates a live list of the current fuel body poses as 3D field-relative poses.
   */
  public List<Pose3d> getPose3dList() {
    List<Pose3d> poses = new ArrayList<>(fuelPieces.size());
    for (Fuel2026 fuelPiece : fuelPieces) {
      var body = fuelPiece.getBody();
      poses.add(new Pose3d(
          body.getTransform().getTranslationX(),
          body.getTransform().getTranslationY(),
          0.0,
          new Rotation3d()));
    }
    return poses;
  }

  /**
   * Converts the current fuel bodies into a Pose3d array and publishes it to NetworkTables.
   */
  public void updatePublishedPose3dList() {
    Pose3d[] poses = new Pose3d[fuelPieces.size()];
    for (int i = 0; i < fuelPieces.size(); i++) {
      var body = fuelPieces.get(i).getBody();
      poses[i] = new Pose3d(
          body.getTransform().getTranslationX(),
          body.getTransform().getTranslationY(),
          0.0,
          new Rotation3d());
    }
    posePublisher.set(poses);
  }

  /**
   * Returns the current fuel body poses as a Pose3d array for external consumers.
   */
  public Pose3d[] getPose3dArray() {
    return getPose3dList().toArray(new Pose3d[0]);
  }

  /**
   * Explicitly publishes the current fuel pose list to NetworkTables.
   */
  public void publishPose3dList() {
    updatePublishedPose3dList();
  }
}