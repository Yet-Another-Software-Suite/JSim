package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.physics.layers.utils.Contact;
import jsim.physics.layers.utils.Cuboid3d;
import jsim.physics.layers.utils.Sphere3d;
import org.junit.jupiter.api.Test;

class Cuboid3dTest {

  @Test
  void containsIsInclusiveOfTheSurface() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 2.0, 4.0, 6.0);

    assertTrue(box.contains(new Translation3d(0, 0, 0)));
    assertTrue(box.contains(new Translation3d(1.0, 2.0, 3.0)), "A corner should count as inside");
    assertFalse(box.contains(new Translation3d(1.01, 0, 0)));
  }

  @Test
  void nearestClampsToTheClosestSurfacePoint() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 2.0, 2.0, 2.0);

    assertEquals(new Translation3d(1, 0, 0), box.nearest(new Translation3d(5, 0, 0)));
    assertEquals(new Translation3d(0.5, 0, 0), box.nearest(new Translation3d(0.5, 0, 0)),
        "A point already inside the box should be its own nearest point");
  }

  @Test
  void getDistanceIsZeroInsideAndPositiveOutside() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 2.0, 2.0, 2.0);

    assertEquals(0.0, box.getDistance(new Translation3d(0.5, 0, 0)), 1e-9);
    assertEquals(4.0, box.getDistance(new Translation3d(5, 0, 0)), 1e-9);
  }

  @Test
  void rotationIsRespectedByContainsAndNearest() {
    // A 2x1x1 box turned 90 degrees about Z looks like a 1x2x1 box in the field frame.
    Cuboid3d box = new Cuboid3d(
        new Pose3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2.0)), 2.0, 1.0, 1.0);

    assertTrue(box.contains(new Translation3d(0.4, 0.9, 0)));
    assertFalse(box.contains(new Translation3d(0.9, 0.4, 0)));
  }

  @Test
  void overlapWithSphereReturnsNullWhenFarAway() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 1.0, 1.0, 1.0);
    assertNull(box.overlapWithSphere(new Sphere3d(new Translation3d(5, 0, 0), 0.1)));
  }

  @Test
  void overlapWithSphereReportsOutwardNormalAndDepth() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 2.0, 2.0, 2.0);
    // Sphere of radius 0.5 centered 1.2m out along +X: penetrates the +X face by 0.3m.
    Contact contact = box.overlapWithSphere(new Sphere3d(new Translation3d(1.2, 0, 0), 0.5));

    assertEquals(new Translation3d(1, 0, 0), contact.normal());
    assertEquals(0.3, contact.depth(), 1e-9);
    assertEquals(new Translation3d(0.3, 0, 0), contact.pushOut());
  }

  @Test
  void overlapWithSphereEscapesThroughTheNearestFaceWhenFullyInside() {
    // A flat, wide box: a sphere centered inside it is much closer to a Z face than any side.
    Cuboid3d box = new Cuboid3d(new Pose3d(), 10.0, 10.0, 0.5);
    Contact contact = box.overlapWithSphere(new Sphere3d(new Translation3d(0, 0, 0), 0.1));

    assertEquals(0.0, contact.normal().getX(), 1e-9);
    assertEquals(0.0, contact.normal().getY(), 1e-9);
    assertEquals(1.0, Math.abs(contact.normal().getZ()), 1e-9,
        "Center-inside escape should leave through the nearest (Z) face, not a side");
    assertEquals(0.35, contact.depth(), 1e-9);
  }

  @Test
  void overlapWithSphereNormalRespectsRotation() {
    Cuboid3d box = new Cuboid3d(
        new Pose3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2.0)), 2.0, 1.0, 1.0);
    // In the box's own (rotated) frame this sphere sits off the local +X face; in field terms
    // that face now points along +Y.
    Contact contact = box.overlapWithSphere(new Sphere3d(new Translation3d(0, 1.2, 0), 0.5));

    assertEquals(0.0, contact.normal().getX(), 1e-9);
    assertEquals(1.0, contact.normal().getY(), 1e-9);
    assertEquals(0.3, contact.depth(), 1e-9);
  }

  @Test
  void overlapWithSphereMarginReportsNegativeDepthForANearMiss() {
    Cuboid3d box = new Cuboid3d(new Pose3d(), 2.0, 2.0, 2.0);
    // Sphere surface is 0.1m short of the box; within a 0.2m margin, but not touching yet.
    Contact contact = box.overlapWithSphere(new Sphere3d(new Translation3d(1.6, 0, 0), 0.5), 0.2);

    assertEquals(new Translation3d(1, 0, 0), contact.normal());
    assertEquals(-0.1, contact.depth(), 1e-9);
  }

  @Test
  void twoCornerConstructorBuildsAnAxisAlignedBoxRegardlessOfCornerOrder() {
    Cuboid3d box = new Cuboid3d(new Translation3d(1, 2, 3), new Translation3d(-1, 0, 1));

    assertEquals(new Translation3d(0, 1, 2), box.getCenter().getTranslation());
    assertEquals(2.0, box.getXWidth(), 1e-9);
    assertEquals(2.0, box.getYWidth(), 1e-9);
    assertEquals(2.0, box.getZWidth(), 1e-9);
    assertEquals(Rotation3d.kZero, box.getRotation());
  }
}
