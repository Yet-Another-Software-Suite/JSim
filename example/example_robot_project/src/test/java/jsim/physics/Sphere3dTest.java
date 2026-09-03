package jsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import jsim.physics.layers.utils.Contact;
import jsim.physics.layers.utils.Sphere3d;
import org.junit.jupiter.api.Test;

class Sphere3dTest {

  @Test
  void containsIsInclusiveOfTheSurface() {
    Sphere3d sphere = new Sphere3d(new Translation3d(), 1.0);

    assertTrue(sphere.contains(new Translation3d(0, 0, 0)));
    assertTrue(sphere.contains(new Translation3d(1, 0, 0)), "A surface point should count as inside");
    assertFalse(sphere.contains(new Translation3d(1.01, 0, 0)));
  }

  @Test
  void nearestClampsToTheSurfaceAlongTheRayFromCenter() {
    Sphere3d sphere = new Sphere3d(new Translation3d(), 1.0);

    assertEquals(new Translation3d(1, 0, 0), sphere.nearest(new Translation3d(5, 0, 0)));
    assertEquals(new Translation3d(0.5, 0, 0), sphere.nearest(new Translation3d(0.5, 0, 0)),
        "A point already inside the sphere should be its own nearest point");
  }

  @Test
  void getDistanceIsZeroInsideAndPositiveOutside() {
    Sphere3d sphere = new Sphere3d(new Translation3d(), 1.0);

    assertEquals(0.0, sphere.getDistance(new Translation3d(0.5, 0, 0)), 1e-9);
    assertEquals(4.0, sphere.getDistance(new Translation3d(5, 0, 0)), 1e-9);
  }

  @Test
  void withCenterKeepsTheRadius() {
    Sphere3d sphere = new Sphere3d(new Translation3d(1, 2, 3), 0.5);
    Sphere3d moved = sphere.withCenter(new Translation3d(9, 9, 9));

    assertEquals(new Translation3d(9, 9, 9), moved.getCenter());
    assertEquals(0.5, moved.getRadius(), 1e-9);
  }

  @Test
  void overlapWithSphereReturnsNullWhenFarAway() {
    Sphere3d a = new Sphere3d(new Translation3d(0, 0, 0), 0.1);
    Sphere3d b = new Sphere3d(new Translation3d(5, 0, 0), 0.1);

    assertNull(a.overlapWithSphere(b));
  }

  @Test
  void overlapWithSphereReportsNormalTowardsOtherAndDepth() {
    // Two 0.5-radius spheres 0.8m apart along +X: combined radius 1.0, so 0.2m of overlap.
    Sphere3d a = new Sphere3d(new Translation3d(0, 0, 0), 0.5);
    Sphere3d b = new Sphere3d(new Translation3d(0.8, 0, 0), 0.5);

    Contact contact = a.overlapWithSphere(b);

    assertEquals(new Translation3d(1, 0, 0), contact.normal(),
        "Normal should point from a's center towards b's");
    assertEquals(0.2, contact.depth(), 1e-9);
  }

  @Test
  void overlapWithSphereIsSymmetric() {
    Sphere3d a = new Sphere3d(new Translation3d(0, 0, 0), 0.5);
    Sphere3d b = new Sphere3d(new Translation3d(0.8, 0, 0), 0.5);

    Contact fromA = a.overlapWithSphere(b);
    Contact fromB = b.overlapWithSphere(a);

    assertEquals(fromA.depth(), fromB.depth(), 1e-9);
    assertEquals(fromA.normal(), fromB.normal().unaryMinus());
  }

  @Test
  void overlapWithSphereMarginReportsNegativeDepthForANearMiss() {
    // Surfaces are 0.1m apart; within a 0.2m margin, but not touching yet.
    Sphere3d a = new Sphere3d(new Translation3d(0, 0, 0), 0.5);
    Sphere3d b = new Sphere3d(new Translation3d(1.1, 0, 0), 0.5);

    Contact contact = a.overlapWithSphere(b, 0.2);

    assertEquals(new Translation3d(1, 0, 0), contact.normal());
    assertEquals(-0.1, contact.depth(), 1e-9);
  }

  @Test
  void coincidentSpheresPickAnArbitraryButValidNormal() {
    Sphere3d a = new Sphere3d(new Translation3d(2, 2, 2), 0.3);
    Sphere3d b = new Sphere3d(new Translation3d(2, 2, 2), 0.3);

    Contact contact = a.overlapWithSphere(b);

    assertEquals(1.0, contact.normal().getNorm(), 1e-9);
    assertEquals(0.6, contact.depth(), 1e-9);
  }
}
