package frc.jsim.collision;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class ColliderShapeTest {

    private static final double EPS = 1e-10;
    // Identity orientation (no rotation)
    private static final double QW = 1, QX = 0, QY = 0, QZ = 0;

    // ------------------------------------------------------------------
    // SphereCollider
    // ------------------------------------------------------------------

    @Test
    void sphere_aabbAtOrigin() {
        SphereCollider s = new SphereCollider(0.5);
        double[] aabb = new double[6];
        s.computeAABB(0, 0, 0, QW, QX, QY, QZ, aabb);
        assertArrayEquals(new double[]{-0.5, -0.5, -0.5, 0.5, 0.5, 0.5}, aabb, EPS);
    }

    @Test
    void sphere_aabbAtOffset() {
        SphereCollider s = new SphereCollider(1.0);
        double[] aabb = new double[6];
        s.computeAABB(3, 4, 5, QW, QX, QY, QZ, aabb);
        assertArrayEquals(new double[]{2, 3, 4, 4, 5, 6}, aabb, EPS);
    }

    @Test
    void sphere_aabbUnaffectedByRotation() {
        // Sphere AABB is rotation-independent
        SphereCollider s = new SphereCollider(0.5);
        double[] aabbId = new double[6], aabbRot = new double[6];
        double sq = Math.sqrt(0.5);
        s.computeAABB(0, 0, 0, QW, QX, QY, QZ, aabbId);
        s.computeAABB(0, 0, 0, sq, 0, 0, sq, aabbRot); // 90° around Z
        assertArrayEquals(aabbId, aabbRot, EPS);
    }

    @Test
    void sphere_negativeRadiusThrows() {
        assertThrows(IllegalArgumentException.class, () -> new SphereCollider(-1));
    }

    // ------------------------------------------------------------------
    // BoxCollider — identity orientation
    // ------------------------------------------------------------------

    @Test
    void box_aabbIdentityOrientation() {
        BoxCollider b = new BoxCollider(1.0, 2.0, 3.0);
        double[] aabb = new double[6];
        b.computeAABB(0, 0, 0, QW, QX, QY, QZ, aabb);
        assertArrayEquals(new double[]{-1, -2, -3, 1, 2, 3}, aabb, EPS);
    }

    @Test
    void box_aabbAtOffset() {
        BoxCollider b = new BoxCollider(0.5, 0.5, 0.5);
        double[] aabb = new double[6];
        b.computeAABB(1, 2, 3, QW, QX, QY, QZ, aabb);
        assertArrayEquals(new double[]{0.5, 1.5, 2.5, 1.5, 2.5, 3.5}, aabb, EPS);
    }

    @Test
    void box_aabb90DegRotationAroundZ_swapsXY() {
        // 90° around Z turns halfX=2, halfY=1 into an AABB halfX=1, halfY=2
        BoxCollider b = new BoxCollider(2.0, 1.0, 0.5);
        double sq = Math.sqrt(0.5);
        double[] aabb = new double[6];
        b.computeAABB(0, 0, 0, sq, 0, 0, sq, aabb); // 90° around Z
        // world extents: x = |R[0,0]|*2 + |R[0,1]|*1 = 0+1=1, y = 0+2=2, z = 0.5
        assertEquals(1.0, aabb[3], 1e-10);  // max X
        assertEquals(2.0, aabb[4], 1e-10);  // max Y
        assertEquals(0.5, aabb[5], 1e-10);  // max Z
    }

    @Test
    void box_aabbEnclosesCube_under45DegRotation() {
        // After a true 45° rotation around Z, both X and Y extents grow to sqrt(2)
        BoxCollider b = new BoxCollider(1.0, 1.0, 0.0);
        // Quaternion for 45° around Z: half-angle = 22.5°
        double qw = Math.cos(Math.PI / 8.0);
        double qz = Math.sin(Math.PI / 8.0);
        double[] aabb = new double[6];
        b.computeAABB(0, 0, 0, qw, 0, 0, qz, aabb);
        assertEquals(Math.sqrt(2.0), aabb[3], 1e-10);
        assertEquals(Math.sqrt(2.0), aabb[4], 1e-10);
    }

    // ------------------------------------------------------------------
    // PlaneCollider
    // ------------------------------------------------------------------

    @Test
    void plane_aabbIsInfinite() {
        PlaneCollider p = new PlaneCollider(0, 0, 1, 0);
        double[] aabb = new double[6];
        p.computeAABB(0, 0, 0, QW, QX, QY, QZ, aabb);
        // min < 0, max > 0 (very large)
        assertTrue(aabb[0] < -1e8);
        assertTrue(aabb[3] > 1e8);
    }

    @Test
    void plane_normalIsNormalized() {
        PlaneCollider p = new PlaneCollider(3, 4, 0, 0); // length 5
        // Stored normal should be unit length
        double len = Math.sqrt(p.nx * p.nx + p.ny * p.ny + p.nz * p.nz);
        assertEquals(1.0, len, EPS);
        assertEquals(3.0 / 5.0, p.nx, EPS);
        assertEquals(4.0 / 5.0, p.ny, EPS);
    }

    // ------------------------------------------------------------------
    // Type tags
    // ------------------------------------------------------------------

    @Test
    void typeTagsAreCorrect() {
        assertEquals(ColliderShape.Type.SPHERE, new SphereCollider(1).getType());
        assertEquals(ColliderShape.Type.BOX,    new BoxCollider(1, 1, 1).getType());
        assertEquals(ColliderShape.Type.PLANE,  new PlaneCollider(0, 0, 1, 0).getType());
    }
}
