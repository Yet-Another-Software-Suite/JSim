package jsim.collision;

import java.util.List;
import jsim.core.Mat3;
import jsim.core.Vec3;
import jsim.dynamics.RigidBody;
import jsim.material.MaterialCombiner;

/**
 * Generates {@link ContactPoint}s from broadphase candidate pairs.
 *
 * <p>Supported shape pairs: Sphere-Sphere, Sphere-Plane, Box-Plane, Box-Sphere, Box-Box.
 * Each test emits at most one contact (box-box emits the deepest corner contact).
 */
public final class NarrowPhase {
    private NarrowPhase() {}

    /**
     * Populate {@code contacts} from each {@link CollisionPair}. Existing entries are
     * cleared first. Only overlapping contacts (penetration > 0) are added.
     *
     * @param pairs    broadphase candidate pairs to test
     * @param contacts output list to populate (cleared before adding new contacts)
     */
    public static void generateContacts(List<CollisionPair> pairs, List<ContactPoint> contacts) {
        contacts.clear();
        for (CollisionPair pair : pairs) {
            dispatch(pair.a, pair.b, contacts);
        }
    }

    // Dispatch

    private static void dispatch(RigidBody a, RigidBody b, List<ContactPoint> out) {
        ColliderShape.Type ta = a.collider.getType();
        ColliderShape.Type tb = b.collider.getType();

        if (ta == ColliderShape.Type.SPHERE && tb == ColliderShape.Type.SPHERE) {
            sphereSphere(a, (SphereCollider) a.collider, b, (SphereCollider) b.collider, out);
        } else if (ta == ColliderShape.Type.SPHERE && tb == ColliderShape.Type.PLANE) {
            spherePlane(a, (SphereCollider) a.collider, b, (PlaneCollider) b.collider, out);
        } else if (ta == ColliderShape.Type.PLANE && tb == ColliderShape.Type.SPHERE) {
            spherePlane(b, (SphereCollider) b.collider, a, (PlaneCollider) a.collider, out);
        } else if (ta == ColliderShape.Type.BOX && tb == ColliderShape.Type.PLANE) {
            boxPlane(a, (BoxCollider) a.collider, b, (PlaneCollider) b.collider, out);
        } else if (ta == ColliderShape.Type.PLANE && tb == ColliderShape.Type.BOX) {
            boxPlane(b, (BoxCollider) b.collider, a, (PlaneCollider) a.collider, out);
        } else if (ta == ColliderShape.Type.BOX && tb == ColliderShape.Type.SPHERE) {
            boxSphere(a, (BoxCollider) a.collider, b, (SphereCollider) b.collider, out);
        } else if (ta == ColliderShape.Type.SPHERE && tb == ColliderShape.Type.BOX) {
            boxSphere(b, (BoxCollider) b.collider, a, (SphereCollider) a.collider, out);
        } else if (ta == ColliderShape.Type.BOX && tb == ColliderShape.Type.BOX) {
            boxBox(a, (BoxCollider) a.collider, b, (BoxCollider) b.collider, out);
        }
    }

    // Sphere vs Sphere

    private static void sphereSphere(RigidBody a, SphereCollider sa,
                                      RigidBody b, SphereCollider sb,
                                      List<ContactPoint> out) {
        double dx = a.posX - b.posX, dy = a.posY - b.posY, dz = a.posZ - b.posZ;
        double distSq = dx * dx + dy * dy + dz * dz;
        double radSum = sa.radius + sb.radius;
        if (distSq >= radSum * radSum) return;

        double dist = Math.sqrt(distSq);
        double[] n;
        if (dist < 1e-8) {
            n = Vec3.of(0, 0, 1); // degenerate: push apart vertically
        } else {
            n = Vec3.of(dx / dist, dy / dist, dz / dist);
        }
        double pen = radSum - dist;
        double cx = (a.posX + b.posX) / 2, cy = (a.posY + b.posY) / 2, cz = (a.posZ + b.posZ) / 2;
        emit(a, b, cx, cy, cz, n[0], n[1], n[2], pen, out);
    }

    // Sphere vs Plane (sphere is dynamic, plane is static)

    private static void spherePlane(RigidBody sphere, SphereCollider sc,
                                     RigidBody plane, PlaneCollider pc,
                                     List<ContactPoint> out) {
        // Transform plane normal to world space
        double[] R = Mat3.fromQuaternion(plane.qW, plane.qX, plane.qY, plane.qZ);
        double[] wn = Mat3.mulVec(R, Vec3.of(pc.nx, pc.ny, pc.nz));
        // A point on the plane in world space: plane.pos + R * (normal * offset)
        double[] op = Vec3.of(pc.offset * pc.nx, pc.offset * pc.ny, pc.offset * pc.nz);
        double[] wp = Mat3.mulVec(R, op);
        wp[0] += plane.posX; wp[1] += plane.posY; wp[2] += plane.posZ;

        // Signed distance from sphere centre to plane
        double[] sp = Vec3.of(sphere.posX - wp[0], sphere.posY - wp[1], sphere.posZ - wp[2]);
        double dist = Vec3.dot(sp, wn);
        double pen = sc.radius - dist;
        if (pen <= 0) return;

        double cx = sphere.posX - wn[0] * sc.radius;
        double cy = sphere.posY - wn[1] * sc.radius;
        double cz = sphere.posZ - wn[2] * sc.radius;
        // Normal points from plane toward sphere
        emit(sphere, plane, cx, cy, cz, wn[0], wn[1], wn[2], pen, out);
    }

    // Box vs Plane

    private static void boxPlane(RigidBody box, BoxCollider bc,
                                  RigidBody plane, PlaneCollider pc,
                                  List<ContactPoint> out) {
        double[] Rp = Mat3.fromQuaternion(plane.qW, plane.qX, plane.qY, plane.qZ);
        double[] wn = Mat3.mulVec(Rp, Vec3.of(pc.nx, pc.ny, pc.nz));
        double[] op = Vec3.of(pc.offset * pc.nx, pc.offset * pc.ny, pc.offset * pc.nz);
        double[] wp = Mat3.mulVec(Rp, op);
        wp[0] += plane.posX; wp[1] += plane.posY; wp[2] += plane.posZ;

        double[] Rb = Mat3.fromQuaternion(box.qW, box.qX, box.qY, box.qZ);
        double[] signs = {-1, 1};
        for (double sx : signs) for (double sy : signs) for (double sz : signs) {
            double[] lc = Vec3.of(sx * bc.halfX, sy * bc.halfY, sz * bc.halfZ);
            double[] wc = Mat3.mulVec(Rb, lc);
            wc[0] += box.posX; wc[1] += box.posY; wc[2] += box.posZ;
            double d = Vec3.dot(Vec3.sub(wc, wp), wn);
            if (d < 0) {
                emit(box, plane, wc[0], wc[1], wc[2], wn[0], wn[1], wn[2], -d, out);
            }
        }
    }

    // Box vs Sphere

    private static void boxSphere(RigidBody box, BoxCollider bc,
                                   RigidBody sphere, SphereCollider sc,
                                   List<ContactPoint> out) {
        // Transform sphere centre to box's local frame
        double[] Rb = Mat3.fromQuaternion(box.qW, box.qX, box.qY, box.qZ);
        double[] RbT = Mat3.transpose(Rb);
        double[] d = Vec3.of(sphere.posX - box.posX, sphere.posY - box.posY, sphere.posZ - box.posZ);
        double[] local = Mat3.mulVec(RbT, d);

        // Clamp to box extents
        double clampX = Math.max(-bc.halfX, Math.min(bc.halfX, local[0]));
        double clampY = Math.max(-bc.halfY, Math.min(bc.halfY, local[1]));
        double clampZ = Math.max(-bc.halfZ, Math.min(bc.halfZ, local[2]));

        double[] closest = Vec3.of(clampX, clampY, clampZ);
        double[] diff = Vec3.sub(local, closest);
        double distSq = Vec3.lengthSq(diff);
        if (distSq >= sc.radius * sc.radius) return;

        double dist = Math.sqrt(distSq);
        double[] localNormal;
        double pen;
        if (dist < 1e-8) {
            // Sphere centre inside box — push out along the shallowest face axis.
            // Penetration = sphere radius + distance from centre to that face.
            double dx = bc.halfX - Math.abs(local[0]);
            double dy = bc.halfY - Math.abs(local[1]);
            double dz = bc.halfZ - Math.abs(local[2]);
            if (dx <= dy && dx <= dz) {
                localNormal = Vec3.of(Math.signum(local[0]), 0, 0);
                pen = sc.radius + dx;
            } else if (dy <= dz) {
                localNormal = Vec3.of(0, Math.signum(local[1]), 0);
                pen = sc.radius + dy;
            } else {
                localNormal = Vec3.of(0, 0, Math.signum(local[2]));
                pen = sc.radius + dz;
            }
        } else {
            localNormal = Vec3.scale(diff, 1.0 / dist);
            pen = sc.radius - dist;
        }

        // Transform normal back to world frame (sphere is bodyA for convention)
        double[] wn = Mat3.mulVec(Rb, localNormal);
        double[] wClosest = Mat3.mulVec(Rb, closest);
        wClosest[0] += box.posX; wClosest[1] += box.posY; wClosest[2] += box.posZ;
        // Normal points from box toward sphere (sphere = bodyA)
        emit(sphere, box, wClosest[0], wClosest[1], wClosest[2], wn[0], wn[1], wn[2], pen, out);
    }

    // Box vs Box (SAT — emit the deepest contact point)

    private static void boxBox(RigidBody a, BoxCollider ba,
                                RigidBody b, BoxCollider bb,
                                List<ContactPoint> out) {
        double[] Ra = Mat3.fromQuaternion(a.qW, a.qX, a.qY, a.qZ);
        double[] Rb = Mat3.fromQuaternion(b.qW, b.qX, b.qY, b.qZ);

        double[] t = Vec3.of(b.posX - a.posX, b.posY - a.posY, b.posZ - a.posZ);

        double[][] axesA = {colOf(Ra, 0), colOf(Ra, 1), colOf(Ra, 2)};
        double[][] axesB = {colOf(Rb, 0), colOf(Rb, 1), colOf(Rb, 2)};
        double[] halfsA = {ba.halfX, ba.halfY, ba.halfZ};
        double[] halfsB = {bb.halfX, bb.halfY, bb.halfZ};

        double minPen = Double.POSITIVE_INFINITY;
        double[] bestAxis = null;
        int bestAxisSign = 1;

        // Test A's 3 face normals
        for (int i = 0; i < 3; i++) {
            double[] ax = axesA[i];
            double dist = Math.abs(Vec3.dot(t, ax)) - (halfsA[i] + projectBox(ax, Rb, halfsB));
            if (dist > 0) return;
            if (-dist < minPen) {
                minPen = -dist;
                bestAxis = ax;
                bestAxisSign = (Vec3.dot(t, ax) < 0) ? 1 : -1;
            }
        }
        // Test B's 3 face normals
        for (int i = 0; i < 3; i++) {
            double[] ax = axesB[i];
            double dist = Math.abs(Vec3.dot(t, ax)) - (projectBox(ax, Ra, halfsA) + halfsB[i]);
            if (dist > 0) return;
            if (-dist < minPen) {
                minPen = -dist;
                bestAxis = ax;
                bestAxisSign = (Vec3.dot(t, ax) < 0) ? 1 : -1;
            }
        }
        // Test 9 edge cross-product axes (required for edge-edge contacts on rotated boxes)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double[] ax = Vec3.cross(axesA[i], axesB[j]);
                double len = Vec3.length(ax);
                if (len < 1e-8) continue; // parallel edges — already covered by face axes
                ax = Vec3.scale(ax, 1.0 / len);
                double dist = Math.abs(Vec3.dot(t, ax))
                        - (projectBox(ax, Ra, halfsA) + projectBox(ax, Rb, halfsB));
                if (dist > 0) return;
                if (-dist < minPen) {
                    minPen = -dist;
                    bestAxis = ax;
                    bestAxisSign = (Vec3.dot(t, ax) < 0) ? 1 : -1;
                }
            }
        }

        if (bestAxis == null) return;

        // Contact normal points from B to A
        double[] n = Vec3.scale(bestAxis, bestAxisSign);

        // Emit one contact point per corner of B that lies inside A along n.
        // pen_corner = dot(c_B - a.pos, n) + projA gives per-corner penetration depth.
        double projA = projectBox(n, Ra, halfsA);
        boolean emitted = false;
        double[] sv = {-1.0, 1.0};
        for (double sx : sv) for (double sy : sv) for (double sz : sv) {
            double[] localB = Vec3.of(sx * bb.halfX, sy * bb.halfY, sz * bb.halfZ);
            double[] wc = Mat3.mulVec(Rb, localB);
            wc[0] += b.posX; wc[1] += b.posY; wc[2] += b.posZ;
            double pen = Vec3.dot(Vec3.of(wc[0] - a.posX, wc[1] - a.posY, wc[2] - a.posZ), n) + projA;
            if (pen > 0) {
                emit(a, b, wc[0], wc[1], wc[2], n[0], n[1], n[2], pen, out);
                emitted = true;
            }
        }
        // Fallback for edge-edge contacts where no corner of B is inside A
        if (!emitted) {
            double[] corner = deepestCorner(b, Rb, bb, n);
            emit(a, b, corner[0], corner[1], corner[2], n[0], n[1], n[2], minPen, out);
        }
    }

    // SAT helpers

    private static double[] colOf(double[] R, int col) {
        return Vec3.of(R[col], R[3 + col], R[6 + col]);
    }

    private static double projectBox(double[] axis, double[] R, double[] halfs) {
        return Math.abs(Vec3.dot(axis, colOf(R, 0))) * halfs[0]
             + Math.abs(Vec3.dot(axis, colOf(R, 1))) * halfs[1]
             + Math.abs(Vec3.dot(axis, colOf(R, 2))) * halfs[2];
    }

    private static double[] deepestCorner(RigidBody body, double[] R, BoxCollider bc, double[] dir) {
        double sx = (Vec3.dot(dir, colOf(R, 0)) >= 0) ? bc.halfX : -bc.halfX;
        double sy = (Vec3.dot(dir, colOf(R, 1)) >= 0) ? bc.halfY : -bc.halfY;
        double sz = (Vec3.dot(dir, colOf(R, 2)) >= 0) ? bc.halfZ : -bc.halfZ;
        double[] local = Vec3.of(sx, sy, sz);
        double[] world = Mat3.mulVec(R, local);
        world[0] += body.posX; world[1] += body.posY; world[2] += body.posZ;
        return world;
    }

    // Shared emitter

    private static void emit(RigidBody a, RigidBody b,
                              double cx, double cy, double cz,
                              double nx, double ny, double nz,
                              double pen, List<ContactPoint> out) {
        ContactPoint cp = new ContactPoint();
        cp.set(a, b, cx, cy, cz, nx, ny, nz, pen);
        cp.combinedFriction    = MaterialCombiner.combineFriction(a.material, b.material);
        cp.combinedRestitution = MaterialCombiner.combineRestitution(a.material, b.material);
        out.add(cp);
    }
}
