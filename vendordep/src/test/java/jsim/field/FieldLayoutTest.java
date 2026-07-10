package jsim.field;

import static org.junit.jupiter.api.Assertions.*;
import jsim.api.SimBody;
import jsim.api.SimBodyBuilder;
import jsim.api.SimWorld;
import jsim.material.Material;
import org.junit.jupiter.api.Test;

class FieldLayoutTest {

    @Test
    void addFrcField_addsFiveStaticBodies() {
        SimWorld world = new SimWorld(0.02, 10);
        int before = world.getBodies().size();
        FieldLayout.addFrcField(world);
        // floor + 4 walls = 5 bodies
        assertEquals(before + 5, world.getBodies().size());
    }

    @Test
    void allFieldBodiesAreStatic() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);
        for (SimBody b : world.getBodies()) {
            assertTrue(b.isStatic(), "Field body '" + b.getName() + "' must be static");
        }
    }

    @Test
    void fieldBodiesDoNotMoveAfterSteps() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);

        // Record initial positions
        double[] xs = world.getBodies().stream().mapToDouble(b -> b.getPosition().getX()).toArray();
        double[] ys = world.getBodies().stream().mapToDouble(b -> b.getPosition().getY()).toArray();
        double[] zs = world.getBodies().stream().mapToDouble(b -> b.getPosition().getZ()).toArray();

        for (int i = 0; i < 100; i++) world.step();

        int i = 0;
        for (SimBody b : world.getBodies()) {
            assertEquals(xs[i], b.getPosition().getX(), 1e-9,
                b.getName() + " X must not change");
            assertEquals(ys[i], b.getPosition().getY(), 1e-9,
                b.getName() + " Y must not change");
            assertEquals(zs[i], b.getPosition().getZ(), 1e-9,
                b.getName() + " Z must not change");
            i++;
        }
    }

    @Test
    void floorPreventsSphereFallingThrough() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);

        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(8, 4, 2)
            .mass(0.235)
            .sphereCollider(0.18)
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) world.step();

        assertTrue(ball.getPosition().getZ() >= -0.1,
            "Ball Z must not go far below floor: " + ball.getPosition().getZ());
    }

    @Test
    void redWallStopsFastProjectile() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);

        // Launch a sphere toward the red (high-X) wall at very high speed
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(8, 4, 0.25)
            .linearVelocity(30, 0, 0)
            .mass(0.235)
            .sphereCollider(0.18)
            .noGravity()
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) world.step();

        assertTrue(ball.getPosition().getX() < FieldLayout.FRC_LENGTH_M + 1.0,
            "Ball must not escape past the red wall: x=" + ball.getPosition().getX());
    }

    @Test
    void blueWallStopsFastProjectile() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);

        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(8, 4, 0.25)
            .linearVelocity(-30, 0, 0)
            .mass(0.235)
            .sphereCollider(0.18)
            .noGravity()
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) world.step();

        assertTrue(ball.getPosition().getX() > -1.0,
            "Ball must not escape past the blue wall: x=" + ball.getPosition().getX());
    }

    @Test
    void nearAndFarWallsContainBall() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFrcField(world);

        // Launch diagonally toward near-Y wall then far-Y wall
        SimBody ball = world.addBody(new SimBodyBuilder("Ball")
            .position(8, 4, 0.25)
            .linearVelocity(0, 30, 0)
            .mass(0.235)
            .sphereCollider(0.18)
            .noGravity()
            .material(Material.RUBBER));

        for (int i = 0; i < 200; i++) world.step();

        assertTrue(ball.getPosition().getY() < FieldLayout.FRC_WIDTH_M + 1.0,
            "Ball must not escape past far wall: y=" + ball.getPosition().getY());
    }

    @Test
    void addField_customDimensions() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addField(world, 10.0, 5.0);
        assertEquals(5, world.getBodies().size());
        for (SimBody b : world.getBodies()) {
            assertTrue(b.isStatic());
        }
    }

    @Test
    void addWalls_addsExactlyFourBodies() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addWalls(world, 16.0, 8.0);
        assertEquals(4, world.getBodies().size());
        for (SimBody b : world.getBodies()) {
            assertTrue(b.isStatic());
        }
    }

    @Test
    void addFloor_addsOnlyOneBody() {
        SimWorld world = new SimWorld(0.02, 10);
        FieldLayout.addFloor(world);
        assertEquals(1, world.getBodies().size());
        assertTrue(world.getBodies().get(0).isStatic());
    }
}
