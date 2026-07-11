package jsim.field;

import jsim.api.SimBodyBuilder;
import jsim.material.Material;

/** Spawns dynamic game pieces from canonical season spec definitions. */
public final class SeasonFieldSpecSpawner {
    private SeasonFieldSpecSpawner() {}

    public static void spawnGamePieces(FieldSimulator sim, SeasonFieldSpec spec) {
        for (SeasonFieldSpec.GamePieceSet set : spec.gamePieceSets()) {
            Material mat = new Material(set.friction(), set.restitution());
            int i = 0;
            for (SeasonFieldSpec.PoseSpec pose : set.poses()) {
                sim.spawnGamePiece(set.variant(), new SimBodyBuilder(set.variant() + i)
                        .position(pose.x(), pose.y(), pose.z())
                        .mass(set.mass())
                        .sphereCollider(set.radius())
                        .material(mat));
                i++;
            }
        }
    }
}
