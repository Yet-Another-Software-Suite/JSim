# Gamepiece Simulation

JSim can simulate any rigid game piece — balls, rings, notes, coral — with realistic contact physics.

## Ball on carpet floor

```java
// Floor (add once, typically in constructor)
simWorld.addBody(new SimBodyBuilder("Floor")
    .planeCollider(0, 0, 1, 0)
    .material(Material.CARPET));

// 2025 Crescendo Note — 36 cm diameter ring, ~0.23 kg
SimBody note = simWorld.addBody(new SimBodyBuilder("Note")
    .position(4.5, 2.0, 0.18)         // 18 cm off the floor (radius)
    .mass(0.235)
    .sphereCollider(0.18)              // approximate as a sphere
    .material(Material.RUBBER));
```

## Spinning projectile (Magnus effect)

For game pieces launched with backspin or topspin, add a Magnus force:

```java
SimBody ball = simWorld.addBody(new SimBodyBuilder("Ball")
    .position(0, 0, 1.2)
    .linearVelocity(8, 0, 3)           // launched at an angle
    .angularVelocity(0, -40, 0)        // topspin about Y axis
    .mass(0.24)
    .sphereCollider(0.085)
    .material(Material.RUBBER));

simWorld.addMagnusForce(ball, 0.15);   // Magnus coefficient (empirical)
```

## Drag on a flying game piece

```java
simWorld.addDragForce(ball, 0.05, 0.01);   // light linear + rotational drag
```

## Detecting when a game piece enters a scoring zone

Since JSim is a physics library, zone detection is implemented by the caller. The simplest approach is distance-based:

```java
Translation3d goal = new Translation3d(8.3, 4.1, 2.0);   // goal position

if (ball.getPosition().getDistance(goal) < 0.25) {
    // ball is within 25 cm of the goal centre
    simWorld.removeBody(ball);                             // despawn it
    scored = true;
}
```

## Tracking multiple game pieces

```java
List<SimBody> notes = new ArrayList<>();

// Spawn five notes at random field positions
for (int i = 0; i < 5; i++) {
    notes.add(simWorld.addBody(new SimBodyBuilder("Note" + i)
        .position(random.nextDouble() * 16, random.nextDouble() * 8, 0.18)
        .mass(0.235)
        .sphereCollider(0.18)
        .material(Material.RUBBER)));
}

// In simulationPeriodic():
for (SimBody note : notes) {
    SmartDashboard.putNumberArray("Note/" + note.getName(),
        new double[] { note.getPosition().getX(),
                       note.getPosition().getY(),
                       note.getPosition().getZ() });
}
```
