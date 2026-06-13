"""
Main simulation runtime loop for JSim.
Handles the fixed timestep update and NT4 synchronization.
"""

import time
import logging

try:
    from jsim_core import PhysicsWorld
except ImportError:
    PhysicsWorld = None

from apps.sim_runtime.sensor_pipeline import SensorPipeline
from apps.sim_runtime.robot_loader import RobotLoader

class SimLoop:
    def __init__(self, tick_rate_hz: float = 50.0, asset_path: str = None):
        self.tick_rate_hz = tick_rate_hz
        self.dt = 1.0 / tick_rate_hz
        self._running = False
        self.physics_world = None
        self.sensor_pipeline = SensorPipeline()
        self.robot_loader = RobotLoader(asset_path) if asset_path else None
        self._initialize_world()

    def start(self):
        """Starts the simulation main loop."""
        self._running = True
        logging.info(f"Starting simulation loop at {self.tick_rate_hz} Hz")
        try:
            self._loop()
        except KeyboardInterrupt:
            self.stop()

    def _initialize_world(self):
        """Creates the physics world and loads robot assets."""
        if PhysicsWorld is None:
            logging.warning("jsim_core not available; physics will not run.")
            return
        
        try:
            # Create default physics world with 0.01s timestep
            self.physics_world = PhysicsWorld(0.01, True)
            logging.info("Physics world initialized")
            
            # Load robot if asset path provided
            if self.robot_loader:
                self.robot_loader.load()
                logging.info("Robot loaded from assets")
        except Exception as e:
            logging.error(f"Failed to initialize physics world: {e}")
            self.physics_world = None
    
    def stop(self):
        """Stops the simulation gracefully."""
        self._running = False
        if self.physics_world:
            try:
                self.physics_world.close()
            except:
                pass
        logging.info("Stopping simulation loop.")

    def _loop(self):
        while self._running:
            start_time = time.perf_counter()
            self._tick()
            elapsed = time.perf_counter() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                logging.warning("Simulation tick trailing behind real-time!")

    def _tick(self):
        """Advances the physics state by one timestep."""
        if self.physics_world:
            try:
                # Step physics by one timestep
                self.physics_world.step(1)
                
                # Update and publish sensor data
                self.sensor_pipeline.update(self.dt)
                self.sensor_pipeline.publish()
            except Exception as e:
                logging.error(f"Physics step error: {e}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    loop = SimLoop()
    loop.start()
