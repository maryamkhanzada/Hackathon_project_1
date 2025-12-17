"""
Basic Isaac Sim Scene Setup
Creates a simple environment with humanoid robot and cameras
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim (headless=False shows GUI)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.sensor import Camera
import numpy as np

# Enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add humanoid robot
humanoid_usd = "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
add_reference_to_stage(usd_path=humanoid_usd, prim_path="/World/Humanoid")

# Add target cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/TargetCube",
        name="target",
        position=np.array([2.0, 0.0, 0.5]),
        scale=np.array([0.2, 0.2, 0.2]),
        color=np.array([1.0, 0.0, 0.0])  # Red
    )
)

# Add camera to robot head
camera = Camera(
    prim_path="/World/Humanoid/head/camera",
    frequency=30,  # 30 FPS
    resolution=(640, 480)
)

# Initialize
camera.initialize()

# Reset world
world.reset()

# Simulation loop
print("Simulation running. Press Ctrl+C to stop.")
frame = 0
while simulation_app.is_running():
    # Step physics
    world.step(render=True)

    # Get camera image every 30 frames (1 second at 30 FPS)
    if frame % 30 == 0:
        rgb_data = camera.get_rgba()
        print(f"Frame {frame}: Camera image shape: {rgb_data.shape}")

    frame += 1

# Cleanup
simulation_app.close()
