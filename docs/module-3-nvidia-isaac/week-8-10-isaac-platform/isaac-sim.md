# Isaac Sim: Photorealistic Robot Simulation

## Introduction

**Isaac Sim** is NVIDIA's photorealistic robot simulator built on the Omniverse platform. It combines physically accurate simulation with RTX-powered ray tracing to create digital twins that closely match real-world behavior, dramatically reducing the sim-to-real gap.

## Why Isaac Sim?

Traditional simulators like Gazebo provide good physics but lack visual fidelity. Isaac Sim bridges this gap:

- **Photorealistic rendering**: RTX ray tracing for accurate lighting and materials
- **Physics accuracy**: PhysX 5 engine with GPU acceleration
- **Synthetic data generation**: Unlimited labeled data for training ML models
- **Domain randomization**: Automatic variation for robust sim-to-real transfer
- **Scalability**: Run thousands of parallel simulations on GPU clusters
- **USD format**: Universal Scene Description for asset interoperability

## Installation and Setup

### System Requirements

**Minimum:**
- GPU: RTX 2070 or higher
- VRAM: 8GB
- RAM: 32GB
- Storage: 50GB

**Recommended:**
- GPU: RTX 4080 or higher
- VRAM: 16GB+
- RAM: 64GB
- Storage: 100GB SSD

### Installing Isaac Sim

**Method 1: Omniverse Launcher (Recommended)**

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

In the launcher:
1. Go to **Exchange** tab
2. Search for "Isaac Sim"
3. Click **Install**
4. Launch Isaac Sim from **Library** tab

**Method 2: Docker Container**

```bash
# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run container
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.1

# Inside container, run Isaac Sim
./runheadless.native.sh
```

### Verifying Installation

```python
# test_isaac_sim.py
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=[0, 0, 1.0],
        size=0.5,
        color=[1.0, 0.0, 0.0]
    )
)

# Run simulation
world.reset()

for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

Run with:
```bash
python test_isaac_sim.py
```

## Isaac Sim Interface

### Python API Basics

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
config = {
    "headless": False,
    "width": 1920,
    "height": 1080
}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.numpy.rotations as rot_utils

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Load robot from USD
robot_usd_path = "/path/to/robot.usd"
add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Robot")

# Initialize world
world.reset()

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

## Importing Robots

### From URDF

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf

# Import URDF
urdf_path = "/path/to/robot.urdf"
imported_robot = _urdf.acquire_urdf_interface().parse_urdf(
    urdf_path=urdf_path,
    import_config=_urdf.ImportConfig(
        merge_fixed_joints=False,
        convex_decomp=False,
        import_inertia_tensor=True,
        fix_base=False
    )
)

# Get prim path
robot_prim_path = imported_robot
```

### From USD (Native Format)

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add robot from USD
robot_prim_path = "/World/Humanoid"
robot_usd_path = "omniverse://localhost/Library/Robots/Humanoid.usd"

add_reference_to_stage(
    usd_path=robot_usd_path,
    prim_path=robot_prim_path
)
```

## Articulation Control

### Position Control

```python
from omni.isaac.core.articulations import Articulation

# Get robot articulation
robot = world.scene.get_object("robot")

# Set joint positions
joint_positions = [0.0, 0.5, -1.0, 0.0, 0.5, -1.0]  # Example for 6 joints
robot.set_joint_positions(joint_positions)

# Get current joint positions
current_positions = robot.get_joint_positions()
```

### Velocity Control

```python
# Set joint velocities
joint_velocities = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
robot.set_joint_velocities(joint_velocities)

# Get current joint velocities
current_velocities = robot.get_joint_velocities()
```

### Effort (Torque) Control

```python
# Apply joint efforts
joint_efforts = [10.0, 5.0, 5.0, 10.0, 5.0, 5.0]
robot.set_joint_efforts(joint_efforts)
```

### PD Control

```python
# Set PD gains
stiffness = [1000.0] * 6  # Position gain
damping = [100.0] * 6     # Velocity gain

robot.set_gains(kps=stiffness, kds=damping)

# Set target positions
target_positions = [0.0, 1.0, -1.5, 0.0, 1.0, -1.5]
robot.set_joint_position_targets(target_positions)
```

## Sensor Simulation

### RGB Camera

```python
from omni.isaac.sensor import Camera
import numpy as np

# Create camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 1.5]),
    frequency=30,
    resolution=(1920, 1080),
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0, 30, 45]), degrees=True
    )
)

# Initialize camera
camera.initialize()

# Get RGB image
world.step(render=True)
rgb_data = camera.get_rgba()[:, :, :3]  # Remove alpha channel

# Save image
from PIL import Image
img = Image.fromarray(rgb_data.astype(np.uint8))
img.save("camera_output.png")
```

### Depth Camera

```python
# Get depth data
depth_data = camera.get_depth()

# Visualize depth
import matplotlib.pyplot as plt
plt.imshow(depth_data, cmap='gray')
plt.colorbar(label='Depth (m)')
plt.savefig('depth_output.png')
```

### Semantic Segmentation

```python
# Enable semantic segmentation
camera.add_semantic_segmentation_to_frame()

# Get semantic labels
semantic_data = camera.get_semantic_segmentation()

# Visualize
plt.imshow(semantic_data)
plt.savefig('semantic_output.png')
```

### LiDAR

```python
from omni.isaac.range_sensor import _range_sensor

# Create LiDAR
lidar_config = _range_sensor.acquire_lidar_sensor_interface()

lidar_path = "/World/Lidar"
lidar_config.create_lidar(
    lidar_path,
    parent=robot_prim_path,
    min_range=0.4,
    max_range=100.0,
    draw_points=True,
    draw_lines=False,
    horizontal_fov=360.0,
    vertical_fov=30.0,
    horizontal_resolution=0.4,
    vertical_resolution=4.0,
    rotation_rate=20.0,
    high_lod=False,
    yaw_offset=0.0
)

# Get LiDAR data
def get_lidar_data():
    depth = lidar_config.get_linear_depth_data(lidar_path)
    azimuth = lidar_config.get_azimuth_data(lidar_path)
    elevation = lidar_config.get_elevation_data(lidar_path)

    # Convert to point cloud
    x = depth * np.cos(elevation) * np.cos(azimuth)
    y = depth * np.cos(elevation) * np.sin(azimuth)
    z = depth * np.sin(elevation)

    return np.stack([x, y, z], axis=-1)
```

## Physics Configuration

### Material Properties

```python
from omni.isaac.core.materials import PhysicsMaterial

# Create physics material
friction_material = PhysicsMaterial(
    prim_path="/World/Materials/HighFriction",
    static_friction=1.0,
    dynamic_friction=0.8,
    restitution=0.1
)

# Apply to object
cube.apply_physics_material(friction_material)
```

### Contact Reporting

```python
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import PhysxSchema

# Enable contact reporting
prim = get_prim_at_path(cube.prim_path)
contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
contact_report_api.CreateThresholdAttr().Set(0.0)

# Get contacts
def get_contacts():
    from omni.physx import get_physx_interface
    physx = get_physx_interface()

    contacts = []
    for contact in physx.get_contact_reports():
        contacts.append({
            'body0': contact.body0,
            'body1': contact.body1,
            'impulse': contact.impulse
        })

    return contacts
```

## Synthetic Data Generation

### Domain Randomization

```python
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.replicator.core import random, modify

# Randomize lighting
def randomize_lighting():
    with modify.Semantics([("class", "light")]):
        random.uniform("intensity", 1000, 10000)
        random.uniform("color", (0.8, 0.8, 0.8), (1.0, 1.0, 1.0))

# Randomize object poses
def randomize_object_poses():
    for obj in scene_objects:
        x = np.random.uniform(-2.0, 2.0)
        y = np.random.uniform(-2.0, 2.0)
        z = np.random.uniform(0.5, 2.0)
        obj.set_world_pose(position=np.array([x, y, z]))

# Randomize textures
def randomize_textures():
    with modify.Semantics([("class", "object")]):
        random.texture()
```

### Automated Data Collection

```python
import omni.replicator.core as rep

# Setup replicator
camera_rep = rep.create.camera(position=(2, 2, 1.5))

# Define randomization
with rep.trigger.on_frame(num_frames=1000):
    # Randomize scene
    with rep.create.group([cube, sphere, cylinder]):
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Randomize lighting
    with rep.create.light():
        rep.modify.attribute("intensity", rep.distribution.uniform(1000, 10000))

# Setup writers
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/output/synthetic_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True
)

# Run data generation
rep.orchestrator.run()
```

## ROS 2 Integration

### Publishing Sensor Data

```python
import rclpy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

class IsaacSimROSBridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('isaac_sim_bridge')

        # Publishers
        self.image_pub = self.node.create_publisher(Image, '/camera/image', 10)
        self.depth_pub = self.node.create_publisher(Image, '/camera/depth', 10)
        self.lidar_pub = self.node.create_publisher(PointCloud2, '/lidar/points', 10)

        self.bridge = CvBridge()

    def publish_camera_data(self, rgb, depth):
        # Publish RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = self.node.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_link'
        self.image_pub.publish(rgb_msg)

        # Publish depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header.stamp = self.node.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'camera_link'
        self.depth_pub.publish(depth_msg)

    def publish_lidar_data(self, points):
        # Convert to PointCloud2 message
        # Implementation details omitted for brevity
        pass
```

### Subscribing to Commands

```python
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.node = rclpy.create_node('robot_controller')

        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # Convert Twist to joint velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Simple differential drive
        left_vel = linear_x - angular_z * 0.5
        right_vel = linear_x + angular_z * 0.5

        self.robot.set_joint_velocities([left_vel, right_vel])
```

## Parallel Simulation

### Running Multiple Environments

```python
from omni.isaac.gym.vec_env import VecEnvBase

class ParallelSimulation(VecEnvBase):
    def __init__(self, num_envs=16):
        self.num_envs = num_envs

        # Create environments
        for i in range(num_envs):
            env_prim_path = f"/World/Env_{i}"
            self.create_environment(env_prim_path, offset=[i * 5.0, 0, 0])

    def create_environment(self, prim_path, offset):
        # Add ground
        add_reference_to_stage(
            usd_path="ground.usd",
            prim_path=f"{prim_path}/Ground"
        )

        # Add robot
        add_reference_to_stage(
            usd_path="robot.usd",
            prim_path=f"{prim_path}/Robot"
        )

        # Set position offset
        robot = world.scene.get_object(f"{prim_path}/Robot")
        robot.set_world_pose(position=np.array(offset))

    def step(self, actions):
        # Apply actions to all environments
        for i, action in enumerate(actions):
            robot = world.scene.get_object(f"/World/Env_{i}/Robot")
            robot.apply_action(action)

        # Step simulation
        world.step(render=True)

        # Collect observations
        observations = []
        for i in range(self.num_envs):
            obs = self.get_observation(i)
            observations.append(obs)

        return observations
```

## Performance Optimization

### GPU Pipeline

```python
# Enable GPU pipeline for faster simulation
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.physx.tensors")

# Use tensor API for batch operations
from omni.isaac.core.utils.torch.maths import tensor_clamp

# Get all joint positions as tensor
joint_positions_tensor = robot.get_joint_positions(clone=False)

# Batch operations on GPU
clamped_positions = tensor_clamp(
    joint_positions_tensor,
    min_val=-3.14,
    max_val=3.14
)

# Set back
robot.set_joint_positions(clamped_positions)
```

### Headless Mode

```bash
# Run without GUI for faster simulation
./runheadless.native.sh -v python my_simulation.py
```

## Practical Example: Humanoid Training Environment

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np

class HumanoidEnv:
    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()

        # Load humanoid
        add_reference_to_stage(
            usd_path="humanoid.usd",
            prim_path="/World/Humanoid"
        )

        self.humanoid = self.world.scene.add(
            Articulation(prim_path="/World/Humanoid")
        )

        # Add target
        self.target = self.world.scene.add(
            DynamicSphere(
                prim_path="/World/Target",
                position=[5.0, 0.0, 0.5],
                radius=0.2,
                color=[0, 1, 0]
            )
        )

        self.world.reset()

    def reset(self):
        # Reset humanoid pose
        self.humanoid.set_joint_positions(np.zeros(self.humanoid.num_dof))

        # Randomize target position
        target_pos = np.array([
            np.random.uniform(3, 7),
            np.random.uniform(-2, 2),
            0.5
        ])
        self.target.set_world_pose(position=target_pos)

        return self.get_observation()

    def step(self, action):
        # Apply action
        self.humanoid.set_joint_position_targets(action)

        # Step simulation
        self.world.step(render=True)

        # Get observation
        obs = self.get_observation()

        # Calculate reward
        reward = self.compute_reward()

        # Check if done
        done = self.is_done()

        return obs, reward, done

    def get_observation(self):
        # Joint positions and velocities
        joint_pos = self.humanoid.get_joint_positions()
        joint_vel = self.humanoid.get_joint_velocities()

        # Humanoid position
        humanoid_pos, _ = self.humanoid.get_world_pose()

        # Target position
        target_pos, _ = self.target.get_world_pose()

        # Combine into observation
        obs = np.concatenate([
            joint_pos,
            joint_vel,
            humanoid_pos,
            target_pos
        ])

        return obs

    def compute_reward(self):
        humanoid_pos, _ = self.humanoid.get_world_pose()
        target_pos, _ = self.target.get_world_pose()

        # Distance to target
        distance = np.linalg.norm(humanoid_pos - target_pos)

        # Reward is negative distance
        reward = -distance

        # Bonus for reaching target
        if distance < 0.5:
            reward += 10.0

        return reward

    def is_done(self):
        # Check if humanoid fell
        humanoid_pos, _ = self.humanoid.get_world_pose()
        if humanoid_pos[2] < 0.3:  # Height threshold
            return True

        return False

# Training loop
env = HumanoidEnv()

for episode in range(100):
    obs = env.reset()
    episode_reward = 0

    for step in range(1000):
        # Random action (replace with RL policy)
        action = np.random.uniform(-1, 1, env.humanoid.num_dof)

        obs, reward, done = env.step(action)
        episode_reward += reward

        if done:
            break

    print(f"Episode {episode}: Reward = {episode_reward}")

simulation_app.close()
```

## Summary

- **Isaac Sim** provides photorealistic simulation with RTX ray tracing
- **USD format** enables asset interoperability and scalability
- **Synthetic data generation** creates unlimited labeled training data
- **Domain randomization** improves sim-to-real transfer
- **ROS 2 integration** enables seamless workflow
- **Parallel simulation** accelerates training with GPU clusters
- **Sensor simulation** includes RGB, depth, semantic, and LiDAR

In the next chapter, we'll explore **Isaac ROS** for hardware-accelerated perception on real robots.
