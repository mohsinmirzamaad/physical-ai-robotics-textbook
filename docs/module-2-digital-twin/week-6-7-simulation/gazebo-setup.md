# Gazebo Simulation Environment

## Introduction

**Gazebo** is an open-source 3D robotics simulator that provides accurate physics simulation, high-quality graphics, and extensive sensor simulation. It's the industry standard for testing robotic algorithms before deploying to real hardware, saving time, money, and preventing potential damage to expensive robots.

## Why Simulation Matters for Physical AI

Before deploying a humanoid robot in the real world, you need to answer critical questions:
- Will the walking gait be stable?
- How will the robot react to unexpected obstacles?
- Can the vision system handle different lighting conditions?
- What happens if a motor fails?

**Digital twins** in Gazebo allow you to:
- **Test safely**: No risk of hardware damage during development
- **Iterate rapidly**: Modify and test algorithms in minutes, not hours
- **Generate data**: Create synthetic training data for ML models
- **Simulate edge cases**: Test scenarios that are dangerous or expensive in reality
- **Parallelize**: Run multiple simulations simultaneously

## Gazebo Architecture

Gazebo consists of several components:

1. **Gazebo Server (gzserver)**: Physics engine and sensor simulation
2. **Gazebo Client (gzclient)**: 3D visualization interface
3. **Physics Engines**: ODE (default), Bullet, Simbody, DART
4. **Sensor Models**: Camera, LiDAR, IMU, force/torque, GPS
5. **Plugin System**: Custom behaviors and interfaces

## Installation

### Ubuntu 22.04 with ROS 2 Humble

```bash
# Install Gazebo Classic (Gazebo 11)
sudo apt update
sudo apt install gazebo11 libgazebo11-dev

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Verify installation
gazebo --version
```

### Testing the Installation

```bash
# Launch Gazebo with empty world
gazebo

# Launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

## World Files: Defining Environments

Gazebo worlds are defined in **SDF (Simulation Description Format)** XML files.

### Basic Empty World

Create `worlds/empty.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch the world:**
```bash
gazebo worlds/empty.world
```

### Adding Objects to the World

```xml
<!-- Simple box obstacle -->
<model name="box">
  <pose>2 0 0.5 0 0 0</pose>
  <static>false</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>10</mass>
      <inertia>
        <ixx>1.67</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1.67</iyy>
        <iyz>0</iyz>
        <izz>1.67</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

## Robot Models: URDF and SDF

### URDF (Unified Robot Description Format)

URDF is the ROS standard for describing robot kinematics and dynamics.

**Simple robot example** (`urdf/simple_robot.urdf`):

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.6" iyz="0.0" izz="0.8"/>
    </inertial>
  </link>

  <!-- Wheel link -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.2 0.25 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Spawning Robots in Gazebo

Create a launch file (`launch/spawn_robot.launch.py`):

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        FindPackageShare('my_robot_description').find('my_robot_description'),
        'urdf',
        'simple_robot.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot
    ])
```

## Gazebo Plugins

Plugins extend Gazebo's functionality and provide ROS 2 interfaces.

### Common Gazebo-ROS Plugins

#### 1. Differential Drive Plugin

Add to URDF:

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <update_rate>50</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.5</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
  </plugin>
</gazebo>
```

#### 2. Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <camera_name>camera</camera_name>
      <image_topic>image_raw</image_topic>
      <camera_info_topic>camera_info</camera_info_topic>
    </plugin>
  </sensor>
</gazebo>
```

#### 3. LiDAR Plugin

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Physics Configuration

### Tuning Physics Parameters

```xml
<physics type="ode">
  <!-- Time step (smaller = more accurate but slower) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor (1.0 = real-time, >1 = faster than real-time) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Update rate -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- Gravity -->
  <gravity>0 0 -9.81</gravity>

  <!-- Solver settings -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Contact Properties

```xml
<gazebo reference="foot_link">
  <mu1>1.0</mu1>  <!-- Friction coefficient -->
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
```

## Practical Example: Humanoid in Gazebo

### Simplified Humanoid URDF Structure

```xml
<robot name="humanoid">
  <!-- Torso -->
  <link name="torso"/>

  <!-- Head -->
  <link name="head"/>
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Arms (left and right) -->
  <link name="left_upper_arm"/>
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>

  <!-- Legs (left and right) -->
  <link name="left_thigh"/>
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Add sensors -->
  <link name="camera_link"/>
  <link name="imu_link"/>

  <!-- Add Gazebo plugins for control and sensing -->
</robot>
```

## Debugging and Visualization

### Gazebo GUI Tools

- **View → Transparent**: See through objects
- **View → Wireframe**: See collision geometries
- **View → Contacts**: Visualize contact points
- **View → Joints**: See joint axes and limits
- **View → Center of Mass**: Display COM for each link

### Command-Line Tools

```bash
# List all models in simulation
gz model --list

# Get model pose
gz model --model-name=simple_robot --pose

# Set model pose
gz model --model-name=simple_robot --pose "1 2 0.5 0 0 0"

# Pause/unpause simulation
gz world --pause
gz world --play

# Reset simulation
gz world --reset-all
```

### ROS 2 Integration

```bash
# List Gazebo topics
ros2 topic list | grep gazebo

# Monitor joint states
ros2 topic echo /joint_states

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Best Practices

1. **Start simple**: Test with basic shapes before complex robots
2. **Tune physics**: Adjust step size and solver iterations for stability
3. **Use appropriate masses**: Incorrect inertia causes unstable simulation
4. **Monitor performance**: Check real-time factor (should be close to 1.0)
5. **Validate sensors**: Compare simulated sensor data with expected values
6. **Version control**: Keep URDF and world files in git
7. **Document assumptions**: Note any simplifications made in the model

## Common Issues and Solutions

### Robot Falls Through Ground
- Check collision geometries exist
- Verify ground plane is static
- Increase contact stiffness (kp)

### Unstable Simulation
- Reduce physics step size
- Increase solver iterations
- Check for unrealistic masses/inertias

### Slow Performance
- Reduce sensor update rates
- Simplify collision geometries
- Disable unnecessary visualizations
- Use headless mode (gzserver only)

## Exercises

1. **Basic World**: Create a world with obstacles (boxes, cylinders) and test spawning a simple robot.

2. **Sensor Integration**: Add a camera and LiDAR to a robot and visualize the data in RViz.

3. **Physics Experiment**: Create a ramp and test how different friction coefficients affect a rolling ball.

4. **Multi-Robot**: Spawn multiple robots in the same world with different namespaces.

5. **Custom Plugin**: Write a simple Gazebo plugin that applies a force to a model when a ROS 2 message is received.

## Summary

- **Gazebo** provides realistic physics simulation for robotics development
- **SDF/URDF** formats describe worlds and robots
- **Plugins** bridge Gazebo and ROS 2 for seamless integration
- **Physics tuning** is critical for stable, accurate simulation
- **Digital twins** enable safe, rapid iteration before real-world deployment

In the next chapter, we'll explore **Unity integration** for high-fidelity rendering and human-robot interaction scenarios.
