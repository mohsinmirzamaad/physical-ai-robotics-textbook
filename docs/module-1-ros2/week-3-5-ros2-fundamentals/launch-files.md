# Launch Files and System Configuration

## Introduction

As robotic systems grow in complexity, manually starting dozens of nodes becomes impractical. **Launch files** in ROS 2 provide a declarative way to start multiple nodes, set parameters, remap topics, and configure your entire robot system with a single command.

## Why Launch Files?

Consider a humanoid robot system that requires:
- Camera drivers (3 nodes for stereo + depth)
- IMU driver
- Motor controllers (12+ nodes for different joints)
- Localization node
- Navigation stack
- Safety monitor
- Logging system

Starting these manually would require 20+ terminal windows. Launch files solve this by:

- **Automation**: Start all nodes with one command
- **Configuration**: Set parameters for different environments (sim vs real)
- **Reproducibility**: Ensure consistent system startup
- **Modularity**: Include other launch files for hierarchical organization

## Launch File Formats

ROS 2 supports three launch file formats:

1. **Python** (`.py`) - Most flexible, recommended for complex logic
2. **XML** (`.xml`) - Declarative, good for simple configurations
3. **YAML** (`.yaml`) - Simplest, limited functionality

We'll focus on Python launch files as they're most powerful for Physical AI applications.

## Basic Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='camera_node',
            name='camera',
            output='screen'
        ),
        Node(
            package='my_robot_pkg',
            executable='controller_node',
            name='controller',
            output='screen'
        )
    ])
```

**Key components:**
- `generate_launch_description()`: Required function that returns LaunchDescription
- `Node`: Action to start a ROS 2 node
- `package`: Name of the ROS 2 package containing the executable
- `executable`: Name of the executable to run
- `name`: Node name (overrides default)
- `output='screen'`: Display node output in terminal

**Running the launch file:**
```bash
ros2 launch my_robot_pkg robot_launch.py
```

## Parameters and Configuration

### Inline Parameters

```python
Node(
    package='my_robot_pkg',
    executable='sensor_node',
    name='lidar',
    parameters=[{
        'frame_id': 'lidar_link',
        'scan_rate': 10.0,
        'min_range': 0.1,
        'max_range': 30.0
    }]
)
```

### Parameters from YAML File

Create `config/robot_params.yaml`:
```yaml
sensor_node:
  ros__parameters:
    frame_id: 'lidar_link'
    scan_rate: 10.0
    min_range: 0.1
    max_range: 30.0
```

Load in launch file:
```python
import os
from ament_index_python.packages import get_package_share_directory

config_file = os.path.join(
    get_package_share_directory('my_robot_pkg'),
    'config',
    'robot_params.yaml'
)

Node(
    package='my_robot_pkg',
    executable='sensor_node',
    name='lidar',
    parameters=[config_file]
)
```

## Launch Arguments

Launch arguments allow runtime configuration:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    # Use arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        Node(
            package='my_robot_pkg',
            executable='controller_node',
            name=robot_name,
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        )
    ])
```

**Usage:**
```bash
ros2 launch my_robot_pkg robot_launch.py use_sim_time:=true robot_name:=atlas
```

## Topic and Service Remapping

Remapping allows connecting nodes with incompatible topic names:

```python
Node(
    package='camera_driver',
    executable='camera_node',
    remappings=[
        ('image', 'camera/image_raw'),
        ('camera_info', 'camera/camera_info')
    ]
)
```

This is crucial when integrating third-party packages that use different naming conventions.

## Namespaces

Namespaces prevent topic name collisions in multi-robot systems:

```python
Node(
    package='my_robot_pkg',
    executable='controller_node',
    namespace='robot1',
    name='controller'
)
```

This node's topics will be prefixed with `/robot1/`, e.g., `/robot1/cmd_vel`.

## Conditional Execution

Execute nodes based on conditions:

```python
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(use_rviz)
        ),

        Node(
            package='my_robot_pkg',
            executable='headless_mode',
            condition=UnlessCondition(use_rviz)
        )
    ])
```

## Including Other Launch Files

Build modular launch systems by including other launch files:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('my_robot_pkg'),
            '/launch/sensors.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ])
    )

    return LaunchDescription([
        sensors_launch,
        navigation_launch
    ])
```

## Event Handlers

React to node lifecycle events:

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera'
    )

    # Start processing node after camera starts
    processing_node = Node(
        package='image_processing',
        executable='processor_node',
        name='processor'
    )

    start_processor = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[processing_node]
        )
    )

    return LaunchDescription([
        camera_node,
        start_processor
    ])
```

## Practical Example: Humanoid Robot Launch

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Robot name'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Config files
    config_dir = PathJoinSubstitution([
        FindPackageShare('humanoid_bringup'),
        'config'
    ])

    # Robot state publisher (URDF)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('humanoid_description'),
            '/launch/robot_state_publisher.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Sensor drivers
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_depth': True,
            'enable_color': True
        }]
    )

    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        namespace=robot_name,
        parameters=[
            PathJoinSubstitution([config_dir, 'imu_params.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Motor controllers
    motor_controller = Node(
        package='humanoid_control',
        executable='motor_controller',
        namespace=robot_name,
        parameters=[
            PathJoinSubstitution([config_dir, 'motor_params.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Balance controller
    balance_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': use_sim_time,
            'kp': 1.5,
            'ki': 0.1,
            'kd': 0.05
        }]
    )

    # RViz (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            config_dir, 'humanoid.rviz'
        ])],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_name,
        declare_use_rviz,
        robot_description,
        camera_node,
        imu_node,
        motor_controller,
        balance_controller,
        rviz_node
    ])
```

**Usage examples:**
```bash
# Real robot with RViz
ros2 launch humanoid_bringup humanoid.launch.py

# Simulation without RViz
ros2 launch humanoid_bringup humanoid.launch.py use_sim_time:=true use_rviz:=false

# Custom robot name
ros2 launch humanoid_bringup humanoid.launch.py robot_name:=atlas_v2
```

## Launch File Organization

Recommended structure for a robot package:

```
my_robot_pkg/
├── launch/
│   ├── robot.launch.py          # Main launch file
│   ├── sensors.launch.py        # Sensor drivers
│   ├── control.launch.py        # Controllers
│   ├── navigation.launch.py     # Navigation stack
│   └── simulation.launch.py     # Gazebo/Isaac Sim
├── config/
│   ├── robot_params.yaml
│   ├── sensor_params.yaml
│   └── navigation_params.yaml
└── rviz/
    └── robot.rviz
```

## Debugging Launch Files

```bash
# Verbose output
ros2 launch my_robot_pkg robot.launch.py --debug

# Show launch arguments
ros2 launch my_robot_pkg robot.launch.py --show-args

# Print launch description without executing
ros2 launch my_robot_pkg robot.launch.py --print-description
```

## Best Practices

1. **Use launch arguments** for configuration that changes between runs
2. **Use YAML files** for parameters that rarely change
3. **Modularize** with included launch files for complex systems
4. **Add descriptions** to all launch arguments
5. **Use namespaces** for multi-robot systems
6. **Set output='screen'** for critical nodes during development
7. **Use event handlers** for nodes with dependencies
8. **Document** expected behavior and required hardware

## Common Patterns

### Simulation vs Real Robot

```python
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Real robot hardware interface
    hardware_node = Node(
        package='robot_hardware',
        executable='hardware_interface',
        condition=UnlessCondition(use_sim_time)
    )

    # Simulation interface
    sim_node = Node(
        package='gazebo_ros',
        executable='spawn_entity',
        condition=IfCondition(use_sim_time)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        hardware_node,
        sim_node
    ])
```

### Multi-Robot Systems

```python
def generate_launch_description():
    robots = []

    for i in range(3):
        robot = Node(
            package='my_robot_pkg',
            executable='controller_node',
            namespace=f'robot{i}',
            parameters=[{
                'robot_id': i,
                'x_pos': i * 2.0
            }]
        )
        robots.append(robot)

    return LaunchDescription(robots)
```

## Exercises

1. **Basic Launch**: Create a launch file that starts a publisher and subscriber node with configurable topic names.

2. **Parameterized System**: Build a launch file that loads parameters from a YAML file and allows overriding them via launch arguments.

3. **Conditional Execution**: Create a launch file that starts different nodes based on a `mode` argument (e.g., 'autonomous', 'manual', 'test').

4. **Modular Launch**: Split a complex system into multiple launch files (sensors, control, visualization) and create a main launch file that includes them all.

5. **Multi-Robot**: Create a launch file that spawns multiple robots in simulation, each in its own namespace.

## Summary

- **Launch files** automate starting and configuring complex robotic systems
- **Python launch files** provide maximum flexibility with programmatic control
- **Launch arguments** enable runtime configuration
- **Included launch files** promote modularity and reusability
- **Conditional execution** adapts behavior based on context (sim vs real)
- **Event handlers** manage node dependencies and lifecycle
- Proper organization and documentation are essential for maintainable systems

With launch files mastered, you can now orchestrate complete robotic systems. In the next module, we'll explore **digital twins** using Gazebo and Unity for physics simulation and visualization.
