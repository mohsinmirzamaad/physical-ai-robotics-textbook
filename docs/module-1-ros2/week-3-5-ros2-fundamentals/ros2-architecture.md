---
sidebar_position: 1
---

# ROS 2 Architecture

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand the core concepts of ROS 2 architecture
- Explain the differences between ROS 1 and ROS 2
- Set up a ROS 2 development environment
- Create and build a basic ROS 2 package
- Understand the DDS middleware layer

## What is ROS 2?

ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. Despite its name, ROS is not an operating system but a middleware that provides:

- **Communication infrastructure**: Message passing between processes
- **Hardware abstraction**: Unified interfaces for sensors and actuators
- **Package management**: Modular, reusable components
- **Tools and libraries**: Visualization, simulation, debugging

### Why ROS 2?

ROS 2 was redesigned from the ground up to address limitations of ROS 1:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time | Limited | Full support |
| Multi-robot | Difficult | Native support |
| Security | Minimal | Built-in (DDS Security) |
| Platforms | Linux only | Linux, Windows, macOS |
| Communication | Custom TCP | DDS standard |
| Lifecycle | None | Managed nodes |

## Core Concepts

### 1. Nodes

Nodes are individual processes that perform computation. Each node should have a single, well-defined purpose.

**Example: Robot with multiple nodes**
- `camera_node`: Captures images
- `object_detector_node`: Detects objects in images
- `motion_planner_node`: Plans robot motion
- `motor_controller_node`: Controls motors

### 2. Topics

Topics enable publish-subscribe communication. Nodes publish messages to topics, and other nodes subscribe to receive them.

**Characteristics:**
- **Asynchronous**: Publishers don't wait for subscribers
- **Many-to-many**: Multiple publishers and subscribers
- **Typed**: Each topic has a specific message type

### 3. Services

Services provide synchronous request-response communication.

**Characteristics:**
- **Synchronous**: Client waits for response
- **One-to-one**: Single client, single server
- **Typed**: Request and response types defined

### 4. Actions

Actions are for long-running tasks with feedback and cancellation.

**Characteristics:**
- **Asynchronous with feedback**: Client receives progress updates
- **Cancellable**: Client can cancel the action
- **Three message types**: Goal, Feedback, Result

## ROS 2 Architecture Layers

```
┌─────────────────────────────────────────┐
│     Application Layer (Your Code)       │
├─────────────────────────────────────────┤
│     ROS 2 Client Libraries (rclcpp/py)  │
├─────────────────────────────────────────┤
│     ROS 2 Core (rcl)                    │
├─────────────────────────────────────────┤
│     DDS Middleware (FastDDS, CycloneDDS)│
├─────────────────────────────────────────┤
│     Operating System (Linux/Windows)     │
└─────────────────────────────────────────┘
```

### DDS (Data Distribution Service)

ROS 2 uses DDS as its communication middleware. DDS provides:

- **Discovery**: Automatic node discovery
- **Quality of Service (QoS)**: Reliability, durability, latency control
- **Security**: Authentication, encryption, access control
- **Scalability**: Efficient multi-robot communication

## Installation

### Ubuntu 22.04 (Recommended)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

### Source the Setup Script

```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --help
```

## Creating Your First Package

### Workspace Structure

```
ros2_ws/
├── src/
│   └── my_package/
│       ├── package.xml
│       ├── setup.py
│       ├── my_package/
│       │   ├── __init__.py
│       │   └── my_node.py
│       └── resource/
└── install/
```

### Create a Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_robot_package

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_robot_cpp_package
```

### Package Structure (Python)

**package.xml** - Package metadata:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**setup.py** - Build configuration:

```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
        ],
    },
)
```

### Simple Publisher Node

**my_robot_package/my_node.py**:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Build and Run

```bash
# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash

# Run the node
ros2 run my_robot_package my_node
```

## ROS 2 Command-Line Tools

### Node Management

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /minimal_publisher

# Kill a node
ros2 lifecycle set /node_name shutdown
```

### Topic Management

```bash
# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic

# Get topic info
ros2 topic info /topic

# Publish to topic
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello'"

# Measure topic frequency
ros2 topic hz /topic
```

### Service Management

```bash
# List services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# Get service type
ros2 service type /service_name
```

### Parameter Management

```bash
# List parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value

# Dump parameters to file
ros2 param dump /node_name > params.yaml

# Load parameters from file
ros2 param load /node_name params.yaml
```

## Quality of Service (QoS)

QoS policies control communication behavior:

### Reliability

- **Reliable**: Guaranteed delivery (TCP-like)
- **Best Effort**: No guarantees (UDP-like)

### Durability

- **Transient Local**: Late-joining subscribers receive last message
- **Volatile**: Only receive messages after subscription

### History

- **Keep Last**: Keep last N messages
- **Keep All**: Keep all messages (memory permitting)

### Example QoS Configuration

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor data QoS (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Create publisher with custom QoS
self.publisher_ = self.create_publisher(
    String,
    'sensor_data',
    sensor_qos
)
```

## Lifecycle Nodes

Lifecycle nodes have managed states for controlled startup and shutdown:

**States:**
1. **Unconfigured**: Initial state
2. **Inactive**: Configured but not active
3. **Active**: Running normally
4. **Finalized**: Shutdown

```python
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

class LifecycleNode(Node):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Start processing
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Stop processing
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
```

## Best Practices

### 1. Single Responsibility

Each node should do one thing well.

### 2. Use Namespaces

Organize nodes and topics with namespaces:

```bash
ros2 run my_package my_node --ros-args -r __ns:=/robot1
```

### 3. Parameter Files

Use YAML files for configuration:

```yaml
my_node:
  ros__parameters:
    update_rate: 10.0
    sensor_topic: "/sensors/camera"
    debug_mode: false
```

### 4. Logging Levels

Use appropriate logging:

```python
self.get_logger().debug('Detailed info')
self.get_logger().info('Normal operation')
self.get_logger().warn('Warning condition')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Critical failure')
```

## Summary

- ROS 2 is a middleware framework for robot development
- Core concepts: nodes, topics, services, actions
- DDS provides robust, scalable communication
- QoS policies control communication behavior
- Lifecycle nodes enable managed startup/shutdown
- Command-line tools facilitate development and debugging

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design](https://design.ros2.org/)
- [DDS Specification](https://www.omg.org/spec/DDS/)

## Next Steps

Continue to [Nodes, Topics, and Services](./nodes-topics-services.md) to learn how to implement communication patterns in ROS 2.
