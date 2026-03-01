# ROS 2 Nodes, Topics, and Services

## Introduction

ROS 2's communication architecture is built on three fundamental patterns: **nodes**, **topics**, and **services**. Understanding these patterns is essential for building distributed robotic systems where multiple processes coordinate to achieve complex behaviors.

## Nodes: The Building Blocks

A **node** is a single executable process in a ROS 2 system. Each node should perform a specific, well-defined task. This modular design allows for:

- **Fault isolation**: If one node crashes, others continue running
- **Reusability**: Nodes can be reused across different robot platforms
- **Parallel development**: Teams can work on different nodes simultaneously
- **Testing**: Individual nodes can be tested in isolation

### Creating Your First Node

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key concepts:**
- `rclpy.init()`: Initializes the ROS 2 Python client library
- `Node` class: Base class for all ROS 2 nodes
- `rclpy.spin()`: Keeps the node alive and processing callbacks
- `destroy_node()` and `shutdown()`: Clean up resources

## Topics: Publish-Subscribe Communication

**Topics** implement a publish-subscribe pattern for continuous data streams. This is ideal for sensor data, robot state, and other information that flows continuously.

### Publisher Example

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status update {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1
```

### Subscriber Example

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Topic Characteristics

- **Many-to-many**: Multiple publishers and subscribers can use the same topic
- **Asynchronous**: Publishers don't wait for subscribers
- **Unidirectional**: Data flows from publisher to subscriber
- **Best for**: Sensor streams, continuous state updates, telemetry

### Quality of Service (QoS)

ROS 2 introduces QoS policies to handle different communication requirements:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # or RELIABLE
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

**Common QoS profiles:**
- **Sensor data**: Best effort, keep last 10
- **Commands**: Reliable, keep last 1
- **Logging**: Best effort, keep all

## Services: Request-Response Communication

**Services** implement a request-response pattern for occasional, synchronous operations. This is ideal for actions that require confirmation or return a result.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client Example

```python
class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request(5, 7)

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

### Service Characteristics

- **One-to-one**: Single client communicates with single server
- **Synchronous**: Client waits for response (or uses async pattern)
- **Bidirectional**: Request goes to server, response returns to client
- **Best for**: Configuration changes, one-time queries, mode switches

## Practical Example: Robot Velocity Controller

Let's combine these concepts in a practical example:

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Publisher: Send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber: Receive laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        # Service: Enable/disable autonomous mode
        self.enable_srv = self.create_service(
            SetBool,
            'enable_autonomous',
            self.enable_callback)

        self.autonomous_enabled = False
        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Check for obstacles in front (simplified)
        min_distance = min(msg.ranges[len(msg.ranges)//2 - 10:len(msg.ranges)//2 + 10])
        self.obstacle_detected = min_distance < 0.5

        if self.autonomous_enabled:
            self.navigate()

    def navigate(self):
        twist = Twist()

        if self.obstacle_detected:
            # Stop if obstacle detected
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn
        else:
            # Move forward
            twist.linear.x = 0.3
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def enable_callback(self, request, response):
        self.autonomous_enabled = request.data
        response.success = True
        response.message = f'Autonomous mode {"enabled" if request.data else "disabled"}'
        return response
```

## Command-Line Tools

ROS 2 provides powerful CLI tools for debugging:

```bash
# List all active nodes
ros2 node list

# Get info about a specific node
ros2 node info /velocity_controller

# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /cmd_vel

# Get topic info
ros2 topic info /scan

# Publish to a topic from command line
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# List all services
ros2 service list

# Call a service
ros2 service call /enable_autonomous std_srvs/srv/SetBool "{data: true}"

# Get service type
ros2 service type /enable_autonomous
```

## Design Patterns and Best Practices

### When to Use Topics vs Services

**Use Topics when:**
- Data flows continuously (sensor readings, state updates)
- Multiple nodes need the same data
- Timing is critical (low latency required)
- Fire-and-forget communication is acceptable

**Use Services when:**
- Operation requires confirmation
- Result must be returned to caller
- Operation is infrequent
- Synchronous behavior is needed

### Node Design Principles

1. **Single Responsibility**: Each node should do one thing well
2. **Loose Coupling**: Nodes should communicate through standard interfaces
3. **Composability**: Nodes should be reusable in different configurations
4. **Fail-Safe**: Handle errors gracefully and log appropriately

### Naming Conventions

```python
# Node names: lowercase with underscores
'velocity_controller'

# Topic names: lowercase with forward slashes for hierarchy
'/robot/sensors/camera/image'
'/robot/actuators/cmd_vel'

# Service names: action-oriented
'/robot/enable_motors'
'/robot/reset_odometry'
```

## Exercises

1. **Temperature Monitor**: Create a publisher node that simulates temperature readings and a subscriber that logs warnings when temperature exceeds a threshold.

2. **Service Calculator**: Implement a service that performs basic math operations (add, subtract, multiply, divide) based on the request.

3. **Robot Controller**: Build a system with three nodes:
   - Sensor node: Publishes simulated distance readings
   - Controller node: Subscribes to sensor data and publishes velocity commands
   - Safety node: Provides a service to enable/disable the controller

4. **Multi-Topic Subscriber**: Create a node that subscribes to multiple sensor topics and publishes a combined status message.

## Summary

- **Nodes** are independent processes that perform specific tasks
- **Topics** enable asynchronous, many-to-many communication for continuous data
- **Services** provide synchronous, request-response communication for occasional operations
- **QoS policies** allow fine-tuning of communication reliability and performance
- Proper design patterns ensure maintainable, scalable robotic systems

In the next chapter, we'll explore **launch files** to orchestrate multiple nodes and configure complex robotic systems.
