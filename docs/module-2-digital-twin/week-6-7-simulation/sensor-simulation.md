# Sensor Simulation and Fusion

## Introduction

Modern robots rely on multiple sensors to perceive their environment. **Sensor fusion** combines data from different sensor modalities to create a more accurate and robust understanding of the world than any single sensor could provide. In this chapter, we'll explore how to simulate various sensors and implement fusion algorithms.

## Why Sensor Fusion?

Individual sensors have limitations:
- **Cameras**: Affected by lighting, lack depth information
- **LiDAR**: Expensive, limited in fog/rain, no color information
- **IMU**: Drifts over time, no absolute position
- **GPS**: Inaccurate indoors, multipath errors

**Sensor fusion** overcomes these limitations by:
- **Redundancy**: If one sensor fails, others compensate
- **Complementary strengths**: Combine different sensor capabilities
- **Improved accuracy**: Statistical methods reduce individual sensor noise
- **Robustness**: System works in varied environmental conditions

## Vision Sensors

### RGB Cameras

RGB cameras provide color images essential for object recognition, scene understanding, and human interaction.

**Gazebo Camera Plugin:**

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="rgb_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>rgb_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Processing RGB Images in ROS 2:**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Image,
            '/robot/camera/processed',
            10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image
        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header = msg.header

        self.publisher.publish(processed_msg)
```

### Depth Cameras (RGB-D)

Depth cameras like Intel RealSense provide both color and depth information.

**Gazebo Depth Camera Plugin:**

```xml
<gazebo reference="depth_camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>depth/image_raw:=depth_camera/depth/image_raw</remapping>
        <remapping>depth/camera_info:=depth_camera/depth/camera_info</remapping>
        <remapping>image_raw:=depth_camera/image_raw</remapping>
        <remapping>camera_info:=depth_camera/camera_info</remapping>
        <remapping>points:=depth_camera/points</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.05</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

**Point Cloud Processing:**

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/robot/depth_camera/points',
            self.pointcloud_callback,
            10)

    def pointcloud_callback(self, msg):
        # Extract points from point cloud
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[:3]

            # Filter points (e.g., within certain range)
            if 0.1 < z < 5.0:
                points.append([x, y, z])

        self.get_logger().info(f'Processed {len(points)} points')

        # Further processing: clustering, plane detection, etc.
```

## LiDAR Sensors

LiDAR (Light Detection and Ranging) provides accurate distance measurements in 2D or 3D.

### 2D LiDAR

**Gazebo 2D LiDAR Plugin:**

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
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
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
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

### 3D LiDAR (Velodyne-style)

```xml
<gazebo reference="velodyne_link">
  <sensor type="ray" name="velodyne">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2617994</min_angle>
          <max_angle>0.2617994</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.9</min>
        <max>130.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_velodyne" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=velodyne_points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>velodyne_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**LiDAR Processing:**

```python
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        # Find closest obstacle
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)
        min_angle = msg.angle_min + min_index * msg.angle_increment

        self.get_logger().info(
            f'Closest obstacle: {min_range:.2f}m at {min_angle:.2f}rad'
        )

        # Detect obstacles in front (±30 degrees)
        front_indices = self.get_front_indices(msg, angle_range=0.52)
        front_ranges = [msg.ranges[i] for i in front_indices]

        if min(front_ranges) < 0.5:
            self.get_logger().warn('Obstacle detected in front!')

    def get_front_indices(self, msg, angle_range):
        """Get indices for rays within angle_range of forward direction"""
        total_rays = len(msg.ranges)
        center_index = total_rays // 2
        range_indices = int(angle_range / msg.angle_increment)

        return range(center_index - range_indices,
                    center_index + range_indices)
```

## Inertial Measurement Unit (IMU)

IMUs measure acceleration and angular velocity, essential for balance and orientation estimation.

**Gazebo IMU Plugin:**

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**IMU Data Processing:**

```python
from sensor_msgs.msg import Imu
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10)

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.last_time = None

    def imu_callback(self, msg):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Extract linear acceleration
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Remove gravity (assuming z-up)
        accel[2] -= 9.81

        # Integrate to get velocity and position (simple dead reckoning)
        self.velocity += accel * dt
        self.position += self.velocity * dt

        # Extract orientation (quaternion)
        orientation = msg.orientation

        # Convert to Euler angles for logging
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        self.get_logger().info(
            f'Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
```

## Force/Torque Sensors

Force/torque sensors measure contact forces, crucial for manipulation and balance.

**Gazebo Force/Torque Plugin:**

```xml
<gazebo reference="wrist_link">
  <sensor name="force_torque_sensor" type="force_torque">
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=wrist_ft</remapping>
      </ros>
      <frame_name>wrist_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Fusion Techniques

### Kalman Filter for Sensor Fusion

Combine IMU and odometry for better state estimation:

```python
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # State: [x, y, vx, vy]
        self.state = np.zeros(4)

        # Covariance matrix
        self.P = np.eye(4) * 0.1

        # Process noise
        self.Q = np.eye(4) * 0.01

        # Measurement noise
        self.R_odom = np.eye(2) * 0.1  # Position measurement
        self.R_imu = np.eye(2) * 0.05   # Velocity measurement

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher for fused estimate
        self.pub = self.create_publisher(Odometry, '/fused_odom', 10)

        self.last_time = None

    def predict(self, dt):
        """Prediction step"""
        # State transition matrix
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_odom(self, z):
        """Update with odometry measurement (position)"""
        # Measurement matrix (observe position only)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Innovation
        y = z - H @ self.state

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_odom

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.P = (np.eye(4) - K @ H) @ self.P

    def update_imu(self, z):
        """Update with IMU measurement (velocity)"""
        # Measurement matrix (observe velocity only)
        H = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Innovation
        y = z - H @ self.state

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.P = (np.eye(4) - K @ H) @ self.P

    def odom_callback(self, msg):
        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.predict(dt)

        self.last_time = current_time

        # Extract position measurement
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        self.update_odom(z)
        self.publish_fused_estimate()

    def imu_callback(self, msg):
        # Extract velocity from IMU (integrated acceleration)
        # In practice, you'd maintain velocity state from IMU
        # This is simplified
        pass

    def publish_fused_estimate(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.twist.twist.linear.x = self.state[2]
        msg.twist.twist.linear.y = self.state[3]

        self.pub.publish(msg)
```

### Robot Localization Package

ROS 2 provides `robot_localization` package for production-ready sensor fusion:

```bash
sudo apt install ros-humble-robot-localization
```

**Configuration** (`config/ekf.yaml`):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: false

    # Odometry input
    odom0: /odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]

    # IMU input
    imu0: /imu
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]

    # Output
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    publish_tf: true
```

**Launch file:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['/path/to/ekf.yaml'],
            output='screen'
        )
    ])
```

## Multi-Sensor Calibration

### Camera-LiDAR Calibration

Calibrating sensors ensures accurate fusion:

```python
import numpy as np
import cv2

class CameraLidarCalibration:
    def __init__(self, camera_matrix, dist_coeffs, T_lidar_to_camera):
        """
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        T_lidar_to_camera: 4x4 transformation matrix from LiDAR to camera frame
        """
        self.K = camera_matrix
        self.D = dist_coeffs
        self.T = T_lidar_to_camera

    def project_lidar_to_image(self, lidar_points, image):
        """Project LiDAR points onto image"""
        # Convert to homogeneous coordinates
        points_homo = np.hstack([lidar_points, np.ones((len(lidar_points), 1))])

        # Transform to camera frame
        points_cam = (self.T @ points_homo.T).T[:, :3]

        # Filter points behind camera
        valid = points_cam[:, 2] > 0
        points_cam = points_cam[valid]

        # Project to image plane
        points_2d = cv2.projectPoints(
            points_cam,
            np.zeros(3),  # No rotation
            np.zeros(3),  # No translation
            self.K,
            self.D
        )[0].reshape(-1, 2)

        # Draw points on image
        for pt in points_2d:
            cv2.circle(image, tuple(pt.astype(int)), 3, (0, 255, 0), -1)

        return image
```

## Practical Example: Humanoid Sensor Suite

```python
class HumanoidSensorFusion(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_fusion')

        # Subscribers
        self.create_subscription(Image, '/camera/image', self.camera_cb, 10)
        self.create_subscription(PointCloud2, '/depth/points', self.depth_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # Publishers
        self.obstacle_pub = self.create_publisher(
            PoseArray, '/detected_obstacles', 10
        )

        # State
        self.latest_image = None
        self.latest_depth = None
        self.latest_scan = None
        self.orientation = None

    def camera_cb(self, msg):
        self.latest_image = msg

    def depth_cb(self, msg):
        self.latest_depth = msg
        self.fuse_vision_data()

    def lidar_cb(self, msg):
        self.latest_scan = msg

    def imu_cb(self, msg):
        self.orientation = msg.orientation

    def fuse_vision_data(self):
        """Fuse camera and depth data for obstacle detection"""
        if self.latest_image is None or self.latest_depth is None:
            return

        # Process combined RGB-D data
        # Detect obstacles, estimate positions, publish results
        pass
```

## Summary

- **Multiple sensor types** provide complementary information about the environment
- **Vision sensors** (RGB, depth) enable object recognition and scene understanding
- **LiDAR** provides accurate distance measurements for mapping and navigation
- **IMU** measures orientation and acceleration for balance and state estimation
- **Sensor fusion** combines data for robust, accurate perception
- **Kalman filters** and **EKF** are standard fusion techniques
- **Calibration** ensures accurate multi-sensor integration

In the next module, we'll explore **NVIDIA Isaac** for advanced AI-powered perception and simulation.
