# Isaac ROS: Hardware-Accelerated Perception

## Introduction

**Isaac ROS** is a collection of hardware-accelerated ROS 2 packages that leverage NVIDIA GPUs and specialized hardware accelerators to deliver real-time performance for robotics applications. These packages are optimized for both discrete GPUs (RTX series) and edge devices (Jetson).

## Why Isaac ROS?

Traditional ROS 2 packages run on CPU, which limits performance for compute-intensive tasks like deep learning inference and image processing. Isaac ROS solves this:

- **GPU acceleration**: 10-100x faster than CPU implementations
- **Hardware accelerators**: Leverage specialized units (Tensor Cores, DLA, PVA)
- **Low latency**: Real-time performance for perception pipelines
- **Power efficiency**: Optimized for Jetson edge devices
- **Drop-in replacement**: Compatible with standard ROS 2 interfaces

## Architecture

Isaac ROS packages use **NVIDIA GXF (Graph Execution Framework)** under the hood for optimized graph execution:

```
ROS 2 Node → GXF Graph → CUDA Kernels → GPU/Accelerators
```

**Key components:**
- **GXF**: Low-latency graph execution framework
- **CUDA**: GPU compute kernels
- **TensorRT**: Optimized deep learning inference
- **VPI (Vision Programming Interface)**: Hardware-accelerated vision algorithms
- **cuDNN**: GPU-accelerated deep neural networks

## Installation

### Prerequisites

**For Jetson:**
```bash
# JetPack 5.1+ (includes CUDA, cuDNN, TensorRT)
sudo apt update
sudo apt install nvidia-jetpack
```

**For x86_64 with discrete GPU:**
```bash
# Install CUDA Toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-0

# Install TensorRT
sudo apt install tensorrt
```

### Installing Isaac ROS

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Clone desired packages (example: DNN inference)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Build using Docker (recommended)
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container, build
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

## Isaac ROS Packages Overview

### Perception Packages

1. **isaac_ros_dnn_inference**: Deep learning inference
2. **isaac_ros_image_proc**: GPU-accelerated image processing
3. **isaac_ros_stereo**: Stereo depth estimation
4. **isaac_ros_visual_slam**: Visual SLAM
5. **isaac_ros_apriltag**: AprilTag detection
6. **isaac_ros_object_detection**: Object detection

### Navigation Packages

1. **isaac_ros_nvblox**: 3D reconstruction and mapping
2. **isaac_ros_freespace_segmentation**: Drivable area detection

## DNN Inference with TensorRT

### Object Detection

```bash
# Launch object detection
ros2 launch isaac_ros_dnn_inference isaac_ros_dnn_inference.launch.py \
    model_file_path:=/path/to/model.onnx \
    engine_file_path:=/path/to/model.plan \
    input_topic:=/camera/image_raw \
    output_topic:=/detections
```

**Python example:**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Subscribe to detections from Isaac ROS
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Subscribe to images for visualization
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_image = None

    def detection_callback(self, msg):
        self.latest_detections = msg.detections
        self.visualize()

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def visualize(self):
        if self.latest_image is None or self.latest_detections is None:
            return

        img = self.latest_image.copy()

        for detection in self.latest_detections:
            # Get bounding box
            bbox = detection.bbox
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            width = int(bbox.size_x)
            height = int(bbox.size_y)

            # Calculate corners
            x1 = center_x - width // 2
            y1 = center_y - height // 2
            x2 = center_x + width // 2
            y2 = center_y + height // 2

            # Draw rectangle
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                text = f"{label}: {score:.2f}"
                cv2.putText(img, text, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Detections', img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Custom Model Deployment

```python
# Convert PyTorch model to ONNX
import torch
import torch.onnx

model = YourModel()
model.eval()

dummy_input = torch.randn(1, 3, 640, 640)

torch.onnx.export(
    model,
    dummy_input,
    "model.onnx",
    export_params=True,
    opset_version=11,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'output': {0: 'batch_size'}
    }
)
```

```bash
# Convert ONNX to TensorRT engine
/usr/src/tensorrt/bin/trtexec \
    --onnx=model.onnx \
    --saveEngine=model.plan \
    --fp16 \
    --workspace=4096
```

## Visual SLAM (cuVSLAM)

Isaac ROS provides hardware-accelerated Visual SLAM for real-time localization and mapping.

### Launch cuVSLAM

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Configuration** (`config/visual_slam.yaml`):

```yaml
visual_slam_node:
  ros__parameters:
    # Camera parameters
    camera_optical_frames: ['camera_infra1_optical_frame', 'camera_infra2_optical_frame']

    # SLAM parameters
    enable_rectified_pose: true
    enable_slam_visualization: true
    enable_landmarks_view: true
    enable_observations_view: true

    # Map parameters
    map_frame: 'map'
    odom_frame: 'odom'
    base_frame: 'base_link'

    # Performance
    num_cameras: 2
    min_num_images: 2

    # Feature tracking
    enable_imu_fusion: true
    gyro_noise_density: 0.000244
    gyro_random_walk: 0.000019
    accel_noise_density: 0.001862
    accel_random_walk: 0.003
```

**Python integration:**

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')

        # Subscribe to SLAM pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.pose_callback,
            10
        )

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(
            f'Position: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, '
            f'z={msg.pose.position.z:.2f}'
        )

    def odom_callback(self, msg):
        # Use odometry for velocity information
        linear_vel = msg.twist.twist.linear
        angular_vel = msg.twist.twist.angular
```

## Stereo Depth Estimation

Hardware-accelerated stereo depth computation using SGM (Semi-Global Matching).

```bash
# Launch stereo depth
ros2 launch isaac_ros_stereo isaac_ros_stereo.launch.py \
    left_image_topic:=/camera/left/image_raw \
    right_image_topic:=/camera/right/image_raw \
    left_camera_info_topic:=/camera/left/camera_info \
    right_camera_info_topic:=/camera/right/camera_info
```

**Processing depth output:**

```python
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        self.disparity_sub = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/depth',
            10
        )

        self.bridge = CvBridge()

    def disparity_callback(self, msg):
        # Convert disparity to depth
        # depth = (focal_length * baseline) / disparity
        focal_length = msg.f
        baseline = msg.t

        disparity = self.bridge.imgmsg_to_cv2(msg.image, 'passthrough')

        # Avoid division by zero
        disparity[disparity == 0] = 0.1

        depth = (focal_length * baseline) / disparity

        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)
```

## Nvblox: 3D Reconstruction

Nvblox creates 3D voxel maps from depth cameras for navigation and planning.

```bash
# Launch nvblox
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

**Configuration:**

```yaml
nvblox_node:
  ros__parameters:
    # Voxel size (meters)
    voxel_size: 0.05

    # ESDF (Euclidean Signed Distance Field)
    esdf_mode: 1  # 0: off, 1: 2D, 2: 3D
    esdf_2d_min_height: 0.0
    esdf_2d_max_height: 1.0

    # Mapping
    max_integration_distance: 10.0
    truncation_distance: 0.2

    # Performance
    max_back_projection_distance: 10.0

    # Mesh generation
    mesh_bandwidth: 10.0
```

**Using the map:**

```python
from nvblox_msgs.msg import DistanceMapSlice
from geometry_msgs.msg import Point

class NavigationPlanner(Node):
    def __init__(self):
        super().__init__('navigation_planner')

        # Subscribe to ESDF slice
        self.esdf_sub = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/map_slice',
            self.esdf_callback,
            10
        )

        self.distance_map = None

    def esdf_callback(self, msg):
        # Store distance map
        self.distance_map = msg

    def is_path_clear(self, start, goal):
        """Check if path between start and goal is collision-free"""
        if self.distance_map is None:
            return False

        # Sample points along path
        num_samples = 20
        for i in range(num_samples):
            t = i / num_samples
            point = Point()
            point.x = start.x + t * (goal.x - start.x)
            point.y = start.y + t * (goal.y - start.y)
            point.z = start.z + t * (goal.z - start.z)

            # Check distance to obstacles
            distance = self.get_distance_at_point(point)
            if distance < 0.3:  # Safety margin
                return False

        return True

    def get_distance_at_point(self, point):
        # Query distance map at point
        # Implementation depends on map structure
        pass
```

## AprilTag Detection

Hardware-accelerated AprilTag detection for localization and object tracking.

```bash
# Launch AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

**Processing detections:**

```python
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class AprilTagTracker(Node):
    def __init__(self):
        super().__init__('apriltag_tracker')

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10
        )

        self.known_tags = {
            0: "charging_station",
            1: "waypoint_1",
            2: "waypoint_2"
        }

    def detection_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            pose = detection.pose.pose.pose

            if tag_id in self.known_tags:
                location = self.known_tags[tag_id]
                self.get_logger().info(
                    f'Detected {location} (ID: {tag_id}) at '
                    f'position: ({pose.position.x:.2f}, '
                    f'{pose.position.y:.2f}, {pose.position.z:.2f})'
                )

                # Use for localization
                self.update_robot_pose(tag_id, pose)

    def update_robot_pose(self, tag_id, tag_pose):
        # Update robot pose based on known tag location
        pass
```

## Image Processing

GPU-accelerated image processing operations.

```bash
# Launch image rectification
ros2 launch isaac_ros_image_proc isaac_ros_image_proc.launch.py
```

**Available operations:**
- Rectification
- Debayering
- Resizing
- Color space conversion
- Image flipping/rotation

**Custom processing pipeline:**

```python
from isaac_ros_image_proc import ImageProc

class CustomImagePipeline(Node):
    def __init__(self):
        super().__init__('custom_pipeline')

        # Input
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Output
        self.processed_pub = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

    def image_callback(self, msg):
        # Isaac ROS handles GPU processing automatically
        # Just republish or do additional CPU processing
        self.processed_pub.publish(msg)
```

## Performance Benchmarks

### Jetson Orin Nano (8GB)

| Package | CPU (ms) | GPU (ms) | Speedup |
|---------|----------|----------|---------|
| Object Detection (ResNet50) | 450 | 15 | 30x |
| Stereo Depth (VGA) | 120 | 8 | 15x |
| Visual SLAM | 80 | 12 | 6.7x |
| AprilTag Detection | 35 | 3 | 11.7x |

### RTX 4080

| Package | CPU (ms) | GPU (ms) | Speedup |
|---------|----------|----------|---------|
| Object Detection (ResNet50) | 450 | 3 | 150x |
| Stereo Depth (1080p) | 200 | 5 | 40x |
| Visual SLAM | 80 | 4 | 20x |
| Nvblox Mapping | 150 | 8 | 18.8x |

## Practical Example: Humanoid Perception Stack

```python
class HumanoidPerceptionStack(Node):
    def __init__(self):
        super().__init__('humanoid_perception')

        # Subscribe to Isaac ROS outputs
        self.create_subscription(
            Detection2DArray, '/detections', self.detection_cb, 10
        )
        self.create_subscription(
            PoseStamped, '/visual_slam/tracking/vo_pose', self.slam_cb, 10
        )
        self.create_subscription(
            DistanceMapSlice, '/nvblox_node/map_slice', self.map_cb, 10
        )
        self.create_subscription(
            AprilTagDetectionArray, '/tag_detections', self.tag_cb, 10
        )

        # State
        self.detected_objects = []
        self.robot_pose = None
        self.local_map = None
        self.known_landmarks = {}

    def detection_cb(self, msg):
        self.detected_objects = msg.detections

    def slam_cb(self, msg):
        self.robot_pose = msg.pose

    def map_cb(self, msg):
        self.local_map = msg

    def tag_cb(self, msg):
        for detection in msg.detections:
            self.known_landmarks[detection.id] = detection.pose

    def get_world_state(self):
        """Fuse all perception data into world state"""
        return {
            'robot_pose': self.robot_pose,
            'objects': self.detected_objects,
            'map': self.local_map,
            'landmarks': self.known_landmarks
        }
```

## Summary

- **Isaac ROS** provides GPU-accelerated ROS 2 packages
- **10-100x speedup** over CPU implementations
- **Real-time performance** on Jetson edge devices
- **Drop-in replacement** for standard ROS 2 packages
- **cuVSLAM** enables robust visual localization
- **Nvblox** creates 3D maps for navigation
- **TensorRT** optimizes deep learning inference
- Complete perception stack for autonomous robots

In the next chapter, we'll explore **Nav2** for autonomous navigation with humanoid robots.
