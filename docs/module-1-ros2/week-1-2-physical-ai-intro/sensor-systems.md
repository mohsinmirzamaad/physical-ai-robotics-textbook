---
sidebar_position: 3
---

# Sensor Systems and Perception

## Learning Outcomes

By the end of this chapter, you will be able to:
- Identify different types of sensors used in robotics
- Understand sensor characteristics (accuracy, precision, range, latency)
- Process sensor data for perception tasks
- Handle sensor noise and uncertainty
- Integrate multiple sensors for robust perception

## Introduction to Robot Sensors

Sensors are the robot's window to the world. They convert physical phenomena into electrical signals that can be processed by computers. For Physical AI, choosing and using the right sensors is critical.

### Sensor Categories

1. **Proprioceptive**: Measure the robot's own state (position, velocity, force)
2. **Exteroceptive**: Measure the environment (distance, light, sound)
3. **Active**: Emit energy and measure reflections (LiDAR, sonar)
4. **Passive**: Measure ambient energy (cameras, microphones)

## Vision Sensors

### RGB Cameras

Standard cameras capture color images, the most information-rich sensor for robotics.

**Specifications:**
- Resolution: 640×480 to 4K+
- Frame rate: 30-120 FPS
- Field of view: 60-180 degrees
- Interface: USB, MIPI, Ethernet

**Use Cases:**
- Object detection and recognition
- Visual servoing
- Human-robot interaction
- Scene understanding

```python
import cv2
import numpy as np

class RGBCamera:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture frame")
        return frame

    def detect_objects(self, frame, model):
        # Preprocess image
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True)

        # Run detection
        model.setInput(blob)
        detections = model.forward()

        return self.parse_detections(detections, frame.shape)

    def parse_detections(self, detections, image_shape):
        objects = []
        for detection in detections:
            confidence = detection[4]
            if confidence > 0.5:
                # Extract bounding box
                center_x = int(detection[0] * image_shape[1])
                center_y = int(detection[1] * image_shape[0])
                width = int(detection[2] * image_shape[1])
                height = int(detection[3] * image_shape[0])

                objects.append({
                    'bbox': [center_x, center_y, width, height],
                    'confidence': confidence,
                    'class_id': np.argmax(detection[5:])
                })

        return objects
```

### Depth Cameras

Depth cameras measure the distance to each pixel, providing 3D information.

**Technologies:**
- **Stereo**: Two cameras triangulate depth
- **Structured light**: Project pattern, measure distortion
- **Time-of-Flight (ToF)**: Measure light travel time

**Popular Models:**
- Intel RealSense (stereo + structured light)
- Microsoft Kinect (ToF)
- ZED Camera (stereo)

```python
import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    def get_3d_point(self, pixel_x, pixel_y, depth_image):
        # Get depth at pixel
        depth = depth_image[pixel_y, pixel_x]

        # Convert to 3D point using camera intrinsics
        depth_intrin = self.pipeline.get_active_profile()\
            .get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        point_3d = rs.rs2_deproject_pixel_to_point(
            depth_intrin, [pixel_x, pixel_y], depth
        )

        return np.array(point_3d)

    def segment_plane(self, depth_image, threshold=0.01):
        # Simple plane segmentation using RANSAC
        points = self.depth_to_pointcloud(depth_image)

        # RANSAC plane fitting
        best_plane = None
        best_inliers = 0

        for _ in range(100):  # 100 iterations
            # Sample 3 random points
            sample = points[np.random.choice(len(points), 3, replace=False)]

            # Fit plane
            plane = self.fit_plane(sample)

            # Count inliers
            distances = np.abs(points @ plane[:3] + plane[3])
            inliers = np.sum(distances < threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = plane

        return best_plane, best_inliers
```

## Range Sensors

### LiDAR (Light Detection and Ranging)

LiDAR uses laser pulses to measure distances, creating precise 3D point clouds.

**Specifications:**
- Range: 0.1m to 100m+
- Accuracy: ±2cm
- Scan rate: 5-20 Hz
- Points per second: 100K to 1M+

**Types:**
- **2D LiDAR**: Single scanning plane (e.g., SICK, Hokuyo)
- **3D LiDAR**: Multiple planes or rotating (e.g., Velodyne, Ouster)
- **Solid-state**: No moving parts (e.g., Livox)

```python
import numpy as np

class LiDAR:
    def __init__(self, num_beams=360, max_range=30.0):
        self.num_beams = num_beams
        self.max_range = max_range
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = (self.angle_max - self.angle_min) / num_beams

    def scan(self):
        # In real implementation, this would read from hardware
        # Here we simulate a scan
        ranges = np.random.uniform(0.5, self.max_range, self.num_beams)
        intensities = np.random.uniform(0, 255, self.num_beams)

        return {
            'ranges': ranges,
            'intensities': intensities,
            'angle_min': self.angle_min,
            'angle_max': self.angle_max,
            'angle_increment': self.angle_increment
        }

    def to_cartesian(self, scan_data):
        # Convert polar coordinates to Cartesian
        angles = np.linspace(
            scan_data['angle_min'],
            scan_data['angle_max'],
            len(scan_data['ranges'])
        )

        x = scan_data['ranges'] * np.cos(angles)
        y = scan_data['ranges'] * np.sin(angles)

        return np.column_stack([x, y])

    def detect_obstacles(self, scan_data, min_distance=0.5):
        # Find points closer than threshold
        close_points = scan_data['ranges'] < min_distance

        if np.any(close_points):
            # Find angle of closest point
            closest_idx = np.argmin(scan_data['ranges'])
            closest_angle = (self.angle_min +
                           closest_idx * self.angle_increment)

            return {
                'obstacle_detected': True,
                'closest_distance': scan_data['ranges'][closest_idx],
                'closest_angle': closest_angle
            }

        return {'obstacle_detected': False}
```

### Ultrasonic Sensors

Ultrasonic sensors emit sound waves and measure echo time to determine distance.

**Specifications:**
- Range: 2cm to 4m
- Accuracy: ±1cm
- Beam angle: 15-30 degrees
- Update rate: 10-50 Hz

**Advantages:**
- Cheap ($1-10)
- Works in any lighting
- Simple interface

**Disadvantages:**
- Limited range
- Wide beam (poor angular resolution)
- Affected by soft materials

```python
import time

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.speed_of_sound = 343  # m/s at 20°C

    def measure_distance(self):
        # Send trigger pulse
        self.trigger_pin.write(1)
        time.sleep(0.00001)  # 10 microseconds
        self.trigger_pin.write(0)

        # Wait for echo
        while self.echo_pin.read() == 0:
            pulse_start = time.time()

        while self.echo_pin.read() == 1:
            pulse_end = time.time()

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * self.speed_of_sound) / 2

        return distance

    def get_filtered_distance(self, num_samples=5):
        # Take multiple measurements and filter
        measurements = []
        for _ in range(num_samples):
            measurements.append(self.measure_distance())
            time.sleep(0.01)

        # Remove outliers and average
        measurements = np.array(measurements)
        median = np.median(measurements)
        filtered = measurements[np.abs(measurements - median) < 0.1]

        return np.mean(filtered) if len(filtered) > 0 else median
```

## Proprioceptive Sensors

### Encoders

Encoders measure rotation angle or position.

**Types:**
- **Incremental**: Count pulses (relative position)
- **Absolute**: Direct angle reading (absolute position)
- **Optical**: Light through slotted disk
- **Magnetic**: Hall effect sensors

```python
class Encoder:
    def __init__(self, resolution=1024):
        self.resolution = resolution  # Pulses per revolution
        self.count = 0
        self.last_count = 0

    def update(self, pulse):
        # Increment or decrement based on direction
        self.count += pulse

    def get_angle(self):
        # Convert count to angle in radians
        return (self.count / self.resolution) * 2 * np.pi

    def get_velocity(self, dt):
        # Calculate angular velocity
        delta_count = self.count - self.last_count
        self.last_count = self.count

        delta_angle = (delta_count / self.resolution) * 2 * np.pi
        return delta_angle / dt

    def reset(self):
        self.count = 0
        self.last_count = 0
```

### IMU (Inertial Measurement Unit)

IMUs measure acceleration and angular velocity using accelerometers and gyroscopes.

**Components:**
- **Accelerometer**: Measures linear acceleration (3 axes)
- **Gyroscope**: Measures angular velocity (3 axes)
- **Magnetometer**: Measures magnetic field (optional, for heading)

```python
import numpy as np
from scipy.spatial.transform import Rotation

class IMU:
    def __init__(self):
        self.orientation = Rotation.from_quat([0, 0, 0, 1])
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.gravity = np.array([0, 0, -9.81])

    def update(self, accel, gyro, dt):
        # Update orientation from gyroscope
        delta_rotation = Rotation.from_rotvec(gyro * dt)
        self.orientation = self.orientation * delta_rotation

        # Remove gravity from acceleration
        accel_world = self.orientation.apply(accel) - self.gravity

        # Integrate acceleration to get velocity
        self.velocity += accel_world * dt

        # Integrate velocity to get position
        self.position += self.velocity * dt

        return {
            'orientation': self.orientation.as_quat(),
            'velocity': self.velocity,
            'position': self.position
        }

    def get_euler_angles(self):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        return self.orientation.as_euler('xyz', degrees=True)
```

### Force/Torque Sensors

Measure forces and torques applied to the robot, essential for manipulation and interaction.

```python
class ForceTorqueSensor:
    def __init__(self, max_force=100, max_torque=10):
        self.max_force = max_force  # Newtons
        self.max_torque = max_torque  # Newton-meters
        self.bias = np.zeros(6)  # 3 forces + 3 torques

    def calibrate(self, num_samples=100):
        # Measure bias when no load applied
        measurements = []
        for _ in range(num_samples):
            measurements.append(self.read_raw())

        self.bias = np.mean(measurements, axis=0)

    def read(self):
        # Read and remove bias
        raw = self.read_raw()
        calibrated = raw - self.bias

        return {
            'force': calibrated[:3],  # Fx, Fy, Fz
            'torque': calibrated[3:]  # Tx, Ty, Tz
        }

    def detect_contact(self, threshold=5.0):
        # Detect if force exceeds threshold
        reading = self.read()
        force_magnitude = np.linalg.norm(reading['force'])

        return force_magnitude > threshold
```

## Sensor Fusion

Combining multiple sensors provides more robust and accurate perception than any single sensor.

### Complementary Filter

Simple fusion of accelerometer and gyroscope for orientation estimation:

```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Weight for gyroscope
        self.angle = np.zeros(3)

    def update(self, accel, gyro, dt):
        # Integrate gyroscope (high-frequency, drifts)
        gyro_angle = self.angle + gyro * dt

        # Calculate angle from accelerometer (low-frequency, noisy)
        accel_angle = np.array([
            np.arctan2(accel[1], accel[2]),
            np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2)),
            0  # Accelerometer can't measure yaw
        ])

        # Fuse: trust gyro for short-term, accel for long-term
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle

        return self.angle
```

### Kalman Filter

Optimal fusion for linear systems with Gaussian noise:

```python
class KalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state = np.zeros(state_dim)
        self.covariance = np.eye(state_dim)

        # Process noise
        self.Q = np.eye(state_dim) * 0.01

        # Measurement noise
        self.R = np.eye(measurement_dim) * 0.1

    def predict(self, F, B=None, u=None):
        # Predict state: x = F*x + B*u
        if B is not None and u is not None:
            self.state = F @ self.state + B @ u
        else:
            self.state = F @ self.state

        # Predict covariance: P = F*P*F' + Q
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement, H):
        # Innovation: y = z - H*x
        innovation = measurement - H @ self.state

        # Innovation covariance: S = H*P*H' + R
        S = H @ self.covariance @ H.T + self.R

        # Kalman gain: K = P*H'*S^-1
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state: x = x + K*y
        self.state = self.state + K @ innovation

        # Update covariance: P = (I - K*H)*P
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance

        return self.state
```

## Sensor Characteristics

### Accuracy vs. Precision

- **Accuracy**: How close to the true value
- **Precision**: How repeatable the measurements

### Latency

Time delay between physical event and measurement availability. Critical for real-time control.

### Bandwidth

Rate at which sensor can provide new measurements. Higher bandwidth enables faster control loops.

### Range and Resolution

- **Range**: Minimum to maximum measurable value
- **Resolution**: Smallest detectable change

## Practical Considerations

### Sensor Placement

- **Field of view**: Ensure sensors cover workspace
- **Occlusion**: Avoid blocking sensors with robot body
- **Redundancy**: Multiple sensors for critical measurements
- **Calibration**: Easy access for maintenance

### Noise Handling

```python
class SensorFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.buffer = []

    def moving_average(self, new_value):
        self.buffer.append(new_value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        return np.mean(self.buffer)

    def median_filter(self, new_value):
        self.buffer.append(new_value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        return np.median(self.buffer)

    def exponential_smoothing(self, new_value, alpha=0.3):
        if not self.buffer:
            self.buffer = [new_value]
        else:
            smoothed = alpha * new_value + (1 - alpha) * self.buffer[-1]
            self.buffer.append(smoothed)

        return self.buffer[-1]
```

## Summary

- Robots use diverse sensors for perception: vision, range, proprioceptive
- Each sensor has trade-offs in accuracy, range, cost, and latency
- Sensor fusion combines multiple sensors for robust perception
- Proper filtering and calibration are essential for reliable operation
- Sensor placement and redundancy improve system robustness

## Further Reading

- Siciliano, B., et al. (2016). "Robotics: Modelling, Planning and Control"
- Thrun, S., et al. (2005). "Probabilistic Robotics"
- Siegwart, R., et al. (2011). "Introduction to Autonomous Mobile Robots"

## Next Steps

Continue to [ROS 2 Architecture](../week-3-5-ros2-fundamentals/ros2-architecture.md) to learn how to integrate these sensors into a robotic system using the Robot Operating System.
