# Humanoid Kinematics and Control

## Introduction

**Humanoid kinematics** is the study of motion in humanoid robots without considering forces. Understanding kinematics is essential for controlling humanoid robots to perform natural, human-like movements. This chapter covers forward kinematics, inverse kinematics, and motion planning for bipedal robots.

## Humanoid Robot Structure

A typical humanoid robot consists of:

- **Head**: 2-3 DOF (pan, tilt, roll)
- **Torso**: 3 DOF (pitch, roll, yaw)
- **Arms**: 7 DOF each (shoulder: 3, elbow: 1, wrist: 3)
- **Hands**: 5-20 DOF (depending on dexterity)
- **Legs**: 6 DOF each (hip: 3, knee: 1, ankle: 2)

**Total DOF**: 30-50+ degrees of freedom

## Coordinate Frames and Transformations

### Denavit-Hartenberg (DH) Parameters

DH parameters describe the relationship between consecutive joints:

- **θ (theta)**: Joint angle (rotation about z-axis)
- **d**: Link offset (translation along z-axis)
- **a**: Link length (translation along x-axis)
- **α (alpha)**: Link twist (rotation about x-axis)

**Transformation matrix:**

```
T = Rot(z, θ) * Trans(z, d) * Trans(x, a) * Rot(x, α)
```

### Example: Simple Arm

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Compute DH transformation matrix"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

# 3-DOF arm example
# Joint 1: Shoulder rotation
T1 = dh_transform(theta1, 0, 0, np.pi/2)

# Joint 2: Shoulder pitch
T2 = dh_transform(theta2, 0, L1, 0)

# Joint 3: Elbow
T3 = dh_transform(theta3, 0, L2, 0)

# End-effector pose
T_end = T1 @ T2 @ T3
```

## Forward Kinematics

Forward kinematics computes the end-effector position given joint angles.

### Implementation

```python
class HumanoidArm:
    def __init__(self):
        # Link lengths (meters)
        self.L1 = 0.3  # Upper arm
        self.L2 = 0.25  # Forearm
        self.L3 = 0.1   # Hand

    def forward_kinematics(self, joint_angles):
        """
        Compute end-effector position from joint angles
        joint_angles: [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow, wrist_pitch, wrist_roll, wrist_yaw]
        """
        theta = joint_angles

        # Shoulder transformations
        T1 = dh_transform(theta[0], 0, 0, np.pi/2)  # Shoulder yaw
        T2 = dh_transform(theta[1], 0, 0, np.pi/2)  # Shoulder pitch
        T3 = dh_transform(theta[2], 0, self.L1, 0)  # Shoulder roll

        # Elbow
        T4 = dh_transform(theta[3], 0, self.L2, 0)

        # Wrist
        T5 = dh_transform(theta[4], 0, 0, np.pi/2)  # Wrist pitch
        T6 = dh_transform(theta[5], 0, 0, np.pi/2)  # Wrist roll
        T7 = dh_transform(theta[6], 0, self.L3, 0)  # Wrist yaw

        # Complete transformation
        T_end = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7

        # Extract position and orientation
        position = T_end[:3, 3]
        rotation = T_end[:3, :3]

        return position, rotation

    def get_jacobian(self, joint_angles):
        """Compute Jacobian matrix for velocity control"""
        epsilon = 1e-6
        n_joints = len(joint_angles)

        # Get current end-effector position
        pos0, _ = self.forward_kinematics(joint_angles)

        # Numerical Jacobian
        J = np.zeros((3, n_joints))

        for i in range(n_joints):
            # Perturb joint i
            theta_plus = joint_angles.copy()
            theta_plus[i] += epsilon

            pos_plus, _ = self.forward_kinematics(theta_plus)

            # Compute derivative
            J[:, i] = (pos_plus - pos0) / epsilon

        return J
```

## Inverse Kinematics

Inverse kinematics computes joint angles needed to reach a desired end-effector pose.

### Analytical IK (2-Link Planar Arm)

```python
def inverse_kinematics_2d(x, y, L1, L2):
    """
    Analytical IK for 2-link planar arm
    Returns: (theta1, theta2) or None if unreachable
    """
    # Distance to target
    d = np.sqrt(x**2 + y**2)

    # Check if reachable
    if d > L1 + L2 or d < abs(L1 - L2):
        return None

    # Law of cosines for theta2
    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)

    # Two solutions (elbow up/down)
    theta2_1 = np.arccos(cos_theta2)
    theta2_2 = -theta2_1

    # Solve for theta1
    k1 = L1 + L2 * np.cos(theta2_1)
    k2 = L2 * np.sin(theta2_1)
    theta1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    k1 = L1 + L2 * np.cos(theta2_2)
    k2 = L2 * np.sin(theta2_2)
    theta1_2 = np.arctan2(y, x) - np.arctan2(k2, k1)

    # Return elbow-down solution
    return (theta1_1, theta2_1)
```

### Numerical IK (Jacobian-based)

```python
def inverse_kinematics_jacobian(arm, target_pos, initial_guess, max_iter=100, tolerance=1e-3):
    """
    Numerical IK using Jacobian pseudo-inverse
    """
    joint_angles = initial_guess.copy()

    for iteration in range(max_iter):
        # Current position
        current_pos, _ = arm.forward_kinematics(joint_angles)

        # Error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            return joint_angles, True

        # Compute Jacobian
        J = arm.get_jacobian(joint_angles)

        # Pseudo-inverse
        J_pinv = np.linalg.pinv(J)

        # Update joint angles
        delta_theta = J_pinv @ error
        joint_angles += delta_theta * 0.1  # Step size

        # Joint limits
        joint_angles = np.clip(joint_angles, -np.pi, np.pi)

    return joint_angles, False
```

### Damped Least Squares (More Stable)

```python
def inverse_kinematics_dls(arm, target_pos, initial_guess, lambda_damping=0.01, max_iter=100):
    """
    IK using Damped Least Squares (Levenberg-Marquardt)
    More stable near singularities
    """
    joint_angles = initial_guess.copy()

    for iteration in range(max_iter):
        current_pos, _ = arm.forward_kinematics(joint_angles)
        error = target_pos - current_pos

        if np.linalg.norm(error) < 1e-3:
            return joint_angles, True

        J = arm.get_jacobian(joint_angles)

        # Damped least squares
        JtJ = J.T @ J
        damping = lambda_damping * np.eye(JtJ.shape[0])
        delta_theta = np.linalg.solve(JtJ + damping, J.T @ error)

        joint_angles += delta_theta * 0.5

        # Joint limits
        joint_angles = np.clip(joint_angles, -np.pi, np.pi)

    return joint_angles, False
```

## Whole-Body Kinematics

For humanoid robots, we need to consider the entire body, not just individual limbs.

```python
class HumanoidKinematics:
    def __init__(self):
        self.left_arm = HumanoidArm()
        self.right_arm = HumanoidArm()
        self.left_leg = HumanoidLeg()
        self.right_leg = HumanoidLeg()

        # Body dimensions
        self.torso_height = 0.5
        self.shoulder_width = 0.4
        self.hip_width = 0.3

    def forward_kinematics_whole_body(self, joint_state):
        """
        Compute positions of all end-effectors
        joint_state: dict with keys 'left_arm', 'right_arm', 'left_leg', 'right_leg', 'torso'
        """
        # Base frame (pelvis)
        base_pos = np.array([0, 0, 0])
        base_rot = np.eye(3)

        # Torso
        torso_angles = joint_state['torso']
        T_torso = self.compute_torso_transform(torso_angles)

        # Left arm (relative to left shoulder)
        left_shoulder_pos = base_pos + T_torso[:3, :3] @ np.array([0, self.shoulder_width/2, self.torso_height])
        left_hand_pos, _ = self.left_arm.forward_kinematics(joint_state['left_arm'])
        left_hand_pos = left_shoulder_pos + T_torso[:3, :3] @ left_hand_pos

        # Right arm
        right_shoulder_pos = base_pos + T_torso[:3, :3] @ np.array([0, -self.shoulder_width/2, self.torso_height])
        right_hand_pos, _ = self.right_arm.forward_kinematics(joint_state['right_arm'])
        right_hand_pos = right_shoulder_pos + T_torso[:3, :3] @ right_hand_pos

        # Legs
        left_foot_pos, _ = self.left_leg.forward_kinematics(joint_state['left_leg'])
        right_foot_pos, _ = self.right_leg.forward_kinematics(joint_state['right_leg'])

        return {
            'left_hand': left_hand_pos,
            'right_hand': right_hand_pos,
            'left_foot': left_foot_pos,
            'right_foot': right_foot_pos
        }

    def compute_torso_transform(self, torso_angles):
        """Compute torso transformation from base"""
        roll, pitch, yaw = torso_angles

        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R = Rz @ Ry @ Rx

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [0, 0, self.torso_height]

        return T
```

## Motion Planning

### Trajectory Generation

```python
class TrajectoryGenerator:
    def __init__(self):
        pass

    def linear_interpolation(self, start, goal, duration, dt):
        """Generate linear trajectory in joint space"""
        num_steps = int(duration / dt)
        trajectory = []

        for i in range(num_steps + 1):
            t = i / num_steps
            point = start + t * (goal - start)
            trajectory.append(point)

        return np.array(trajectory)

    def cubic_polynomial(self, start, goal, duration, dt):
        """Generate smooth trajectory using cubic polynomial"""
        num_steps = int(duration / dt)
        trajectory = []

        # Cubic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        # Boundary conditions: q(0)=start, q(T)=goal, q'(0)=0, q'(T)=0

        a0 = start
        a1 = 0
        a2 = 3 * (goal - start) / (duration**2)
        a3 = -2 * (goal - start) / (duration**3)

        for i in range(num_steps + 1):
            t = i * dt
            point = a0 + a1*t + a2*(t**2) + a3*(t**3)
            trajectory.append(point)

        return np.array(trajectory)

    def quintic_polynomial(self, start, goal, duration, dt):
        """Generate very smooth trajectory using quintic polynomial"""
        num_steps = int(duration / dt)
        trajectory = []

        # Quintic: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Boundary: q(0)=start, q(T)=goal, q'(0)=0, q'(T)=0, q''(0)=0, q''(T)=0

        T = duration
        a0 = start
        a1 = 0
        a2 = 0
        a3 = 10 * (goal - start) / (T**3)
        a4 = -15 * (goal - start) / (T**4)
        a5 = 6 * (goal - start) / (T**5)

        for i in range(num_steps + 1):
            t = i * dt
            point = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
            trajectory.append(point)

        return np.array(trajectory)
```

### Cartesian Space Planning

```python
def plan_cartesian_path(arm, start_pos, goal_pos, num_waypoints=50):
    """
    Plan path in Cartesian space, then convert to joint space
    """
    # Generate Cartesian waypoints
    cartesian_path = []
    for i in range(num_waypoints + 1):
        t = i / num_waypoints
        waypoint = start_pos + t * (goal_pos - start_pos)
        cartesian_path.append(waypoint)

    # Convert to joint space using IK
    joint_path = []
    current_joints = np.zeros(7)  # Initial guess

    for waypoint in cartesian_path:
        joints, success = inverse_kinematics_dls(arm, waypoint, current_joints)

        if not success:
            print(f"IK failed at waypoint {waypoint}")
            return None

        joint_path.append(joints)
        current_joints = joints  # Use as next initial guess

    return np.array(joint_path)
```

## ROS 2 Integration

### Joint State Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Kinematics
        self.arm = HumanoidArm()
        self.trajectory_gen = TrajectoryGenerator()

        # Timer for control loop
        self.create_timer(0.01, self.control_loop)  # 100 Hz

        self.current_trajectory = None
        self.trajectory_index = 0

    def move_to_position(self, target_pos):
        """Move arm to target position"""
        # Get current joint angles
        current_joints = self.get_current_joints()

        # Compute IK
        goal_joints, success = inverse_kinematics_dls(
            self.arm,
            target_pos,
            current_joints
        )

        if not success:
            self.get_logger().error('IK failed')
            return

        # Generate trajectory
        self.current_trajectory = self.trajectory_gen.quintic_polynomial(
            current_joints,
            goal_joints,
            duration=2.0,
            dt=0.01
        )

        self.trajectory_index = 0

    def control_loop(self):
        """Execute trajectory"""
        if self.current_trajectory is None:
            return

        if self.trajectory_index >= len(self.current_trajectory):
            self.current_trajectory = None
            return

        # Get current waypoint
        joint_angles = self.current_trajectory[self.trajectory_index]

        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = joint_angles.tolist()

        self.joint_state_pub.publish(msg)

        self.trajectory_index += 1

    def get_current_joints(self):
        """Get current joint angles (placeholder)"""
        return np.zeros(7)
```

## Practical Example: Reaching Task

```python
class ReachingTask:
    def __init__(self):
        self.arm = HumanoidArm()
        self.controller = HumanoidController()

    def reach_object(self, object_position):
        """Reach for an object at given position"""
        # Add offset for hand
        target_pos = object_position + np.array([0, 0, 0.1])

        # Check if reachable
        distance = np.linalg.norm(target_pos)
        max_reach = self.arm.L1 + self.arm.L2 + self.arm.L3

        if distance > max_reach:
            print(f"Object out of reach: {distance:.2f}m > {max_reach:.2f}m")
            return False

        # Move to position
        self.controller.move_to_position(target_pos)

        return True

    def grasp_object(self):
        """Close gripper to grasp object"""
        # Send gripper command
        pass

    def place_object(self, place_position):
        """Place object at target position"""
        self.controller.move_to_position(place_position)
        # Open gripper
        pass
```

## Summary

- **Forward kinematics** computes end-effector pose from joint angles
- **Inverse kinematics** computes joint angles for desired pose
- **DH parameters** describe robot geometry
- **Jacobian** relates joint velocities to end-effector velocities
- **Numerical IK** (Jacobian-based, DLS) handles complex robots
- **Trajectory generation** creates smooth motion profiles
- **Whole-body kinematics** coordinates multiple limbs
- ROS 2 integration enables real-time control

In the next chapter, we'll explore **bipedal locomotion** for humanoid walking.
