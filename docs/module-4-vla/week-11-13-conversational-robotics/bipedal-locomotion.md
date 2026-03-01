# Bipedal Locomotion and Balance Control

## Introduction

**Bipedal locomotion** is one of the most challenging problems in robotics. Unlike wheeled or quadruped robots, bipedal humanoids must maintain balance on two legs while walking, requiring sophisticated control algorithms and real-time sensor feedback.

## Fundamentals of Bipedal Walking

### Gait Cycle

A complete walking cycle consists of two phases for each leg:

1. **Stance Phase** (60%): Foot is on the ground, supporting body weight
2. **Swing Phase** (40%): Foot is in the air, moving forward

**Key events:**
- **Heel Strike**: Foot contacts ground
- **Flat Foot**: Entire foot on ground
- **Heel Off**: Heel lifts, weight on toes
- **Toe Off**: Foot leaves ground
- **Mid Swing**: Foot at highest point
- **Heel Strike**: Cycle repeats

### Center of Mass (CoM) and Center of Pressure (CoP)

**Center of Mass (CoM)**: Average position of all mass in the robot
**Center of Pressure (CoP)**: Point where ground reaction force acts

**Stability condition**: CoP must remain within the support polygon (area defined by contact points with ground).

```python
import numpy as np

class BipedalRobot:
    def __init__(self, mass=50.0):
        self.mass = mass  # kg
        self.g = 9.81     # m/s^2
        self.com_height = 0.9  # m

    def compute_com(self, joint_positions, link_masses, link_com_positions):
        """
        Compute center of mass from joint configuration
        """
        total_mass = sum(link_masses)
        com = np.zeros(3)

        for i, (mass, pos) in enumerate(zip(link_masses, link_com_positions)):
            com += mass * pos

        com /= total_mass
        return com

    def compute_cop(self, left_foot_force, right_foot_force,
                    left_foot_pos, right_foot_pos):
        """
        Compute center of pressure from foot forces
        """
        total_force = left_foot_force + right_foot_force

        if total_force < 1.0:  # Threshold to avoid division by zero
            return None

        cop = (left_foot_force * left_foot_pos +
               right_foot_force * right_foot_pos) / total_force

        return cop

    def is_stable(self, com, cop, support_polygon):
        """
        Check if robot is statically stable
        CoP must be inside support polygon
        """
        # Project CoM to ground
        com_ground = com[:2]  # x, y only

        # Check if CoP is inside support polygon
        return self.point_in_polygon(cop[:2], support_polygon)

    def point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside
```

## Zero Moment Point (ZMP)

The **Zero Moment Point** is a point on the ground where the net moment from ground reaction forces is zero. For stable walking, ZMP must remain inside the support polygon.

```python
class ZMPController:
    def __init__(self):
        self.g = 9.81

    def compute_zmp(self, com_pos, com_vel, com_acc, com_height):
        """
        Compute ZMP from CoM dynamics
        ZMP = CoM_xy - (CoM_height / g) * CoM_acc_xy
        """
        zmp_x = com_pos[0] - (com_height / self.g) * com_acc[0]
        zmp_y = com_pos[1] - (com_height / self.g) * com_acc[1]

        return np.array([zmp_x, zmp_y, 0])

    def compute_desired_com_acc(self, com_pos, zmp_desired, com_height):
        """
        Compute desired CoM acceleration to achieve desired ZMP
        """
        acc_x = (self.g / com_height) * (com_pos[0] - zmp_desired[0])
        acc_y = (self.g / com_height) * (com_pos[1] - zmp_desired[1])

        return np.array([acc_x, acc_y, 0])
```

## Linear Inverted Pendulum Model (LIPM)

The LIPM simplifies the humanoid to a point mass at constant height, connected to the ground by a massless leg.

```python
class LIPM:
    def __init__(self, com_height=0.9, mass=50.0):
        self.h = com_height
        self.m = mass
        self.g = 9.81
        self.omega = np.sqrt(self.g / self.h)  # Natural frequency

    def dynamics(self, state, zmp):
        """
        LIPM dynamics: x_ddot = omega^2 * (x - zmp)
        state: [x, x_dot, y, y_dot]
        """
        x, x_dot, y, y_dot = state

        x_ddot = self.omega**2 * (x - zmp[0])
        y_ddot = self.omega**2 * (y - zmp[1])

        return np.array([x_dot, x_ddot, y_dot, y_ddot])

    def simulate_step(self, state, zmp, dt):
        """Simulate one time step using Euler integration"""
        state_dot = self.dynamics(state, zmp)
        new_state = state + state_dot * dt
        return new_state

    def predict_trajectory(self, initial_state, zmp_trajectory, dt):
        """Predict CoM trajectory given ZMP trajectory"""
        trajectory = [initial_state]
        state = initial_state.copy()

        for zmp in zmp_trajectory:
            state = self.simulate_step(state, zmp, dt)
            trajectory.append(state.copy())

        return np.array(trajectory)
```

## Footstep Planning

```python
class FootstepPlanner:
    def __init__(self):
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters
        self.step_height = 0.05 # meters
        self.step_duration = 0.8  # seconds

    def plan_forward_walk(self, num_steps, start_pos=[0, 0, 0]):
        """
        Plan footsteps for forward walking
        Returns list of footstep poses: [(x, y, z, yaw), ...]
        """
        footsteps = []
        current_pos = np.array(start_pos)

        # Start with left foot
        left_foot = True

        for i in range(num_steps):
            if left_foot:
                # Left foot steps forward
                step_pos = current_pos + np.array([
                    self.step_length,
                    self.step_width / 2,
                    0
                ])
            else:
                # Right foot steps forward
                step_pos = current_pos + np.array([
                    self.step_length,
                    -self.step_width / 2,
                    0
                ])

            footsteps.append({
                'position': step_pos,
                'orientation': 0.0,  # yaw
                'foot': 'left' if left_foot else 'right',
                'duration': self.step_duration
            })

            current_pos = step_pos
            left_foot = not left_foot

        return footsteps

    def plan_turn(self, angle, num_steps):
        """Plan footsteps for turning in place"""
        footsteps = []
        angle_per_step = angle / num_steps

        current_angle = 0
        left_foot = True

        for i in range(num_steps):
            current_angle += angle_per_step

            if left_foot:
                step_pos = np.array([0, self.step_width / 2, 0])
            else:
                step_pos = np.array([0, -self.step_width / 2, 0])

            footsteps.append({
                'position': step_pos,
                'orientation': current_angle,
                'foot': 'left' if left_foot else 'right',
                'duration': self.step_duration
            })

            left_foot = not left_foot

        return footsteps
```

## Trajectory Generation for Walking

```python
class WalkingTrajectoryGenerator:
    def __init__(self):
        self.lipm = LIPM()
        self.footstep_planner = FootstepPlanner()

    def generate_com_trajectory(self, footsteps, dt=0.01):
        """
        Generate CoM trajectory from footstep plan
        """
        com_trajectory = []
        zmp_trajectory = []

        for step in footsteps:
            # During single support, ZMP is at support foot
            num_samples = int(step['duration'] / dt)

            for i in range(num_samples):
                # ZMP at support foot
                if step['foot'] == 'left':
                    # Right foot is support
                    zmp = np.array([step['position'][0] - self.footstep_planner.step_length,
                                   -self.footstep_planner.step_width / 2])
                else:
                    # Left foot is support
                    zmp = np.array([step['position'][0] - self.footstep_planner.step_length,
                                   self.footstep_planner.step_width / 2])

                zmp_trajectory.append(zmp)

        # Simulate LIPM to get CoM trajectory
        initial_state = np.array([0, 0, 0, 0])  # [x, x_dot, y, y_dot]
        com_trajectory = self.lipm.predict_trajectory(
            initial_state,
            zmp_trajectory,
            dt
        )

        return com_trajectory, zmp_trajectory

    def generate_swing_foot_trajectory(self, start_pos, end_pos, duration, dt=0.01):
        """
        Generate swing foot trajectory (parabolic arc)
        """
        num_samples = int(duration / dt)
        trajectory = []

        for i in range(num_samples):
            t = i / num_samples

            # Linear interpolation in x and y
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])

            # Parabolic arc in z
            z = 4 * self.footstep_planner.step_height * t * (1 - t)

            trajectory.append(np.array([x, y, z]))

        return np.array(trajectory)
```

## Balance Control

### PID Controller for Balance

```python
class BalanceController:
    def __init__(self):
        # PID gains
        self.kp = 100.0  # Proportional
        self.ki = 10.0   # Integral
        self.kd = 20.0   # Derivative

        # State
        self.error_integral = np.zeros(2)
        self.last_error = np.zeros(2)

    def compute_ankle_torque(self, com_pos, com_vel, zmp_desired, dt):
        """
        Compute ankle torque to maintain balance
        """
        # Current ZMP (simplified)
        zmp_current = com_pos[:2]

        # Error
        error = zmp_desired[:2] - zmp_current

        # Integral
        self.error_integral += error * dt

        # Derivative
        error_derivative = (error - self.last_error) / dt
        self.last_error = error

        # PID control
        torque = (self.kp * error +
                 self.ki * self.error_integral +
                 self.kd * error_derivative)

        return torque

    def reset(self):
        """Reset controller state"""
        self.error_integral = np.zeros(2)
        self.last_error = np.zeros(2)
```

### Model Predictive Control (MPC)

```python
import cvxpy as cp

class MPCBalanceController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.lipm = LIPM()

    def solve(self, current_state, zmp_reference):
        """
        Solve MPC problem to find optimal ZMP trajectory
        current_state: [x, x_dot, y, y_dot]
        zmp_reference: desired ZMP trajectory (horizon x 2)
        """
        # Decision variables
        zmp = cp.Variable((self.horizon, 2))
        com = cp.Variable((self.horizon + 1, 4))

        # Cost function
        cost = 0
        for t in range(self.horizon):
            # Track reference ZMP
            cost += cp.sum_squares(zmp[t] - zmp_reference[t])

            # Minimize CoM velocity
            cost += 0.1 * cp.sum_squares(com[t, [1, 3]])

        # Constraints
        constraints = []

        # Initial condition
        constraints.append(com[0] == current_state)

        # Dynamics
        for t in range(self.horizon):
            # LIPM dynamics (linearized)
            omega_sq = self.lipm.omega ** 2

            # x direction
            constraints.append(
                com[t+1, 0] == com[t, 0] + self.dt * com[t, 1]
            )
            constraints.append(
                com[t+1, 1] == com[t, 1] + self.dt * omega_sq * (com[t, 0] - zmp[t, 0])
            )

            # y direction
            constraints.append(
                com[t+1, 2] == com[t, 2] + self.dt * com[t, 3]
            )
            constraints.append(
                com[t+1, 3] == com[t, 3] + self.dt * omega_sq * (com[t, 2] - zmp[t, 1])
            )

        # ZMP constraints (support polygon)
        for t in range(self.horizon):
            constraints.append(zmp[t, 0] >= -0.1)
            constraints.append(zmp[t, 0] <= 0.1)
            constraints.append(zmp[t, 1] >= -0.05)
            constraints.append(zmp[t, 1] <= 0.05)

        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status == cp.OPTIMAL:
            return zmp.value, com.value
        else:
            return None, None
```

## Whole-Body Control

```python
class WholeBodyController:
    def __init__(self, robot):
        self.robot = robot
        self.balance_controller = BalanceController()

    def compute_joint_torques(self, desired_com_acc, desired_foot_forces):
        """
        Compute joint torques to achieve desired CoM acceleration
        and foot forces using inverse dynamics
        """
        # Get robot state
        q = self.robot.get_joint_positions()
        q_dot = self.robot.get_joint_velocities()

        # Compute Jacobians
        J_com = self.robot.compute_com_jacobian(q)
        J_left_foot = self.robot.compute_foot_jacobian(q, 'left')
        J_right_foot = self.robot.compute_foot_jacobian(q, 'right')

        # Desired joint accelerations (from CoM acceleration)
        q_ddot_desired = np.linalg.pinv(J_com) @ desired_com_acc

        # Compute torques using inverse dynamics
        M = self.robot.compute_mass_matrix(q)
        C = self.robot.compute_coriolis_matrix(q, q_dot)
        G = self.robot.compute_gravity_vector(q)

        # Torque = M*q_ddot + C*q_dot + G - J^T*F
        tau = (M @ q_ddot_desired + C @ q_dot + G -
               J_left_foot.T @ desired_foot_forces['left'] -
               J_right_foot.T @ desired_foot_forces['right'])

        return tau
```

## ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

class BipedalWalkingController(Node):
    def __init__(self):
        super().__init__('bipedal_walking_controller')

        # Controllers
        self.footstep_planner = FootstepPlanner()
        self.trajectory_gen = WalkingTrajectoryGenerator()
        self.balance_controller = BalanceController()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # State
        self.current_velocity = np.zeros(2)
        self.current_orientation = np.zeros(3)
        self.walking = False

        # Control loop
        self.create_timer(0.01, self.control_loop)

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.current_velocity[0] = msg.linear.x
        self.current_velocity[1] = msg.angular.z

        if np.linalg.norm(self.current_velocity) > 0.01:
            self.start_walking()
        else:
            self.stop_walking()

    def imu_callback(self, msg):
        """Receive IMU data for balance"""
        # Extract orientation
        orientation = msg.orientation
        # Convert to Euler angles
        # self.current_orientation = ...

    def start_walking(self):
        """Start walking behavior"""
        if not self.walking:
            self.get_logger().info('Starting walking')
            self.walking = True

            # Plan footsteps based on velocity
            num_steps = 10
            self.footsteps = self.footstep_planner.plan_forward_walk(num_steps)

            # Generate trajectories
            self.com_trajectory, self.zmp_trajectory = \
                self.trajectory_gen.generate_com_trajectory(self.footsteps)

            self.trajectory_index = 0

    def stop_walking(self):
        """Stop walking behavior"""
        if self.walking:
            self.get_logger().info('Stopping walking')
            self.walking = False

    def control_loop(self):
        """Main control loop"""
        if not self.walking:
            return

        if self.trajectory_index >= len(self.com_trajectory):
            self.stop_walking()
            return

        # Get desired CoM state
        desired_com_state = self.com_trajectory[self.trajectory_index]
        desired_zmp = self.zmp_trajectory[self.trajectory_index]

        # Compute balance control
        ankle_torque = self.balance_controller.compute_ankle_torque(
            desired_com_state[:2],
            desired_com_state[2:],
            desired_zmp,
            0.01
        )

        # Convert to joint commands
        joint_commands = self.compute_joint_commands(
            desired_com_state,
            ankle_torque
        )

        # Publish
        msg = Float64MultiArray()
        msg.data = joint_commands.tolist()
        self.joint_cmd_pub.publish(msg)

        self.trajectory_index += 1

    def compute_joint_commands(self, com_state, ankle_torque):
        """Convert high-level commands to joint-level commands"""
        # Placeholder - would use inverse kinematics
        return np.zeros(12)  # 6 DOF per leg

def main():
    rclpy.init()
    controller = BipedalWalkingController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

## Practical Example: Simple Walking Gait

```python
class SimpleWalkingGait:
    def __init__(self):
        self.step_length = 0.2
        self.step_height = 0.05
        self.step_duration = 0.8
        self.phase = 0.0

    def update(self, dt):
        """Update gait phase"""
        self.phase += dt / self.step_duration
        if self.phase >= 1.0:
            self.phase = 0.0

    def get_foot_positions(self):
        """Get left and right foot positions for current phase"""
        # Simple sinusoidal gait
        left_x = self.step_length * np.sin(2 * np.pi * self.phase)
        left_z = self.step_height * np.sin(np.pi * self.phase) if self.phase < 0.5 else 0

        right_x = self.step_length * np.sin(2 * np.pi * (self.phase + 0.5))
        right_z = self.step_height * np.sin(np.pi * (self.phase + 0.5)) if self.phase >= 0.5 else 0

        return {
            'left': np.array([left_x, 0.1, left_z]),
            'right': np.array([right_x, -0.1, right_z])
        }
```

## Summary

- **Bipedal walking** requires maintaining balance while moving
- **ZMP** must stay inside support polygon for stability
- **LIPM** provides simplified model for planning
- **Footstep planning** determines where to place feet
- **Trajectory generation** creates smooth CoM and foot paths
- **Balance control** uses feedback to maintain stability
- **MPC** optimizes future trajectory for robust walking
- **Whole-body control** coordinates all joints
- ROS 2 integration enables real-time control

In the next chapter, we'll explore **conversational robotics** using LLMs for natural language interaction.
