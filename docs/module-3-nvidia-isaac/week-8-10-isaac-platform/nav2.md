# Nav2: Navigation for Humanoid Robots

## Introduction

**Nav2 (Navigation2)** is the ROS 2 navigation stack that enables autonomous mobile robots to navigate from point A to point B while avoiding obstacles. While originally designed for wheeled robots, Nav2 can be adapted for bipedal humanoid robots with careful configuration.

## Nav2 Architecture

Nav2 consists of several interconnected servers:

1. **Map Server**: Provides static maps
2. **AMCL (Localization)**: Adaptive Monte Carlo Localization
3. **Planner Server**: Global path planning
4. **Controller Server**: Local trajectory following
5. **Recoveries Server**: Behavior recovery actions
6. **Behavior Tree Navigator**: Coordinates navigation behaviors
7. **Waypoint Follower**: Follows sequences of waypoints
8. **Lifecycle Manager**: Manages node lifecycles

```
Goal → BT Navigator → Planner → Controller → cmd_vel
         ↑              ↑          ↑
         |              |          |
      AMCL ←─── Map ───┴──────────┘
```

## Installation

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install visualization tools
sudo apt install ros-humble-rviz2 ros-humble-nav2-rviz-plugins

# Install SLAM Toolbox (for mapping)
sudo apt install ros-humble-slam-toolbox
```

## Creating a Map

### Using SLAM Toolbox

```bash
# Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Drive robot around to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

This creates two files:
- `my_map.yaml`: Map metadata
- `my_map.pgm`: Occupancy grid image

**Map YAML format:**

```yaml
image: my_map.pgm
resolution: 0.05  # meters per pixel
origin: [-10.0, -10.0, 0.0]  # x, y, yaw
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## Nav2 Configuration

### Basic Configuration File

Create `config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    # DWB controller (Dynamic Window Approach)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vth_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "my_map.yaml"

amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

## Launch File

Create `launch/navigation.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('my_robot_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_dir, 'maps', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'))

    # Map server
    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        map_server_cmd,
        rviz_cmd
    ])
```

## Using Nav2 Programmatically

### Simple Navigation Client

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.navigator = BasicNavigator()

        # Wait for Nav2 to be ready
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')

    def navigate_to_pose(self, x, y, yaw):
        """Navigate to a specific pose"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)

        self.get_logger().info(f'Navigating to ({x}, {y}, {yaw})')
        self.navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                distance = feedback.distance_remaining
                self.get_logger().info(f'Distance remaining: {distance:.2f}m')

            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached!')
            return True
        else:
            self.get_logger().error('Navigation failed!')
            return False

    def follow_waypoints(self, waypoints):
        """Follow a sequence of waypoints"""
        goal_poses = []

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)
            goal_poses.append(pose)

        self.navigator.followWaypoints(goal_poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_waypoint = feedback.current_waypoint
                self.get_logger().info(f'Executing waypoint {current_waypoint + 1}/{len(waypoints)}')

            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All waypoints reached!')
            return True
        else:
            self.get_logger().error('Waypoint following failed!')
            return False

def main():
    rclpy.init()
    nav_client = NavigationClient()

    # Navigate to single goal
    nav_client.navigate_to_pose(2.0, 1.0, 0.0)

    # Follow waypoints
    waypoints = [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 1.57),
        (1.0, 2.0, 3.14),
        (0.0, 0.0, 0.0)
    ]
    nav_client.follow_waypoints(waypoints)

    nav_client.destroy_node()
    rclpy.shutdown()
```

## Adapting Nav2 for Humanoid Robots

### Footprint Configuration

Humanoid robots have different footprints than wheeled robots:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Use polygon footprint for humanoid
      footprint: "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"
      # Or use circular approximation
      # robot_radius: 0.25
```

### Custom Controller for Bipedal Motion

```python
from nav2_core import Controller
from geometry_msgs.msg import TwistStamped

class BipedalController(Controller):
    def __init__(self):
        super().__init__()
        self.max_linear_vel = 0.3  # Slower for stability
        self.max_angular_vel = 0.5

    def computeVelocityCommands(self, pose, velocity, goal_checker):
        """Compute velocity commands for bipedal robot"""
        # Get path from global planner
        path = self.getPlan()

        # Find closest point on path
        closest_point = self.find_closest_point(pose, path)

        # Calculate desired velocity
        cmd_vel = TwistStamped()

        # Linear velocity (forward/backward)
        distance_to_goal = self.distance(pose, path[-1])
        if distance_to_goal > 0.5:
            cmd_vel.twist.linear.x = self.max_linear_vel
        else:
            # Slow down near goal
            cmd_vel.twist.linear.x = self.max_linear_vel * (distance_to_goal / 0.5)

        # Angular velocity (turning)
        angle_to_path = self.angle_to_path(pose, closest_point)
        cmd_vel.twist.angular.z = np.clip(
            angle_to_path * 2.0,
            -self.max_angular_vel,
            self.max_angular_vel
        )

        # Add stability constraints
        if abs(cmd_vel.twist.angular.z) > 0.3:
            # Reduce linear velocity when turning
            cmd_vel.twist.linear.x *= 0.5

        return cmd_vel

    def find_closest_point(self, pose, path):
        """Find closest point on path to current pose"""
        min_dist = float('inf')
        closest = None

        for point in path:
            dist = self.distance(pose, point)
            if dist < min_dist:
                min_dist = dist
                closest = point

        return closest

    def distance(self, pose1, pose2):
        """Calculate Euclidean distance"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return np.sqrt(dx**2 + dy**2)

    def angle_to_path(self, pose, target):
        """Calculate angle to target point"""
        dx = target.position.x - pose.position.x
        dy = target.position.y - pose.position.y
        target_angle = np.arctan2(dy, dx)

        # Get current yaw from quaternion
        current_yaw = self.quaternion_to_yaw(pose.orientation)

        # Calculate angle difference
        angle_diff = target_angle - current_yaw

        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        return angle_diff
```

## Behavior Trees

Nav2 uses behavior trees for complex navigation logic. You can customize the behavior tree:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <SequenceStar name="RecoveryActions">
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </SequenceStar>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Practical Example: Humanoid Patrol

```python
class HumanoidPatrol(Node):
    def __init__(self):
        super().__init__('humanoid_patrol')
        self.navigator = BasicNavigator()

        # Define patrol waypoints
        self.patrol_points = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57)
        ]

        self.current_waypoint = 0

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

    def patrol(self):
        """Execute patrol behavior"""
        while rclpy.ok():
            # Get next waypoint
            x, y, yaw = self.patrol_points[self.current_waypoint]

            # Create goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.z = np.sin(yaw / 2.0)
            goal.pose.orientation.w = np.cos(yaw / 2.0)

            # Navigate
            self.get_logger().info(f'Patrolling to waypoint {self.current_waypoint}')
            self.navigator.goToPose(goal)

            # Wait for completion
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)

            # Check result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Waypoint reached')
                # Move to next waypoint
                self.current_waypoint = (self.current_waypoint + 1) % len(self.patrol_points)
            else:
                self.get_logger().error('Navigation failed, retrying...')

            # Pause at waypoint
            time.sleep(2.0)

def main():
    rclpy.init()
    patrol = HumanoidPatrol()
    patrol.patrol()
    patrol.destroy_node()
    rclpy.shutdown()
```

## Summary

- **Nav2** provides autonomous navigation for ROS 2 robots
- **Behavior trees** coordinate complex navigation behaviors
- **Costmaps** represent obstacles and free space
- **AMCL** provides localization using particle filters
- **Planners** compute global paths to goals
- **Controllers** follow paths while avoiding obstacles
- **Recoveries** handle stuck situations
- Humanoid robots require custom controllers for bipedal motion
- Nav2 integrates seamlessly with Isaac ROS for perception

In the next module, we'll explore **Vision-Language-Action (VLA)** models for natural language control of humanoid robots.
