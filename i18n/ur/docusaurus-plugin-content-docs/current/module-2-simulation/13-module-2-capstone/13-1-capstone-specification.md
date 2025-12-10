# Lesson 13.1: The Specification

As in the Module 1 capstone, we begin by writing a clear and unambiguous technical specification.

## High-Level Requirements

1.  The system shall consist of a simulated two-wheeled robot in a world with walls.
2.  The robot shall be equipped with a 360-degree LiDAR sensor.
3.  A "wall follower" node shall control the robot.
4.  The wall follower node shall cause the robot to move forward while maintaining a configurable distance from a wall to its right.
5.  The entire system shall be started with a single launch file.

## System Components

1.  **Gazebo World:** An SDF file containing a world with at least one long, straight wall for the robot to follow.
2.  **Robot Model:** A URDF file for a two-wheeled robot, including a LiDAR sensor plugin.
3.  **Wall Follower Node:** A Python ROS 2 node that implements the control logic.
4.  **Launch File:** A Python launch file to start and configure the system.

## Node-by-Node Specification

### 1. `wall_follower_node`

*   **Node Name:** `wall_follower_node`
*   **Purpose:** Subscribes to LiDAR data and publishes velocity commands to follow a wall.
*   **Parameters:**
    *   `desired_distance` (float, default: 1.0): The target distance to maintain from the wall (in meters).
    *   `forward_velocity` (float, default: 0.2): The constant forward speed of the robot.
    *   `proportional_gain` (float, default: 0.5): The 'k' value for the proportional controller that determines the turning rate.
*   **Topics Subscribed:**
    *   `/scan` (`sensor_msgs/msg/LaserScan`): The topic the LiDAR plugin is publishing on.
*   **Topics Published:**
    *   `/cmd_vel` (`geometry_msgs/msg/Twist`): The topic the differential drive plugin is listening to.
*   **Behavior:**
    1.  The node runs a continuous control loop (triggered by the reception of a `/scan` message).
    2.  In the loop, it reads the distance measurement from the ray pointing directly to the right (the 90-degree or -π/2 radians index of the `ranges` array).
    3.  It calculates the `error = desired_distance - measured_distance`.
    4.  It calculates the `angular_velocity = proportional_gain * error`.
    5.  It publishes a `Twist` message with `linear.x` set to `forward_velocity` and `angular.z` set to the calculated `angular_velocity`.

### 2. Gazebo Plugins (within URDF)

*   **LiDAR Plugin:**
    *   Must be configured to publish on the `/scan` topic.
    *   Must have a 360-degree horizontal field of view.
*   **Differential Drive Plugin:**
    *   Must be configured to subscribe to the `/cmd_vel` topic.
    *   `wheel_separation` and `wheel_radius` must accurately match the URDF's geometry.

## Launch File Specification

*   **Name:** `module2_capstone.launch.py`
*   **Actions:**
    1.  Start Gazebo with the specified world file.
    2.  Spawn the robot URDF model into Gazebo using the `ros_gz_sim` create node.
    3.  Start the `wall_follower_node`.
    4.  Pass the default parameter values (`desired_distance`, etc.) to the `wall_follower_node`.

This specification provides a complete blueprint for our system. In the next lesson, you will implement the world, the robot, and the launch file.
