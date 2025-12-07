# Lesson 21.1: Capstone Specification

For this capstone, we will define a "fetch" task for a simulated humanoid robot.

## High-Level Requirements

1.  The system shall be capable of receiving a natural language voice command from a human user.
2.  The system shall use an LLM to interpret the command and generate a structured plan.
3.  The robot shall navigate to a specified location in a simulated environment.
4.  The robot shall use its perception system to identify a target object.
5.  The robot shall manipulate the object (e.g., pick it up, push it).
6.  The entire system shall be launched with a single command.

## Scenario

**Robot:** A simplified simulated humanoid (e.g., our two-wheeled robot with a camera, LiDAR, and a simple gripper arm added).
**Environment:** A simulated indoor environment in Isaac Sim with furniture and objects.
**Task:** The user says, "Go to the kitchen and bring me the red mug."

## System Components

1.  **Speech-to-Text Node:** Converts human speech to text.
2.  **LLM Planner Node:** Interprets text commands, generates JSON plans.
3.  **Action Executor Node:** Translates JSON plans into ROS 2 actions/services.
4.  **Navigation Stack:** Uses VSLAM and Nav2 for autonomous movement.
5.  **Perception Stack:** Uses camera and object detection for finding objects.
6.  **Manipulation Stack:** Uses MoveIt2 for arm control.
7.  **Simulation:** Isaac Sim with a humanoid model and a realistic environment.

## Node-by-Node Specification

### 1. `voice_command_node`
*   **Purpose:** Captures audio, sends to STT, publishes text.
*   **Topics Published:** `/voice_command_text` (`std_msgs/msg/String`).

### 2. `llm_planner_node`
*   **Purpose:** Subscribes to text commands, prompts LLM, publishes JSON plan.
*   **Topics Subscribed:** `/voice_command_text` (`std_msgs/msg/String`).
*   **Topics Published:** `/robot_plan` (`std_msgs/msg/String` - containing JSON).
*   **Behavior:** Uses a pre-defined prompt to query an external LLM API (e.g., OpenAI, Gemini).

### 3. `action_executor_node`
*   **Purpose:** Subscribes to JSON plans, calls appropriate ROS 2 actions/services.
*   **Topics Subscribed:** `/robot_plan` (`std_msgs/msg/String`).
*   **Services Called:** Nav2 `/navigate_to_pose`, MoveIt2 `/move_group/action`.
*   **Behavior:** Parses JSON, orchestrates calls to navigation, perception, and manipulation.

### 4. Navigation Stack (Nav2 + Isaac ROS VSLAM)
*   **Purpose:** Autonomous movement to target locations.
*   **Inputs:** Camera, IMU data (from Isaac Sim). Goal pose (from `action_executor_node`).
*   **Outputs:** `/cmd_vel` (`geometry_msgs/msg/Twist`).

### 5. Perception Stack (Object Detection)
*   **Purpose:** Locates specified objects.
*   **Inputs:** Camera data (from Isaac Sim).
*   **Services Provided:** `/find_object` (`my_custom_interfaces/srv/FindObject`).
*   **Behavior:** Runs a pre-trained object detection model.

### 6. Manipulation Stack (MoveIt2)
*   **Purpose:** Plans and executes arm movements for grasping.
*   **Inputs:** Target object pose (from `action_executor_node`).
*   **Services Provided:** `/pickup_object` (`my_custom_interfaces/srv/PickupObject`).
*   **Behavior:** Uses MoveIt2 to plan and execute a grasp.

## Launch File Specification (`humanoid_vla_capstone.launch.py`)

*   Start Isaac Sim.
*   Spawn the humanoid robot model.
*   Launch `voice_command_node`.
*   Launch `llm_planner_node`.
*   Launch `action_executor_node`.
*   Launch Nav2 stack (including VSLAM, local and global planners).
*   Launch object detection node.
*   Launch MoveIt2 planning and execution nodes.

This is a comprehensive system. While the full implementation is beyond the scope of this lesson, the specification lays out the entire architecture. In the next lessons, we'll focus on the high-level integration points.
