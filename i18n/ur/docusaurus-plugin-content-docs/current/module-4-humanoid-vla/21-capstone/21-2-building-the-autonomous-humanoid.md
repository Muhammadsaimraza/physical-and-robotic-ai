# Lesson 21.2: Building the Autonomous Humanoid (High-Level Integration)

Building the full autonomous humanoid outlined in the specification involves integrating a large number of complex components. This lesson focuses on how you would connect these pieces at a high level.

## 1. Simulated Humanoid in Isaac Sim

*   **Model:** Use a pre-existing humanoid model from Isaac Sim's assets (e.g., Carter, Franka Emika Panda with an integrated mobile base, or a more complex bipedal humanoid if available).
*   **Sensors:** Configure the model in Isaac Sim to include:
    *   **RGB-D Camera:** For perception (object detection, VSLAM).
    *   **IMU:** For localization and balance.
    *   **Joint State Publishers:** To provide joint angles for kinematics.
*   **Actuators:** Configure the robot's joints to be controllable via ROS 2 topics (e.g., `JointState` messages or directly to `JointController` action servers if using `ros_control`).

## 2. Setting Up the VLA Pipeline (Voice-to-Action)

### a. Speech-to-Text Node (`voice_command_node`)
*   **Input:** Microphone audio stream.
*   **Process:** Use a Python node with a library like `SpeechRecognition` to send audio to a cloud-based STT service (e.g., Google Speech-to-Text, OpenAI Whisper API).
*   **Output:** Publishes `std_msgs/msg/String` to `/voice_command_text`.

### b. LLM Planner Node (`llm_planner_node`)
*   **Input:** Subscribes to `/voice_command_text`.
*   **Process:** Formulates a prompt incorporating the available robot actions and current environment context. Sends the prompt to an LLM API (e.g., OpenAI, Gemini). Parses the JSON response.
*   **Output:** Publishes `std_msgs/msg/String` (JSON plan) to `/robot_plan`.

### c. Action Executor Node (`action_executor_node`)
*   **Input:** Subscribes to `/robot_plan`.
*   **Process:**
    *   **GOTO:** Creates a `NavigateToPose` goal and sends it to the Nav2 Action Server (`/navigate_to_pose`).
    *   **FIND:** Calls a custom service `/find_object` from the Perception Stack, passing the object name.
    *   **PICKUP/DELIVER:** Creates a `MoveGroupGoal` for MoveIt2 (target pose for gripper) and sends it to the `/move_group` Action Server.
*   **Output:** Orchestrates calls to various ROS 2 action/service servers.

## 3. Navigation Stack

*   **Isaac ROS VSLAM Node:** Subscribes to stereo camera and IMU data from Isaac Sim (bridged to ROS 2). Publishes `/tf` (odom to base_link) and `/map` (point cloud).
*   **Nav2 Stack:** Launches the `amcl`, `global_planner`, `local_planner`, and `controller_server` nodes. Configured to use the VSLAM outputs for localization and mapping. Subscribes to goals from `action_executor_node`. Publishes `/cmd_vel` to the robot's base controller.

## 4. Perception Stack

*   **Object Detection Node:** Subscribes to camera images from Isaac Sim. Runs a pre-trained object detection model (e.g., YOLO, EfficientDet) on the GPU (potentially using Isaac ROS `detectnet_v2`).
*   **`find_object` Service:** Provides a service interface to query for the location of a detected object. When called, it uses the results of the object detection model to find the 3D pose of the requested object.

## 5. Manipulation Stack (MoveIt2)

*   **MoveIt2 Nodes:** The MoveIt Setup Assistant (from Chapter 19) is used to generate the configuration for your humanoid arm. This launches `move_group`, `rviz`, and other necessary nodes.
*   **`pickup_object` Service:** This custom service is integrated with MoveIt2. When called, it:
    1.  Calculates a pre-grasp, grasp, and post-grasp pose for the target object.
    2.  Sends these poses as goals to the `/move_group` action server.
    3.  Commands the gripper to open/close.

## Orchestration with a Master Launch File

The entire system needs to be launched with a single command. This involves a complex Python launch file (similar to your Module 2 capstone, but much larger).

```python
# humanoid_vla_capstone.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
# ... other imports for Isaac Sim, Nav2, MoveIt2 ...

def generate_launch_description():
    # 1. Launch Isaac Sim (as a separate process or include an Isaac Sim launch file)
    isaac_sim_launch = ExecuteProcess(...) # Or IncludeLaunchDescription for Isaac Sim

    # 2. Launch Navigation (Nav2 with Isaac ROS VSLAM)
    nav2_launch = IncludeLaunchDescription(...) # Use a pre-configured Nav2 launch file

    # 3. Launch Perception (Object Detection node, find_object service)
    perception_launch = Node(...) # Your object detection node

    # 4. Launch Manipulation (MoveIt2)
    moveit_launch = IncludeLaunchDescription(...) # Your robot's MoveIt2 launch file

    # 5. Launch VLA Pipeline nodes
    voice_command_node = Node(...)
    llm_planner_node = Node(...)
    action_executor_node = Node(...)

    return LaunchDescription([
        isaac_sim_launch,
        nav2_launch,
        perception_launch,
        moveit_launch,
        voice_command_node,
        llm_planner_node,
        action_executor_node,
    ])
```
This is a high-level overview. Each component itself is a complex system. The goal of this capstone is to understand the full system architecture and how these pieces fit together. In the final lesson, we will discuss how to test such a complex system.
