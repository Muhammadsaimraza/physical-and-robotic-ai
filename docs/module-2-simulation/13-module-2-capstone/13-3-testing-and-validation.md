# Lesson 13.3: Testing & Validation

Your integrated system is running. Now it's time to validate its performance against our specification from Lesson 13.1.

Launch your system using `ros2 launch urdf_tutorial module2_capstone.launch.py`.

## Test Procedures

### Requirement 1 & 2: Robot and World
*   **Test:** Does the Gazebo simulation start? Do you see a world with a ground plane and a long wall? Does your two-wheeled robot spawn into the world?
*   **Verification:** Visual inspection of the Gazebo GUI.
*   **Result:** PASS / FAIL

### Requirement 3: Wall Follower Node Starts
*   **Test:** Does the `wall_follower_node` start correctly?
*   **Verification:** In a new terminal, run `ros2 node list`. You should see `/wall_follower_node` in the list, in addition to the Gazebo nodes.
*   **Result:** PASS / FAIL

### Requirement 4: Closed-Loop Control
*   **Test:** Does the robot move forward and attempt to maintain a constant distance from the wall on its right?
*   **Verification:**
    1.  Observe the robot's behavior in the Gazebo GUI. It should drive alongside the wall.
    2.  Use `ros2 topic echo /cmd_vel`. You should see a constant `linear.x` value and a fluctuating `angular.z` value as the controller makes corrections.
    3.  Use `ros2 topic echo /scan`. Look at the value for `ranges[90]`. Does it stay close to the `desired_distance` parameter you set in the launch file (e.g., 1.5 meters)? It will fluctuate, but it should hover around the target value.
*   **Result:** PASS / FAIL

### Requirement 5: Configurable Parameters
*   **Test:** Can we change the behavior by changing the launch parameters?
*   **Verification:**
    1.  Stop the launch file (`Ctrl+C`).
    2.  Edit `module2_capstone.launch.py`. Change the parameters, for example:
        ```python
        parameters=[
            {'desired_distance': 0.5}, # Closer to the wall
            {'forward_velocity': 0.1}, # Slower
            {'proportional_gain': 1.2}  # More aggressive turning
        ]
        ```
    3.  Re-launch the simulation.
    4.  Observe the robot's behavior. Does it now try to stay much closer to the wall? Is its movement slower and more twitchy (due to the higher gain)?
*   **Result:** PASS / FAIL

### Requirement 6: Single Launch File
*   **Test:** Does the entire system (Gazebo, robot spawn, controller node) start with a single command?
*   **Verification:** The `ros2 launch` command successfully starts everything.
*   **Result:** PASS

## Conclusion

If you have passed these validation steps, you have successfully completed Module 2. You have demonstrated an end-to-end understanding of robotics simulation:

*   You modeled a robot's physical form (URDF).
*   You created a world for it to live in (SDF).
*   You gave it senses (Gazebo sensor plugins).
*   You gave it a brain (a ROS 2 controller node).
*   You integrated and tested the complete system.

These are the fundamental skills of a robotics simulation engineer. The complexity of the robots and the worlds will grow, but the principles you have learned here will remain the same.

You are now well-prepared for the advanced topics in Module 3, where we will explore how to use high-fidelity simulators like NVIDIA Isaac Sim to train AI-based perception and control systems.
