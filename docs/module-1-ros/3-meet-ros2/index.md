# Chapter 3: Meet ROS 2

The theory is over. It's time to get your hands dirty. In this chapter, you will set up your ROS 2 environment and take your first steps into the world of robotics middleware. You won't be writing any Python code just yet. Instead, you'll use the powerful `ros2` command-line interface (CLI) to explore a live ROS 2 system, control a simulated robot, and inspect how different components communicate with each other.

This chapter is about building intuition. By the end, you'll have a feel for how a ROS 2 system is structured and how to use its powerful built-in tools for introspection and debugging.

*   **Duration:** 4 lessons, 4 hours total
*   **Layer Breakdown:** L1→L2 (You'll be given commands to run, but also prompts to explore on your own with AI assistance)
*   **Hardware Tier:** Tier 1 (Cloud ROS 2 environment like The Construct, or a local installation)
*   **Prerequisites:** Completion of Chapter 2

## Learning Objectives
By the end of this chapter, you will be able to:

*   Navigate a ROS 2 environment using the command line.
*   Use the `ros2 run` command to start a node and control a simulated robot (Turtlesim).
*   Use `ros2 topic list`, `ros2 topic echo`, and `ros2 topic pub` to inspect and publish messages.
*   Use `ros2 service list`, `ros2 service info`, and `ros2 service call` to interact with services.
*   Use `ros2 param list` and `ros2 param set` to inspect and change node parameters.

## Lessons
*   **Lesson 3.1: Environment Setup** (60 minutes)
    *   Get your ROS 2 environment running, either through a cloud platform or by installing it locally.

*   **Lesson 3.2: Your First Robot (Turtlesim)** (60 minutes)
    *   Meet Turtlesim, the "Hello World" of robotics simulators, and learn to control it from the command line.

*   **Lesson 3.3: Nodes & Topics** (60 minutes)
    *   Dissect the Turtlesim system, identify its nodes, and "spy" on the topic messages that control the robot's movement.

*   **Lesson 3.4: Services & Parameters** (60 minutes)
    *   Go deeper by calling services to manipulate the simulation and changing parameters to alter the robot's appearance.
