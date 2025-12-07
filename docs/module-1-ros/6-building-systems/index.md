# Chapter 6: Building Robot Systems

You can now write individual ROS 2 nodes that communicate using topics and services. But a real robot is not one or two nodes; it's a complex system of dozens of nodes all working together. How do you start, configure, and manage such a system?

In this chapter, you will learn to use **parameters** to make your nodes configurable and **launch files** to start and orchestrate your entire multi-node system with a single command. This is the bridge from writing individual components to designing and integrating complete robotic applications.

*   **Duration:** 3 lessons, 3 hours total
*   **Layer Breakdown:** L3: Intelligence Design (Recognizing and using system-level patterns)
*   **Hardware Tier:** Tier 1 (Cloud ROS 2 environment or local install)
*   **Prerequisites:** Completion of Chapter 5

## Learning Objectives
By the end of this chapter, you will be able to:

*   Declare and use parameters within a Python node.
*   Create a ROS 2 launch file in Python.
*   Use a launch file to start multiple nodes simultaneously.
*   Pass parameters to nodes from a launch file.
*   Use `ros2doctor` and `rqt_graph` to debug a multi-node system.

## Lessons
*   **Lesson 6.1: Parameters** (60 minutes)
    *   Learn how to make your nodes configurable from the outside world without changing the code.

*   **Lesson 6.2: Launch Files** (60 minutes)
    *   Write a Python launch script to start your talker/listener system with a single command.

*   **Lesson 6.3: Debugging Systems** (60 minutes)
    *   Learn to use graphical and command-line tools to visualize and debug your running system.
