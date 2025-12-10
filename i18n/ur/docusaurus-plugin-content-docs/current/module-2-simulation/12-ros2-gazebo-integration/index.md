# Chapter 12: ROS 2 + Gazebo Integration

You have a simulated robot in a simulated world, and you have a collection of ROS 2 nodes. How do you get them to talk to each other?

The answer is the **ROS-Gazebo bridge**. This is a dedicated ROS 2 node that acts as a translator, converting ROS 2 messages into Gazebo messages and vice-versa. The plugins you have been using (like the camera and diff drive plugins) are actually convenience wrappers around this bridge.

In this chapter, you will learn how the bridge works and how to use it to create a closed-loop control system for your simulated robot.

## Learning Objectives
By the end of this chapter, you will be able to:

*   Understand the role of the `ros_gz_bridge`.
*   Use the bridge to pass messages between ROS 2 and Gazebo.
*   Write a simple ROS 2 node that subscribes to sensor data from Gazebo and publishes control commands back.
*   Implement a closed-loop "wall follower" robot.

## Lessons
*   **Lesson 12.1: The `ros_gz_bridge`**
*   **Lesson 12.2: Spawning Robots** (Review)
*   **Lesson 12.3: Closed-Loop Control**
*   **Lesson 12.4: Creating Skills**
