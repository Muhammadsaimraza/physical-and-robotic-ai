# Chapter 11: Sensors in Simulation

A robot is blind without its sensors. A key advantage of a simulator like Gazebo is the ability to model a wide variety of sensors and have them produce realistic data, just as a physical sensor would.

In Gazebo, a sensor is a **plugin** that you attach to a link in your robot's model. This plugin generates data based on the state of the simulated world and publishes it to a ROS 2 topic.

In this chapter, you will learn how to add the most common types of sensors—cameras, LiDAR, and IMUs—to your simulated robot.

## Learning Objectives
By the end of this chapter, you will be able to:

*   Add a camera sensor to a URDF and view its image feed in ROS 2.
*   Add a LiDAR sensor and visualize its point cloud data.
*   Add an IMU sensor to get orientation data.
*   Use RViz and other tools to debug your simulated sensor data.

## Lessons
*   **Lesson 11.1: Camera Simulation**
*   **Lesson 11.2: LiDAR Simulation**
*   **Lesson 11.3: IMU & Contact Sensors**
*   **Lesson 11.4: Sensors Debugging**
