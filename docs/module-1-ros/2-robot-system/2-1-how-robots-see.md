# Lesson 2.1: How Robots See (Sensors)

A robot's understanding of the world is only as good as the data it receives. **Sensors** are the hardware components that convert physical phenomena from the environment into electrical signals that the robot's brain can process. They are the robot's eyes, ears, and sense of balance.

We can group most robot sensors into three broad categories:

### 1. Proprioceptive Sensors: The Sense of Self

These sensors measure the internal state of the robot itself.
*   **Joint Encoders:** These are the most fundamental proprioceptive sensors. They are attached to each motor and measure the precise angle of the joint. Without encoders, the robot would not know the position of its own limbs.
*   **Inertial Measurement Units (IMUs):** An IMU is a small chip that typically contains an accelerometer (to measure linear acceleration) and a gyroscope (to measure rotational velocity). By combining these readings, an IMU can estimate the robot's orientation (roll, pitch, yaw) and track its movement. It is the robot's sense of balance, crucial for walking and maintaining stability.

### 2. Exteroceptive Sensors: The Sense of the World

These sensors measure the external environment.
*   **Cameras:** The most common and information-rich sensor. A standard RGB camera provides a 2D color image of the world, just like our own eyes. This data is the foundation for most modern AI-based perception, including object detection, tracking, and scene understanding.
*   **LiDAR (Light Detection and Ranging):** A LiDAR sensor works by sending out pulses of laser light and measuring the time it takes for the light to bounce back. This provides a precise 3D "point cloud" of the environment. LiDAR is excellent for mapping and obstacle avoidance because it provides direct distance measurements, something a standard camera cannot do.
*   **Depth Cameras:** A depth camera is a hybrid sensor that combines a regular camera with an infrared projector and sensor. It produces a "depth image," where each pixel's value corresponds to its distance from the camera. This provides 3D information in a camera-like format, and it is a key sensor for navigation and manipulation. The Intel RealSense D435i, used in this course, is a prime example.

### 3. Force/Tactile Sensors: The Sense of Touch

These sensors measure the forces and torques that result from interaction with the environment.
*   **Force-Torque (F/T) Sensors:** Often placed in a robot's wrist or ankle, these sensors measure the forces and torques being applied to the end-effector (the hand or foot). This is critical for tasks that require a delicate touch, like assembling parts or avoiding excessive force.
*   **Tactile Sensors:** These are arrays of small sensors that can be placed on a robot's fingertips to give it a sense of touch, allowing it to detect the shape, texture, and pressure of an object it is holding.

A robust robot uses a combination of these sensors—a process called **sensor fusion**—to build a complete and reliable model of itself and its environment. In the next lesson, we will look at the components that allow the robot to act on this information.
