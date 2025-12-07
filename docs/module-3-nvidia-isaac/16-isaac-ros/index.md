# Chapter 16: Isaac ROS & GPU-Accelerated Perception

You've trained a model on synthetic data. Now you need to run it on your robot. The challenge is that many modern perception algorithms, like Visual SLAM or object detection with large neural networks, are too computationally expensive to run in real-time on a robot's CPU.

This is where **Isaac ROS** comes in. It is a collection of ROS 2 packages that have been optimized to run on NVIDIA GPUs. By offloading the heavy computation to the GPU, these packages enable real-time performance for complex AI and computer vision tasks on embedded platforms.

## Learning Objectives
By the end of this chapter, you will be able to:

*   Explain why GPU acceleration is critical for modern robotics.
*   Describe the purpose and architecture of the Isaac ROS VSLAM package.
*   Integrate the output of Isaac ROS VSLAM with the Nav2 stack for autonomous navigation.

## Lessons
*   **Lesson 16.1: The Need for Speed: GPU Acceleration**
*   **Lesson 16.2: Isaac ROS VSLAM**
