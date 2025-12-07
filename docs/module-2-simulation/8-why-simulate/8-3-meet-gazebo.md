# Lesson 8.3: Meet Gazebo

There are many robotics simulators, but for the open-source robotics community, **Gazebo** has been the dominant tool for over a decade.

Gazebo is a 3D rigid-body physics simulator. This means it is very good at modeling how solid objects move and interact under the influence of forces like gravity and collisions. It was created and is still maintained by the same organization that created ROS. As a result, the integration between ROS and Gazebo is excellent.

*(Note: The simulator was recently renamed from "Gazebo" to "Ignition," and is now being renamed back to "Gazebo." You may see all three names used in documentation. For this course, we are using the modern version of the simulator, but will refer to it by its classic name, Gazebo.)*

## Why Gazebo?

*   **Physics-Focused:** Gazebo's primary strength is its robust and battle-tested physics engine (it uses the Open Dynamics Engine - ODE). It excels at simulating robot locomotion, grasping, and other tasks where accurate physics is important.
*   **ROS Integration:** As a core part of the ROS ecosystem, Gazebo has a rich set of plugins that provide a seamless bridge to ROS 2. You can add a camera plugin to your simulated robot, and it will publish `sensor_msgs/msg/Image` messages on a ROS 2 topic, just like a real camera.
*   **Open Source:** Gazebo is free and open-source, with a large and active community of users and developers.
*   **Extensible:** A plugin-based architecture allows you to extend the simulator with new sensors, actuators, and physics models.

## Gazebo vs. Other Simulators

How does Gazebo compare to a game engine like Unity, which we discussed in Module 1?

| Feature | Gazebo | Unity / Isaac Sim |
| :--- | :--- | :--- |
| **Primary Strength** | Physics Simulation | Photorealistic Rendering |
| **Best For** | Locomotion, manipulation, control algorithms. | Perception, computer vision, AI training. |
| **Visuals** | Functional, but not photorealistic. | State-of-the-art, real-time ray tracing. |
| **ROS Integration** | Excellent, native support. | Good, but requires a bridge package. |
| **Community**| Open-source, academic, research-focused. | Commercial, game development, enterprise. |

**The Bottom Line:**
*   If your primary challenge is **control and dynamics**, Gazebo is an excellent choice.
*   If your primary challenge is **vision-based perception and AI**, a game engine like Isaac Sim is a better choice.

For this module, we will be using Gazebo. It is the perfect tool for learning the fundamentals of simulation, as it is tightly integrated with ROS and has a lower hardware requirement than a high-fidelity simulator like Isaac Sim. The principles you learn here—describing your robot, building a world, and connecting it to ROS—will apply directly to any other simulator you use in the future.

In the next chapter, we will learn how to describe the physical form of our robot using the Unified Robot Description Format (URDF).
