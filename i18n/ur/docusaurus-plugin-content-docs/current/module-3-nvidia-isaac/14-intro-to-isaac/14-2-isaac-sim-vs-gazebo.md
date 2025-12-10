# Lesson 14.2: Isaac Sim vs. Gazebo

You've spent the last module learning Gazebo. Now we are introducing a new simulator, Isaac Sim. What's the difference? When would you choose one over the other?

The choice of simulator depends on your primary goal.

## The Tale of Two Simulators

| Feature | Gazebo | Isaac Sim |
| :--- | :--- | :--- |
| **Core Strength** | **Physics.** Fast and robust rigid-body dynamics. | **Photorealism.** State-of-the-art rendering for realistic sensor data. |
| **Primary Use Case** | Developing and testing control algorithms, locomotion, and manipulation. | Training and testing AI/ML perception models. Synthetic data generation. |
| **Rendering Quality** | Functional, but not realistic. Lighting and textures are basic. | Physically-based, real-time ray tracing. Looks like a modern video game. |
| **Physics Engine** | ODE (Open Dynamics Engine) - proven and stable. | NVIDIA PhysX 5 - advanced engine that also supports soft bodies and fluids. |
| **Hardware Requirement** | Can run on a standard CPU. | Requires a powerful, modern NVIDIA GPU (RTX series). |
| **Community & Cost** | Open-source, large academic community, free. | Proprietary (but free to use), enterprise-focused. |
| **ROS Integration** | Excellent and native. | Excellent and native. |

## An Analogy: The Wind Tunnel vs. The Movie Set

*   **Gazebo is like a wind tunnel.** It might not look pretty, but it is a highly effective and scientifically accurate tool for testing the dynamics of your system. If you want to know if your control algorithm will be stable, Gazebo is the perfect tool.
*   **Isaac Sim is like a movie set.** It is designed to look and feel exactly like the real world. If you want to fool a camera and an AI model into thinking they are in a real environment, Isaac Sim is the perfect tool.

## Which One Should You Use?

In modern robotics, the answer is often **both**.

A common professional workflow:
1.  Use Gazebo for the initial stages of development. It's lightweight and fast for testing basic control loops, motor controllers, and locomotion.
2.  Use Isaac Sim for the perception and AI stack. Once the basic control is working, you move to Isaac Sim to generate data, train your vision models, and test your autonomous navigation system in a realistic environment.

This course focuses on Gazebo in Module 2 because it is more accessible and is excellent for teaching the fundamentals of simulation. In this module, we introduce Isaac Sim because it is the state-of-the-art tool for the AI-driven tasks that represent the future of robotics.

The skills you learned with Gazebo—creating a URDF, understanding plugins, and integrating with ROS 2—are all directly transferable to Isaac Sim. The core concepts are the same; only the specific tools and the visual quality have changed.
