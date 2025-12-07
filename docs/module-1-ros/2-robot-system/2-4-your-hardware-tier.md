# Lesson 2.4: Your Hardware Tier

This course is designed to be accessible to a wide range of students, from those with just a laptop to those with a full robotics lab. To accommodate this, we have defined four "Hardware Tiers."

Your tier determines *how* you will run the code and simulations in this course. While all of Module 1 can be completed on Tier 1, later modules will require more powerful hardware to run high-fidelity simulations and deploy to physical robots.

It is important to identify your tier now, as it will shape your learning path.

## The Four Tiers

### Tier 1: The Cloud Roboticist
*   **Equipment:** A standard laptop (Windows, Mac, or Linux) with a modern web browser.
*   **How it Works:** You will use cloud-based platforms to run all your ROS 2 code and simulations.
    *   **The Construct:** A web-based platform that gives you a virtual machine with ROS 2 and various simulators pre-installed.
    *   **Pyodide/MockROS:** For some exercises, you will run ROS 2 directly in your browser using a technology called Pyodide that compiles Python and ROS to run in a web page.
*   **Advantages:** No complex setup, nothing to install on your local machine. You can learn from anywhere.
*   **Limitations:** You are dependent on an internet connection and the performance of the cloud provider. You cannot connect to physical hardware.

### Tier 2: The Simulation Specialist
*   **Equipment:** A powerful desktop workstation with a dedicated NVIDIA GPU (e.g., RTX 3070 or better) running Ubuntu Linux.
*   **How it Works:** You will install ROS 2, Gazebo, and other simulation tools directly on your local machine.
*   **Advantages:** Much faster and more responsive than cloud-based simulation. You have full control over your environment.
*   **Limitations:** Requires a significant hardware investment and a more complex software setup (including partitioning your drive to install Linux).

### Tier 3: The Edge AI Developer
*   **Equipment:** Everything in Tier 2, plus an NVIDIA Jetson Orin Nano Developer Kit.
*   **How it Works:** You will use your workstation for high-fidelity simulation (like NVIDIA Isaac Sim) and then deploy your trained AI models to the Jetson for real-time inference.
*   **Advantages:** This tier allows you to experience the full "sim-to-real" workflow and work with hardware-accelerated AI for perception and navigation. This is representative of a professional robotics workflow.
*   **Limitations:** Requires purchasing and setting up the Jetson kit.

### Tier 4: The Full-Stack Roboticist
*   **Equipment:** Everything in Tier 3, plus a complete physical robot (e.g., a Unitree Go2 or G1 humanoid).
*   **How it Works:** You will deploy your code to a real, mobile robot, testing its ability to navigate and interact with the real world.
*   **Advantages:** This is the ultimate ground truth. It provides invaluable experience in debugging the subtle and challenging issues that only appear when interacting with the physical world.
*   **Limitations:** This is the most expensive and complex tier.

## What Tier Are You?

Take a moment to assess your available hardware.

*   If you only have a standard laptop, you are **Tier 1**.
*   If you have a powerful gaming PC with an NVIDIA graphics card that you are willing to install Linux on, you are **Tier 2**.
*   If you have a Tier 2 workstation and have also purchased a Jetson Orin kit, you are **Tier 3**.
*   If you have all of the above plus a physical robot, you are **Tier 4**.

**No matter your tier, you can successfully complete all of Module 1.** The course is designed to provide a complete learning experience for cloud-based students. Higher tiers will have a smoother experience in later modules and will be able to complete the physical deployment exercises.

This concludes Chapter 2. You now have a complete high-level view of a robot system, from sensors to actuators to the middleware that connects them. In Chapter 3, we will get our hands dirty for the first time and start interacting with a live ROS 2 system.
