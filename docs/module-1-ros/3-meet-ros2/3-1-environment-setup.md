# Lesson 3.1: Environment Setup

Before you can interact with a ROS 2 system, you need access to one. In this lesson, you will get your environment up and running.

Based on your Hardware Tier from the previous chapter, you have two primary options.

---

### Option A: Tier 1 (Cloud-Based)

For Tier 1 students, we recommend using a cloud-based robotics platform. This is the fastest and easiest way to get started, as it requires no local installation.

**Recommendation: The Construct**

1.  **Sign Up:** Go to [The Construct](https://www.theconstructsim.com/) and sign up for a free account.
2.  **Launch a ROSject:** Find a ROS 2 Humble course or create a new ROSject. This will spin up a virtual machine in your browser that has ROS 2, a code editor, and a graphical desktop all pre-configured.
3.  **Open a Terminal:** The integrated desktop will have a terminal application. This is where you will run all the `ros2` commands for this chapter.
4.  **Source the Environment:** In ROS, you must "source" a setup file in every new terminal to make the ROS commands available. The exact command may vary by platform, but it will look something like this:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    Your cloud environment may even do this for you automatically.

**Advantages:**
*   Zero installation on your local machine.
*   Standardized environment ensures all examples will work out of the box.

---

### Option B: Tier 2+ (Local Installation)

For students with a dedicated Linux machine or a dual-boot setup, installing ROS 2 locally is the best long-term option.

**Official Installation Guide**

The official ROS 2 documentation is the single source of truth for installation instructions. Follow the guide for your specific operating system.

**Target Distribution: ROS 2 Humble Hawksbill**

*   **Go to the ROS 2 Humble Installation Guide:** [docs.ros.org](https://docs.ros.org/en/humble/Installation.html)
*   **Select your OS:** The primary supported OS is **Ubuntu 22.04 (Jammy Jellyfish)**.
*   **Follow the Debian instructions:** This will guide you through adding the ROS 2 repositories and installing the `ros-humble-desktop` package.

**Key Steps:**
1.  **Set Locale and Sources:** Ensure your system is configured to accept packages from `packages.ros.org`.
2.  **Install ROS 2 Packages:** The main command will be:
    ```bash
    sudo apt install ros-humble-desktop
    ```
3.  **Source the Environment:** Just like in the cloud, you must source the setup file in every terminal you open. Add this line to the end of your `~/.bashrc` file to have it run automatically for every new terminal:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
    Then, run `source ~/.bashrc` in your current terminal.

**Verification**

No matter which option you chose, you can verify your installation by running a simple command:
```bash
ros2 --help
```
If your environment is set up correctly, this will print a list of available `ros2` commands. If you get a "command not found" error, it means you have not sourced the setup file correctly.

---

You are now ready to take your first steps in the ROS 2 ecosystem. In the next lesson, you'll meet Turtlesim, your first simulated robot.
