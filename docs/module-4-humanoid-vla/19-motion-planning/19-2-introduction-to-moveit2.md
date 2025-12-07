# Lesson 19.2: Introduction to MoveIt2

**MoveIt** is the most widely used software for motion planning in the ROS ecosystem. It is a large and complex framework that provides everything you need to get a robot arm moving.

MoveIt2 is the ROS 2 version of the framework.

## What MoveIt2 Does

MoveIt is not a single node. It is a collection of nodes, plugins, and configuration files that work together to provide a complete motion planning service.

At its core, MoveIt:
1.  **Takes your URDF:** It reads your robot's URDF to understand its kinematic structure.
2.  **Loads the Environment:** It subscribes to topics that provide information about the world, including a 3D representation of the environment from a sensor like a depth camera. This is used for collision checking.
3.  **Provides an Action Server:** It provides a ROS 2 Action server, typically called `/move_action`, that allows you to send it goals. A goal can be a target pose for the end-effector or a target set of joint angles.
4.  **Plans a Path:** When it receives a goal, it uses its configured motion planning plugin (which often uses an algorithm like RRT) to find a collision-free path.
5.  **Executes the Trajectory:** It converts the planned path into a smooth trajectory and sends the commands to the robot's motor controllers (often via another action server called `/follow_joint_trajectory`).

## The MoveIt Setup Assistant

Configuring MoveIt2 for a new robot by hand is a very complex process. To simplify this, the MoveIt team provides a GUI tool called the **MoveIt Setup Assistant**.

The Setup Assistant guides you through the process:
1.  **Load URDF:** You load your robot's URDF file.
2.  **Define Planning Groups:** You define a "planning group" for the part of the robot you want to control (e.g., you create an "arm" group that includes all the links and joints from the shoulder to the wrist).
3.  **Define Robot Poses:** You can define pre-set poses, like a "home" or "stow" position.
4.  **Configure End-Effectors:** You define which link is the end-effector (the "hand").
5.  **Generate Configuration Files:** The tool then automatically generates a collection of configuration files and launch files that are specific to your robot.

These generated files create a complete MoveIt configuration package. You can then launch this package to start all the necessary MoveIt nodes.

## Using MoveIt

Once the MoveIt nodes are running, you can command your robot from your own Python node by creating a client for the `/move_action` server.

A simplified workflow:
1.  Create an action client for `/move_action`.
2.  Create a `PoseStamped` message to define your target pose in 3D space.
3.  Send the goal to the MoveIt action server.
4.  MoveIt will then do all the hard work: call an IK solver, plan a collision-free path, and execute the trajectory on the real or simulated robot.

MoveIt provides the bridge from high-level "go here" commands to low-level joint control. It is a fundamental building block for any robot that needs to perform manipulation tasks.
