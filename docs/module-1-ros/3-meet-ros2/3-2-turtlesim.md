# Lesson 3.2: Your First Robot (Turtlesim)

Turtlesim is a simple, lightweight robot simulator that is part of the standard ROS 2 installation. It is the "Hello, World!" of robotics. While it may seem basic, it's the perfect tool for learning the core concepts of ROS 2 without the complexity of a full 3D simulator.

## Launching Turtlesim

Let's start the simulation. Open a terminal (with your ROS 2 environment sourced) and run the following command:
```bash
ros2 run turtlesim turtlesim_node
```
This command does three things:
*   `ros2 run`: The standard command to execute a single ROS 2 node.
*   `turtlesim`: The name of the **package** we want to use.
*   `turtlesim_node`: The name of the **executable (node)** we want to run within that package.

You should see a new window appear with a blue background and a single turtle in the middle. Congratulations, you are now running your first robot simulation.

![Turtlesim Window](https://raw.githubusercontent.com/ros/ros_tutorials/humble-devel/turtlesim/images/turtlesim.png)

This window is a 2D world where our turtle robot lives. The `turtlesim_node` is the program that manages this world, draws the turtle, and listens for commands.

## Controlling the Turtle

The `turtlesim_node` itself doesn't have a user interface for control. It just listens for commands from the ROS 2 network. To control it, we need to run a second node that publishes these commands.

Open a **second terminal** (make sure to source your ROS 2 environment again) and run:
```bash
ros2 run turtlesim turtle_teleop_key
```
This command runs the `turtle_teleop_key` node from the `turtlesim` package. This node listens for keystrokes in the terminal and publishes them as velocity commands for the turtle.

Click on the second terminal window (the one running `turtle_teleop_key`) and use the arrow keys on your keyboard. You should see the turtle in the first window start to move!

## What's Happening?

You have just created your first multi-node ROS 2 system.
1.  The `turtlesim_node` is running, managing the simulation. It is **subscribing** to a topic to listen for velocity commands.
2.  The `turtle_teleop_key` node is running. It is **publishing** velocity commands to that same topic based on your key presses.

The two nodes are completely independent. They don't know anything about each other. They are only connected by the anonymous ROS 2 topic that sits between them.

This is the core principle of ROS in action. We are composing a complex behavior (keyboard-controlled movement) by combining two simple, single-purpose nodes.

### Exploration
*   Try driving the turtle around the screen.
*   What happens if you close the `turtle_teleop_key` terminal? Does the simulation crash? (It shouldn't, because the `turtlesim_node` is a separate process).
*   What happens if you run a *second* `turtle_teleop_key` node in a third terminal? Can both nodes control the turtle?

In the next lesson, we will use ROS 2's command-line tools to look under the hood and inspect the nodes and topics that make this system work.
