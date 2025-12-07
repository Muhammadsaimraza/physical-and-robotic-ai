# Lesson 3.3: Nodes & Topics

In the last lesson, you got Turtlesim running. You have a `turtlesim_node` process and a `turtle_teleop_key` process. But from the perspective of ROS, these are just generic processes. Let's use the `ros2` CLI to see what's really going on.

With your two Turtlesim nodes still running, open a **third terminal** and source your ROS 2 environment.

## Discovering Nodes

The `ros2 node` command lets us introspect the running nodes in the ROS graph.
```bash
ros2 node list
```
You should see output similar to this:
```
/turtlesim
/teleop_turtle
```
These are the names of the two nodes that are currently running. Note that `/teleop_turtle` is the default name for the `turtle_teleop_key` executable.

## Discovering Topics

Nodes communicate using topics. Let's see what topics are currently active in the system.
```bash
ros2 topic list
```
This will show you a list of all the topics that nodes are either publishing to or subscribing from. You'll see a few, but the most important ones are:
```
/turtle1/cmd_vel
/turtle1/pose
/parameter_events
/rosout
```
*   `/turtle1/cmd_vel`: This is the topic where the teleop node publishes velocity commands. The name `cmd_vel` is a standard convention in ROS for "command velocity."
*   `/turtle1/pose`: The `turtlesim_node` publishes the turtle's current position and orientation (its "pose") on this topic. This is how other nodes could know where the turtle is.
*   The other two topics are used for system-wide events and logging.

## Spying on a Topic

We can use `ros2 topic echo` to print the messages that are being published on a topic in real-time. Let's listen to the command velocity topic.

In your third terminal, run:
```bash
ros2 topic echo /turtle1/cmd_vel
```
At first, you won't see anything. But now, go to the terminal running `turtle_teleop_key` and press the arrow keys. You will see messages start to appear in your third terminal!
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```
You are "echoing" the `Twist` messages that the teleop node is publishing. This is an incredibly powerful debugging tool. It lets you see the raw data flowing between any two nodes in your system without having to modify their code.

Now try echoing the `/turtle1/pose` topic while you drive the turtle around.
```bash
ros2 topic echo /turtle1/pose
```
You will see a continuous stream of messages showing the turtle's changing `x`, `y`, and `theta` (angle) values.

## Publishing from the Command Line

We can also publish messages directly from the command line, without needing a separate node. This is useful for testing.

Let's make the turtle move in a circle. The command is a bit long:
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
Let's break this command down:
*   `ros2 topic pub`: The command to publish a message.
*   `--rate 1`: Publish the message at a rate of 1 Hz (once per second).
*   `/turtle1/cmd_vel`: The topic to publish to.
*   `geometry_msgs/msg/Twist`: The **type** of message we are publishing.
*   `"{...}"`: The actual data for the message, in YAML format. We are setting the linear x velocity to 2.0 and the angular z velocity to 1.8.

When you run this, you will see the turtle start to move in a circle, even if the `turtle_teleop_key` node is not running! You are now controlling the robot directly from the command line. Press `Ctrl+C` in that terminal to stop publishing.

You now have the fundamental tools for inspecting and debugging a ROS 2 system. In the next lesson, we'll explore the other main communication patterns: services and parameters.
