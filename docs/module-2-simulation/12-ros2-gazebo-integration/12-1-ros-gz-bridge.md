# Lesson 12.1: The `ros_gz_bridge`

Gazebo and ROS are two separate, independent systems. Gazebo has its own internal publish/subscribe system called "Gazebo Transport." ROS has its own publish/subscribe system (DDS). They do not speak the same language.

The `ros_gz_bridge` is a special ROS 2 node that translates between these two worlds.

## How it Works

The bridge node subscribes to a Gazebo topic, converts the message to its ROS 2 equivalent, and then publishes it on a ROS 2 topic. It can also do the reverse.

For example, when you use the `gz-sim-ros2-camera-system` plugin:
1.  The Gazebo camera simulation publishes a Gazebo `Image` message on an internal Gazebo topic.
2.  The plugin, which has an embedded bridge, receives this message.
3.  It converts the Gazebo `Image` message into a ROS 2 `sensor_msgs/msg/Image` message.
4.  It publishes the ROS 2 message on the topic you specified (e.g., `/image_raw`).

## Using the Bridge Manually

While the sensor and controller plugins are convenient because they have a pre-configured bridge built-in, you can also run the bridge as a standalone node to pass any message type between the two systems.

This is done with the `ros_gz_bridge` executable.

**Example:**
Let's say you want to get the pose of your robot model, `my_robot`, from Gazebo and publish it to a ROS 2 topic.

You can run the bridge node like this:
```bash
ros2 run ros_gz_bridge parameter_bridge /model/my_robot/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose
```

### Breakdown
*   `ros2 run ros_gz_bridge parameter_bridge`: Runs the bridge node.
*   **`/model/my_robot/pose`**: The name of the Gazebo topic to subscribe to. (Gazebo automatically publishes the pose of every model on a topic with this naming convention).
*   **`@geometry_msgs/msg/PoseStamped`**: The ROS 2 message type to convert to. This is specified after the first `@`.
*   **`@gz.msgs.Pose`**: The Gazebo message type to convert from. This is specified after the second `@`.

This command will create a ROS 2 topic named `/model/my_robot/pose` and publish the robot's pose to it. Your ROS 2 nodes can then subscribe to this topic to know where the robot is.

## Finding Gazebo Topics

How do you know what Gazebo topics are available? You can use the `gz` command line tool.
```bash
gz topic -l
```
This will list all the topics currently being published inside the Gazebo simulation. You can use this list to find the topics you want to bridge to ROS 2.

The plugins we have used so far handle this bridging for us automatically.
*   The `DiffDrive` plugin bridges the ROS 2 `/cmd_vel` topic to the Gazebo physics engine.
*   The `Camera` plugin bridges the internal Gazebo camera data to the ROS 2 `/image_raw` topic.
*   The `Lidar` plugin bridges the internal Gazebo laser data to the ROS 2 `/scan` topic.

Understanding that this bridge is the underlying mechanism is key to debugging and to creating more complex integrations between the two systems. In the next lessons, we will leverage these bridged topics to create a closed-loop controller.
