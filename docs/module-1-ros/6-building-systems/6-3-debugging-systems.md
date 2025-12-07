# Lesson 6.3: Debugging Systems

When you have a dozen nodes all started from a launch file, how do you figure out what's going on? If something isn't working, where do you start? ROS 2 provides several powerful tools for inspecting and debugging a live system.

## `ros2doctor`

`ros2doctor` is your first line of defense. It's a general-purpose diagnostic tool that checks for common problems in your system.

Run your `talker_listener.launch.py` file. In a second terminal, run the doctor:
```bash
ros2 doctor
```
It will check your network configuration, running nodes, topic publishers, and more, and it will print a report with any warnings or errors it finds. If you're ever having a strange network-related issue where nodes can't seem to communicate, `ros2 doctor` is the first tool you should reach for.

## RQT Graph

The most powerful debugging tool in the ROS ecosystem is `rqt`. RQT is a graphical user interface framework that hosts a wide variety of plugins. The most useful of these is `rqt_graph`.

To start it, simply run:
```bash
rqt_graph
```
This will open a window that shows a visual representation of your ROS graph.
*   **Ovals** are nodes.
*   **Rectangles** are topics.
*   **Arrows** show the direction of communication (who is publishing and who is subscribing).

![RQT Graph Example](https://raw.githubusercontent.com/ros-visualization/rqt_graph/ros2/rqt_graph.png)

With your talker/listener system running, `rqt_graph` will show you:
*   The `/my_talker` node.
*   The `/my_listener` node.
*   The `/chatter` topic.
*   An arrow from `/my_talker` to `/chatter`.
*   An arrow from `/chatter` to `/my_listener`.

This gives you an immediate, intuitive overview of your entire system architecture. You can instantly see if a node is subscribed to the wrong topic or if a connection is missing. As your systems become more complex, with dozens of nodes and topics, `rqt_graph` becomes an indispensable tool for understanding how everything fits together.

## Other RQT Plugins

RQT has many other useful plugins for debugging. You can start them from the `rqt` main window, under the "Plugins" menu. Some of the most useful include:

*   **Topic Monitor:** Graphically view all the topics and the rate at which they are being published.
*   **Service Caller:** A GUI for calling services, similar to `ros2 service call`.
*   **Parameter Reconfigure:** A GUI for inspecting and changing node parameters, similar to `ros2 param set`.
*   **Plot:** Plot the numeric values from any topic in real-time. This is great for visualizing sensor data or control loop errors.

## Systematic Debugging

When faced with a bug in a complex system, the key is to be systematic.

1.  **Check the graph:** Use `rqt_graph` to confirm that your nodes are running and connected as you expect. Is there a missing link?
2.  **Check the data:** Use `ros2 topic echo` to inspect the raw data being published on a topic. Is the data in the format you expect? Are the values reasonable?
3.  **Isolate the problem:** Try to run nodes one at a time using `ros2 run`. Can you narrow down which node is causing the issue?
4.  **Check the logs:** Look at the console output of your nodes. Are there any error messages? You can increase the logging verbosity to get more detail.

By combining the command-line tools you learned in Chapter 3 with the graphical tools from `rqt`, you have a complete toolkit for debugging even the most complex robotic systems.

This concludes Chapter 6. You can now build individual nodes, configure them with parameters, and integrate them into a complete application using launch files. In the final chapter of this module, you will put all of these skills together to build a complete robot controller from a specification.
