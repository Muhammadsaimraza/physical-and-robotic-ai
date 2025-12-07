# Lesson 11.4: Sensors Debugging

Your simulation is now publishing a lot of data. But how do you know if the data is correct? Debugging simulated sensors is a critical skill. Here are the primary tools and techniques you should use.

## 1. `ros2 topic echo`

This is always your first step. Before you try to visualize anything, look at the raw data.
```bash
ros2 topic echo <topic_name>
```
*   **Is anything being published?** If you see no messages, it means your plugin is not running or is configured incorrectly. Check your URDF and the Gazebo console for errors.
*   **Is the data reasonable?** If you're echoing `/scan` and all the ranges are `inf` (infinity), it might mean your LiDAR is inside another object or pointing at the sky. If you're echoing `/image_raw`, you won't see the image data itself (it's too large), but you should see the message headers being published.

## 2. RViz

RViz is the primary tool for visualizing sensor data in a spatial context.

*   **TF Tree:** The first thing to check is your TF tree. Add the "TF" display. Do all the links appear? Are they connected correctly? If your `camera_link` is floating in space instead of being attached to your `chassis`, you have a problem in your joint's `<origin>` tag.
*   **LaserScan Display:** When you add a LaserScan display and set the topic, you should see the points appear around your robot model. If the points are offset or rotated incorrectly, it usually means the `<frame_name>` in your sensor plugin configuration is wrong, or the sensor's `<pose>` inside the URDF is incorrect.
*   **Image Display:** RViz can also display camera images. Add an "Image" display and set the topic. This is useful, but for simple viewing, `rqt_image_view` is often easier.
*   **IMU Display:** You can't directly visualize the IMU's orientation, but you can see its effect on the TF tree. The IMU data is often used by a localization algorithm (like SLAM) to publish the robot's orientation. You would see this by watching the `chassis` frame rotate in RViz as you move the physical robot in Gazebo.

## 3. Gazebo GUI

Don't forget that Gazebo itself is a powerful debugging tool.

*   **Visualizers:** In the Gazebo GUI, you can turn on visualizers for many sensor types. For example, you can enable "View" -> "Lidar visualization" to see the actual laser beams being cast in the simulation. This can help you understand why your sensor isn't seeing an object you think it should. You can do the same for camera frustums and other sensors.
*   **Inspect Tool:** Use the "Inspect" tool (the magnifying glass) to click on any link of your robot. The panel on the right will show you all of its properties, including its mass, inertia, and whether it is in a collision state.
*   **Console Output:** The terminal where you launched Gazebo is filled with useful information and error messages. If a plugin fails to load or a model can't be found, the error will be printed here.

## Common Problems & Solutions

*   **Problem:** No messages on the topic.
    *   **Solution:** Check the Gazebo console for errors. Did the plugin load correctly? Is the topic name in your URDF spelled correctly?
*   **Problem:** Data is in the wrong place in RViz (e.g., LiDAR scan is floating in space).
    *   **Solution:** Your TF tree is broken. Check the `<frame_name>` or `<frame_id>` in your plugin configuration. It must match the name of the link the sensor is attached to.
*   **Problem:** The simulation is very slow.
    *   **Solution:** Your sensor settings might be too high. Try reducing the `update_rate` or the number of `samples` (for LiDAR) or the `width`/`height` (for a camera). A `gpu_lidar` is much faster than a CPU `lidar`.

By combining these tools, you can systematically diagnose and fix almost any issue with your simulated sensors. This concludes Chapter 11. You can now build a robot and give it senses. In the next chapter, you'll learn how to connect your ROS 2 code to the robot to create a closed-loop control system.
