# Lesson 11.2: LiDAR Simulation

LiDAR (Light Detection and Ranging) is a critical sensor for navigation and obstacle avoidance. It provides a precise 2D or 3D scan of the environment by measuring the distance to objects with laser beams.

Let's add a LiDAR sensor to our robot. We'll attach it to the `camera_link` we created in the last lesson.

## Add the Gazebo LiDAR Plugin

Just like the camera, a LiDAR sensor is a plugin that we attach to a link. We will add a second `<sensor>` block inside the `<gazebo reference="camera_link">` tag. It's perfectly fine to have multiple sensors attached to the same link.

```xml
<!-- Add this INSIDE the <gazebo reference="camera_link"> tag in your URDF -->

<sensor name="lidar_sensor" type="gpu_lidar">
  <topic>scan</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>20.0</max>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgz-sim-ros2-lidar-system.so">
      <topic_name>scan</topic_name>
      <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```
### Breakdown

*   **`<sensor type="gpu_lidar">`**: Defines the sensor. `gpu_lidar` is a hardware-accelerated version that is much more performant than the standard `lidar` type.
*   **`<topic>` (optional):** This is an internal Gazebo topic. We can ignore it as the ROS 2 plugin will handle publishing.
*   **`<update_rate>`**: 10 scans per second.
*   **`<ray>` block**: Defines the properties of the laser.
    *   `<scan><horizontal>`: We are defining a 2D horizontal scan.
        *   `<samples>`: 360 points in the scan.
        *   `<min_angle>` & `<max_angle>`: A full 360-degree (-π to +π) scan.
    *   `<range>`: The sensor can detect objects between 0.1m and 20.0m away.
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-lidar-system.so"`: The plugin that simulates a LiDAR and publishes `sensor_msgs/msg/LaserScan` messages to ROS 2.
    *   `<topic_name>`: The ROS 2 topic to publish on. We'll use `/scan`.
    *   `<frame_name>`: The TF frame of the scan, which is the link the sensor is attached to.

## 2. Visualizing the LiDAR Data

Now, rebuild and launch your `gazebo.launch.py`. Your robot is now "seeing" the world with its new laser scanner.

The best tool to visualize a `LaserScan` message is **RViz**.

1.  **Launch RViz:** In a new terminal, run `rviz2`.
2.  **Set Fixed Frame:** In the "Displays" panel, set the "Fixed Frame" to `chassis`.
3.  **Add TF Display:** Click "Add" and add a "TF" display. This will show you the coordinate frames of your robot model. You should see `chassis`, `left_wheel`, `right_wheel`, and `camera_link`.
4.  **Add LaserScan Display:** Click "Add" and add a "LaserScan" display.
5.  **Configure LaserScan:** In the settings for the LaserScan display, change the "Topic" to `/scan`.

You should now see red dots appear in RViz, representing the points where the laser beams are hitting objects in the Gazebo world (like the construction cone or the walls). As you drive the robot around using `/cmd_vel`, you will see the scan update in real-time.

This is the data that a navigation algorithm like SLAM (Simultaneous Localization and Mapping) would use to build a map of the environment. You have now equipped your robot with two of the most important senses for autonomous navigation.
