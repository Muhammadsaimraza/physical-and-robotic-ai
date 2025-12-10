# Lesson 11.1: Camera Simulation

A camera is the richest sensor in robotics, providing the data needed for object detection, navigation, and interaction. Let's add a camera to our two-wheeled robot.

A sensor in Gazebo needs two things:
1.  A `<link>` to attach it to.
2.  A `<sensor>` tag within a `<gazebo>` block to define the plugin and its properties.

## 1. Add a Camera Link

First, we need a physical link for the camera. Let's add a small box to the front of our chassis.

```xml
<!-- Add to your URDF file -->

<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
      <geometry>
          <box size="0.05 0.05 0.05"/>
      </geometry>
  </collision>
  <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyyy="0.0" izz="1e-6"/>
  </inertial>
</link>

<!-- Camera Joint -->
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.225 0 0.075" rpy="0 0 0"/>
</joint>
```
We've created a small red box and attached it to the front of the chassis (`x=0.225` is half the chassis length, `z=0.075` is half the chassis height plus half the camera height).

## 2. Add the Gazebo Camera Plugin

Now, we add the `<gazebo>` tag to attach the camera sensor *to the `camera_link`*.

```xml
<!-- Add to your URDF file -->

<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgz-sim-ros2-camera-system.so">
      <topic_name>image_raw</topic_name>
      <camera_info_topic_name>camera_info</camera_info_topic_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
### Breakdown
*   **`<gazebo reference="camera_link">`**: Associates this block with the `camera_link`.
*   **`<sensor type="camera">`**: Defines the sensor.
*   **`<update_rate>`**: 30 frames per second.
*   **`<camera>` block**: Defines the camera's intrinsic properties.
    *   `<horizontal_fov>`: The horizontal field of view in radians (1.396 rad = 80 deg).
    *   `<image>`: The resolution and pixel format.
    *   `<clip>`: The near and far clipping planes.
*   **`<plugin>`**: This is the most important part.
    *   `filename="libgz-sim-ros2-camera-system.so"`: This specifies the Gazebo plugin that simulates a camera and publishes its images to ROS 2.
    *   `<topic_name>`: The ROS 2 topic to publish the images on. We'll use `image_raw`.
    *   `<frame_name>`: The TF frame to associate with the image data, which should be the link the sensor is attached to.

## 3. Visualizing the Camera Feed

Now, rebuild and launch your `gazebo.launch.py`. Your robot will spawn with the red camera box on top.

To see the camera data, you need to use another ROS 2 tool: **RViz** or **`rqt_image_view`**.

In a new terminal, run:
```bash
ros2 run rqt_image_view rqt_image_view
```
This will open a GUI window. In the topic dropdown at the top, select `/image_raw`. You should now see a real-time video feed from the perspective of your simulated robot!

Drive the robot around using `ros2 topic pub` commands to the `/cmd_vel` topic, and you will see the image in `rqt_image_view` update.

You have successfully given your robot vision. In the next lesson, you'll give it another powerful sense: LiDAR.
