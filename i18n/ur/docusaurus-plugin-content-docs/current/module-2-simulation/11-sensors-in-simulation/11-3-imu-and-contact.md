# Lesson 11.3: IMU & Contact Sensors

Let's round out our sensor suite with two more important types: an IMU for balance and orientation, and a contact sensor to detect collisions.

## 1. IMU Sensor

An Inertial Measurement Unit (IMU) measures orientation, angular velocity, and linear acceleration. It's the robot's inner ear, providing its sense of balance.

We will attach the IMU to the `chassis` link.
```xml
<!-- Add this INSIDE the <gazebo reference="chassis"> tag in your URDF -->

<sensor name="imu_sensor" type="imu">
  <update_rate>50</update_rate>
  <plugin name="imu_controller" filename="libgz-sim-ros2-imu-system.so">
    <topic_name>imu</topic_name>
    <frame_id>chassis</frame_id>
  </plugin>
</sensor>
```
### Breakdown
*   **`<sensor type="imu">`**: Defines the sensor type.
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-imu-system.so"`: The plugin that simulates an IMU and publishes `sensor_msgs/msg/Imu` messages.
    *   `<topic_name>`: The ROS 2 topic to publish on, `/imu`.
    *   `<frame_id>`: The TF frame of the data.

After rebuilding and launching, you can echo the topic to see the raw data:
```bash
ros2 topic echo /imu
```
You will see a stream of messages containing orientation (as a quaternion), angular velocity, and linear acceleration.

## 2. Contact/Bumper Sensor

Sometimes you need to know if your robot has physically run into something. A contact or bumper sensor detects collisions.

Let's add a "bumper" to the front of our chassis. We'll attach it to the same `camera_link` at the front.

```xml
<!-- Add this INSIDE the <gazebo reference="camera_link"> tag in your URDF -->

<sensor name="bumper_sensor" type="contact">
  <contact>
    <collision>camera_link_collision</collision>
  </contact>
  <update_rate>10</update_rate>
  <plugin name="bumper_controller" filename="libgz-sim-ros2-bumper-system.so">
    <topic_name>bumper_state</topic_name>
    <frame_id>camera_link</frame_id>
  </plugin>
</sensor>
```
### Breakdown
*   **`<sensor type="contact">`**: Defines the sensor type.
*   **`<contact><collision>`**: This is important. It specifies which **collision geometry** this sensor is attached to. The name `camera_link_collision` refers to the `<collision>` element of the `camera_link` link. This means the sensor will only trigger if that specific collision shape hits something.
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-bumper-system.so"`: The plugin that publishes `gz_ros2_interfaces/msg/Contact` messages when a collision is detected.
    *   `<topic_name>`: The topic to publish on, `/bumper_state`.

After rebuilding and launching, you can echo the topic:
```bash
ros2 topic echo /bumper_state
```
Initially, it will be empty. Now, drive your robot forward until it hits the construction cone or a wall. As soon as the `camera_link`'s collision geometry makes contact, you will see messages appear on the topic, describing the collision.

Your robot is now equipped with a comprehensive suite of sensors, allowing it to perceive the world through sight, lasers, and touch, and to understand its own orientation in space. In the final lesson of this chapter, we'll review the tools we use to make sure all this data is correct.
