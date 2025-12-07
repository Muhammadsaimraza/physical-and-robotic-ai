# Lesson 10.3: Physical Configuration (Gazebo-Specific Tags)

URDF is a general-purpose format, but simulators often need more information than it can provide. To solve this, we can add Gazebo-specific tags to our URDF file inside a `<gazebo>` block. The simulator will read these tags, but other ROS tools will simply ignore them.

These tags allow us to define friction, damping, material properties, and to connect our URDF to Gazebo plugins.

## The `<gazebo>` Tag

You can add a `<gazebo>` tag inside a `<link>` or a `<joint>` tag. If you are defining properties for a link, the tag is `<gazebo reference="link_name">`.

## Adding Friction

Let's add friction properties to our wheels and caster so they don't slide around like they're on ice. We will define friction coefficients for the collision geometry.

```xml
<!-- Inside two_wheeled_robot.urdf -->

<gazebo reference="left_wheel">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="right_wheel">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="caster">
  <mu1>0.0</mu1>
  <mu2>0.0</mu2>
  <material>Gazebo/Grey</material>
</gazebo>
```
*   **`reference="link_name"`**: This attribute tells Gazebo which link these properties apply to.
*   **`<mu1>` and `<mu2>`**: These are the coefficients of friction for the primary and secondary friction directions. For a wheel, we want high friction. For our simple caster, we'll set it to zero to make it slippery.
*   **`<material>`**: This tells Gazebo to use one of its built-in material scripts for rendering. This will make the robot look much nicer in the simulator than the simple colors we defined in the `<visual>` tag.

## Adding a Gazebo Plugin

The most powerful use of the `<gazebo>` tag is to attach plugins. A plugin is a piece of code that runs inside the simulation and can interact with your robot and the ROS 2 network.

We need a plugin to control our differential drive robot. The `ros_gz_bridge` package provides one called `gz::sim::systems::DiffDrive`.

We add this plugin to our URDF, associated with the `chassis` link.
```xml
<gazebo>
  <plugin
    name="gz::sim::systems::DiffDrive"
    filename="gz-sim-diff-drive-system">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.35</wheel_separation>
    <wheel_radius>0.1</wheel_radius>
    <odom_publish_frequency>10</odom_publish_frequency>
    <topic>/cmd_vel</topic>
  </plugin>
</gazebo>
```
*   **`<plugin>`**: Defines the plugin to load.
*   **`<left_joint>` and `<right_joint>`**: The names of the wheel joints.
*   **`<wheel_separation>`**: The distance between the two wheels (in meters). This must match the `y` value in your joint origins.
*   **`<wheel_radius>`**: The radius of the wheels. This must match the radius in your link geometry.
*   **`<topic>`**: This is the crucial part. The plugin will subscribe to this ROS 2 topic for `geometry_msgs/msg/Twist` messages. This is how we will control our robot. We've set it to `/cmd_vel`.

## The Full URDF

Putting it all together, your URDF file is now a hybrid file containing both standard URDF tags and Gazebo-specific extensions. This is a very common pattern in ROS development.

When you launch your `gazebo.launch.py` file now, your robot should spawn and sit correctly on the ground. It is now a physically realistic model.

Even better, it is now controllable. While the simulation is running, open a new terminal and use `ros2 topic pub` to send a `Twist` message to the `/cmd_vel` topic.
```bash
ros2 topic pub --rate 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```
Your simulated robot will start to move forward! The `DiffDrive` plugin is receiving the ROS 2 message and converting it into forces that it applies to the wheel joints inside the physics simulation.

You have now created a complete digital twin. In the next chapters, we will explore how to add simulated sensors and build more complex, closed-loop controllers.
