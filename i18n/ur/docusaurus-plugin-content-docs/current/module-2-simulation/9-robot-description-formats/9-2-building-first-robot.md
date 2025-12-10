# Lesson 9.2: Building Your First Robot

Let's apply the concepts from the last lesson to build a simple, two-wheeled mobile robot in URDF. Our robot will have:
*   A chassis (the main body).
*   Two wheels.
*   A caster wheel at the front for balance.

We will define three links (`chassis`, `left_wheel`, `right_wheel`) and three joints (`left_wheel_joint`, `right_wheel_joint`, `caster_joint`).

## The URDF File

Create a new package for this chapter's work (e.g., `ros2 pkg create --build-type ament_python urdf_tutorial`). Inside that package, create a directory called `urdf`, and inside that, a file named `two_wheeled_robot.urdf`.

```xml
<!-- two_wheeled_robot.urdf -->
<robot name="two_wheeled_robot">

  <!-- ***** LINKS ***** -->

  <!-- Chassis Link -->
  <link name="chassis">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Right Wheel Link -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  
  <!-- Caster Wheel Link -->
  <link name="caster">
      <visual>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
          <material name="grey">
              <color rgba="0.5 0.5 0.5 1.0"/>
          </material>
      </visual>
  </link>

  <!-- ***** JOINTS ***** -->

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Caster Joint -->
  <joint name="caster_joint" type="fixed">
      <parent link="chassis"/>
      <child link="caster"/>
      <origin xyz="0.2 0 -0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

### Breakdown of Key Tags

*   **Chassis:** This is the root link. It's a simple box.
*   **Wheels (`left_wheel`, `right_wheel`):**
    *   The geometry is a cylinder. By default, a cylinder's axis is aligned with the Z-axis.
    *   `<origin rpy="1.5707 0 0">`: We rotate the visual geometry by 90 degrees (1.5707 radians) around the X-axis so that the cylinder lies flat, like a wheel. This `<origin>` tag is *inside* the `<visual>` tag, so it only affects the visual representation, not the link's coordinate frame itself.
*   **Joints (`left_wheel_joint`, `right_wheel_joint`):**
    *   `type="continuous"`: This is a revolute joint with no limits, perfect for a wheel that can spin forever.
    *   `parent="chassis"`: Both wheels are connected directly to the chassis.
    *   `<origin xyz="..."`: This is critical. It positions the wheel relative to the chassis's origin (which is its center).
        *   `xyz="0 0.175 -0.05"`: For the left wheel, we move it 0.175 units in the +Y direction (left) and -0.05 units in Z (down).
    *   `<axis xyz="0 1 0"/>`: The wheels should rotate around the Y-axis.
*   **Caster Joint:**
    *   `type="fixed"`: For this simple model, we'll make the caster a fixed sphere attached to the front of the chassis. It won't actually rotate or pivot.

## Visualizing the Robot

How can you see if your URDF is correct? You can use **RViz**, the standard ROS 2 visualization tool.

1.  **Install Joint State Publisher GUI:** This is a tool that lets you move your robot's joints with a slider.
    ```bash
    sudo apt install ros-humble-joint-state-publisher-gui
    ```
2.  **Create a Launch File:** Create a launch file to start RViz and publish the state of your robot.
    ```python
    # urdf_tutorial/launch/display.launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.substitutions import Command

    def generate_launch_description():
        pkg_share = get_package_share_directory('urdf_tutorial')
        urdf_file_path = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot.urdf')

        return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
            ),
        ])
    ```
3.  **Build and Launch:** After building and sourcing, run your launch file.
    ```bash
    ros2 launch urdf_tutorial display.launch.py
    ```
    This will open RViz and the Joint State Publisher GUI.
4.  **Configure RViz:**
    *   In the RViz "Displays" panel on the left, change the "Fixed Frame" to `chassis`.
    *   Click the "Add" button and add a "RobotModel" display.
    *   Add a "TF" display to see the coordinate frames.

You should now see your robot! Use the sliders in the Joint State Publisher GUI to rotate the wheel joints and see your robot model move in RViz.

You have now built and visualized your first robot. However, it's just a visual model. It has no mass or substance. In the next lesson, we'll add the physical properties that a simulator needs.
