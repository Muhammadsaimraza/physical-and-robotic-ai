# Lesson 13.2: Building the Simulation

This lesson involves assembling the pieces of the simulation based on our specification. You will create the world, ensure your URDF is complete, and write the final launch file.

## 1. Create the World

Create a new world file, `worlds/wall_follower_world.sdf`. This world needs a wall for the robot to follow. A simple box is sufficient.

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="wall_follower_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <model name="wall">
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </visual>
      </link>
      <static>true</static>
    </model>

  </world>
</sdf>
```
This creates a long, thin box to act as a wall at `y=5`.

## 2. Update the Robot URDF

Ensure your `two_wheeled_robot.urdf` from the previous chapters is complete and matches the specification. It must include:
*   The `chassis`, `left_wheel`, and `right_wheel` links with `<visual>`, `<collision>`, and `<inertial>` tags.
*   The `DiffDrive` plugin configured to listen on `/cmd_vel`.
*   A `lidar_link` and `lidar_joint`.
*   The `gpu_lidar` sensor plugin attached to the `lidar_link` and configured to publish on `/scan`.

## 3. Implement the Wall Follower Node

This is the `wall_follower.py` node you wrote in the previous chapter. Ensure it is complete and correctly implements the logic from the specification. Make sure it declares and uses the required parameters.

## 4. Create the Capstone Launch File

Create `launch/module2_capstone.launch.py`. This file will integrate all the components.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file = os.path.join(pkg_share, 'worlds', 'wall_follower_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot.urdf')

    # 1. Launch Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen',
    )
    
    # 3. Start the Wall Follower Node
    wall_follower = Node(
        package='my_first_package', # Assuming the node is in this package
        executable='wall_follower', # The name from your setup.py
        name='wall_follower_node',
        output='screen',
        parameters=[
            {'desired_distance': 1.5},
            {'forward_velocity': 0.3},
            {'proportional_gain': 0.8}
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        wall_follower,
    ])
```

## 5. Build and Run

1.  Make sure all your nodes (`wall_follower`) are correctly registered in their respective `setup.py` files and all dependencies are in `package.xml`.
2.  `colcon build`
3.  `source install/setup.bash`
4.  `ros2 launch urdf_tutorial module2_capstone.launch.py`

When you launch the system, you should see Gazebo open, the world with the wall appear, your robot spawn, and the robot should immediately start moving and trying to follow the wall.

You have now built an integrated, closed-loop robotics simulation. In the final lesson, you will formally test it.
