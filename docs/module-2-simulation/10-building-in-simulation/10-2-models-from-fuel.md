# Lesson 10.2: Models from Fuel

Building every object in your world from scratch would be time-consuming. Fortunately, Gazebo has access to a large online database of pre-made models called the **Ignition Fuel** marketplace. You can browse it at [app.ignitionrobotics.org](https://app.ignitionrobotics.org/fuel/models).

You can include any model from this marketplace in your SDF world file using the `<include>` tag, just as we did for the sun and ground plane.

## Including a Fuel Model

Let's add a construction cone to our world.

First, find the model on the Fuel website. If you search for "construction cone," you'll find it. The "owner" of the model is `OpenRobotics`.

Now, we can add it to our `empty_world.sdf` file.
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="empty_world">

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- A construction cone from Fuel -->
    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Construction Cone</uri>
      <name>cone_1</name>
      <pose>2.0 1.0 0.0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Breakdown
*   **`<uri>`:** Instead of `model://`, we provide the full URL to the model on the Fuel server. The format is `https://fuel.ignitionrobotics.org/1.0/<OWNER>/models/<MODEL_NAME>`.
*   **`<name>`:** We must give this instance of the model a unique name in our world, e.g., `cone_1`.
*   **`<pose>`:** This tag lets us set the initial position and orientation of the model in the world. The format is `x y z roll pitch yaw`. Here, we are placing the cone at `(x=2.0, y=1.0, z=0.0)`.

## Caching

The first time you launch a world that includes a Fuel model, Gazebo will download it from the internet and cache it on your local machine (usually in `~/.ignition/fuel`). The next time you launch the world, it will load the model from the local cache, which is much faster.

## Adding Your Robot

How do we add our own robot, which we defined in a URDF file? We can't include a URDF file directly in a world SDF file.

Instead, we will use a special "spawner" node from the `ros_gz_sim` package to add the robot to the simulation after it has started.

We will modify our `gazebo.launch.py` to do this.
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file_path],
            output='screen'
        ),
        
        # Spawn the robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', urdf_file_path,
                '-name', 'my_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen',
        ),
    ])
```
### Breakdown
1.  We add a new `Node` action.
2.  **`package='ros_gz_sim'`, `executable='create'`**: This runs the spawner node.
3.  **`arguments=[...]`**: We pass command-line arguments to the spawner.
    *   `-file`: The path to our URDF file.
    *   `-name`: A unique name for our robot in the simulation.
    *   `-x`, `-y`, `-z`: The initial coordinates to spawn the robot at.

Now, build and launch. You will see your empty world with the construction cone, and after a moment, your two-wheeled robot will appear at the origin.

It will probably fall over and look strange. Why? Because while our URDF has `<inertial>` tags, it doesn't have the Gazebo-specific tags needed for a complete physical simulation. We will fix that in the next lesson.
