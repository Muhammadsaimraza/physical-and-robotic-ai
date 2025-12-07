# Lesson 10.1: SDF World Basics

An SDF file is an XML file that describes a simulation world. Let's create a simple, empty world with just a light source and a ground plane.

Create a `worlds` directory in your `urdf_tutorial` package, and inside it, a file named `empty_world.sdf`.

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

  </world>
</sdf>
```

### Breakdown

*   **`<sdf version="1.7">`**: The root tag, specifying the version of the SDF specification being used.
*   **`<world name="empty_world">`**: This tag contains everything in our world.
*   **`<include>`**: This is a powerful tag that lets us pull in pre-existing models. Gazebo comes with several built-in models, including `sun` and `ground_plane`. The `model://` prefix tells Gazebo to look for these models in its standard library.

## Launching a World in Gazebo

To run this simulation, we need a launch file that starts the Gazebo simulator and tells it which world file to load.

We will use the `ros_gz_sim` package, which provides the bridge between ROS 2 and Gazebo.

Create a new launch file, `launch/gazebo.launch.py`.
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file_path],
            output='screen'
        ),
    ])
```

### Breakdown
1.  We find the full path to our `empty_world.sdf` file.
2.  We use the `ExecuteProcess` action to run a command line process.
3.  The command is `gz sim -r empty_world.sdf`.
    *   `gz sim`: The command to run the Gazebo simulator.
    *   `-r`: This flag tells Gazebo to start the simulation paused. This is good practice, as it gives all your nodes time to start up before the physics engine begins running.
    *   `world_file_path`: The path to the world we want to load.

## Build and Launch

1.  **Add Dependency:** Add a dependency on `ros_gz_sim` in your `urdf_tutorial/package.xml`.
    ```xml
    <exec_depend>ros_gz_sim</exec_depend>
    ```
2.  **Build:** `colcon build`.
3.  **Source:** `source install/setup.bash`.
4.  **Launch:**
    ```bash
    ros2 launch urdf_tutorial gazebo.launch.py
    ```
This will open the Gazebo GUI. You will see a flat, gray ground plane under a bright light. At the bottom, you will see that the simulation time is `0.000` and it is paused. You can press the "Play" button in the GUI to start the simulation.

You have now created and launched your first Gazebo world. In the next lessons, we'll learn how to add more interesting objects to it.
