# Lesson 6.2: Launch Files

So far, every time you've wanted to run your talker/listener system, you've had to open two terminals, source the environment in both, and run a `ros2 run` command in each. This is tedious and doesn't scale.

A **Launch File** is a script that automates this process. It allows you to define a whole system of nodes, configure them, and start them all with a single command. In ROS 2, launch files are written in Python.

## Creating a Launch File

1.  **Create a `launch` directory:** Inside your `my_first_package` directory, create a new directory called `launch`.
2.  **Create the launch file:** Inside the `launch` directory, create a new Python file, for example `talker_listener.launch.py`.

```python
# my_first_package/launch/talker_listener.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the talker and listener system.
    """
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='talker',
            name='my_talker'
        ),
        Node(
            package='my_first_package',
            executable='listener',
            name='my_listener'
        )
    ])
```

### Code Breakdown

1.  **Imports:** We import `LaunchDescription` and the ROS-specific `Node` action.
2.  **`generate_launch_description()`:** This is the main function that ROS 2 will look for when you ask it to run the launch file. It must return a `LaunchDescription` object.
3.  **`LaunchDescription([...])`**: This object holds a list of all the **actions** you want to perform. The most common action is `Node`.
4.  **`Node(...)`**: This action tells the launch system to start a node. You must specify the `package` and the `executable` (the name you defined in `setup.py`). You can also optionally provide a unique `name` for the node.

## Installing the Launch File

You need to tell `colcon` to install your `launch` directory so that `ros2 launch` can find it. Open your `setup.py` and add the following `data_files` entry:

```python
# setup.py
import os
from glob import glob
...
setup(
    ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install all files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    ...
)
```

## Build and Launch

1.  **Build:** Run `colcon build`.
2.  **Source:** `source install/setup.bash`.
3.  **Launch:** Now, instead of two `ros2 run` commands, you can use one `ros2 launch` command:
    ```bash
    ros2 launch my_first_package talker_listener.launch.py
    ```
You will see the output from both the talker and the listener interleaved in the same terminal. Both nodes have been started by a single command.

## Setting Parameters in a Launch File

Launch files are also the perfect place to set parameters. Let's create a new launch file for our `talker_with_params` node.

```python
# my_first_package/launch/params_talker.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='talker_with_params',
            name='my_params_talker',
            parameters=[
                {'greeting': 'Bonjour'}
            ]
        )
    ])
```
The `parameters` argument takes a list of dictionaries.

Now, when you build and run this launch file, the node will start up and immediately begin publishing "Bonjour World: ..." without needing any command-line arguments.
```bash
ros2 launch my_first_package params_talker.launch.py
```

Launch files are the standard way to run any non-trivial ROS 2 system. They are essential for managing the complexity of a real robot. In the final lesson of this chapter, we'll look at the tools that help you debug these complex systems.
