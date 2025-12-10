# Lesson 12.2: Spawning Robots (Review)

We briefly covered this in Chapter 10, but it's worth looking at the mechanism for adding your robot to the simulation in more detail.

You cannot include a URDF file directly in a world SDF file. The SDF standard does not have an `<include>` tag for URDFs. Instead, the standard workflow is:
1.  Start the Gazebo simulation with a world file that contains the environment (ground, lights, obstacles, etc.).
2.  Use a separate mechanism to **spawn** the robot's URDF model into the running simulation.

## The `create` Executable

The `ros_gz_sim` package provides a helpful executable for this purpose, called `create`. As we saw before, we can use it in a launch file like this:

```python
# From a launch file

robot_urdf = os.path.join(get_package_share_directory('my_package'), 'urdf', 'my_robot.urdf')

spawn_robot_node = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file', robot_urdf,
        '-name', 'my_robot',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.1'
    ],
    output='screen',
)
```

## How it Works Under the Hood

The `create` executable is a convenience wrapper that does two things:
1.  **It reads your URDF file.** It parses the XML and holds the robot's description in memory.
2.  **It calls a Gazebo service.** Gazebo has a built-in service that allows you to add new models to the world while it's running. The `create` node calls this service and passes it the robot's description from the URDF file.

The service call is essentially saying, "Hey Gazebo, please add a new model to your world. Here is the description of the model." Gazebo then adds the robot to its physics engine and starts simulating it.

## Why Spawn Instead of Include?

This two-step process might seem more complicated than just including the robot in the world file, but it provides a critical advantage: **modularity**.

*   **Robot and World are Decoupled:** Your robot's URDF file is completely separate from your world SDF file. This means you can test the *same robot* in *many different worlds* without having to edit any files. You can simply pass a different world file path to your launch file.
*   **Dynamic Spawning:** You can spawn robots at any time, not just at the beginning of the simulation. You could have a ROS 2 node that decides to spawn a new robot in response to some event.

This separation of the robot from its environment is a fundamental concept in ROS and Gazebo. It allows you to create reusable robot models and reusable test environments, and then mix and match them as needed.

In the next lesson, we will use the topics provided by our spawned robot's sensors to create a closed-loop controller.
