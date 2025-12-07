# Lesson 7.2: Building the Controller

With a clear specification, the coding becomes much more straightforward. We are no longer designing on the fly; we are simply implementing the plan we've already made.

This lesson is a culmination of everything you've learned. You will need to create a new service interface, write a complex node that acts as both a service server and a publisher, and create a launch file to tie it all together.

## 1. Create the Custom Service

First, let's create the `DrawShape.srv` file in your `my_custom_interfaces` package.
1.  **Create file:** `my_custom_interfaces/srv/DrawShape.srv`
    ```
    float32 edge_length
    int32 sides
    ---
    bool success
    ```
2.  **Edit `CMakeLists.txt`:** Add the new service file to the `rosidl_generate_interfaces` call.
    ```cmake
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Person.msg"
      "srv/SendPerson.srv"
      "srv/DrawShape.srv"  # Add this
    )
    ```

## 2. Implement the `shape_drawer_node`

Create a new file, `my_first_package/shape_drawer.py`. The code for this node is more complex as it combines several concepts.

```python
# my_first_package/my_first_package/shape_drawer.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_custom_interfaces.srv import DrawShape
import time
import math

class ShapeDrawerNode(Node):
    def __init__(self):
        super().__init__('shape_drawer_node')
        
        # Declare parameters
        self.declare_parameter('linear_velocity', 1.0)
        self.declare_parameter('angular_velocity', 1.0)
        
        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create the service server
        self.srv = self.create_service(DrawShape, 'draw_shape', self.draw_shape_callback)
        
        self.get_logger().info('Shape Drawer Node has been started.')

    def draw_shape_callback(self, request, response):
        linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        
        edge_length = request.edge_length
        sides = request.sides

        self.get_logger().info(f'Drawing a shape with {sides} sides of length {edge_length}.')

        # Calculate times needed for movement and turning
        move_duration = edge_length / linear_vel
        turn_angle = 2 * math.pi / sides
        turn_duration = turn_angle / angular_vel

        # Loop to draw the shape
        for i in range(sides):
            # Move forward
            self.publish_velocity(linear_vel, 0.0)
            time.sleep(move_duration)
            
            # Stop
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.1)

            # Turn
            self.publish_velocity(0.0, angular_vel)
            time.sleep(turn_duration)

            # Stop
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.1)

        self.get_logger().info('Shape drawing complete.')
        response.success = True
        return response

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    shape_drawer_node = ShapeDrawerNode()
    rclpy.spin(shape_drawer_node)
    shape_drawer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Note:** The use of `time.sleep()` here is a simple approach for this exercise. In a real, production-quality robot, you would use more sophisticated feedback-based control (e.g., checking the turtle's pose from the `/turtle1/pose` topic) rather than relying on fixed-time movements.

## 3. Create the Launch File

Create `launch/capstone.launch.py` in your `my_first_package`.

```python
# my_first_package/launch/capstone.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='my_first_package',
            executable='shape_drawer',
            name='shape_drawer_node',
            parameters=[
                {'linear_velocity': 0.5},
                {'angular_velocity': 0.5}
            ]
        )
    ])
```

## 4. Register, Build, and Run

1.  **Register Node:** Add `shape_drawer = my_first_package.shape_drawer:main` to your `setup.py`.
2.  **Add Dependency:** Add `<depend>my_custom_interfaces</depend>` to the `package.xml` of `my_first_package`.
3.  **Build:** Run `colcon build` from your workspace root. It should build both `my_custom_interfaces` and `my_first_package`.
4.  **Source:** `source install/setup.bash`.
5.  **Launch the system:**
    ```bash
    ros2 launch my_first_package capstone.launch.py
    ```
    This will start both the Turtlesim simulator and your controller node. Your system is now running and waiting for a command.

In the final lesson, we will test the system against our specification.
