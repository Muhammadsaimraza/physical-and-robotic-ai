# Lesson 6.1: Parameters

Hard-coding values inside your code is generally a bad practice. What if you want to change the timer rate of your talker node? Or change the `chatter` topic to `whisper`? You could edit the code and rebuild, but there's a much better way: **Parameters**.

Parameters are a way to provide configuration values to a node when it starts up. Every node has a built-in parameter server that other nodes can access.

## Declaring Parameters

Let's modify our talker node to use a parameter for the message content.

```python
# my_first_package/talker_with_params.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerWithParamsNode(Node):
    def __init__(self):
        super().__init__('talker_with_params_node')
        
        # Declare the parameter
        self.declare_parameter('greeting', 'Hello')
        
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker with params node has been started.')

    def timer_callback(self):
        # Get the parameter value
        greeting = self.get_parameter('greeting').get_parameter_value().string_value
        
        msg = String()
        msg.data = f'{greeting} World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# ... main function remains the same ...
def main(args=None):
    rclpy.init(args=args)
    node = TalkerWithParamsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown
1.  **`self.declare_parameter('greeting', 'Hello')`**:
    *   This function does two things: it tells the node that a parameter named `greeting` exists, and it provides a **default value** (`'Hello'`).
    *   If no value is provided for this parameter when the node starts, it will use the default.
2.  **`self.get_parameter('greeting').get_parameter_value().string_value`**:
    *   This is how you retrieve the parameter's current value inside your node.
    *   It's a bit verbose, but it's explicit: you get the parameter object, then get its value object, then get the actual value cast to the correct type (`string_value`, `integer_value`, etc.).

## Register, Build, and Run

1.  **Register Node:** Add `talker_with_params = my_first_package.talker_with_params:main` to your `setup.py`.
2.  **Build:** `colcon build`.
3.  **Source:** `source install/setup.bash`.
4.  **Run:**
    *   **Without setting the parameter:**
        ```bash
        ros2 run my_first_package talker_with_params
        ```
        The node will publish "Hello World: ...".

    *   **Setting the parameter from the command line:**
        ```bash
        ros2 run my_first_package talker_with_params --ros-args -p greeting:="Hola"
        ```
        Now the node will publish "Hola World: ...".

The `--ros-args -p <param_name>:=<param_value>` syntax is the standard way to set a parameter for a node at startup.

## Changing Parameters at Runtime

You can also change a parameter while the node is running, just like you did with Turtlesim in Chapter 3.

1.  Start the node without setting the parameter.
2.  In a second terminal, list the parameters for the node:
    ```bash
    ros2 param list
    ```
    You will see `/talker_with_params_node` and its `greeting` parameter.
3.  Now, change the parameter:
    ```bash
    ros2 param set /talker_with_params_node greeting "Bonjour"
    ```
You will immediately see the output of your running node change from "Hello World" to "Bonjour World".

Parameters are the key to creating reusable and flexible nodes. By externalizing configuration, you can adapt your node to different robots and different situations without ever touching the source code. In the next lesson, we'll see how to make this even easier by setting parameters from a launch file.
