# Lesson 4.2: Writing a Publisher

Let's write our first real ROS 2 node. We'll create a "talker" node that publishes a simple string message to a topic at a regular interval.

This node will be a Python class that inherits from the `rclpy.node.Node` class.

## The Code

Inside your `my_first_package` directory, create a new file: `my_first_package/talker.py`.
```python
# my_first_package/my_first_package/talker.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """
    A ROS 2 node that publishes a string message every second.
    """
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    talker_node = TalkerNode()
    
    try:
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1.  **Imports:** We import `rclpy` (the ROS 2 client library for Python), `Node`, and the `String` message type from `std_msgs.msg`.
2.  **`TalkerNode` Class:**
    *   Our class inherits from `Node`.
    *   `super().__init__('talker_node')`: We call the parent constructor and give our node a name, `talker_node`.
    *   `self.create_publisher(String, 'chatter', 10)`: This is the core of the publisher. We create a publisher that sends `String` messages on the topic named `chatter`. The `10` is the "queue size," which is a quality-of-service setting that we'll explore later.
    *   `self.create_timer(1.0, self.timer_callback)`: We create a timer that will call our `timer_callback` method once every second.
3.  **`timer_callback` Method:**
    *   This function is executed every time the timer fires.
    *   `msg = String()`: We create a new `String` message object.
    *   `msg.data = ...`: We set the `data` field of the message.
    *   `self.publisher_.publish(msg)`: We publish the message to the topic.
    *   `self.get_logger().info(...)`: We write a log message to the console.
4.  **`main` Function:**
    *   `rclpy.init()`: Initializes the ROS 2 client library.
    *   `talker_node = TalkerNode()`: We create an instance of our node.
    *   `rclpy.spin(talker_node)`: This is a crucial line. It "spins" the node, which means it enters a loop that keeps the node alive and allows its callbacks (like our timer callback) to be processed. The program will stay in this loop until you press `Ctrl+C` in the terminal.
    *   `destroy_node()` and `shutdown()`: These clean up the node and the ROS 2 client library when the program exits.

## Registering the Node

Now we need to tell ROS 2 that this file contains an executable node. We do this in `setup.py`. Open the `setup.py` file in your package and add the `entry_points` section inside the `setup()` function:
```python
# setup.py

...
setup(
    ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
        ],
    },
)
```
This tells `colcon` to create an executable named `talker` that runs the `main` function from the `talker.py` file in your `my_first_package` library.

## Build and Run

1.  **Build:** Navigate to the root of your workspace (`ros2_ws`) and run `colcon build`.
2.  **Source:** In a new terminal, source your workspace: `source install/setup.bash`.
3.  **Run:** Now you can run your new node!
    ```bash
    ros2 run my_first_package talker
    ```
You should see your log messages printed to the screen once per second.

You can also use the command-line tools you learned in the last chapter to inspect your node. In another terminal (don't forget to source!), try:
```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```
You have successfully created and run your first ROS 2 publisher node. In the next lesson, you'll create a subscriber to listen to it.
