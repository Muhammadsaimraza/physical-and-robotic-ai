# Lesson 4.3: Writing a Subscriber

A publisher isn't very useful on its own. We need another node to receive the messages. In this lesson, you will create a "listener" node that subscribes to the `chatter` topic and prints the messages it receives.

## The Code

Create a new file in your package: `my_first_package/listener.py`.
```python
# my_first_package/my_first_package/listener.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    """
    A ROS 2 node that subscribes to a string message and prints it.
    """
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info('Listener node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    
    listener_node = ListenerNode()
    
    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        pass
        
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

The structure is very similar to the publisher.

1.  **`ListenerNode` Class:**
    *   `super().__init__('listener_node')`: We give the node a unique name.
    *   `self.create_subscription(...)`: This is the core of the subscriber.
        *   `String`: The type of message to listen for.
        *   `'chatter'`: The name of the topic to subscribe to. This **must** match the topic name used by the publisher.
        *   `self.listener_callback`: The name of the function to call whenever a new message is received.
        *   `10`: The queue size.
2.  **`listener_callback` Method:**
    *   This function is the **callback**. It is executed automatically by `rclpy` every time a message arrives on the `chatter` topic.
    *   The message itself is passed as the first argument, `msg`.
    *   `self.get_logger().info(...)`: We simply log the data field of the received message.
3.  **`main` Function:**
    *   The `main` function is almost identical to the publisher's. We initialize `rclpy`, create an instance of our node, and then call `rclpy.spin()` to keep it alive and process incoming messages.

## Registering the Node

Just like with the publisher, we need to add an entry point for our new node in `setup.py`.
```python
# setup.py

...
setup(
    ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
            'listener = my_first_package.listener:main', # Add this line
        ],
    },
)
```

## Build and Run

1.  **Build:** Go to the root of your workspace (`ros2_ws`) and run `colcon build` again. Colcon is smart enough to only rebuild what has changed.
2.  **Source:** You need to source your workspace again in any new terminals you open. `source install/setup.bash`.
3.  **Run:** Now you can run both nodes.
    *   In one terminal, run the talker:
        ```bash
        ros2 run my_first_package talker
        ```
    *   In a **second** terminal, run the listener:
        ```bash
        ros2 run my_first_package listener
        ```

You should see the talker publishing its "Hello World" messages, and you will see the listener printing "I heard: ..." for every message it receives.

You have now created a complete, multi-node communication system in ROS 2. You have two independent Python programs communicating with each other using the publish/subscribe pattern, orchestrated by the ROS 2 middleware.

In the next lesson, we'll explore how you can use AI to accelerate this development process.
