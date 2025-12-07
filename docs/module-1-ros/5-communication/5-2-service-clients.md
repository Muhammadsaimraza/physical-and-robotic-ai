# Lesson 5.2: Service Client

Calling a service from the command line is useful for testing, but in a real robot, you'll have another node that acts as the **client**. The client is responsible for sending the request and waiting for the response.

A key difference from subscribers is that a service call is typically **synchronous**. The client code will pause and wait until the server sends back the response.

## The Code

Let's create a client node to call our `add_two_ints` service. Create a new file: `my_first_package/add_two_ints_client.py`.
```python
# my_first_package/my_first_package/add_two_ints_client.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    """
    A ROS 2 node that calls the add_two_ints service.
    """
    def __init__(self):
        super().__init__('add_two_ints_client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    client_node = AddTwoIntsClientNode()
    response = client_node.send_request(5, 10)
    
    if response:
        client_node.get_logger().info(
            f'Result of add_two_ints: for {5} + {10} = {response.sum}')
    else:
        client_node.get_logger().error('Service call failed')
        
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1.  **`AddTwoIntsClientNode` Class:**
    *   `self.create_client(AddTwoInts, 'add_two_ints')`: We create a client for a service of type `AddTwoInts` on the topic `'add_two_ints'`.
    *   `self.client.wait_for_service(...)`: This is a crucial step. The client will wait here until it can connect to a server. This prevents the client from trying to send a request before the server is ready.
2.  **`send_request` Method:**
    *   `self.req.a = a; self.req.b = b`: We populate the request object.
    *   `self.client.call_async(self.req)`: We send the request. This call is **asynchronous**—it returns immediately with a "Future" object that represents a task that will be completed in the future.
    *   `rclpy.spin_until_future_complete(self, self.future)`: This is the synchronous part. We spin the node, blocking our program, until the Future object is populated with a result from the server.
    *   `return self.future.result()`: We return the result contained within the Future.
3.  **`main` Function:**
    *   We create the client node.
    *   We call `send_request` with some numbers.
    *   We log the response we get back.
    *   **Note:** We don't need a `try/except` block with `rclpy.spin` here because our node doesn't need to run forever. It sends one request, gets one response, and then exits.

## Registering and Building

1.  **Register Node:** Add the new client node to your `setup.py`:
    ```python
    'console_scripts': [
        'add_two_ints_server = my_first_package.add_two_ints_server:main',
        'add_two_ints_client = my_first_package.add_two_ints_client:main',
    ],
    ```
2.  **Build:** Run `colcon build`.
3.  **Source:** `source install/setup.bash`.
4.  **Run:**
    *   In one terminal, make sure your server is running:
        ```bash
        ros2 run my_first_package add_two_ints_server
        ```
    *   In a second terminal, run your new client:
        ```bash
        ros2 run my_first_package add_two_ints_client
        ```

The client will start, send its request, get the response, print the result, and then exit. You have now built a complete request/response system.

In the next lesson, you'll learn how to break free from the built-in message types and create your own.
