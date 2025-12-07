# Lesson 5.1: Service Server

A ROS 2 Service is a request/response communication pattern. A **server** node advertises a service, and a **client** node can call that service. The client then waits for the server to send back a response.

In this lesson, we will create a server node that provides a simple service to add two integers.

## The Code

We will use the built-in service type `example_interfaces/srv/AddTwoInts`. You can inspect its structure by running:
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```
You will see that the request contains two 64-bit integers, `a` and `b`, and the response contains one 64-bit integer, `sum`.

Now, let's create the server. Create a new file in your package: `my_first_package/add_two_ints_server.py`.
```python
# my_first_package/my_first_package/add_two_ints_server.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    """
    A ROS 2 node that provides a service to add two integers.
    """
    def __init__(self):
        super().__init__('add_two_ints_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server_node = AddTwoIntsServerNode()
    rclpy.spin(add_two_ints_server_node)
    add_two_ints_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1.  **Imports:** We import the service type `AddTwoInts` from `example_interfaces.srv`.
2.  **`AddTwoIntsServerNode` Class:**
    *   `self.create_service(...)`: This is the core of the server.
        *   `AddTwoInts`: The type of the service.
        *   `'add_two_ints'`: The name of the service.
        *   `self.add_two_ints_callback`: The function to call when a request is received.
3.  **`add_two_ints_callback` Method:**
    *   This function takes two arguments: `request` and `response`. `request` holds the incoming data (`a` and `b`), and `response` is an empty object that we must fill out.
    *   `response.sum = request.a + request.b`: We perform the addition and set the `sum` field of the response object.
    *   `return response`: We return the populated response object. This sends the response back to the client.

## Registering and Building

1.  **Add Dependency:** You must add a dependency on `example_interfaces` in your `package.xml`:
    ```xml
    <depend>example_interfaces</depend>
    ```
2.  **Register Node:** Add the new node to your `setup.py`:
    ```python
    'console_scripts': [
        'add_two_ints_server = my_first_package.add_two_ints_server:main',
        ...
    ],
    ```
3.  **Build:** Run `colcon build` from your workspace root.
4.  **Source:** Source your workspace in a new terminal with `source install/setup.bash`.
5.  **Run:** Run your server node:
    ```bash
    ros2 run my_first_package add_two_ints_server
    ```
    You should see the "server has been started" log message.

## Testing from the CLI

Your server is now running and waiting for requests. We can test it from the command line using `ros2 service call`.

Open a second terminal and run:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```
In the server terminal, you will see the log message showing the incoming request and the sum it calculated. In the client terminal, you will see the response it received:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=10)

response:
example_interfaces.srv.AddTwoInts_Response(sum=15)
```
You have successfully created a ROS 2 service. In the next lesson, you will create a dedicated client node to call it.
