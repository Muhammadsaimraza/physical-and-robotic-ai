# Lesson 5.3: Custom Interfaces

So far, we have used built-in message and service types like `std_msgs/msg/String` and `example_interfaces/srv/AddTwoInts`. But for any real robot, you will need to define your own data structures. In ROS 2, these are called **interfaces**.

There are three types of interfaces:
*   `.msg`: Message files, for topics.
*   `.srv`: Service files, for services.
*   `.action`: Action files, for long-running tasks (which we'll cover later).

Let's create a package to hold our custom interfaces and then use them in our Python nodes.

## Creating an Interface Package

It is a best practice to keep your interface definitions in a separate package from the nodes that use them.

1.  **Create Package:** In the `src` directory of your workspace, create a new package.
    ```bash
    ros2 pkg create --build-type ament_cmake my_custom_interfaces
    ```
    **Important:** Interface packages must be `ament_cmake` packages, not `ament_python`.

2.  **Create Directories:** Inside the new package, create directories named `msg` and `srv`.
    ```bash
    cd my_custom_interfaces
    mkdir msg
    mkdir srv
    ```

3.  **Define a `.msg` file:** Create a new file `msg/Person.msg`.
    ```
    # msg/Person.msg
    string first_name
    string last_name
    uint8 age
    ```
    The format is simple: `type name`.

4.  **Define a `.srv` file:** Create `srv/SendPerson.srv`.
    ```
    # srv/SendPerson.srv
    my_custom_interfaces/msg/Person person_to_send
    ---
    bool success
    ```
    The `---` separates the request from the response. Here, the request is our custom `Person` message, and the response is a simple boolean.

## Configuring the Build

Because this is a `ament_cmake` package, the build configuration is different from our Python package.

1.  **Edit `CMakeLists.txt`:** You need to tell CMake to find the ROS 2 interface generation tools and then specify which files to build. Add these lines:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Person.msg"
      "srv/SendPerson.srv"
    )
    ```

2.  **Edit `package.xml`:** You need to add build dependencies.
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

## Using the Custom Interfaces

Now you can use these interfaces in your Python package.

1.  **Add Dependency:** In your `my_first_package/package.xml`, add a dependency on your new interface package.
    ```xml
    <depend>my_custom_interfaces</depend>
    ```

2.  **Import and Use:** Now you can import `Person` and `SendPerson` just like any other message or service type.
    ```python
    # In your python node
    from my_custom_interfaces.msg import Person
    from my_custom_interfaces.srv import SendPerson
    
    # ...
    
    # Publishing a custom message
    person_msg = Person()
    person_msg.first_name = "John"
    person_msg.last_name = "Doe"
    person_msg.age = 30
    self.publisher_.publish(person_msg)
    
    # ...
    
    # Inside a service callback
    # request.person_to_send will be a Person object
    self.get_logger().info(f'Received person: {request.person_to_send.first_name}')
    response.success = True
    return response
    ```

## Build and Run

1.  **Build:** Go to your workspace root (`ros2_ws`) and run `colcon build`. Colcon will be smart enough to build `my_custom_interfaces` *before* `my_first_package` because of the dependency you added.
2.  **Source:** `source install/setup.bash`.
3.  **Verify:** You can now see your new types with the `ros2 interface` command:
    ```bash
    ros2 interface show my_custom_interfaces/msg/Person
    ros2 interface show my_custom_interfaces/srv/SendPerson
    ```

Creating custom interfaces is a fundamental skill. It allows you to create clean, self-documenting APIs for all the nodes in your system. In the final lesson of this chapter, we'll discuss when to choose a topic versus a service for your API.
