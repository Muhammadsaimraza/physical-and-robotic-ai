# Lesson 7.1: The Specification

Before writing a single line of code for a complex system, a good engineer writes a specification. A specification is a detailed description of *what* the system should do, not *how* it should do it. It defines the components, their interfaces, and their expected behavior.

For this capstone, we will design a system that commands a turtle in the Turtlesim simulator to draw a shape.

## High-Level Requirements

1.  The system shall be able to command a turtle to draw a polygon (e.g., a square).
2.  The shape's size (edge length) and drawing speed shall be configurable.
3.  The drawing process shall be triggered by a service call.
4.  The system shall be started with a single launch file.

## System Components

Based on these requirements, we can identify the need for at least two nodes:

1.  **A Controller Node:** This node will be responsible for the "business logic" of drawing the shape. It will translate the high-level goal ("draw a square") into a sequence of low-level velocity commands.
2.  **A Turtlesim Node:** The simulator itself, which we will launch as part of our system.

## Interface Design (The "API")

How will our nodes communicate?

1.  **Controller → Turtlesim:** The controller needs to send velocity commands to the simulator. We will use the standard `/turtle1/cmd_vel` topic with `geometry_msgs/msg/Twist` messages, just as `turtle_teleop_key` did.
2.  **User → Controller:** The user needs a way to trigger the drawing process. A service is a perfect fit for this. We will create a service that the user can call to start the drawing.

Let's define our service. We'll need a custom service type.

### Custom Service: `DrawShape.srv`

Let's create this in our `my_custom_interfaces` package.
```
# srv/DrawShape.srv
float32 edge_length
int32 sides
---
bool success
```
*   **Request:** `edge_length` defines the size of the shape, and `sides` defines how many sides the polygon has (e.g., 4 for a square).
*   **Response:** A simple boolean to indicate if the drawing was completed successfully.

## Node-by-Node Specification

### 1. `shape_drawer_node`

*   **Node Name:** `shape_drawer_node`
*   **Purpose:** Provides a service to draw a shape and publishes the necessary velocity commands.
*   **Parameters:**
    *   `linear_velocity` (float, default: 1.0): The forward speed of the turtle while drawing.
    *   `angular_velocity` (float, default: 1.0): The turning speed of the turtle at the corners.
*   **Services Provided:**
    *   `/draw_shape` (`my_custom_interfaces/srv/DrawShape`): Receives a request to draw a shape.
*   **Topics Published:**
    *   `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`): Publishes velocity commands to move the turtle.
*   **Behavior:**
    1.  When the `/draw_shape` service is called, the node receives the `edge_length` and `sides`.
    2.  It calculates the required turning angle for the corners (360 / sides).
    3.  It enters a loop that repeats for the number of sides.
    4.  In each loop iteration, it:
        a.  Publishes a `Twist` message with `linear.x` set to the `linear_velocity` parameter for a duration of (`edge_length` / `linear_velocity`) seconds.
        b.  Publishes a `Twist` message with `angular.z` set to the `angular_velocity` parameter for a duration of (`turn_angle` / `angular_velocity`) seconds.
    5.  After the loop completes, it publishes a zero-velocity `Twist` message to stop the turtle.
    6.  It returns `success: true` in the service response.

### 2. `turtlesim_node`

*   This is the standard node from the `turtlesim` package. We don't need to write it, just launch it.

## Launch File Specification

*   **Name:** `capstone.launch.py`
*   **Actions:**
    1.  Start the `turtlesim_node` from the `turtlesim` package.
    2.  Start our `shape_drawer_node`.
    3.  Pass default parameter values to the `shape_drawer_node` (e.g., `linear_velocity: 0.5`, `angular_velocity: 0.5`).

---

With this specification, we have a complete and unambiguous plan. We know exactly what to build, how the components will interact, and what the final behavior should be. In the next lesson, you will implement this plan.
