# Lesson 3.4: Services & Parameters

Topics are great for continuous streams of data, but sometimes you need a request/response interaction. For example, "What is the current time?" or "Please reset the simulation." This is what **Services** are for.

**Parameters**, on the other hand, are a way to configure a node at runtime without having to restart it.

Let's explore these concepts using Turtlesim. Keep your `turtlesim_node` running.

---

## Services: The Request/Response Model

A service has a **server** (which provides the service) and a **client** (which calls the service). It's a two-way conversation.

Let's see what services the `turtlesim_node` provides. In a new terminal, run:
```bash
ros2 service list
```
You will see a long list, including:
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
...
```
These are all the actions you can "request" from the `turtlesim_node`.

Let's try calling the `/clear` service. This service erases the trail the turtle has left behind. To call a service from the command line, we use `ros2 service call`.
```bash
ros2 service call /clear std_srvs/srv/Empty
```
*   `/clear`: The name of the service.
*   `std_srvs/srv/Empty`: The **type** of the service. In this case, it's an "Empty" service, meaning the request doesn't need to contain any data.

When you run this, you'll see the turtle's trail disappear.

### Services with Arguments

Some services require arguments. Let's spawn a new turtle. First, let's see what information the `/spawn` service needs with `ros2 service info`.
```bash
ros2 service info /spawn
```
This will tell you the service type is `turtlesim/srv/Spawn`. We need more detail.
```bash
ros2 interface show turtlesim/srv/Spawn
```
This shows the structure of the request and response.
```
float32 x
float32 y
float32 theta
string name # Optional. A unique name will be created if this is empty.
---
string name
```
The request (above the `---`) needs an x, y, theta, and an optional name. The response (below the `---`) will return the name of the new turtle.

Now let's call it:
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0.5, name: 'turtle2'}"
```
A new turtle named `turtle2` will appear at position (1, 1) in the simulation!

---

## Parameters: Configuring a Node

Parameters allow us to change the configuration of a node while it's running.

Let's see what parameters the `turtlesim_node` has.
```bash
ros2 param list
```
You should see:
```
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```
The `/turtlesim` node has parameters to control the background color of the simulation.

Let's read the current value of the red component.
```bash
ros2 param get /turtlesim background_r
```
It will return `69`.

Now, let's change it. We can use `ros2 param set` to change the background to a nice yellow color (Red=255, Green=255, Blue=0).
```bash
ros2 param set /turtlesim background_r 255
ros2 param set /turtlesim background_g 255
ros2 param set /turtlesim background_b 0
```
As you run these commands, you will see the background color of the Turtlesim window change in real-time. You have just reconfigured a running ROS 2 node without restarting it.

This is an extremely powerful feature for tuning and calibrating robots. You can adjust motor controller gains, sensor filter settings, and more, all while the system is live.

---

This concludes Chapter 3. You have now mastered the fundamental command-line tools for interacting with a ROS 2 system. You can inspect nodes, topics, services, and parameters. With this foundation, you are ready to start writing your own ROS 2 nodes in the next chapter.
