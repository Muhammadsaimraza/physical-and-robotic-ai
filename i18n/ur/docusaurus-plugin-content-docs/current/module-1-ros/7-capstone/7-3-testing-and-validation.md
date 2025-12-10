# Lesson 7.3: Testing & Validation

Your system is built and running. Now, how do you know if it works? You must test it against the requirements laid out in your specification.

With your capstone system running (via `ros2 launch my_first_package capstone.launch.py`), open a new terminal and source your workspace. We will now go through the requirements from Lesson 7.1 and validate each one.

---

### Requirement 1: The system shall be able to command a turtle to draw a polygon.

This is the core requirement. We can test this by calling our `/draw_shape` service. Let's try to draw a square (4 sides).

```bash
ros2 service call /draw_shape my_custom_interfaces/srv/DrawShape "{sides: 4, edge_length: 2.0}"
```
**Expected Behavior:** The turtle in the Turtlesim window should move forward, turn, move forward, turn, and so on, until it has drawn a complete square.
**Actual Behavior:** Observe the turtle. Does it draw a square?

Let's try a triangle.
```bash
ros2 service call /draw_shape my_custom_interfaces/srv/DrawShape "{sides: 3, edge_length: 3.0}"
```
**Expected Behavior:** The turtle should draw an equilateral triangle.
**Actual Behavior:** Does it?

**Result:** PASS / FAIL

---

### Requirement 2: The shape's size and drawing speed shall be configurable.

Our specification called for the size to be configured via the service call and the speed to be configured via parameters.

**Test 2a: Size Configuration**
Call the service with a different `edge_length`.
```bash
ros2 service call /draw_shape my_custom_interfaces/srv/DrawShape "{sides: 4, edge_length: 4.0}"
```
**Expected Behavior:** The turtle should draw a larger square than the first test.
**Actual Behavior:** Does it?

**Result:** PASS / FAIL

**Test 2b: Speed Configuration**
Our launch file set the default speeds to 0.5. We can override these from the command line when we launch. Stop your current launch file (`Ctrl+C`) and restart it with new parameters:
```bash
ros2 launch my_first_package capstone.launch.py linear_velocity:=1.5 angular_velocity:=1.5
```
Now, call the service to draw a square again:
```bash
ros2 service call /draw_shape my_custom_interfaces/srv/DrawShape "{sides: 4, edge_length: 2.0}"
```
**Expected Behavior:** The turtle should draw the square noticeably faster than in the first test.
**Actual Behavior:** Is it faster?

**Result:** PASS / FAIL

---

### Requirement 3: The drawing process shall be triggered by a service call.

We have already validated this in the previous tests. The turtle only moves when we explicitly call the `/draw_shape` service.

**Result:** PASS

---

### Requirement 4: The system shall be started with a single launch file.

We have also validated this. The `ros2 launch my_first_package capstone.launch.py` command successfully starts both the `turtlesim_node` and our `shape_drawer_node`. We can verify this by using `ros2 node list` in another terminal while the system is running.

**Result:** PASS

---

## Conclusion

If you have passed all the validation steps, congratulations! You have successfully designed, implemented, and tested a multi-node ROS 2 system from a specification. You have demonstrated mastery of all the core concepts of this module: nodes, topics, services, custom interfaces, parameters, and launch files.

This is a significant achievement. The workflow you practiced in this chapter—spec, build, test—is the same workflow used to build some of the most complex robots in the world.

You are now ready to move on to Module 2, where you will learn to apply these skills to more complex, 3D robots in high-fidelity simulators.
