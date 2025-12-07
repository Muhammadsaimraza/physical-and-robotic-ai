# Lesson 12.4: Creating Skills

The "wall follower" you built in the last lesson is an example of a **skill** or a **behavior**. It's a self-contained ROS 2 node (or collection of nodes) that accomplishes a specific, reusable task.

Modern robotics is moving away from monolithic, hard-coded control programs and towards a library of composable skills. A high-level planner can then sequence these skills to accomplish complex goals.

## Characteristics of a Good Skill

*   **Modular:** It should be a self-contained ROS 2 package with a clear interface.
*   **Configurable:** It should use parameters to allow for easy tuning and adaptation to new robots or environments. Our wall follower could have parameters for `desired_distance` and `k_proportional`.
*   **Well-Defined API:** It should have a clear way to start, stop, and monitor its progress. For simple skills, just running the node is enough. For more complex skills, you would typically use a ROS 2 **Action** (which we will cover in a future module). An action provides a formal way to send a goal (e.g., "follow the wall for 10 meters"), receive feedback, and get a final result.

## Example: A "Go to Goal" Skill

Let's design another skill. This one will take our robot to a specific (x, y) coordinate.

*   **Node Name:** `go_to_goal_node`
*   **Subscribes to:**
    *   `/odom` (`nav_msgs/msg/Odometry`): To get the robot's current position and orientation. We would get this topic by using the `ros_gz_bridge` to bridge Gazebo's pose information for our robot.
*   **Publishes to:**
    *   `/cmd_vel` (`geometry_msgs/msg/Twist`): To control the robot's movement.
*   **Action Server:**
    *   `/go_to_goal` (`my_interfaces/action/GoToGoal`): An action server to receive the target coordinate.
*   **Logic:**
    1.  Receive a goal with a target (x, y) pose.
    2.  In a control loop, continuously:
        a.  Read the current pose from `/odom`.
        b.  Calculate the error between the current pose and the goal pose.
        c.  Calculate a `linear.x` and `angular.z` velocity to reduce the error (this is a more complex proportional controller than the wall follower).
        d.  Publish the velocity command to `/cmd_vel`.
        e.  Publish feedback on the action topic with the current distance to the goal.
    3.  When the error is close to zero, stop the robot and report that the goal was successful.

## Composing Skills

Once you have a library of these skills, a high-level "brain" node can use them to perform complex tasks.

Imagine you have skills for:
*   `GoToGoal`
*   `FindObject` (using a camera)
*   `PickUpObject` (using a robot arm)

A brain node could receive a command like "fetch me the red ball from the other room" and translate it into a sequence of calls to these skill action servers:
1.  Call `GoToGoal` with the coordinates of the "other room."
2.  Once that succeeds, call `FindObject` with "red ball" as the target.
3.  Once that succeeds (returning the position of the ball), call `GoToGoal` with the position of the ball.
4.  Once that succeeds, call `PickUpObject`.

This hierarchical, behavior-based architecture is a powerful and scalable way to build complex autonomous systems. You are not just writing code; you are creating a library of reusable robotic capabilities.

This concludes Chapter 12. You now understand how to integrate your ROS 2 nodes with a Gazebo simulation to create closed-loop behaviors. In the final chapter of this module, you will put all of these pieces together to complete the Module 2 capstone project.
