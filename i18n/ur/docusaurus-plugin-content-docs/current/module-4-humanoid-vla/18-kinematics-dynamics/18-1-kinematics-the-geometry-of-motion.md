# Lesson 18.1: Kinematics: The Geometry of Motion

Kinematics is the "geometry of motion." It's a mathematical framework for describing the position, orientation, and velocity of the robot's body parts without considering the forces involved. For a robot arm or leg, there are two fundamental kinematic problems.

## 1. Forward Kinematics (FK)

**The Question:** "Given a set of joint angles, where is the robot's hand?"

This is the "easy" problem. Your URDF file is essentially a description of the forward kinematics of your robot. It defines the length of each link and the position and orientation of each joint.

If you know the angle of the shoulder joint, the elbow joint, and the wrist joint, you can use a series of matrix multiplications (one for each link's transform) to calculate the exact 3D position and orientation of the robot's hand (its "end-effector") relative to its base.

The ROS `robot_state_publisher` node does this continuously. It reads the joint angles from the `/joint_states` topic, performs the forward kinematics calculation, and publishes the resulting link poses as a TF tree. This is how RViz knows where to draw your robot.

## 2. Inverse Kinematics (IK)

**The Question:** "If I want to place the robot's hand at a specific target position, what should the joint angles be?"

This is the "hard" problem. It is the inverse of the FK problem, and it is much more difficult to solve.

*   **Multiple Solutions:** For a typical robot arm with 6 or 7 joints, there are often multiple, and sometimes infinite, possible sets of joint angles that will result in the same end-effector position. Think about touching your nose: you can do it with your elbow high or your elbow low. Both are valid IK solutions.
*   **No Solution:** If you try to reach for a target that is outside the robot's workspace, there is no solution.
*   **Computational Cost:** Solving the IK equations can be computationally expensive, and it needs to be done very quickly for real-time control.

## The IK Solver

Because this problem is so hard, we don't solve it ourselves. We use a specialized piece of software called an **IK solver**.

An IK solver is a library that takes a target pose for the end-effector and the robot's URDF as input, and it calculates a valid set of joint angles as output.

ROS has a standard framework for motion planning called **MoveIt**, which has a pluggable architecture for IK solvers. You can choose from a variety of different solvers based on your robot and your needs.

When you use a motion planner to say "move the arm to this target," the first thing the planner does is call the IK solver to find out what the final joint angles need to be. The planner then creates a trajectory through the joint space to move from the current angles to the target angles while avoiding collisions.
