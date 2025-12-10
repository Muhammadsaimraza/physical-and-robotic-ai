# Lesson 19.1: What is Motion Planning?

Motion planning is the process of finding a valid sequence of movements to get a robot from a starting configuration to a goal configuration. It seems simple, but it is a computationally very hard problem.

## Configuration Space (C-Space)

To understand why, we need to think about the robot's **Configuration Space (C-Space)**. C-Space is an abstract mathematical space where each point represents a unique configuration of the robot's joints.
*   For a simple 2-joint arm, the C-Space is a 2D square.
*   For a 6-DoF arm, the C-Space is a 6-dimensional hyper-volume.

Every point in this high-dimensional space is a set of joint angles. The motion planning problem is to find a path from the starting point to the goal point in this C-Space.

## The Obstacles

The problem is that not all of C-Space is valid. Some configurations are forbidden.
*   **Self-Collisions:** Some joint configurations would cause the robot's arm to run into its own body.
*   **Environmental Collisions:** Some joint configurations would cause the arm to run into a table, a wall, or another object in the environment.

These invalid configurations form "obstacles" in C-Space. The motion planner's job is to find a path through C-Space that avoids these obstacles.

## Sampling-Based Planners

For a high-dimensional space, it is impossible to map out all the obstacles. Instead, modern motion planners use **sampling-based algorithms**, like **RRT (Rapidly-exploring Random Tree)**.

The RRT algorithm works like this:
1.  Start with the robot's current configuration.
2.  Randomly pick a new, "goal" configuration in C-Space.
3.  Try to move a small step from the current configuration toward the goal configuration.
4.  **Collision Check:** Check if this new, small step results in a collision (either with the robot itself or the environment). This is done in the 3D "workspace" using the robot's collision geometry.
5.  If there is no collision, add the new configuration to a "tree" of valid states and repeat from the new state.
6.  If there is a collision, discard the new configuration and try a different random goal.

By repeating this process thousands of times, the algorithm rapidly explores the valid, collision-free regions of the C-Space, building a tree of reachable configurations that eventually connects the start state to the goal state. The path through this tree is the planned motion.

This process—randomly sampling, checking for collisions, and building a tree—is the core of almost all modern motion planning systems.
