# Lesson 10.4: World Building with AI

Just as with URDF, writing complex SDF world files by hand can be tedious. You can use an AI assistant to quickly generate interesting and complex worlds for your robot to train and test in.

## Using AI for World Generation

Let's ask an LLM to create a world with a few obstacles.

---
**Prompt:**

> "You are an expert Gazebo and ROS 2 developer. Please generate a complete SDF world file named `obstacle_course.sdf`.
>
> The world should contain:
> 1. A ground plane.
> 2. A sun for lighting.
> 3. A "Construction Cone" model from the OpenRobotics Fuel marketplace at pose (3, 3, 0).
> 4. A "Jersey Barrier" model from the OpenRobotics Fuel marketplace at pose (5, -2, 0).
> 5. A simple box shape with size 1x5x1 meters at pose (0, 7, 0.5) to act as a wall.
>
> The SDF should be version 1.7, well-formatted, and commented."

---

**Review the Output:**
The AI should generate a complete `.sdf` file.
*   Does it include the `ground_plane` and `sun`?
*   Does it correctly include the two models from the Fuel marketplace with the correct poses?
*   Does it create a new model for the box wall? A primitive shape like a box must be contained within a `<model>` tag, which contains `<link>`, `<collision>`, and `<visual>` tags, just like in URDF. Did the AI do this correctly?

## Iterating with AI

This generated world can be a great starting point. You can now iterate on it.

---
**Follow-up Prompt:**

> "Thank you. Now, please add a second box wall to the `obstacle_course.sdf` file. It should be perpendicular to the first one, at pose (7, 4.5, 0.5)."

---

The AI should be able to take the previous context and add the new model to it, generating a complete, updated file.

## Your Role as a World Builder

This AI-assisted workflow is extremely powerful for creating test environments. You can quickly generate dozens of different worlds to test your robot's navigation and perception algorithms.

Your role is to:
1.  **Design the scenario:** What do you want to test? Obstacle avoidance? Navigating a maze? Finding a specific object?
2.  **Prompt the AI:** Give the AI a clear, high-level description of the world you want to create.
3.  **Validate and Refine:** Load the generated world in Gazebo. Does it look right? Are the objects in the correct places? Are there any errors in the console?
4.  **Integrate:** Use your generated world files in your launch scripts to run your tests.

By leveraging AI, you can spend more time thinking about *what* to test and less time on the tedious mechanics of writing XML.

This concludes Chapter 10. You can now create simulated worlds, populate them with objects, and spawn your robot into them. In the next chapter, we will give your simulated robot the ability to see by adding simulated sensors.
