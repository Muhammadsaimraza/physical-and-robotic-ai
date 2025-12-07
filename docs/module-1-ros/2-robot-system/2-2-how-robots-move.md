# Lesson 2.2: How Robots Move (Actuators)

If sensors are how a robot perceives the world, **actuators** are how it affects the world. An actuator is a device that converts an electrical signal into physical motion. They are the muscles of the robot.

In robotics, the most common type of actuator is an **electric motor**. The combination of a motor, its associated gearing, and the control electronics is often referred to as a **servo**.

## The Anatomy of a Robot Joint

A single robot joint, like an elbow or a knee, is a complex assembly.
1.  **The Motor:** The core of the actuator. Most modern robots use brushless DC motors for their high efficiency and power density.
2.  **The Gearbox:** A motor typically spins very fast but with low torque (rotational force). A gearbox is a system of gears that reduces the speed of the motor's output while increasing its torque. This is what gives the robot its strength. The **gear ratio** (e.g., 100:1) determines how much the speed is reduced and the torque is multiplied.
3.  **The Encoder:** As we learned in the last lesson, the encoder is the sensor that measures the angle of the joint. It is a critical part of the feedback loop.
4.  **The Motor Controller:** This is a small electronic circuit that takes a high-level command (e.g., "move to 90 degrees") and translates it into the precise electrical signals needed to make the motor turn. It uses the feedback from the encoder to ensure the motor reaches and holds the target position.

This entire assembly—motor, gearbox, encoder, and controller—is what allows a robot to perform precise, controlled movements.

## The Control Loop

How does a robot move its arm to a specific point in space? It uses a **control loop**.

1.  **Goal:** The robot's "brain" (a high-level software component) decides on a goal, for example, "set the elbow joint to 90 degrees."
2.  **Command:** This goal is sent to the motor controller for the elbow joint.
3.  **Action:** The motor controller sends power to the motor, causing it to turn.
4.  **Sensing:** As the motor turns, the joint encoder measures the new angle of the joint.
5.  **Feedback:** The new angle is sent back to the motor controller.
6.  **Comparison:** The controller compares the current angle to the goal angle.
    *   If they are the same, the controller stops the motor (or applies a small amount of power to hold the position against gravity).
    *   If they are different, the controller calculates the error and sends a new command to the motor to reduce the error.

This loop repeats hundreds or thousands of times per second. This is known as a **closed-loop control system**, and it is the fundamental principle that enables all modern robotics.

## Degrees of Freedom (DoF)

The number of independent joints a robot has is called its **degrees of freedom (DoF)**. Each joint that can be independently controlled adds one degree of freedom.

*   A simple robot arm might have 6 DoF (3 at the shoulder, 1 at the elbow, 2 at the wrist).
*   A humanoid robot can have 20, 30, or even more DoF.

The more degrees of freedom a robot has, the more complex and dextrous its movements can be. However, controlling a high-DoF system is also a significantly harder computational problem, as the robot's brain must coordinate all of these joints simultaneously to produce a single, smooth motion.

This coordination challenge is the subject of our next lesson.
