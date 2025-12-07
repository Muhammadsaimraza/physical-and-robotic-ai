# Lesson 18.2: Dynamics: The Physics of Motion

Kinematics describes *how* a robot can move. Dynamics describes *why* it moves. It is the study of motion in relation to the forces, torques, and energy that produce it.

For a wheeled robot, the dynamics are relatively simple. For a legged humanoid robot, the dynamics are incredibly complex, and they all revolve around one central problem: **balance**.

A walking robot is often described as an "inverted pendulum." It is an inherently unstable system that is constantly in a state of controlled falling. Mastering the dynamics of this controlled fall is the key to bipedal locomotion.

## Center of Mass (CoM)

The **Center of Mass (CoM)** is the single point on the robot where the force of gravity can be considered to be acting. It is the average position of all the mass in the robot.

To be statically stable (i.e., balanced without moving), the robot's CoM must be located vertically above its **support polygon**. The support polygon is the area on the ground enclosed by the robot's feet.
*   If the robot is standing on two feet, the support polygon is the area that includes both feet.
*   If the robot lifts one foot, the support polygon shrinks dramatically to just the area of the single foot that is on the ground.

A primary goal of any walking algorithm is to move the robot's body in such a way that the CoM stays within the support polygon.

## Zero Moment Point (ZMP)

The CoM is useful for static balance, but walking is a *dynamic* process. The robot is accelerating and decelerating, which creates inertial forces that also affect its balance.

The **Zero Moment Point (ZMP)** is a more advanced concept that accounts for these dynamic forces. The ZMP is the point on the ground where the total net moment (or torque) from both gravity and the inertial forces is zero.

The rule for dynamic stability is simple: **for the robot not to fall over, the ZMP must always remain inside the support polygon.**

A walking gait is a carefully choreographed dance to keep the ZMP under control.
1.  Before taking a step, the robot shifts its body to move the ZMP onto the foot that will remain on the ground.
2.  It then lifts the other foot and swings it forward.
3.  As the swinging foot moves forward, the robot adjusts its torso and arms to keep the ZMP within the tiny support polygon of the single stationary foot.
4.  As the swinging foot lands, the support polygon expands to include both feet. The robot can then shift its weight and the ZMP onto the new support foot to begin the next step.

Modern humanoid control systems are complex algorithms that are constantly calculating the current position of the ZMP and commanding the robot's joints to move in a way that keeps the ZMP within the stable region. This is the fundamental challenge of making a robot walk like a human.
