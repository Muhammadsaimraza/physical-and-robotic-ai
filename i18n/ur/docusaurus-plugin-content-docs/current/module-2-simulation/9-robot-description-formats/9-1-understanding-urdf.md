# Lesson 9.1: Understanding URDF

The **Unified Robot Description Format (URDF)** is an XML-based file format for representing a robot model. At its core, a URDF file describes the robot as a tree of **links** and **joints**.

*   **Link:** A rigid part of the robot's body (e.g., a leg, a wheel, a chassis).
*   **Joint:** A connection between two links that defines how one link can move relative to the other.

## The Link-Joint Tree

A URDF model must be a tree. This means there are no closed loops. One link must be specified as the **root link**, and all other links are connected to it through a chain of joints.

```
base_link (root)
  ├── right_leg_joint
  │   └── right_leg_link
  │       └── right_foot_joint
  │           └── right_foot_link
  ├── left_leg_joint
  │   └── left_leg_link
  ...
```

## The `<robot>` Tag

Every URDF file starts and ends with a `<robot>` tag. This tag has one attribute, `name`, which is the name of the robot.
```xml
<robot name="my_robot">
  ... all your links and joints go here ...
</robot>
```

## The `<link>` Tag

The `<link>` tag defines a rigid part of the robot.
```xml
<link name="my_link_name">
  ...
</link>
```
Inside the link tag, you will typically define three things: `<visual>`, `<collision>`, and `<inertial>`.

*   **`<visual>`:** This describes what the link *looks like*. It's used for visualization in tools like RViz and Gazebo.
    *   `<geometry>`: Can be a simple shape like `<box>`, `<cylinder>`, or `<sphere>`, or it can be a 3D mesh file like `<mesh filename="package://my_package/meshes/my_link.stl" />`.
    *   `<material>`: Defines the color of the link.
*   **`<collision>`:** This describes the physical bounding box of the link for the physics engine. It is often a simpler shape than the visual geometry to speed up collision checking. It has its own `<geometry>` tag.
*   **`<inertial>`:** This describes the physical properties of the link: its mass and its moment of inertia. This is crucial for accurate physics simulation.

## The `<joint>` Tag

The `<joint>` tag connects two links together.
```xml
<joint name="my_joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 1.0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```
*   **`name` and `type`:** Every joint has a name and a type. The most common types are:
    *   `revolute`: A hinge joint that rotates around a single axis (like an elbow).
    *   `continuous`: A revolute joint with no angle limits (like a wheel).
    *   `prismatic`: A sliding joint that moves along an axis.
    *   `fixed`: A joint that doesn't move. This is used to rigidly connect two links.
*   **`<parent>` and `<child>`:** These tags define the tree structure. The joint connects the `child` link to the `parent` link.
*   **`<origin>`:** This is a crucial tag. It defines the **transform** (position and orientation) of the child link's origin relative to the parent link's origin. `xyz` is the position offset, and `rpy` (roll, pitch, yaw) is the orientation offset.
*   **`<axis>`:** For revolute and prismatic joints, this defines the axis of motion. `xyz="0 0 1"` means the joint rotates around the Z-axis.
*   **`<limit>`:** For revolute and prismatic joints, this defines the joint's limits: `lower` and `upper` for position, `effort` for the maximum force it can apply, and `velocity` for the maximum speed.

By combining these simple building blocks, you can describe the kinematics of almost any robot, from a simple mobile cart to a complex humanoid. In the next lesson, you'll use these tags to build your first robot.
