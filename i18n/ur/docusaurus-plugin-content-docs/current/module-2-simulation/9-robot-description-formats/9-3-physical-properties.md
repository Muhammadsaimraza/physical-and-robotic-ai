# Lesson 9.3: Physical Properties

Your robot *looks* right in RViz, but as far as a physics engine is concerned, it's a ghost. It has no mass, no friction, and no substance. To simulate the robot in a tool like Gazebo, we need to add its physical properties.

This is done using two tags inside each `<link>`: `<collision>` and `<inertial>`.

## `<collision>`

The `<collision>` tag defines the shape of the link for the physics engine. It's often a good idea to make this shape simpler than the `<visual>` shape. For example, you might have a very detailed visual mesh with thousands of polygons, but use a simple box or cylinder for the collision geometry. This makes the physics calculations much faster.

For our simple robot, the visual and collision geometries can be the same.

```xml
<link name="chassis">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
    ...
  </visual>
  
  <collision>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>
  
  ...
</link>
```
You should add a `<collision>` tag to every link in your robot that you want to interact with the world.

## `<inertial>`

The `<inertial>` tag is the most critical for a realistic simulation. It defines the mass and moment of inertia for the link.

```xml
<link name="chassis">
  ...
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia 
      ixx="0.04" ixy="0.0" ixz="0.0"
      iyy="0.1" iyyy="0.0"
      izz="0.12"/>
  </inertial>
</link>
```

*   **`<origin>`:** The center of mass for the link, relative to the link's origin. For a symmetrical shape, this is usually `xyz="0 0 0"`.
*   **`<mass>`:** The mass of the link in kilograms.
*   **`<inertia>`:** The **moment of inertia tensor**. This 3x3 matrix describes how the link's mass is distributed around its center of mass. It determines how the link will react to rotational forces (torques).
    *   `ixx`, `iyy`, `izz`: These are the moments of inertia about the X, Y, and Z axes.
    *   `ixy`, `ixz`, `iyz`: These are the products of inertia. For symmetrical shapes, these are often zero.

### How to Calculate Inertia?

Calculating the inertia tensor for a complex shape is difficult. Fortunately, for common shapes, there are standard formulas:
*   **Box:** `Ixx = (1/12) * m * (y^2 + z^2)`
*   **Cylinder:** `Ixx = (1/12) * m * (3*r^2 + h^2)`
*   **Sphere:** `Ixx = (2/5) * m * r^2`

Most CAD programs can also calculate the mass and inertia tensor for a model automatically if you define the material (e.g., aluminum, steel). For our purposes, we can use approximate values.

## Adding to Our Robot

Let's add these properties to our `two_wheeled_robot.urdf`.

```xml
<!-- two_wheeled_robot.urdf -->
<robot name="two_wheeled_robot">

  <!-- ***** LINKS ***** -->

  <link name="chassis">
    <visual> ... </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.1" iyyy="0.0" izz="0.12"/>
    </inertial>
  </link>

  <link name="left_wheel">
    <visual> ... </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyyy="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ... Do the same for right_wheel and caster ... -->

  <!-- ***** JOINTS ***** -->
  ...
</robot>
```
You need to add `<collision>` and `<inertial>` tags to every link.

With these tags added, your robot is no longer a ghost. It is a physical object that a simulator like Gazebo can bring to life. In the next chapter, we will do exactly that: import this robot into a Gazebo world and make it move.
