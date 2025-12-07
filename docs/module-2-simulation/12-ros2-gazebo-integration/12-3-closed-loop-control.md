# Lesson 12.3: Closed-Loop Control

So far, our control has been **open-loop**. We publish a velocity command and just hope the robot does what we want. We are not using any sensor feedback to adjust our commands.

**Closed-loop control** is the process of using sensor feedback to continuously correct a robot's behavior. This is the key to creating intelligent and robust robots.

Let's build a classic closed-loop behavior: a wall follower. The goal is to have our robot drive forward while maintaining a constant distance from a wall on its right side.

## The Logic

1.  **Sense:** Use the LiDAR scanner to measure the distance to the wall on the right.
2.  **Compare:** Compare this measured distance to our desired distance (our "setpoint").
3.  **Act:**
    *   If the robot is too far from the wall, turn slightly to the right.
    *   If the robot is too close to the wall, turn slightly to the left.
    *   If the robot is at the correct distance, drive straight.

This is a simple form of a **proportional controller**. The turning command is proportional to the error between the desired distance and the measured distance.

## The Code

We will create a new node that subscribes to the `/scan` topic and publishes to the `/cmd_vel` topic.

Create a new file, `my_first_package/wall_follower.py`.

```python
# my_first_package/my_first_package/wall_follower.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.get_logger().info('Wall Follower Node has been started.')

    def scan_callback(self, msg):
        # We want to look at the laser reading directly to our right.
        # Our scan is 360 points, from -pi to +pi.
        # The point to the right is at -pi/2 radians, which is the 90th point.
        # (The indices go from 0 to 359, mapping to the angles)
        
        distance_to_right = msg.ranges[90]
        
        # Simple proportional controller
        desired_distance = 1.0
        error = desired_distance - distance_to_right
        
        # The 'k' is our proportional gain. It's a tuning parameter.
        k_proportional = 0.5
        
        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Always move forward
        twist_msg.angular.z = k_proportional * error # Turn based on the error
        
        self.get_logger().info(f'Dist: {distance_to_right:.2f}, Err: {error:.2f}, Ang. Vel: {twist_msg.angular.z:.2f}')
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the System

1.  **Register and Build:** Add the new node to your `setup.py` and run `colcon build`.
2.  **Create a Launch File:** Create a new launch file, `launch/wall_follower.launch.py`. This should:
    *   Launch Gazebo with a world that has some walls (e.g., your `obstacle_course.sdf`).
    *   Spawn your two-wheeled robot URDF.
    *   Launch your new `wall_follower` node.
3.  **Launch and Observe:** Run your new launch file. Place your robot near a wall in the simulation. It should start to drive forward, adjusting its angle to try and stay 1.0 meter away from the wall.

You have now created a true robotic "skill." Your system is no longer just executing pre-programmed movements; it is sensing its environment and reacting to it in real-time. This perception-action loop is the essence of intelligent robotics.
