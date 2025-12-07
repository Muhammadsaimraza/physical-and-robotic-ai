# Lesson 5.4: Design Patterns (Topics vs. Services)

You are now armed with two powerful communication tools: topics (publish/subscribe) and services (request/response). The mark of an experienced ROS developer is knowing which tool to use for which job. This is a question of system design.

There is no single right answer, but there are well-established patterns and trade-offs.

## Topics: For Continuous Data Streams

Use a **topic** when the data is being produced continuously, and you don't care if anyone is listening. Think of it as a broadcast.

**Key Characteristics:**
*   **One-to-Many:** One publisher, many subscribers.
*   **Asynchronous:** The publisher sends the message and moves on. It doesn't wait for anyone.
*   **Stateless:** The publisher doesn't know who is subscribed, and subscribers don't know who published.

**Good For:**
*   **Sensor Data:** Camera images, LiDAR scans, IMU readings. These are continuous streams of data about the state of the world.
*   **Robot State:** Publishing the robot's current position (`/tf`), joint angles (`/joint_states`), or battery level.
*   **Commands:** Sending continuous velocity commands (`/cmd_vel`) to a robot base.

**Anti-Patterns (When NOT to use a topic):**
*   Don't use a topic if you need a direct confirmation that the message was received and processed.
*   Don't use a topic for a single, discrete action where you need a result (e.g., "compute the inverse kinematics for this position").

## Services: For Discrete Actions & Queries

Use a **service** when you need to trigger a specific action and get a direct response. Think of it as a function call.

**Key Characteristics:**
*   **One-to-One:** One client, one server.
*   **Synchronous:** The client sends a request and blocks (waits) until it receives a response.
*   **Stateful Transaction:** The client and server are in a direct, two-way conversation.

**Good For:**
*   **Triggering an Action:** `/reset_simulation`, `/capture_image`, `/move_arm_to_home_position`.
*   **Querying for Data:** `/get_map`, `/lookup_transform`, `/what_is_your_name`.
*   **Performing a Calculation:** `/add_two_ints`, `/compute_ik_solution`.

**Anti-Patterns (When NOT to use a service):**
*   Don't use a service for a long-running task. If the action will take more than a few seconds, the client will be blocked for too long. For this, we use **Actions** (covered in a later chapter).
*   Don't use a service for streaming data. Publishing camera images over a service would be incredibly inefficient.

## A Hybrid Example: Taking a Picture

Let's consider a camera node. How should it provide its data? It should use both!

1.  **Topic for Streaming:** The node should continuously **publish** `sensor_msgs/msg/Image` messages to a topic like `/camera/image_raw`. This allows any number of other nodes to use the live video feed for object detection, tracking, etc.
2.  **Service for Triggering:** The node could also provide a **service** called `/camera/save_image` of type `my_interfaces/srv/SaveImage`. The request could contain a `filepath`, and the response could contain a `bool success`. This allows a client to request that a single, specific frame be saved to disk and to get a confirmation that the action was completed.

By providing both interfaces, the camera node becomes much more flexible and reusable.

## Conclusion

Choosing between topics and services is a fundamental design decision in ROS 2. By thinking about the nature of the data and the interaction required, you can build clean, robust, and maintainable robotic systems.

*   **Continuous stream? Unidirectional?** Use a topic.
*   **Discrete action? Bidirectional? Need a response?** Use a service.

This concludes Chapter 5. You are now a master of ROS 2's core communication patterns. In the next chapter, we will learn how to assemble the nodes you've built into a complete system using launch files.
