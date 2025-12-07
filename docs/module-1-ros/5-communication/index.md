# Chapter 5: Communication Mastery

You've mastered the publish/subscribe pattern, the one-to-many broadcast mechanism that forms the backbone of ROS 2. But what happens when you need a two-way conversation? What if you need to guarantee that a request was received and get a direct response?

This is where ROS 2 **Services** come in. In this chapter, you'll master the request/response pattern, learn how to create your own custom message and service types, and explore the design patterns that help you choose the right communication method for the job.

*   **Duration:** 4 lessons, 4 hours total
*   **Layer Breakdown:** L2-L3 (AI Collaboration and an introduction to reusable patterns)
*   **Hardware Tier:** Tier 1 (Cloud ROS 2 environment or local install)
*   **Prerequisites:** Completion of Chapter 4

## Learning Objectives
By the end of this chapter, you will be able to:

*   Write a ROS 2 service server node that provides a request/response capability.
*   Write a ROS 2 service client node that calls a service and processes the response.
*   Define your own custom `.msg` and `.srv` interface files.
*   Build and use packages containing custom interfaces.
*   Analyze a communication problem and choose the appropriate tool (topic, service, or action).

## Lessons
*   **Lesson 5.1: Service Server** (60 minutes)
    *   Write a Python node that provides a service to add two integers.

*   **Lesson 5.2: Service Client** (60 minutes)
    *   Write a client node that calls the addition service and prints the result.

*   **Lesson 5.3: Custom Interfaces** (60 minutes)
    *   Define your own message and service types, moving beyond the built-in `String` and `Int32` types.

*   **Lesson 5.4: Design Patterns** (60 minutes)
    *   Learn the trade-offs between topics and services and develop the intuition for when to use each.
