# Chapter 7: Capstone - Robot Controller

Welcome to the capstone project for Module 1. In this chapter, you will put everything you have learned—nodes, topics, services, parameters, and launch files—together to build a complete, multi-node robot controller system for Turtlesim.

This chapter introduces the final layer of our teaching method: **Spec-Driven Integration**. You will not just be writing code; you will be acting as a systems integrator. You will start by writing a clear technical specification for the desired behavior, and then you will implement and test your system against that specification. This is how professional robotics projects are managed.

*   **Duration:** 3 lessons, 4.5 hours total
*   **Layer Breakdown:** L4: Spec-Driven Integration
*   **Hardware Tier:** Tier 1 (Turtlesim simulation)
*   **Prerequisites:** Completion of Chapter 6

## Learning Objectives
By the end of this chapter, you will be able to:

*   Write a technical specification for a multi-node ROS 2 system.
*   Implement a system from a specification, combining multiple nodes, topics, and services.
*   Create a launch file to run and configure the entire system.
*   Systematically test your implementation against your specification to validate its correctness.

## Project Goal

You will build a system that makes the turtle draw a shape of a configurable size and speed. The system will be controlled by a service call, not by running a node directly.

## Lessons
*   **Lesson 7.1: The Specification** (60 minutes)
    *   First, we will write a detailed plan. What nodes do we need? What topics and services will they use? What parameters will they have?

*   **Lesson 7.2: Building the Controller** (120 minutes)
    *   With a clear specification in hand, you will implement the nodes and launch file for the system.

*   **Lesson 7.3: Testing & Validation** (90 minutes)
    *   You will run your system and systematically test each requirement from your specification to ensure it works as designed.
