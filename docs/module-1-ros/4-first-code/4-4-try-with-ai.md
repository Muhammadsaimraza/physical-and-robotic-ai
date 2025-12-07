# Lesson 4.4: Try With AI

You've now written a publisher and a subscriber manually. You understand the components: the node class, the `create_publisher`/`create_subscription` calls, the callbacks, and the `rclpy.spin()` loop.

Now, let's explore how a generative AI assistant can act as your co-pilot, helping you to write, understand, and modify this code faster.

## The AI as a "Rubber Duck"

"Rubber duck debugging" is a classic programming technique where you explain your code, line-by-line, to a rubber duck. The act of explaining it often reveals the bug to you.

An AI assistant is like a super-powered rubber duck that can talk back.

**Your Task:**
1.  Open your `talker.py` file.
2.  Copy the entire contents of the file.
3.  Go to your favorite large language model (like ChatGPT, Gemini, Claude, etc.).
4.  Paste the code and ask the following prompt.

---
**Prompt 1: Code Explanation**

> "You are an expert ROS 2 developer. Please explain this Python code to me line by line. I am a beginner. Explain what each part does, especially the `rclpy` functions."

---

Review the AI's answer. Does it match your own understanding? Did it point out anything you missed? The AI is very good at providing detailed, line-by-line explanations of existing code.

## The AI as a Code Generator

Now let's ask the AI to modify the code. Let's say we want to change the message type from a simple `String` to a more complex message that includes a counter and a timestamp. The `sensor_msgs/msg/TimeReference` message is a good candidate.

---
**Prompt 2: Code Modification**

> "Excellent, thank you. Now, please modify the `talker.py` code. I want to publish a `sensor_msgs/msg/TimeReference` message instead of a `std_msgs/msg/String` message.
>
> In the timer callback, you should:
> 1. Set the `time_ref.sec` part of the message's timestamp to the current value of the counter `i`.
> 2. Continue to log the message content to the screen.
> 3. Do the same for the `listener.py` node, so it can subscribe to and parse this new message type."

---

The AI should provide you with two new blocks of code, one for the talker and one for the listener.

**Your Task:**
1.  Create two new files, `talker_advanced.py` and `listener_advanced.py`.
2.  Paste the AI-generated code into these files.
3.  **Crucially, you must register these new nodes in your `setup.py` file:**
    ```python
    # setup.py
    'console_scripts': [
        'talker = my_first_package.talker:main',
        'listener = my_first_package.listener:main',
        'talker_advanced = my_first_package.talker_advanced:main',
        'listener_advanced = my_first_package.listener_advanced:main',
    ],
    ```
4.  You also need to add the new dependency to your `package.xml`:
    ```xml
    <!-- package.xml -->
    <depend>sensor_msgs</depend>
    ```
5.  Rebuild your workspace with `colcon build`.
6.  Run your new nodes:
    *   Terminal 1: `ros2 run my_first_package talker_advanced`
    *   Terminal 2: `ros2 run my_first_package listener_advanced`

Did it work? If not, can you debug the issue? Often, the AI will get the code 95% right, but you, the human developer, need to provide the final 5% of correction and integration. This is the essence of AI-assisted development.

## The Power of Collaboration

This workflow—write a little, get it working, then ask an AI to explain and extend it—is the core of the L2 AI Collaboration layer. The AI is a powerful tool, but it is not a replacement for understanding. By building the simple version yourself first, you gain the context needed to effectively prompt the AI and to understand and debug its output.

This concludes Chapter 4. You are now not only a ROS 2 developer, but you are an AI-augmented ROS 2 developer. In the next chapter, we will use this workflow to explore ROS 2's other primary communication pattern: services.
