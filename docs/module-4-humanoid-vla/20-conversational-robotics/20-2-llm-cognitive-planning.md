# Lesson 20.2: LLM Cognitive Planning

The heart of the Voice-to-Action pipeline is the **LLM Cognitive Planner**. This is where the robot truly "understands" the human's intent and translates it into a sequence of executable robot actions.

We are treating the Large Language Model (LLM) as the robot's high-level brain. It's not controlling individual joint movements, but rather orchestrating a sequence of higher-level skills.

## The Role of Prompt Engineering

The key to effective LLM cognitive planning is **prompt engineering**. You need to give the LLM clear instructions on its role, the actions it can take, and the desired output format.

A well-designed prompt will typically include:
1.  **Role Definition:** "You are the planning brain for a service robot."
2.  **Available Actions:** A list of the robot's high-level skills, along with their parameters.
    *   `GOTO(location_name)`
    *   `FIND(object_name)`
    *   `PICKUP(object_name)`
    *   `DELIVER(object_name, person_name)`
3.  **Known Context:** Information about the environment (e.g., "Known locations are: kitchen, living_room, bedroom. Known objects are: water_bottle, book, keys.")
4.  **Output Format:** "Output your plan in JSON format."
5.  **Example Conversational Turn:** A few examples of human commands and the expected robot plan.

## Example Prompt

```text
You are the planning brain for a household service robot named Rosie. Your goal is to generate a structured plan in JSON format to fulfill user requests.

Available Actions:
- GOTO(location: str): Move to a specified location. Valid locations: 'kitchen', 'living_room', 'bedroom', 'office'.
- FIND(object_name: str): Locate a specified object within the current room using visual perception.
- PICKUP(object_name: str): Pick up a specified object. Assumes the robot is at the object's location.
- DELIVER(object_name: str, recipient_name: str): Deliver an object to a person. Valid recipients: 'user', 'guest'.

Current Robot Location: 'living_room'

User Command: "Rosie, can you please bring me the water bottle from the kitchen?"

Output your plan as a JSON array of action objects.

Example:
User: "Go to the kitchen."
Rosie:
[
  {"action": "GOTO", "parameters": {"location": "kitchen"}}
]
```

## LLM Output

If prompted correctly, the LLM would then output a JSON plan like this:

```json
[
  {"action": "GOTO", "parameters": {"location": "kitchen"}},
  {"action": "FIND", "parameters": {"object_name": "water_bottle"}},
  {"action": "PICKUP", "parameters": {"object_name": "water_bottle"}},
  {"action": "GOTO", "parameters": {"location": "user"}},
  {"action": "DELIVER", "parameters": {"object_name": "water_bottle", "recipient_name": "user"}}
]
```
This structured JSON is then passed to the "Plan-to-Action" execution module, which translates these high-level actions into calls to ROS 2 action servers and service clients.

## Multi-Modal Interaction

While we've focused on speech-to-text, real-world human-robot interaction is often **multi-modal**.

*   **Speech + Gesture:** "Pick up *that* object" (with a pointing gesture).
*   **Speech + Vision:** "Find the *red* book." (using visual attributes).

LLMs are becoming increasingly capable of processing multi-modal inputs, allowing them to fuse information from text, images, and potentially other sensor streams to form a more complete understanding of the user's intent. This is the essence of **Vision-Language-Action (VLA)** models.

The future of robotic control is not just about writing code, but about training and prompting powerful AI models to reason about tasks in human terms and then seamlessly translating those abstract plans into physical actions.
