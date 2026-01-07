AI Integration
==============

Integrate Mini-Arm with LLMs and AI systems for intelligent, natural language control.

Natural Language Commands
-------------------------

Use an LLM to interpret commands and generate robot actions:

.. code-block:: python

    from mini_arm import MiniArm
    from openai import OpenAI

    arm = MiniArm()
    arm.connect()

    client = OpenAI()

    SYSTEM_PROMPT = """
    You are a robot arm controller. Given a natural language command,
    output a JSON object with the action to perform.

    Available actions:
    - {"action": "move_to", "x": float, "y": float, "z": float}
    - {"action": "set_joints", "angles": [j0, j1, j2, j3, j4, j5]}
    - {"action": "gripper", "state": "open" | "close" | float(0-100)}
    - {"action": "home"}
    - {"action": "wave"}

    The robot workspace is approximately:
    - X: -150 to 150 mm
    - Y: -150 to 150 mm
    - Z: 0 to 200 mm

    Respond with ONLY the JSON object.
    """

    def execute_command(text: str):
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": text}
            ],
            response_format={"type": "json_object"}
        )

        action = json.loads(response.choices[0].message.content)

        if action["action"] == "move_to":
            arm.move_to(x=action["x"], y=action["y"], z=action["z"])
        elif action["action"] == "set_joints":
            arm.set_joints(action["angles"])
        elif action["action"] == "gripper":
            if action["state"] == "open":
                arm.gripper_open()
            elif action["state"] == "close":
                arm.gripper_close()
            else:
                arm.gripper_set(action["state"])
        elif action["action"] == "home":
            arm.home()
        elif action["action"] == "wave":
            wave_motion()

    # Interactive loop
    while True:
        command = input("Command: ")
        if command.lower() == "quit":
            break
        execute_command(command)

Voice Control
-------------

Add speech recognition for hands-free operation:

.. code-block:: python

    import speech_recognition as sr

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Say a command...")

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_whisper(audio)
        print(f"Heard: {text}")
        execute_command(text)
    except sr.UnknownValueError:
        print("Could not understand audio")

Vision-Language Integration
---------------------------

Combine object detection with LLM reasoning:

.. code-block:: python

    from mini_arm.vision import Camera, detect_objects

    camera = Camera()

    def find_and_pick(object_description: str):
        # Capture image
        frame = camera.capture()

        # Use vision model to find object
        detections = detect_objects(
            frame,
            prompt=object_description,
            model="grounding-dino"
        )

        if not detections:
            print(f"Could not find: {object_description}")
            return False

        # Get object position in robot frame
        obj = detections[0]
        robot_pos = camera.pixel_to_robot(obj.center, obj.depth)

        # Pick the object
        arm.move_to(x=robot_pos.x, y=robot_pos.y, z=robot_pos.z + 50)
        arm.gripper_open()
        arm.move_to(z=robot_pos.z)
        arm.gripper_close()
        arm.move_to(z=robot_pos.z + 50)

        return True

    # Example usage
    find_and_pick("the red apple on the left")

Agentic Control
---------------

Create an autonomous agent that can plan and execute complex tasks:

.. code-block:: python

    from langchain.agents import create_tool_calling_agent
    from langchain_openai import ChatOpenAI
    from langchain.tools import tool

    @tool
    def move_arm(x: float, y: float, z: float) -> str:
        """Move the robot arm to XYZ position in millimeters."""
        arm.move_to(x=x, y=y, z=z)
        return f"Moved to ({x}, {y}, {z})"

    @tool
    def open_gripper() -> str:
        """Open the robot gripper."""
        arm.gripper_open()
        return "Gripper opened"

    @tool
    def close_gripper() -> str:
        """Close the robot gripper."""
        arm.gripper_close()
        return "Gripper closed"

    @tool
    def get_position() -> str:
        """Get the current position of the robot arm."""
        pos = arm.get_pose()
        return f"Current position: ({pos.x}, {pos.y}, {pos.z})"

    @tool
    def look_at_workspace() -> str:
        """Take a photo and describe what's on the workspace."""
        frame = camera.capture()
        # Use vision model to describe scene
        description = describe_scene(frame)
        return description

    tools = [move_arm, open_gripper, close_gripper, get_position, look_at_workspace]

    llm = ChatOpenAI(model="gpt-4o")
    agent = create_tool_calling_agent(llm, tools, prompt)

    # Agent can now autonomously complete tasks
    agent.run("Pick up the blue block and place it on top of the red block")

Safety Considerations
---------------------

When using AI for robot control, always implement safety measures:

.. code-block:: python

    from mini_arm.safety import SafetyWrapper

    # Wrap arm with safety checks
    safe_arm = SafetyWrapper(
        arm,
        max_velocity=90,          # deg/s
        workspace_limits={        # mm
            "x": (-150, 150),
            "y": (-150, 150),
            "z": (0, 200),
        },
        collision_objects=[...],  # Known obstacles
        require_confirmation=True  # Ask before executing
    )

    # AI commands go through safety filter
    safe_arm.move_to(x=100, y=100, z=50)  # Validated before execution

Example: AI Teaching Assistant
------------------------------

Use Mini-Arm as an interactive teaching tool:

.. code-block:: python

    # Student asks about robot kinematics
    # LLM explains while demonstrating on the arm

    def teach(topic: str):
        if "forward kinematics" in topic.lower():
            response = """
            Forward kinematics calculates the end-effector position
            from joint angles. Watch as I demonstrate:
            """
            print(response)

            # Demonstrate by moving joints
            for i in range(6):
                print(f"Moving joint {i}...")
                arm.set_joint(i, 30)
                time.sleep(1)
                pos = arm.get_pose()
                print(f"End-effector now at: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                arm.set_joint(i, 0)

Next Steps
----------

- :doc:`/ros2/setup` - ROS2 integration for advanced AI
- `MicroLLM Project <https://github.com/Jshulgach/MicroLLM>`_ - On-device LLM for Pico
