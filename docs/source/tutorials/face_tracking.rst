Face Tracking
=============

Use computer vision to track faces and have Mini-Arm follow them.

Requirements
------------

.. code-block:: bash

    pip install opencv-python mediapipe

Basic Face Tracking
-------------------

.. code-block:: python

    import cv2
    import mediapipe as mp
    from mini_arm import MiniArm

    # Initialize
    arm = MiniArm()
    arm.connect()

    mp_face = mp.solutions.face_detection
    face_detection = mp_face.FaceDetection(min_detection_confidence=0.7)

    cap = cv2.VideoCapture(0)

    # Camera center
    frame_center_x = 320
    frame_center_y = 240

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Detect faces
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_detection.process(rgb)

            if results.detections:
                # Get first face
                detection = results.detections[0]
                bbox = detection.location_data.relative_bounding_box

                # Calculate face center
                h, w, _ = frame.shape
                face_x = int((bbox.xmin + bbox.width / 2) * w)
                face_y = int((bbox.ymin + bbox.height / 2) * h)

                # Calculate error from center
                error_x = (face_x - frame_center_x) / frame_center_x
                error_y = (face_y - frame_center_y) / frame_center_y

                # Move arm to track face
                arm.jog_joint(0, -error_x * 5)  # Pan (base)
                arm.jog_joint(1, error_y * 3)   # Tilt (shoulder)

                # Draw tracking info
                cv2.circle(frame, (face_x, face_y), 5, (0, 255, 0), -1)
                cv2.rectangle(frame,
                    (int(bbox.xmin * w), int(bbox.ymin * h)),
                    (int((bbox.xmin + bbox.width) * w), int((bbox.ymin + bbox.height) * h)),
                    (0, 255, 0), 2)

            cv2.imshow("Face Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.disconnect()

PID Control for Smooth Tracking
-------------------------------

.. code-block:: python

    class PIDController:
        def __init__(self, kp=1.0, ki=0.0, kd=0.1):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.integral = 0
            self.prev_error = 0

        def update(self, error, dt=0.033):
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            return output

    # Use PID for smoother tracking
    pan_pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
    tilt_pid = PIDController(kp=1.5, ki=0.1, kd=0.3)

    # In tracking loop:
    pan_correction = pan_pid.update(error_x)
    tilt_correction = tilt_pid.update(error_y)

    arm.jog_joint(0, pan_correction)
    arm.jog_joint(1, tilt_correction)

Multiple Face Handling
----------------------

.. code-block:: python

    if results.detections:
        # Track the largest (closest) face
        largest_face = max(results.detections,
            key=lambda d: d.location_data.relative_bounding_box.width *
                         d.location_data.relative_bounding_box.height)

        # Or track a specific face (face re-identification)
        # ... advanced implementation

Face Distance Estimation
------------------------

Use face size to estimate distance and adjust arm position:

.. code-block:: python

    # Approximate distance from face size
    face_width_pixels = bbox.width * w
    KNOWN_FACE_WIDTH_CM = 15  # Average face width
    FOCAL_LENGTH = 600        # Calibrated for your camera

    distance_cm = (KNOWN_FACE_WIDTH_CM * FOCAL_LENGTH) / face_width_pixels

    # Adjust arm reach based on distance
    target_z = 100 + (distance_cm - 50) * 0.5  # Example mapping
    arm.move_to(z=target_z)

Hand Gesture Integration
------------------------

Combine with hand tracking for gesture control:

.. code-block:: python

    import mediapipe as mp

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1)

    # In loop:
    hand_results = hands.process(rgb)

    if hand_results.multi_hand_landmarks:
        hand = hand_results.multi_hand_landmarks[0]

        # Check for closed fist (gripper close)
        if is_fist(hand):
            arm.gripper_close()
        else:
            arm.gripper_open()

Complete Demo
-------------

Run the full face tracking demo:

.. code-block:: bash

    python examples/03_face_detection/face_track.py

Next Steps
----------

- :doc:`ai_integration` - Add LLM-based interaction
- :doc:`/ros2/setup` - ROS2 integration for advanced vision
