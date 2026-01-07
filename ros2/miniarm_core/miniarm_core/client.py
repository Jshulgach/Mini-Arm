"""
Mini-Arm Client

Serial communication interface for the Mini-Arm robot.
"""

import time
from typing import List, Optional, Tuple, Union

import numpy as np
import serial

__all__ = ["MiniArmClient"]


class MiniArmClient:
    """A client for the Mini-Arm robot arm that handles serial communication.

    This class provides methods for connecting to and controlling the Mini-Arm
    robot via serial communication. It can send commands and receive responses
    from the Raspberry Pi Pico running the Mini-Arm firmware.

    Parameters
    ----------
    name : str, optional
        Name identifier for the client instance. Default is 'MiniArmClient'.
    port : str, optional
        Serial port to connect to. Default is 'COM3' (Windows).
        Use '/dev/ttyACM0' for Linux.
    baudrate : int, optional
        Baudrate for the serial connection. Default is 115200.
    command_delimiter : str, optional
        Character(s) used to terminate commands. Default is ';'.
    timeout : float, optional
        Serial read timeout in seconds. Default is 1.0.
    verbose : bool, optional
        Enable/disable verbose output logging. Default is False.

    Attributes
    ----------
    connected : bool
        True if serial connection is established and open.
    s : serial.Serial
        The underlying serial connection object.

    Examples
    --------
    Basic usage:

    >>> from mini_arm import MiniArmClient
    >>> client = MiniArmClient(port='COM3')
    >>> client.home()
    >>> pose = client.get_pose()
    >>> print(f"Current position: {pose}")
    >>> client.disconnect()

    Using context manager:

    >>> with MiniArmClient(port='COM3') as client:
    ...     client.home()
    ...     client.set_pose(0.135, 0.0, 0.22)
    """

    # Default home position
    HOME_POSITION = [0.135, 0.0, 0.215]

    # Workspace limits (meters)
    WORKSPACE_LIMITS = {
        "x": (0.05, 0.25),
        "y": (-0.15, 0.15),
        "z": (0.10, 0.30),
    }

    def __init__(
        self,
        name: str = "MiniArmClient",
        port: str = "COM3",
        baudrate: int = 115200,
        command_delimiter: str = ";",
        timeout: float = 1.0,
        verbose: bool = False,
    ):
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.command_delimiter = command_delimiter
        self.timeout = timeout
        self.verbose = verbose

        # Internal state
        self.s: Optional[serial.Serial] = None
        self.connected: bool = False
        self._last_pose: Optional[np.ndarray] = None
        self._last_joints: Optional[List[float]] = None

        # Attempt connection
        self._connect()

    def _connect(self):
        """Establish serial connection to Mini-Arm."""
        try:
            self.s = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
            self.connected = self.s.is_open

            if self.connected:
                # Clear any startup messages
                time.sleep(0.5)
                self.get_buffer()
                self.logger(f"Connected to {self.port} at {self.baudrate} baud")

        except serial.SerialException as e:
            self.s = None
            self.connected = False
            self.logger(f"Failed to connect: {e}", warning=True)

    def logger(self, *argv, warning: bool = False):
        """Log messages with timestamp.

        Parameters
        ----------
        *argv : str
            Message components to join and log.
        warning : bool, optional
            If True, prefix message with '(Warning)'. Default is False.
        """
        msg = "".join(str(a) for a in argv)
        if warning:
            msg = "⚠️  " + msg
        print(f"[{time.monotonic():.3f}][{self.name}] {msg}")

    # =========================================================================
    # Core Communication Methods
    # =========================================================================

    def send_message(self, message: str) -> None:
        """Send a message to the Pico over serial.

        Automatically adds the command terminator if not present.

        Parameters
        ----------
        message : str
            The command message to send.
        """
        if not self.s or not self.s.is_open:
            self.logger("Serial connection not available", warning=True)
            return

        if not message.endswith(self.command_delimiter):
            message += self.command_delimiter

        try:
            self.s.write(message.encode())
            if self.verbose:
                self.logger(f"TX: {message.strip()}")
            time.sleep(0.01)  # 10ms delay to prevent buffer overflow
        except serial.SerialException as e:
            self.logger(f"Error sending: {e}", warning=True)

    def send(self, command: str) -> Optional[str]:
        """Send a command and optionally read the response.

        Parameters
        ----------
        command : str
            Command string to send to Mini-Arm.

        Returns
        -------
        str or None
            Response from the device, if any.
        """
        self.send_message(command)
        time.sleep(0.05)
        return self.get_buffer()

    def get_buffer(self) -> Optional[str]:
        """Read all available bytes from the serial connection.

        Returns
        -------
        str or None
            The buffer contents as a string, or None if no data available.
        """
        if not self.s or not self.s.is_open:
            return None

        try:
            if self.s.in_waiting > 0:
                msg = ""
                while self.s.in_waiting > 0:
                    msg += self.s.readline().decode(errors="ignore")
                    time.sleep(0.01)
                if self.verbose and msg:
                    self.logger(f"RX: {msg.strip()}")
                return msg.strip() if msg else None
        except serial.SerialException as e:
            self.logger(f"Error reading: {e}", warning=True)
        return None

    def wait_for_response(self, timeout: float = 2.0) -> Optional[str]:
        """Wait for a response from the device.

        Parameters
        ----------
        timeout : float
            Maximum time to wait in seconds.

        Returns
        -------
        str or None
            Response string or None if timeout.
        """
        start = time.time()
        while time.time() - start < timeout:
            response = self.get_buffer()
            if response:
                return response
            time.sleep(0.05)
        return None

    # =========================================================================
    # Motion Commands
    # =========================================================================

    def home(self) -> None:
        """Move the robot arm to its home position."""
        self.send_message("home")
        if self.verbose:
            self.logger("Moving to home position")

    def ik_home(self) -> None:
        """Move to IK home position (all joints at 0)."""
        self.send_message("ikhome")

    def set_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        """Move the end-effector to a Cartesian position.

        Parameters
        ----------
        x : float
            X coordinate in meters.
        y : float
            Y coordinate in meters.
        z : float
            Z coordinate in meters.
        roll : float, optional
            Roll angle in radians. Default is 0.0.
        pitch : float, optional
            Pitch angle in radians. Default is 0.0.
        yaw : float, optional
            Yaw angle in radians. Default is 0.0.
        """
        pose_str = f"[{x},{y},{z},{roll},{pitch},{yaw}]"
        self.send_message(f"set_pose:{pose_str}")
        if self.verbose:
            self.logger(f"Moving to pose: ({x:.3f}, {y:.3f}, {z:.3f})")

    def set_pose_xyz(
        self, position: Union[List[float], Tuple[float, float, float]]
    ) -> None:
        """Move to XYZ position (convenience method).

        Parameters
        ----------
        position : list or tuple
            [x, y, z] coordinates in meters.
        """
        if len(position) >= 3:
            self.set_pose(position[0], position[1], position[2])
        else:
            self.logger(
                "Position must have at least 3 elements (x, y, z)", warning=True
            )

    def set_delta_pose(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        droll: float = 0.0,
        dpitch: float = 0.0,
        dyaw: float = 0.0,
    ) -> None:
        """Move the end-effector by a relative delta.

        Parameters
        ----------
        dx : float
            Delta X in meters.
        dy : float
            Delta Y in meters.
        dz : float
            Delta Z in meters.
        droll : float, optional
            Delta roll in radians.
        dpitch : float, optional
            Delta pitch in radians.
        dyaw : float, optional
            Delta yaw in radians.
        """
        delta_str = f"[{dx},{dy},{dz},{droll},{dpitch},{dyaw}]"
        self.send_message(f"set_delta_pose:{delta_str}")
        if self.verbose:
            self.logger(f"Delta move: ({dx:.4f}, {dy:.4f}, {dz:.4f})")

    def get_pose(self) -> Optional[np.ndarray]:
        """Get the current end-effector pose.

        Returns
        -------
        np.ndarray or None
            Array [x, y, z, roll, pitch, yaw] or None if reading fails.
        """
        self.send_message("get_pose")
        time.sleep(0.1)
        data = self.get_buffer()

        if data:
            try:
                # Parse response: "cords: [x: 0.135, y: 0.000, z: 0.215]angles: [...]"
                if "cords:" in data:
                    coords_part = data.split("cords:")[1].split("angles:")[0]
                    coords_part = coords_part.replace("[", "").replace("]", "").strip()
                    pose = []
                    for p in coords_part.split(","):
                        if ":" in p:
                            pose.append(float(p.split(":")[1].strip()))
                    if len(pose) >= 3:
                        self._last_pose = np.array(pose[:3])
                        return self._last_pose
            except Exception as e:
                if self.verbose:
                    self.logger(f"Error parsing pose: {e}", warning=True)

        return self._last_pose

    def get_current_pose(self) -> Optional[np.ndarray]:
        """Alias for get_pose() for backward compatibility."""
        return self.get_pose()

    # =========================================================================
    # Joint Commands
    # =========================================================================

    def set_joint(self, joint_index: int, angle: float) -> None:
        """Move a single joint to a specified angle.

        Parameters
        ----------
        joint_index : int
            Joint index (0-5 for arm joints, 6 for gripper).
        angle : float
            Target angle in degrees.
        """
        self.send_message(f"movemotor:{joint_index}:{angle}")
        if self.verbose:
            self.logger(f"Joint {joint_index} -> {angle}°")

    def set_joints(self, angles: List[float]) -> None:
        """Set all joint angles.

        Parameters
        ----------
        angles : list of float
            Joint angles in degrees [j0, j1, j2, j3, j4, j5] or
            [j0, j1, j2, j3, j4, j5, gripper].
        """
        angles_str = ",".join(str(a) for a in angles)
        self.send_message(f"movemotors:[{angles_str}]")
        if self.verbose:
            self.logger(f"Setting joints: {angles}")

    def get_joints(self) -> Optional[List[float]]:
        """Get current joint angles.

        Returns
        -------
        list of float or None
            Joint angles in degrees, or None if reading fails.
        """
        self.send_message("get_joints")
        time.sleep(0.1)
        data = self.get_buffer()

        if data:
            try:
                # Parse joint values from response
                data = data.replace("[", "").replace("]", "")
                joints = [float(j.strip()) for j in data.split(",") if j.strip()]
                if joints:
                    self._last_joints = joints
                    return joints
            except Exception as e:
                if self.verbose:
                    self.logger(f"Error parsing joints: {e}", warning=True)

        return self._last_joints

    # =========================================================================
    # Gripper Commands
    # =========================================================================

    def set_gripper(self, value: Union[float, str]) -> None:
        """Control the gripper.

        Parameters
        ----------
        value : float or str
            Gripper position (0-180 degrees) or state ('open', 'close').
        """
        self.send_message(f"set_gripper:{value}")
        if self.verbose:
            self.logger(f"Gripper -> {value}")

    def gripper_open(self) -> None:
        """Open the gripper fully."""
        self.set_gripper("open")

    def gripper_close(self) -> None:
        """Close the gripper fully."""
        self.set_gripper("close")

    # =========================================================================
    # LED Commands
    # =========================================================================

    def set_led(self, r: int, g: int, b: int) -> None:
        """Set the RGB LED color.

        Parameters
        ----------
        r : int
            Red value (0-255).
        g : int
            Green value (0-255).
        b : int
            Blue value (0-255).
        """
        self.send_message(f"set_led:[{r},{g},{b}]")
        if self.verbose:
            self.logger(f"LED -> RGB({r}, {g}, {b})")

    def led_off(self) -> None:
        """Turn off the LED."""
        self.set_led(0, 0, 0)

    def led_red(self) -> None:
        """Set LED to red."""
        self.set_led(255, 0, 0)

    def led_green(self) -> None:
        """Set LED to green."""
        self.set_led(0, 255, 0)

    def led_blue(self) -> None:
        """Set LED to blue."""
        self.set_led(0, 0, 255)

    # =========================================================================
    # Trajectory Commands
    # =========================================================================

    def start_trajectory(
        self, trajectory: str = "circle", repeat: bool = False
    ) -> None:
        """Start a predefined trajectory.

        Parameters
        ----------
        trajectory : str
            Trajectory name (e.g., 'circle') or list of points.
        repeat : bool
            Whether to repeat the trajectory continuously.
        """
        repeat_str = "true" if repeat else "false"
        self.send_message(f"trajectory:{trajectory}:{repeat_str}")
        if self.verbose:
            self.logger(f"Starting trajectory: {trajectory} (repeat={repeat})")

    def stop_trajectory(self) -> None:
        """Stop any running trajectory."""
        self.send_message("stop")
        if self.verbose:
            self.logger("Stopping trajectory")

    # =========================================================================
    # System Commands
    # =========================================================================

    def get_info(self) -> Optional[str]:
        """Get robot system information.

        Returns
        -------
        str or None
            Robot info string.
        """
        self.send_message("info")
        time.sleep(0.2)
        info = self.get_buffer()
        if info:
            print(info)
        return info

    def get_help(self) -> Optional[str]:
        """Get list of available commands.

        Returns
        -------
        str or None
            Help text with available commands.
        """
        self.send_message("help")
        time.sleep(0.3)
        help_text = self.get_buffer()
        if help_text:
            print(help_text)
        return help_text

    def set_debug(self, enabled: bool) -> None:
        """Enable or disable debug mode on the robot.

        Parameters
        ----------
        enabled : bool
            True to enable debug output, False to disable.
        """
        mode = "on" if enabled else "off"
        self.send_message(f"debug:{mode}")
        if self.verbose:
            self.logger(f"Debug mode: {mode}")

    def set_debug_mode(self, mode: Union[bool, str]) -> None:
        """Alias for set_debug() for backward compatibility."""
        if isinstance(mode, bool):
            self.set_debug(mode)
        else:
            enabled = mode.lower() in ("on", "true", "1")
            self.set_debug(enabled)

    def set_rate(self, hz: int) -> None:
        """Set the main loop rate on the robot.

        Parameters
        ----------
        hz : int
            Loop rate in Hz.
        """
        self.send_message(f"set_rate:{hz}")
        if self.verbose:
            self.logger(f"Loop rate set to {hz} Hz")

    def test(self) -> None:
        """Send a test command to verify communication."""
        self.send_message("test")

    def play_music(self, song: str) -> None:
        """Play a music file on the robot.

        Parameters
        ----------
        song : str
            Name of the song to play.
        """
        self.send_message(f"play_music:{song}")

    def read_fsr(self) -> Optional[str]:
        """Read force-sensitive resistor values.

        Returns
        -------
        str or None
            FSR sensor values.
        """
        self.send_message("fsr")
        time.sleep(0.1)
        return self.get_buffer()

    # =========================================================================
    # Xbox Controller Support
    # =========================================================================

    def send_controller_data(self, controller_data: str) -> None:
        """Send Xbox controller data for teleoperation.

        Parameters
        ----------
        controller_data : str
            Formatted controller state string.
        """
        self.send_message(f"controller:{controller_data}")

    # =========================================================================
    # Connection Management
    # =========================================================================

    def send_ctrl_c(self) -> None:
        """Send Ctrl+C to interrupt current operation."""
        if self.s:
            self.s.write(b"\x03")
            time.sleep(0.01)

    def send_ctrl_d(self) -> None:
        """Send Ctrl+D (soft reset)."""
        if self.s:
            self.s.write(b"\x04")
            time.sleep(0.01)

    def reconnect(self) -> bool:
        """Attempt to reconnect to the robot.

        Returns
        -------
        bool
            True if reconnection successful.
        """
        self.disconnect()
        time.sleep(0.5)
        self._connect()
        return self.connected

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.s:
            try:
                self.s.close()
            except Exception:
                pass
            self.connected = False
            self.logger("Disconnected")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False

    def __repr__(self) -> str:
        status = "connected" if self.connected else "disconnected"
        return f"MiniArmClient(port='{self.port}', {status})"
