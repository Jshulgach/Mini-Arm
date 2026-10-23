# Mini-Arm software repair — September 6, 2026

This repair prepares a coherent model and corrects command handling. It does not establish a calibrated robot or enable Homebase controls. The first physical bring-up is now complete: status communication works on COM3 and repaired Python firmware is installed on the Pico 2 W, with servo power disconnected. No motion commands were sent.

## Model corrections

- Applied millimeter-to-meter scaling to every visual and collision STL. Actual binary STL bounds were checked; the original base spanned about 121 units while URDF link origins were in meters.
- Replaced missing mesh references with repository CAD exports: base → `Base.STL`, electronics → `electronics_deck.STL`, fingers → `gripper_finger.STL`, knuckles → `gripper_link.STL`. The gripper body now references `link_6.STL`; the previous `gripper_link.STL` is the small linkage, not the body.
- These mesh assignments are inferred from the repository part names/dimensions. Mesh origins, axis orientation, articulation and assembled collision geometry still require comparison against the actual arm/CAD assembly. Existing joint transforms and bounds were preserved, not represented as newly measured.
- Aligned the six-axis controller with the existing URDF wrist name `miniarm_gripper_link_joint`. Corrected the SRDF chain, virtual root, hand group, end-effector reference and kinematics group name.
- Removed invalid ready/transport poses instead of widening limits or guessing replacement physical poses. A mathematical `zero` pose is provided for the mock model only.
- Replaced unverified collision exemptions with structural adjacency pairs. This is not a sampled, validated collision matrix.
- Added the missing gripper command/state interfaces and corrected the initial-state syntax. The launch uses only [ROS Jazzy mock hardware](https://docs.ros.org/en/jazzy/p/hardware_interface/doc/mock_components_userdoc.html), with no physical plugin selector.
- Planning speed/acceleration caps are explicitly simulation values. Mass/inertia, camera extrinsics and the L70 mounting transform remain unmeasured and are not fabricated.

## Firmware and client corrections

- Boot homing is disabled by default (`home_on_start=False`), including the supplied `code.py`. The explicit `home` command still requests motion.
- The supplied firmware entry point uses the client's semicolon terminator. `set_joints` now aliases `movemotors`; documented gripper and pose aliases are accepted.
- Every six-axis batch is validated after servo offsets/flips before any write. An invalid optional gripper angle also rejects the whole batch. NaN, infinity, malformed lengths and invalid arm indices are rejected.
- Single-joint commands preserve fractional degrees. Stored command angles advance only after a successful write. Driver I/O failures are propagated; a failure partway through a valid batch can still leave partial physical motion.
- Fixed signed interpolation and double-applied direction flips. The speed helper now uses its RPM argument, but remains synchronous and cannot be treated as an interruptible trajectory controller or emergency stop.
- Gripper open/close and numeric inputs share validation and work in simulation. Failed IK no longer reports a requested pose as achieved or applies the gripper first.
- Public client pose angles remain radians and are converted to the firmware's degree-based wire format. Absolute XYZ requests enforce the client's existing configurable workspace bounds. These bounds do not prove reachability or collision clearance; raw commands and relative moves are not a substitute for a hardware safety layer.
- Firmware math imports prefer CircuitPython `ulab`, falling back to desktop numpy for tests. Startup and non-motion status queries are now verified on the actual Pico 2 W; motion and Cartesian execution remain unverified.

Servo offsets/signs in firmware and URDF joint coordinates are not yet calibrated to each other. The nominal 0–180 servo-command check is an electrical command bound, not a measured mechanical limit. Reported servo positions are commanded values, not independent encoder feedback.

## Reproduce model checks

```powershell
$env:PYTHONPATH='ros2/miniarm_core'
python -m pytest tests/test_robot_model.py tests/test_firmware_commands.py tests/test_client.py
python docs/smart-home/audit_model.py
```

For a ROS workspace with consistent Jazzy dependencies:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select miniarm_description miniarm_moveit_config
source install/setup.bash
ros2 launch miniarm_moveit_config mock.launch.py rviz:=true
```

`mock.launch.py` starts the robot-state publisher, mock ros2_control system, arm trajectory controller, gripper controller and joint-state broadcaster. RViz is optional and defaults off. It does not launch MoveIt planning or claim collision validation.

## Verified results and local ROS environment

- 40 pytest cases pass, including model contracts, actual firmware methods with mocked servo writes, boot behavior and existing client tests.
- ROS Jazzy `check_urdf` accepts both the source model and expanded installed Xacro. Both model packages build/install successfully.
- The guarded [runtime smoke test](../../tests/ros_mock_smoke.py) verified exactly one `mock_components/GenericSystem`, three active controllers, successful six-joint trajectory and gripper actions, fresh joint-state values and return to mathematical zero. Simulation processes were stopped afterward.
- The original installed `joint_trajectory_controller` 4.34.0 binary crashed in `on_activate` against the installed controller-manager/hardware-interface 4.45.2 environment. Rebuilding upstream 4.34.0 controller sources against the current headers resolved activation with the same model/configuration. This points to an installed binary compatibility problem; no system packages were upgraded.
- The verification overlay is `/tmp/miniarm-verify.6SLNOb` in WSL. It includes rebuilt `joint_trajectory_controller` and `gripper_controllers` from upstream commit `03e4ec7fc458174785c1c44951eea64b7c309492`. Missing build helpers (`ros2_control_cmake`, `rsl`, `libexpected-dev`) were downloaded and extracted locally, not installed globally. The overlay is temporary and should not be treated as a persistent deployment.

To repeat the runtime test while that overlay exists, launch in one WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/miniarm-verify.6SLNOb/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch miniarm_moveit_config mock.launch.py
```

In another terminal with the same environment, run `python3 /mnt/c/Users/jonat/Documents/GitHub/Mini-Arm/tests/ros_mock_smoke.py`. The script checks actual hardware-plugin identity before sending simulated actions and refuses to run outside the specified localhost-only domain. Stop the launch after testing. Building only the Mini-Arm packages against this PC's original controller binaries will not incorporate the isolated compatibility repair.

To recreate the overlay, create a fresh temporary workspace, clone `https://github.com/ros-controls/ros2_controllers.git` at tag `4.34.0` and verify the commit above. In a `dependencies` subdirectory use `apt download ros-jazzy-ros2-control-cmake ros-jazzy-rsl libexpected-dev`, then extract the archives with `dpkg-deb -x` into `dependencies/extracted`. Prepend its `usr` and `opt/ros/jazzy` directories to `CMAKE_PREFIX_PATH`. Build the two model packages and the `joint_trajectory_controller`/`gripper_controllers` source subdirectories with `colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF`, then source its install setup. This is a local compatibility workaround, not a general requirement for a consistent ROS installation.

## Before physical bring-up

The board and installed firmware have been identified and servo-power isolation confirmed for the software update. Next, record and physically verify each servo's channel, neutral angle, direction and actual travel. Then compare the model to the real assembly, verify a fixed arm-to-L70 mount and measure the gripper camera transform. A working stop/power-isolation procedure is required before testing motion. Do not attach a fabricated camera/L70 transform just to make the frame tree complete.

The non-motion identification/status exchange passed. The updated board clears configured PWM outputs during startup and does not automatically home. Restoring the original backup would restore automatic homing; keep servo power disconnected during firmware maintenance.


## Pico 2 W bring-up — September 6, 2026

- Identified COM3 and CIRCUITPY on D: as the same Raspberry Pi Pico 2 W, running CircuitPython 9.2.4. The user confirmed the external servo supply was disconnected before serial testing and changes.
- `test;`, `info;` and `get_joints;` responded before and after the update. Returned zeros and poses are internal command/model state, not measured physical joint positions.
- Backed up the original `code.py`, `boot_out.txt` and complete `lib` tree (60 files with SHA-256 manifest) to local ignored `build/pico2w-backups/20260906-211847/`. Account settings were neither read nor changed.
- Preserved the board's installed PCA9685 channel order: base 15, shoulder 14, elbow 13, wrist rotation 12, wrist bend 2, gripper link 1, gripper 0. Preserved its elbow orientation setting, offsets, direction flags, and default CPU clock. These are recovered settings, not newly calibrated measurements.
- Installed five repaired Python files: `code.py`, `lib/arm_control/async_controller.py`, `lib/arm_control/robotarm.py`, `lib/arm_utils/armTransforms.py`, and `lib/arm_utils/robot_models.py`. All matched repository bytes after copying. Controller source version is 1.4.9; CircuitPython and third-party libraries were not upgraded.
- The PCA9685 retained old output values across the initial soft restart. Added explicit clearing of all seven configured outputs in `RobotArm` construction. A regression test covers all seven wired channels and preserves unrelated outputs.
- After restarting, status reported six disabled arm outputs (`None`); direct read-only I2C register inspection confirmed all seven configured channels held `[0, 0, 0, 0]`. The program was restarted again and the final status queries passed. This verifies register state, not an oscilloscope measurement or a physical motion test.
- Final serial log: local ignored `build/pico2w-status-after-update.log`. COM3 was closed after verification. The controller program is running; servo power remains disconnected by the user's last confirmation.
- Auto-reload was disabled for file replacement with `supervisor.runtime.autoreload = False`, following the [CircuitPython supervisor API](https://docs.circuitpython.org/en/stable/shared-bindings/supervisor/index.html). It remained off during this session. A power reset may restore normal auto-reload behavior.
- Homebase has updated setup notes but no continuous serial connection or physical controls. Calibration, rigid mounting, camera alignment, and an interruptible motion/stop workflow remain outstanding.


Subsequent USB transport update: see [protocol, compatibility and measured latency](usb-protocol.md). Controller 1.4.10 and the buffered reader are now installed; the combined Python suite has 51 passing tests. See [camera preparation](../../firmware/esp32-camera/README.md) for the separately compile-tested, user-flashed ESP32 firmware.
