# Development checkpoint - 2026-10-23

This branch preserves the standalone Homebase extraction, Mini-Arm calibration and USB protocol work, gripper-camera firmware, and ROS model repairs. Homebase application development continues in its own repository.

## Fresh validation

- 122 software tests passed on Python 3.11, using mocked serial and servo I/O.
- Ruff and Black passed for the Python client package. Black's old exclusion silently skipped the entire ROS tree; the client check now runs.
- Source and wheel distributions built and passed Twine metadata validation. No package release was published.
- Static model checks found no missing mesh, semantic-link, control-joint or joint-state limit references. The ROS controller remains `mock_components/GenericSystem`.
- Host-compiled camera handler tests passed for JSON parsing, bounds, origins, auto/manual settings and failure handling. No firmware was flashed.
- Two newer Python string methods were replaced to retain declared Python 3.8 compatibility. The full supported-version matrix still needs CI verification.

## Remaining physical work

Servo commands are not encoder measurements. Model-to-servo signs, offsets, mechanical joint limits, camera-to-gripper transform and coordinated motion require supervised validation. The current URDF has no inertial definitions or camera links; it is not a validated dynamics model. No actuator was commanded during this checkpoint.

User screenshots, local camera Wi-Fi configuration, household inventory, generated builds and recovery backups remain outside Git. Historical smart-home design notes describe the state at their original dates; current Homebase integration status belongs in the standalone Homebase repository.
