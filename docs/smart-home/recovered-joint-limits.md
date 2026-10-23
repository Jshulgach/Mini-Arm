# Recovered joint-limit references

Inspected September 7, 2026 after the user recalled saved assembly limits.

The strongest per-joint candidate is `ros2/miniarm_description/urdf/miniarm.urdf`. Its limits already appear in commit `d9b7419` (January 5, 2025) under `miniarm_ros/miniarm_description/urdf/miniarm.urdf`; the 2026 restructure moved the file. The user reports the servos accept 0–180 degrees and considers the supply sufficient for holding all servos. Supply capability and these mechanical ranges have not been measured in this session.

| Joint | Saved URDF model range, approximately | Recovered servo conversion | Corresponding servo commands, approximately |
| --- | --- | --- | --- |
| Base | −90° to +90° | 90 − model angle | 0–180° |
| Shoulder | −90° to +90° | 90 − model angle | 0–180° |
| Elbow | −45° to +135° | 45 + model angle | 0–180° |
| Wrist rotation | −90° to +90° | 90 + model angle | 0–180° |
| Wrist bend | −90° to +90° | 110 − model angle | **20–200°; conflicts with servo API** |
| Gripper link | −90° to +90° | 90 + model angle | 0–180° |
| Gripper knuckle | −135° to 0° | No verified linkage conversion | Unknown |

URDF bounds are in radians: ±1.5708 for most arm joints, −0.785 to 2.3561 for the elbow, and −2.35619 to 0 for the gripper. Rounding above is intentional; these are not new measured limits. Firmware conversions use the installed channel order, offsets, direction flags and orientation flags recovered from the Pico before the repair. In particular, wrist bend uses `110 - model_degrees`. Intersecting its mathematical range with the servo API would restrict model angles to about −70° through +90°, but that calculation alone does not establish clearance or a safe mechanical limit.

Other sources conflict or do not provide assembly calibration:

- `docs/source/getting_started/quickstart.rst` lists base/wrist rotation/gripper link at ±180°, shoulder/wrist bend at ±90°, and elbow at ±135°. That table arrived with the January 2026 documentation restructure and differs from the URDF.
- `docs/source/getting_started/configuration.rst` contains an example `mini_arm_config.yaml`; no such saved assembly configuration was found in this checkout.
- `ros2/miniarm_moveit_config/config/joint_limits.yaml` contains velocity/acceleration overrides, not per-joint position bounds.
- The original Pico backup, `build/pico2w-backups/20260906-211847/lib/arm_control/robotarm.py`, uses generic ±2π constraints for every arm joint. Its `constraints` property names `j1_range` through `j6_range`, but these attributes are not initialized there.
- The May 2023 history initially defines those named ranges uniformly at ±2π; commit `ebd2a78` replaces them with uniform ±π/2 constraints. Later revisions use broad ±4π or ±2π defaults. These are not distinct measured assembly limits.

No recovered range has been installed as a new physical motion limit or copied into the worksheet's measurement fields. Homebase shows the URDF references beside the existing recovered servo mappings, including the wrist conflict. Full joint-limit adoption needs matching model axes/zero poses to the assembled arm and resolving the gripper linkage conversion.

The supervised diagnostic now supports all seven servos individually. Each hold repeats that joint's last pulse target and retains its own one-degree change / ±5-degree test window. These are raw servo tests; the conflicting wrist model limits are not applied. Tests do not validate mechanical clearance, coordinate joints, or assess the supply under combined load. Only the base pulse and hold have been physically verified by the user so far.
