# USB communication update — September 6, 2026

The Pico 2 W now runs controller source version **1.4.10**. The USB reader, controller and entry point were installed on COM3 with servo power disconnected. Firmware backups are in ignored `build/pico2w-backups/20260906-214404-usb/`. No motion command was sent during verification. All six arm outputs still reported disabled afterward; the prior seven-channel startup clearing remains in place.

## Processing acknowledgments

The updated Python client requires firmware 1.4.10 or newer. It sends one printable ASCII command at a time, wrapped in unique processing fences, for example:

```text
sync:UNIQUE_HEX_TOKEN-b;get_joints;sync:UNIQUE_HEX_TOKEN-e;
```

The controller returns `@sync:UNIQUE_HEX_TOKEN-b`, command output, then `@sync:UNIQUE_HEX_TOKEN-e`, each newline-terminated. The client waits for the matching pair, serializes requests using a lock, ignores older output, and never automatically retries after a timeout. These markers report handler progress: they are **not proof of physical target attainment**, and legacy handlers do not all return structured success/error results. The client raises for explicit parser errors; full structured command-result reporting is a future protocol extension. Asynchronous trajectory output can be interleaved; it is not correlated telemetry.

Fixed 10 ms post-write, 50 ms generic response, and 100–300 ms query delays have been removed. The host reads buffered bytes with a short configured timeout and bounded response size. It does not reconfigure the Windows serial port inside the receive loop. Joint queries now parse the real timestamp-prefixed firmware output correctly.

The firmware reads up to 256 bytes at a time, preserves partial and back-to-back frames, and caps each command at 512 bytes. Oversized frames are discarded through their delimiter; their tail cannot execute as a truncated command. Invalid ASCII is rejected. The host accepts only one command per exchange; legacy raw serial tools can still use the semicolon protocol but must manage their own pacing.

CircuitPython's console title output initially obscured a reply fence. The entry point now disables `supervisor.status_bar.console`, and the client also tolerates terminal escape sequences. Regression tests cover this actual failure.

## Measured results

Twenty measured exchanges per command, after three warmups, through the updated client on COM3:

| Query | Median | Maximum |
|---|---:|---:|
| test | 8.29 ms | 18.26 ms |
| get_joints | 8.95 ms | 17.96 ms |
| info | 26.07 ms | 35.74 ms |

Local output: `build/usb-benchmark.json`. These include two processing fences and response parsing. The earlier 2.69 ms measurement was a simpler raw test query with no fences, so it is not an equivalent benchmark. The improvement is removal of the old client's deliberate waits, complete-response handling and request pacing, not a faster USB physical link. No sustained motion/control rate is inferred from these measurements.

51 Python tests pass, including chunk boundaries, overflow recovery, fragmented replies, stale fences, timeout-without-retry, errors, and actual console escape sequences. The updated client successfully parsed real joint-status output and completed all hardware benchmark exchanges.

## Remaining motion work

This update does not replace the existing synchronous interpolation helper or make Windows/CircuitPython hard real-time. Timed local motion, interruption/watchdog behavior and calibrated joint limits still need a separate motion-controller implementation before powered motion testing. USB receipt acknowledgments must not be used as a safety stop or completion signal.

## Base-test extension (controller 1.4.11)

Installed September 7 with servo power disconnected. `cal_probe` returns one `@cal:` JSON line containing protocol `miniarm-base-pulse-v1`, channel 15, duration 400 ms, one-degree maximum target change, five-degree window, initial/last requested targets, fault/mode state and all seven PWM command states. Processing fences still wrap the response.

`cal_pulse:TOKEN:SERVO_DEGREES:EXPECTED_PREVIOUS` is the explicit base-only movement command. `EXPECTED_PREVIOUS` is `none` for the first target, otherwise the last acknowledged target. The first angle is user-selected, with no inference from model zero. The controller rejects malformed/nonfinite targets, another active output, running trajectories, duplicates, stale state, immediate repeats and changes beyond the test range. On acceptance it locks legacy commands, writes only base channel 15 and releases that output on its own 400 ms deadline. It returns structured success only after the release attempt and status read; exceptions return `ok:false`. A write/release fault prevents further tests. The recent duplicate-token cache holds 16 entries; clients must never automatically retry a timed-out pulse.

The async handler yields while waiting for the deadline. The regular controller update loop also checks the deadline; it is not dependent on further USB traffic. This is a best-effort software deadline, not a certified watchdog or physical stop. I²C latency, driver/controller failures and servo mechanics remain outside that guarantee. The servo may not reach its requested target during the pulse and may drift when unpowered. Only PWM command state is observed. Disconnect servo power if behavior is unexpected or release is not confirmed.

The browser base panel checks exact protocol capabilities before exposing tests and requires the output-release report to match the requested target and test center. No motion command was issued during installation verification; a live `cal_probe` confirmed all seven outputs off. See the [Homebase base-test workflow](../../apps/homebase/README.md#base-servo-desk-test-september-7-update) for use and validation.

## Supervised hold extension (controller 1.4.12)

Installed September 7 with user-confirmed servo power disconnected. Four files were backed up in `build/pico2w-backups/20260907-093625-hold-test/` and copied with hash verification. A live read-only probe confirmed hold-v1, a 3000 ms deadline, all seven outputs off and no fault or active session. No pulse or hold command was sent; physical hold behavior remains unverified.

`cal_probe` retains the pulse-v1 contract and adds `hold_protocol: "miniarm-base-hold-v1"`, `hold_ms: 3000`, `mode` and `active_token`. Existing pulse clients remain compatible.

`cal_hold:TOKEN:EXPECTED_PREVIOUS` starts a three-second hold of the last successfully commanded pulse angle. It cannot select a new angle and rejects use before a pulse, while a trajectory is active, or while another output/session is active. It retains the pulse limiter's duplicate protection and cooldown. Activation replies immediately with structured status; the parser can then process `cal_release:TOKEN` before the deadline. Only the matching active hold can be released by that token. Release after timeout is idempotent; it performs no servo write when inactive.

The controller's update loop releases the hold at the deadline, with no renewal command. The browser observes for up to three seconds, then sends an owned release/status request; Release now or tab hiding requests it earlier. It attempts release even after uncertain activation, using a fresh signal after a canceled request. Completion requires all seven outputs off and unchanged last-target/center state. A release error is surfaced rather than reported as success. No hardware pulse or hold is sent by the installation checks.

## Shoulder extension (controller 1.4.13)

The base `cal_*` commands retain their existing wire format. Shoulder uses `shoulder_probe`, `shoulder_pulse:TOKEN:ANGLE:EXPECTED_PREVIOUS`, `shoulder_hold:TOKEN:EXPECTED_PREVIOUS`, and `shoulder_release:TOKEN`. Replies remain `@cal:` JSON within processing fences, identifying `miniarm-shoulder-pulse-v1`, `miniarm-shoulder-hold-v1`, and channel 14. The seven-output array keeps arm order: base at index 0, shoulder at index 1, gripper at index 6.

Each joint has separate target/window/token/session state. Shared controller checks refuse new motion while any session is active or faulted; the existing all-output-off check is also retained. The update loop services every deadline, and controller shutdown releases every active session. Any session's calibration lock disables legacy commands. The browser binds request prefix, expected response protocol, hardware channel, hold protocol and output index to the chosen joint, rejecting mismatches before accepting a test as active or complete.

## All-joint desk-test extension

Installed September 7 without another revision increment. Capability identifiers determine compatibility. All joints expose the same probe/pulse/hold/release argument formats described above:

| Joint | Command prefix | PCA9685 channel | Output-array index |
| --- | --- | --- | --- |
| Base | `cal` | 15 | 0 |
| Shoulder | `shoulder` | 14 | 1 |
| Elbow | `elbow` | 13 | 2 |
| Wrist rotation | `wrist_rotation` | 12 | 3 |
| Wrist bend | `wrist_bend` | 2 | 4 |
| Gripper link | `gripper_link` | 1 | 5 |
| Gripper | `gripper` | 0 | 6 |

Protocol IDs use hyphenated names, e.g. `miniarm-wrist-bend-pulse-v1` and `miniarm-wrist-bend-hold-v1`. These tests send explicit raw servo angles, including for the gripper. They do not apply URDF limits, infer positions, or coordinate joints. The 400 ms pulse, three-second hold, one-degree change, five-degree window, duplicate protection, fault latch and owned release apply to every joint.

The installer verified the Pico identity, backed up four files to `build/pico2w-backups/20260907-101806-all-joints/`, disabled autoreload during replacement and verified hashes before restarting. Read-only probes before and after the update reported all seven PWM outputs off. Post-update probes verified every joint's protocol, channel, deadlines and empty target state. No movement command was sent; PWM status does not measure supply power. Installation log: `build/pico2w-all-joints-update.log`.

## Drag-to-move slider

Installed September 7 after the user asked to continue. Servo power was instructed to remain disconnected; software does not measure supply power. No revision increment. Each probe adds `miniarm-JOINT-sweep-v1`, `sweep_ms:3000`, `sweep_watchdog_ms:600`, `sweep_rate:10`, `sweep_target` and `sweep_sequence`. The hyphenated joint identifiers and command prefixes follow the table above.

- `PREFIX_sweep:TOKEN:EXPECTED_PREVIOUS` starts at the last commanded angle. A prior explicit pulse establishes that target; existing all-output-off, cooldown, duplicate-token, trajectory, fault and global active-session checks apply.
- `PREFIX_target:TOKEN:SEQUENCE:ANGLE` changes the desired angle within the existing five-degree window, clipped by the 0–180° API range. Sequences must increment exactly from one, and the token must own the active sweep. Invalid or stale packets never refresh the watchdog. An expired session cannot be revived.
- The normal controller loop slews the commanded angle at at most 10°/s, with each tick limited to 50 ms of progress so a stalled loop cannot cause a catch-up jump. It attempts PWM release at 600 ms without a valid target update or the original three-second deadline, whichever comes first. Heartbeats never extend the latter. Write/release faults latch. These are software deadlines, not a hardware emergency stop.
- `PREFIX_release:TOKEN` now also accepts an owned sweep. It releases output immediately when processed. The browser sends the newest target only, sequentially, with 100 ms between exchanges and no queue or retry. Pointer/key release, cancellation, loss of pointer capture, focus loss and hidden/unmounted pages request release; the controller watchdog remains independent of browser cleanup. After any uncertain start/update the browser attempts owned release with a fresh signal.

The sliders report desired and last-commanded angles, never encoder positions. A restarted or released servo can physically drift from its last command. The all-joint dashboard initializes an untested joint with a 400 ms pulse when the user changes and releases its initial-angle slider; later drags use the sweep protocol. The original pulse/hold controls remain expandable. One shared connection checks all seven joints without initializing them. Hardware supports one joint at a time; this does not adopt the unresolved URDF limits or enable full-range sweeps.

Installation backed up four Python files to `build/pico2w-backups/20260907-115306-slider/`, verified the copied hashes and restarted the Pico. Before installation all seven sessions were idle and fault-free. Afterward, read-only probes verified every joint's slider protocol, channel, 3000 ms maximum, 600 ms watchdog, 10°/s rate and empty target state. All PWM outputs read off; no movement command was sent. COM3 was released. Log: `build/pico2w-slider-update.log`. Physical slider behavior awaits supervised user testing.

## Immediate jogging and Home

The immediate-jog update adds `jog_protocol: miniarm-JOINT-jog-v1` without incrementing the firmware revision. The dashboard requires it on every joint before enabling the Jog toggle. Toggle-on sends no motion; toggle-off, Release, page hiding and navigation cancel the active command. Sliders preview while disabled. While enabled, the first new slider value starts a session immediately, with no pointer-release initialization step. A first target establishes the current ±5° window; subsequent targets remain within it. The initial physical position is unknown, so the first write may produce a large move. One joint at a time, 10°/s slew, three-second absolute deadline and 600 ms watchdog remain.

- `PREFIX_jog:TOKEN:TARGET:EXPECTED_PREVIOUS` starts a session at an explicit new target. `none` is allowed only when the joint has no last command. Existing target/window, fault, duplicate-token and all-output-off checks apply.
- `PREFIX_target:TOKEN:SEQUENCE:TARGET` is sent only if the desired position changes.
- `PREFIX_keep:TOKEN:SEQUENCE` refreshes the watchdog without carrying or changing a position. Sequence and ownership validation are identical to target updates. It cannot revive an expired session or extend the absolute deadline. Controller interpolation writes only when its commanded angle actually changes.
- `PREFIX_release:TOKEN` ends either jog or legacy sweep/hold sessions. No timed-out motion is retried.

Home is a host-side, prevalidated sequence using the existing firmware model-zero references: base 90°, shoulder 90°, elbow 45°, wrist rotation 90°, wrist bend 110°, gripper link 90°. The gripper is excluded. Home checks all seven statuses and mappings plus every target's current window before sending anything, skips unchanged commands, then moves one joint at a time with release confirmation and cooldown between joints. Cancellation or failure stops the remaining sequence. Home does not bypass calibration locks, invoke legacy homing, infer encoder positions, or promise the released arm maintains that pose.

Validation before installation: 122 Python tests and 69 dashboard tests passed, including new first-jog setup, idle keep-alives without repeated writes, watchdog expiry, target deduplication, Home preflight and cancellation. TypeScript, lint and production build passed. Subsequent Connect/Disconnect changes brought the dashboard suite to 70 passing tests.

Installed September 7 at 13:10 local time after the user confirmed servo power disconnected and USB connected. `build/install-immediate-jog.py` verified the COM3/CIRCUITPY identity, backed up four files to `build/pico2w-backups/20260907-131029-immediate-jog/`, disabled autoreload during replacement, verified copied hashes and soft-restarted the controller. All seven read-only post-update probes confirmed immediate-jog, pulse, hold and sweep capabilities, expected channels, timing limits, no active session or fault, empty target state and all seven PWM outputs off. COM3 was released. No movement command was sent. Log: `build/pico2w-immediate-jog-update.log`. Physical jogging and Home remain for supervised user testing.
