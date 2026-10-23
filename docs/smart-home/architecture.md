# Observable home and robotics architecture

Research and local inspection: September 6, 2026. This is a proposed architecture and bring-up plan, not a deployed system. A read-only HTTP connection attempt to the supplied OctoPi address timed out; no devices were moved. Device support remains unverified on the owner's hardware. See [the device integration plan](device-integration-plan.md) for the updated owner inventory and adapter choices.

## Recommendation

**Updated user decision: build our own dashboard.** [Homebase](../../apps/homebase/README.md) is the local frontend, with a house schematic, relationship view, editable inventory and deterministic task previews. It has no Home Assistant dependency. The backend described here remains a proposal.

Build a local home service around our dashboard, a shared world model and a small durable task coordinator. Home Assistant may be added as one optional device adapter. Put ROS 2 robot skills behind the same capability interface as household devices. An interchangeable language model interprets requests and proposes typed tasks; local code checks permissions, prerequisites, execution and outcomes. Start with one coordinator and ordinary specialist services. Multiple conversational agents are optional later, not a prerequisite.

The first milestone is an observable L70: state, battery, connectivity, then supervised pause/dock/clean operations with state verification. Treat mobile manipulation as a separate milestone gated by base control, mounting, calibration and reach.

## Components and ownership

| Component | Initial implementation proposal | Responsibility |
|---|---|---|
| Device gateway | Custom adapter registry; optional Home Assistant bridge | Devices, entities, areas, vendor integrations and routine automations |
| Coordinator | Small Python service with typed schemas and a persistent task state machine | Interpret, resolve targets, authorize, dispatch, cancel, verify, report |
| Model gateway | OpenAI API or local Ollama/llama.cpp adapter | Common application-level contract for structured tasks; backend-specific request translation |
| World model | PostgreSQL tables for entities, relations, observations, tasks and events | Timestamped evidence and relationships; export graph-shaped JSON to the UI |
| Event transport | HA WebSocket events; local MQTT where needed | State changes and availability; ROS stays on ROS transports |
| Vision | Frigate for compatible fixed cameras; separate ESP32 camera adapter | Detection events, room zones, timestamped images and camera health |
| Robot skills | ROS 2 Jazzy, MoveIt 2; Nav2 only if base interfaces qualify | Motion planning, trajectories, transforms, controlled recovery |
| Views | Graph view plus a Three.js-based house view as a later UI choice | Shared entity selection and configuration, object locations, evidence age |
| Operations | Structured logs and metrics, persistent volumes and backups | Task traces, connector health, authentication expiry, latency and storage usage |

Home Assistant exposes state subscriptions and correlated API responses over its [WebSocket API](https://developers.home-assistant.io/docs/api/websocket/). Reuse that integration surface instead of writing a second implementation of every vendor API. Use MQTT for custom hardware events, not as a replacement for the camera media pipeline or high-frequency robot control.

The gateway exposes functions such as `home.get_state`, `home.set_cover_position`, `perception.last_observation`, `vacuum.return_to_dock`, and eventually `robot.pick_place`. Tool schemas specify targets, units, argument bounds and expected results. [OpenAI function calling](https://developers.openai.com/api/docs/guides/function-calling) provides the model-to-application exchange; the application still implements validation and execution. An OpenAI-compatible local endpoint does not guarantee identical tool or vision behavior, so each backend needs contract checks.

## Task lifecycle

Use `received -> planned -> ready -> executing -> verifying -> succeeded`, with explicit `blocked`, `cancelled`, `failed` and `outcome_unknown` states. Persist the task before dispatch. Every operation carries a task ID, command ID, entity ID, deadline and configuration revision. Acquire a per-device lease so automation, a user joystick and the coordinator cannot command the same actuator concurrently.

Reads can retry. Before retrying an action after a timeout, reconcile actual device state: a missing reply does not prove that nothing happened. Never replay retained MQTT movement commands. A restart may resume observation, but must reconcile physical tasks before resuming them. No automatic cloud fallback when local-only processing is selected.

Record desired state, acknowledged command and observed state separately. A serial write is not proof that a joint moved; a vacuum accepting a docking command is not proof that it docked. Completion requires an observable postcondition. If it cannot be observed, report that limitation.

Routine home automations remain functional when the LLM is unavailable. A robot's stop, command timeout and actuator limits must work without the cloud, coordinator, Wi-Fi or graphical interface. Household controls can have standing authorization; early robot experiments use an explicit armed session and local stop controls. Do not require a new confirmation for every ordinary authorized action.

## What the example requests entail

### “Where is my son?”

Resolve the household member, query recent authorized observations, compare room and camera coverage, and return the location with evidence age. Example output: “Last seen in the living room 20 seconds ago; I don't have a newer observation.” If confidence or coverage is inadequate, return unknown. Occupancy, phone Wi-Fi presence and a particular person's location are different facts.

Frigate supports local [face recognition](https://docs.frigate.video/configuration/face_recognition/) and timestamped tracking integrations. Enrolled household identities, occupancy sensors and optional tags can be fused, but identity and room transitions require validation in the actual home. Camera placement, occlusion and stale frames impose real limits. Keep raw interior video and identity data local by default; cloud image processing should be an explicit per-camera setting. Provide household access controls, private zones and configurable retention.

The current [Ring integration](https://www.home-assistant.io/integrations/ring/) supports live camera feeds and events through Ring's cloud. Last recordings require a Ring subscription; two-way live audio is not supported. Test each installed model's stream availability and latency before relying on it for continuous perception. A working HA camera entity does not establish a dependable local RTSP stream. Prefer a verified local stream for robot perception.

### “Put the toy on the couch.”

Resolve which toy and which couch, obtain a metric object pose, check graspability and load, find reachable pickup and placement poses, approach, stabilize the base, plan the arm trajectory, grasp, verify retention, transport, place and verify the object is released at the destination. Ask only when the target cannot be resolved reliably.

[MoveIt Task Constructor](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html) can compose manipulation stages. [Nav2](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/) is an option when the base supplies suitable control, localization, sensor data and transforms. Neither creates missing Eufy interfaces. A cleaning-room command is not a navigation pose goal.

The ESP32 wrist camera is useful for inspection and visual alignment. One uncalibrated RGB camera does not directly measure metric depth. First use a known object and calibrated tabletop/fiducials; evaluate depth sensing or validated multiview estimation for less constrained pickup. Calibrate camera intrinsics, camera-to-gripper, tool-center and base-to-arm transforms.

An arm merely resting on the vacuum has no reliable fixed transform. It needs a rigid mount, a measured pose, payload/power assessment and clearance around the vacuum sensors. Check both floor and couch reach from the installed height before promising this task. A simulation scene and a plausible-looking trajectory do not establish physical feasibility.

## L70 integration decision

Two candidates are [jeppesens/eufy-clean](https://github.com/jeppesens/eufy-clean), a Home Assistant custom integration that lists the L70/T2190 family, and the upstream [martijnpoppen/eufy-clean SDK](https://github.com/martijnpoppen/eufy-clean), which describes cloud/MQTT discovery and a local Tuya path. These are community projects, not an official Eufy robotics API.

The integration says its principal test devices are X10 Pro Omni and C28 Omni. Transport is discovered per device, not reliably inferred from a marketing name. Its Tuya path lacks map/live XY access; the richer map features must not be assumed for an L70. MQTT support by itself does not imply offline operation. These limitations make capability discovery the first gate. Start with the HA integration if compatible; use the SDK as an adapter fallback, not a parallel competing controller.

Bring-up sequence:

1. Confirm exact model number, firmware, Eufy app operation, account region and the device's reserved LAN address. Enter credentials in the local integration UI, not this repository.
2. Pin the reviewed integration version and inventory only. Record transport, read-only state fields and supported actions. Do not initialize using an example that starts cleaning.
3. Verify battery, state and availability against the app. Observe disconnect/reconnect; label old data stale.
4. With the attachment secured or removed and the path clear, test individual pause, dock and short cleaning commands. Verify each resulting state. Demonstrate a local stop method.
5. Test offline behavior in a separate controlled trial and explicitly record cloud dependencies. Inspect map, pose, manual-drive and sensor capabilities individually.
6. Enable only proven capabilities. Arbitrary movement, `cmd_vel`, odometry, scan access and pose goals remain unverified. If unavailable, keep the L70 as a vacuum integration and use an open mobile base for manipulation, or undertake a separate measured localization/control project.

Acceptance: one dashboard entity reports current state with a timestamp; a command has a trace from request to observed outcome; an outage produces unavailable/unknown rather than a stale success. There is no claim of autonomous mobile manipulation at this stage.

## Mini-Arm audit

Checkout inspected: `666bbb4` (2026-01-07). Run `python docs/smart-home/audit_model.py` for repeatable, read-only XML/reference checks.

| Finding | Consequence / required work |
|---|---|
| Four unique referenced meshes absent: `base_link.STL`, `electronics_link.STL`, `finger_link.STL`, `knuckle_link.STL` | Recover correct CAD exports and verify scale/origins; similar filenames are not evidence of equivalent geometry |
| SRDF refers to absent `miniarm_link0`, `miniarm_link6`, `miniarm_link7`, `miniarm_link8` | Rebuild groups, virtual joint and collision exclusions against the actual chain |
| Controller requests absent `miniarm_joint6`; URDF wrist joint is `miniarm_gripper_link_joint` | Choose a canonical joint mapping and update URDF, SRDF, controller and teleop together |
| Named states put joint 4 below its URDF lower limit | Revalidate ready/transport poses against measured mechanical limits |
| `fake_components/GenericSystem` backend | Supply a real hardware bridge with explicit cancellation and state semantics |
| No camera links; no inertial elements | Add measured wrist-camera frames; obtain mass/inertia for dynamics simulation |
| Host `set_joints` exists but firmware parser disables that command | Reconcile the wire protocol before exposing trajectory execution |
| Client declares workspace limits but `set_pose` sends without checking them | Implement validated limits at the gateway and controller, including finite values and units |
| Firmware homes during initialization; `stop` command stops a trajectory | Review startup motion and distinguish trajectory cancellation from physical emergency stop |
| Joint state comes from stored angles/PWM command state in the inspected path | Treat as commanded/estimated unless independent sensor feedback is established |

The ESP32 test contains an old hardcoded stream address. That is a historical setting, not a verified current endpoint. Move it to device configuration. Robot runtime files have not been edited: correcting physical geometry requires measurements and comparison to the real assembly.

## Device configuration and account management

Keep separate records for devices, entities, capabilities, integrations, credentials, frames and observations. A washer can be one device with cycle, remaining-time and power entities. A camera has both a network endpoint and a spatial pose. Each capability has a status such as unverified, available or unavailable, supported arguments, prerequisites, evidence and verification date.

`device-registry.example.json` is an illustrative application schema, not valid HA configuration and not an executable deployment. The UI should edit a validated configuration model, version changes, test connections and apply updates transactionally. Fields cover hardware/firmware, placement, endpoints, adapter version, credentials reference, calibration, units, permissions and health. Keep manufacturer facts distinct from user labels and observations.

Let native integrations own vendor login/session renewal. Use OAuth/device authorization when the vendor supports it; otherwise keep credentials in an integration-local secret store. Expose only connection status and a reconnect flow to the coordinator. A `secret_ref` is a reference, not the secret itself. HA `secrets.yaml` provides separation but is not encryption. Apply filesystem protection and protected backups; never place keys in graph exports, logs, URLs or prompts. Scope gateway actions independently rather than assuming a general HA token has per-device permissions.

Inventory router/switch/AP relationships from their management interfaces or exports; DHCP and ARP alone cannot establish the full physical topology. Record `connected_to`, `located_in`, `mounted_on`, `observed_by` and `controlled_by` as distinct relation types with source and freshness. Use stable IDs despite IP changes. For isolation, plan a trusted service network plus IoT/camera segments with explicit permitted flows, adjusting discovery relays for the actual router. No broad network scan has been performed.

For other appliances, select adapters after model inventory. Prefer manufacturer/local HA integrations for blinds and laundry; appliances with remote-start locks retain their physical prerequisites. [OctoPrint](https://www.home-assistant.io/integrations/octoprint/) is a candidate for compatible printers; Klipper/Moonraker and vendor-specific printers need their respective adapters. Monitor temperatures/jobs first and expose only validated print controls.

## House and graph views

Use a single entity ID in every view. Graph mode can switch between physical network topology, room membership and control dependencies; an edge must say what it means. The 3D view shows room geometry, devices, camera coverage, robot pose uncertainty and current task overlays. Selecting a device opens the same settings/evidence panel in both views. A frontend model has no authority to command actuators directly.

The supplied address is insufficient to reconstruct the interior. Start with a measured floor plan or owner-supplied scan, verify dimensions and produce a lightweight glTF/GLB model for display. Keep simplified navigation and collision geometry separately, with a measured transformation between house, vacuum map and ROS frames. Example layouts are illustrative until replaced with measurements. A vacuum's 2D map, even if exportable, does not supply ceiling heights or complete furniture geometry.

## Compute and NVIDIA

Verified on this PC: Ryzen 5 5600X, 6 cores/12 threads, approximately 64 GB RAM, RTX 3070 Ti with 8,192 MiB VRAM and driver 591.86. WSL2 Ubuntu 24.04 is running; Docker Desktop's WSL distribution is installed but was stopped. Approximately 146 GB free on C: and 138 GB on G: were reported. These are snapshots, not performance measurements.

Start here: HA and coordinator on CPU; benchmark one small quantized instruction model with the real tool schemas; add one camera workload and measure combined latency/VRAM. A 3–8B quantized model is a sizing experiment, not a throughput or fit guarantee: context cache, vision and concurrency consume additional memory. [llama.cpp server](https://github.com/ggml-org/llama.cpp/blob/master/tools/server/README.md) is a local option. HA has separate [OpenAI](https://www.home-assistant.io/integrations/openai_conversation/) and [Ollama](https://www.home-assistant.io/integrations/ollama/) integrations; its OpenAI integration is not a generic endpoint for local providers. API operation requires an API key and enabled API billing.

The owner confirmed this PC as the initial host, with portability to a dedicated PC later. Runtime checks subsequently found that the current HA guide excludes Docker Desktop, correcting the initial Windows container proposal. Recommend HAOS in a VM for the hub and Docker Compose for custom services; alternatively, provision a dedicated native Linux Docker Engine environment and validate its networking. Selection is pending; see [runtime decision](runtime-decision.md). The accompanying Compose files are a native-engine foundation, not a validated Desktop deployment. For development, reuse WSL Ubuntu for ROS while checking USB and discovery behavior. For continuous operation, prefer an always-on local host/VM for household services so desktop sleep or experiments do not interrupt automations. GPU jobs can initially remain on this PC. No hardware purchase is needed for the first integration milestone.

NVIDIA developments checked:

- [March 16, 2026 model announcements](https://investor.nvidia.com/news/press-release-details/2026/NVIDIA-Expands-Open-Model-Families-to-Power-the-Next-Wave-of-Agentic-Physical-and-Healthcare-AI/default.aspx): Nemotron for agentic work, Cosmos 3 for physical AI and GR00T N1.7 for robotics. These occupy different roles in this design.
- [July 7, 2026 GR00T workflow](https://developer.nvidia.com/blog/develop-humanoid-robot-policies-end-to-end-with-nvidia-isaac-gr00t/): covers teleoperation, LeRobot-format datasets, post-training and evaluation with GR00T 1.7. It is an attractive later learning pipeline, not a ready-made policy for this custom arm/vacuum assembly.
- [Current Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html): list RTX 4080 and 16 GB VRAM as minimum GPU specifications. This PC is below that GPU/VRAM baseline. Start model inspection in RViz and use a lightweight simulation path; evaluate a suitable remote GPU or upgrade only for a concrete Isaac workload.
- [Jetson Thor](https://www.nvidia.com/en-au/autonomous-machines/embedded-systems/jetson-thor/) is a potential future onboard inference platform. First establish power, mounting and measured inference requirements; a dedicated edge computer is not needed just to connect the L70.

## Implementation gates

1. **Inventory and L70 visibility:** configure the hub, discover capabilities, display evidence age and verify basic control.
2. **Arm and camera bench setup:** repair description/configuration, calibrate the assembly, reconcile firmware, validate stop and supervised teleoperation.
3. **Observable home:** add fixed cameras and appliance entities, event history, graph links, authentication health and a measured floor plan.
4. **Language interface:** same validated tools through API and local backends; evaluate ambiguous targets, stale data, offline devices and interrupted commands.
5. **Bounded pickup:** known light object at a measured surface, fixed base, low-speed motion, measured placement and grasp verification.
6. **Mobile manipulation:** only after base interfaces, localization, rigid mounting, reach, stability and obstacle handling pass dedicated trials.

Confirmed by owner: initial host is this PC; the previous Raspberry Pi 3B HA installation is unavailable; router is Verizon CR1000A; printer is Snapmaker 2.0 through OctoPi; laundry models are GE GFW550SSNB1WW and GFD55GSSN0WW. Living-room shade item 2318932 and kitchen shade item 2318939 share FCC ID 2AVVDAC136-06; the device plan documents their 433.92 MHz RF path. Remaining inputs: L70 attachment/app status, current OctoPi address, Bond bridge ownership/compatibility check, SmartHQ enrollment status, Ring models, arm firmware/serial connection, camera endpoint, mounting measurements and floor plan or scan. Supply account credentials only through local setup interfaces.
