# Historical implementation notes

These notes were written while Homebase lived in `Mini-Arm/apps/homebase`. Paths, setup instructions and validation claims below refer to their recorded stage. Use the root README for current setup. Robot firmware history remains associated with Mini-Arm.

# Homebase

A custom local dashboard for this home. Home Assistant is optional; this app does not depend on it or require a HA installation.

## Implemented

- House view with the two identified rooms, selectable devices and a tilted schematic view.
- Connection view for reported camera → arm → L70 mounting and OctoPi → printer relationships, with zoom and selection.
- Searchable inventory of 12 entries, including the PC webcam and an unexpanded Ring camera group.
- Device inspector with known specifications, provenance, next integration step and editable name, room and endpoint.
- A persistent SQLite gateway, served with the dashboard, owns device names, rooms, endpoints, monitoring settings, relationships and notification rules.
- `/systems`: device editor, room creation, selectable relationship graph, activity timeline, local notifications, rule simulation, and reviewed settings backup/import. Original browser settings can be imported explicitly; they are not erased.
- Opt-in TCP port monitoring every 30 seconds with source, timestamp, latency and stale-state labels. All devices start with monitoring off. The unavailable OctoPi and disconnected arm have owner-reported notes, not fabricated live states.
- Three deterministic task previews for docking, household-member location and toy pickup. Every preview states what is missing. There is no language-model inference or execution.

On-demand camera previews and the separate Mini-Arm controls remain available. Vendor account authorization, appliance state adapters, network discovery, coordinated robot motion and a measured 3D model are not implemented. Monitoring checks TCP reachability only, not identity or appliance state. Room dimensions and adjacency in the schematic are illustrative. An assigned room is configuration, not a live location observation.

## Run locally

Use Node 24.20.0 or later and npm. Node 24.13.0 on this Windows PC produced a native process-shutdown assertion after generating the build; an isolated Node 24.20.0 build completed successfully. The system Node installation was not modified.

```powershell
cd C:\Users\jonat\Documents\GitHub\Mini-Arm\apps\homebase
npm ci
npm run dev
```

Open http://127.0.0.1:5173. The server is bound to this PC only. For a production static build:

```powershell
npm run build
npm start
```

On this PC, while retaining the older system Node, use the verified isolated runtime for the build:

```powershell
npm exec --yes --package=node@24.20.0 -- node node_modules/vinext/dist/cli.js build
npm start
```

Build and use `npm start` for the full application: it serves `dist/client` and `/api/*` together on loopback port 5173. The Vite development server is frontend-only. Stop it before starting the full service. Set `PORT` to change the port. Set `HOMEBASE_DATA_DIR` to choose a persistent data directory; the default is `apps/homebase/data/`, ignored by Git and Docker build context.

## Docker and moving to another PC

```powershell
docker compose up --build -d
```

Compose builds the dashboard and runs a Node service with a persistent `homebase-data` volume at the same loopback URL. Stop any existing process on port 5173 before using Compose. On September 8, 2026, the Docker image built successfully and ran as the unprivileged Node user on a temporary loopback port. HTTP checks verified `/concepts` and `/api/home`, with 12 seeded devices and monitoring disabled. The temporary container was removed; the existing local service remains running. The dependency lockfile was repaired to include a missing transitive runtime required by a clean install. Settings backup/import transfers configuration to another PC, with monitoring and rules disabled on restore. For a full history backup, stop the service and copy the complete data directory. The API enforces loopback Host and same-origin browser requests; remote-access authentication is not implemented.

## Chat and adaptive canvas

The canvas starts in a simplified everyday view: chat, saved workspace tabs, a compact registry summary, and widget content. **Edit canvas** reveals Add widget, Reset layout, workspace management, reorder/remove controls and keyboard/pointer resize handles. **Done editing** removes those controls from the interface without changing the saved layout. Editing mode itself is not persisted.

## Virtual-home playground

Open **Virtual home** from the canvas or visit `/playground`. This independent page uses three fictional devices and four scenarios: laundry completion, lost connection, arrival, and connection recovery. **Simulate event** advances a virtual clock, changes a virtual device state and evaluates enabled preview rules. Matched rules can create an in-page simulated notification, expand/highlight a virtual card, or show a proposed welcome-home action. No physical-action execution control exists.

Preview rules can be toggled before an event. Disabled rules produce no effects; repeating an unchanged state suppresses duplicate effects even if a rule was subsequently enabled. Event history preserves the original rule decisions. Selecting an old event changes the displayed review, not current virtual state. Reset clears virtual state/history while retaining the current rule choices; leaving the page discards the entire session. History is bounded to 20 events and 12 simulated notifications.

The simulation model has no network or hardware adapters, and the playground does not import the live gateway, cameras, or assistant. Virtual events are never written into the real registry or real notifications. September 9 validation: 91 tests passed, plus TypeScript, lint and production build. Browser checks covered edit-mode controls, resizing, all four scenarios, duplicate suppression, disabled rules, historical decisions, reset/navigation and mobile layouts. A before/after digest of real devices, observations, events, notifications and rules was unchanged. No camera was activated or robot command sent.

## Saved workspaces and home-aware chat

The canvas now has **Overview**, **Cameras** and **Workshop** workspaces. Each saves its own widget selection, order and sizes in this browser. Your previous canvas migrates to Overview. **Manage workspace** can copy the current layout into a named workspace, rename a workspace, or delete a custom one (up to eight total). **Reset layout** affects only the selected workspace. Switching cancels pending chat, clears the current conversation and unmounts camera previews; cameras do not start automatically in the destination workspace.

Home-aware questions work without an AI key: try **What's offline?**, **Show my cameras**, **What's in the kitchen?**, **How is OctoPi?**, or **Recent activity**. The gateway supplies the registry snapshot itself; browser-provided device facts are ignored. Answers include snapshot time, observation time/source where available, and monitoring state. A recent failed TCP check is described as a port failure; missing observations and observations older than 90 seconds are kept separate. A reachable port does not prove that a device is operating normally. Asking does not initiate monitoring, camera capture, or device commands.

Related widgets appear and temporarily expand; **Return to my layout** restores their saved sizes and clears the device filter. Revealed widgets remain on that workspace until removed. Camera answers expose the available PC/gripper previews, with Ring still listed as unintegrated. Home answers are deterministic and stay local even with optional AI enabled; their structured records and associated question/answer pairs are excluded from later AI conversation history.

Home-aware chat/workspace validation: 86 dashboard tests pass, plus TypeScript, lint and the production build. New tests cover stale/missing/future observations, changed endpoints, server-owned registry authority, local-only home answers, and independent workspace persistence. Browser checks verified offline/camera/room/activity questions, paused preview defaults, workspace switching, independent resize persistence, custom copy/rename/delete, and mobile layouts. The original canvas migrated to Overview. No hardware control, camera activation or paid AI request was performed.

Open `/canvas` using **Chat & canvas** on the overview, or the link above the concept gallery. **Add widget** opens a library containing Ask Homebase, Weather, Home status, Recent activity, PC webcam and Gripper camera. Add each once, remove it anytime, or start from an empty canvas. The original three-card layout remains compatible. All cards have draggable corner handles, keyboard arrow resizing, width/height presets and earlier/later controls. Cards snap to the grid; narrow screens stack them. Layout and name/unit preferences save in this browser. Chat history clears on reload or workspace switch.

Camera widgets start stopped and use the existing opt-in previews; adding one does not activate a camera. The gripper card reads its address from the home registry. Recent activity shows real gateway events. A short Weather card shows the selected day's essentials and **Show full forecast**; chat can restore a removed Weather card and bring it into focus.

Ask **What's the weather tomorrow?** to retrieve a Pittsburgh forecast and temporarily expand/highlight its widget. **Return to my layout** restores the saved size; resizing the focused card makes that size a manual choice. Chat results include a forecast card that can reopen the associated result. Today, tomorrow and the day after tomorrow are supported. The built-in forecast response uses data and simple conditional advice; it is not LLM-generated or a general web search.

The service requests city-level weather from [Open-Meteo](https://open-meteo.com/en/docs), validates dates/units/missing values, and caches results for up to 15 minutes. Cards show retrieval time and forecast dates. No exact street address is sent. The home-status widget reads the existing registry; chat has no device-action tools.

Optional OpenAI conversation uses the [Responses API](https://developers.openai.com/api/reference/typescript/resources/beta/subresources/responses/methods/create). Set these in an ignored `apps/homebase/.env.local`, then restart the Node service:

```dotenv
OPENAI_API_KEY=your-api-key
OPENAI_MODEL=your-available-model-id
```

After configuration, turn on **Use AI** in chat. For general conversation and weather only, this sends the question, up to eight eligible recent messages, first name, home city and any retrieved forecast to OpenAI. Home-registry answers and their associated questions are excluded from that history. Requests use `store: false`, no tools, an output bound and a timeout. A failed AI call falls back to a clearly labeled built-in answer. Provider credentials never go to the browser. General AI replies have no live web search and must not be treated as verified current information. AI integration is tested with mocked responses; a live paid API call has not been made. The live weather provider has been checked.

For Docker, pass environment variables at runtime or add a local Compose override with `env_file: .env.local`. The image excludes `.env*`; never bake keys into it. Optional home-weather environment settings are `HOMEBASE_WEATHER_CITY`, `HOMEBASE_WEATHER_LATITUDE`, `HOMEBASE_WEATHER_LONGITUDE`, and `HOMEBASE_WEATHER_TIMEZONE`; defaults are Pittsburgh city coordinates and `America/New_York`.

The canvas uses `/api/assistant` (GET configuration availability / POST chat) and `/api/weather` (POST forecast), behind the existing same-origin/loopback checks. Tests cover home-timezone rollover, incomplete provider values, cache coalescing, optional AI boundaries, corrupt or out-of-range layouts, empty/subset layouts and duplicate widget prevention.

September 8 canvas validation: 82 dashboard tests, TypeScript, lint and the production build passed. Interactive browser checks covered real forecast answers and focus/restore, adding/removing widgets, paused camera defaults, reload persistence including an empty canvas, recovery from an empty canvas, pointer and keyboard resizing, and narrow-screen canvas/library layouts. A brief local service outage produced a forecast error; retry recovered after service restart. No camera was started and no robot command was sent during these checks. Live AI and camera streaming were not exercised in this pass.

## Design concepts

Open `/concepts` (also linked from the home page) to compare five interactive presentation studies:

- **Violet:** a close refinement of the existing purple device dashboard.
- **Daylight:** bright room navigation and everyday household actions.
- **Relay:** a dense operations console with an attention filter and event history.
- **Atlas:** an illustrative floor plan with a room inspector.
- **Ion:** a cyan and electric-blue HUD on near-black navy, with Simple / Detailed views, a responsive shade dial, presets, and optional slow ring motion. It respects reduced-motion preferences. Expand its design references for source links and palette hex values. Ion is initially selected.

Ion also includes pointer-following light, button hover/press/sheen feedback, keyboard focus halos, responsive panel edges, collapsible connections/routine/timing/activity panels, room and detail-view reveals, slider and toggle feedback, a focused shade workspace, a scene-preview dialog, selected-node emphasis, alert/result reveals and palette hover feedback. The scene dialog applies a 20% shade position to sample state only. Motion off and reduced-motion preferences cover these effects and the portaled dialog. Collapse panels using their headers; restore the focused workspace with **Show all panels**.

Room selection, shade percentage, routine state and notification dismissal use in-memory sample data shared across concepts. Reloading resets the study. These controls never import the gateway, request a camera, save settings or dispatch device commands. The floor plan is not a measured model of the house. The studies are comparison prototypes; they do not apply a theme to the working dashboard.

The test suite server-renders all five layouts and checks their controls in two sample states, plus Ion's simple and detailed views. TypeScript, lint and the production build are also checked; browser interaction and visual QA have not been performed.

## Architecture

- `app/page.tsx`: home workspace and shared selection across views.
- `app/globals.css`: visual tokens, layout and responsive states.
- `lib/home.ts`: typed inventory, reported relationships, configuration validation, preview templates and future read-only adapter contract.
- `lib/settings-store.ts`: shared persistent registry with a browser-only fallback for the legacy frontend.
- `scripts/gateway.mjs`: SQLite storage, bounded TCP checks, transition notifications, simulation and local API protections.
- `lib/gateway.ts`: typed frontend API and shared refresh store.
- `app/systems/`: persistent home operations workspace.
- `lib/agent-tools.ts`: optional, feature-detected WebMCP inventory read and task-preview tools. No execution tool exists.
- `scripts/serve.mjs`: full local service and 30-second monitor scheduler.

Next, add verified state adapters and account authorization behind this gateway, plus explicit capabilities for permitted actions. The current automation action is a local notification only; no email, vendor requests, LLM calls or motor commands are dispatched. Robot control and its limits remain in the separate controller.

Accounts, passwords and API tokens belong in the future gateway, never in browser storage or this repository. The editable endpoint field rejects credentials in URLs, query parameters, fragments and non-HTTP schemes. It is an address placeholder, not an account form.

### Gateway behavior and validation

`GET /api/home` returns current configuration, timestamped observations, latest 100 events and notifications. `PUT /api/home` validates and saves settings with a revision check to reject stale edits. Explicit `POST /api/check` performs a TCP connection to the saved endpoint without sending HTTP bytes. Monitoring accepts private LAN IPv4 addresses only and does not scan address ranges. Saving an enabled monitor opts into background checks. An endpoint change invalidates its old observation, and delayed checks cannot overwrite changed endpoints.

Rules match first observations or subsequent state transitions, never every unchanged poll. Only local notification records are produced. `POST /api/rule-test` evaluates a synthetic state without persisting a device observation or notification. The service retains 2,000 activity events and 500 notifications; the UI shows the latest 100 of each. Observations older than 90 seconds are labeled stale. Backups contain settings only; imports have a review step and disable monitoring/rules. Device endpoint credentials are rejected. Core inventory IDs are retained for compatibility with existing camera and arm views.

Gateway tests cover persistence across restart, conflicting revisions, validation, private-address restrictions, transition deduplication, simulation isolation, pending-check races and cross-site/DNS-rebinding rejection. The Systems component is rendered with seeded data in the local test suite. No actual device probes or physical commands were used in these tests.

## Verification

`npm test` covers configuration round trips, invalid/credential-bearing endpoints, malformed persisted settings and honest task-preview unknowns. `npm run lint` checks application-owned sources, excluding unchanged starter UI primitives with upstream lint findings. `npx tsc --noEmit` checks the full TypeScript project. The static export is generated with `npm run build` under Node 24.20.0.

Browser interaction/visual QA was not performed. No supported WebMCP validation context was available; optional tool registration is not claimed as verified. A production HTTP smoke check verifies the generated root/assets and that source paths and write methods are not served. The dependency audit reported zero known vulnerabilities after the starter was updated on September 6, 2026; this is a dated dependency snapshot.

### Gripper camera preview

Select Gripper camera, save its base HTTP address in Configure, and press Start preview in Overview. Homebase requests uncached JPEG snapshots directly from `/capture`, at up to 1, 3 or 5 fps, with one request in flight and a four-second frame timeout. It shows the timestamp of the last successfully decoded frame, stops on failure and pauses when hidden/unmounted. It does not record or proxy images. Browser local-network permissions or HTTPS mixed-content rules can block access; this dashboard is intended for local HTTP use. The separate port-81 MJPEG link opens the camera stream directly.

Firmware and user-managed flashing instructions: [ESP32 camera README](../../firmware/esp32-camera/README.md). The AI Thinker reference build compiles; the actual board model, Wi-Fi configuration and live feed remain unverified. No ESP32 was flashed during development. Dashboard browser interaction QA was not performed; TypeScript, lint, URL tests and production compilation were checked.

The gripper preview also offers an **Image settings** panel. It embeds the camera's `/settings` page on demand, so all sensor-control requests stay on the camera's origin without a cloud or dashboard proxy. Requires Gripper Camera firmware 1.1.0, which the user must upload manually. Firmware 1.0.0 will continue streaming normally but returns a missing page for these new controls. Close/reopen or refresh the panel after uploading. Sensor adjustments persist until camera restart.

### PC webcam

Choose **Devices → PC webcam → Start webcam**, then allow the browser's camera permission. The camera dropdown lists available video inputs; device labels may appear only after the first permission grant. Stop before changing camera or preferred resolution. The preview reports the actual negotiated dimensions/frame rate and has a display-only mirror toggle.

This uses the webcam attached to the device running the browser, independently of the ESP32 gripper camera. It requests video only (`audio: false`), does not record or upload, and releases its tracks on Stop, cancellation, unmount, page exit or tab hiding. A permission response arriving after cancellation is discarded and its tracks are stopped. Camera choices are session-local; device name/room still use the existing browser settings store.

Requires browser `getUserMedia` support and a secure context; the local `http://127.0.0.1:5173` origin is suitable in supporting browsers. If the embedded browser does not expose camera access, open the same dashboard in Chrome or Edge on this PC. A remote HTTP LAN URL may require HTTPS. See [browser camera access](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia).

Validation covers capture-session cleanup and races with fake media streams, video-only constraints, TypeScript, lint and the production build. The user subsequently confirmed the webcam works. No webcam was activated by the assistant.

### Mini-Arm calibration workspace

Open **Devices → Mini-Arm → Open calibration workspace**, or `/arm` directly. The workspace includes the PC webcam and gripper camera, each started explicitly by the user.

The USB check uses [Web Serial](https://developer.chrome.com/docs/capabilities/serial). Choose the Pico 2 W port (previously COM3) in the browser permission picker. Close other serial monitors first. Use the first check with servo power disconnected. The code filters CircuitPython USB VID/PID `239A:8162`; these identify a device class, not the individual board's serial number. The user must choose the correct port. Firmware 1.4.10 or newer must already be running; the page does not enter the REPL, reset or flash the board.

The only commands are `test` and `info`, bracketed by unique `sync` markers. A 3.5-second response deadline begins after opening the port; opening/picker duration is browser controlled. Parsing is bounded to 32 KB and requires the expected physical-controller response. There are no automatic retries. Success, timeout, cancellation and disconnect all release stream locks and close the port. A canceled picker or delayed open is checked before writing. Tab hiding/page exit cancels the check. Every result is labeled as a timestamped snapshot; neither model angles nor servo command values are measured positions. The existing report omits gripper output state.

The worksheet preserves the recovered seven channel numbers as unverified wiring references. Measured neutral, travel and direction start blank. Inputs use servo command degrees, with unique channels 0–15 and consistent bounds within the servo API's 0–180° range. That API range is not a measured mechanical limit. Wiring confirmations are manual observations and never authorize motion. Save persists only to `homebase.miniarm-calibration.v1` in this browser; Export/Import JSON transfers the worksheet to another browser/PC. Import loads a reviewable draft before Save replaces stored data. Updates from another tab refresh the editor, so use one tab for editing.

The worksheet never applies calibration to firmware. The separate base-test panel described below can send an explicit, bounded servo pulse. Physical mapping, measured mechanical travel and pose alignment remain outstanding; neither a completed worksheet nor a pulse establishes safe coordinated motion.

Validation: 22 dashboard tests pass, covering worksheet validation/import/export and USB framing, cancellation, timeout, wrong-device selection, disconnect, bounded output and resource cleanup with simulated streams, alongside existing camera/configuration tests. TypeScript, lint and the production static build pass. Browser interaction and this new browser USB path still need the user's hardware check. The local server resolves exported routes such as `/arm` without requiring `.html`; the Docker Nginx configuration includes equivalent extension lookup, but Docker has not been built or run.

### Base servo desk test (September 7 update)

The user confirmed that the browser USB check works and asked to use the recovered wiring as a working assumption for desk testing. The worksheet's wiring checkboxes remain manual records; assumptions have not been marked as measured verification.

With servo power disconnected, choose **Connect base test**. It probes for firmware protocol `miniarm-base-pulse-v1` without moving anything and reads all seven output states, including the gripper. The base must be assigned to channel 15. Close other USB serial tools while testing. Support the arm against tipping and keep a physical servo-power cutoff accessible.

Enter the first servo target explicitly. The recovered 90° reference is unverified and is not preselected. A first pulse can move abruptly from an unknown physical angle. Connect servo power only for the supervised test, then click **Pulse base** once. Each pulse requests that absolute angle for 400 ms before the Pico attempts PWM release. A pulse does not establish that the servo reached the angle. Re-energizing an unpowered joint can move it again if it drifted. Subsequent target changes are at most 1° per request and remain within ±5° of the first target; each requires another explicit click. A one-second cooldown prevents immediate repeats. There is no continuous hold, sweep, automatic retry or motion on connection. The other six outputs must already be off.

The Pico enforces pulse duration, finite API limits, the initial test window, previous-target matching, duplicate-token rejection and cooldown. It rejects tests while a trajectory is active. Once a pulse is attempted, it locks legacy commands until a restart with servo power disconnected, because raw calibration does not update the model pose. After the output-release attempt, the structured reply reports actual PWM command registers. Driver errors latch a fault. Software timing and release are not a hardware emergency stop: an I²C fault or controller failure can leave PWM active. Cut physical servo power on abnormal behavior. An interrupted browser check discards its connection and never retries the pulse.

Controller 1.4.11 was installed on the identified Pico 2 W with user-confirmed servo power disconnected. Four files were copied and hash-verified, with originals backed up in ignored `build/pico2w-backups/20260907-012542-base-test/`. A live read-only `cal_probe` returned all seven outputs off, no fault and no previous target. No movement command was sent. The ESP32 camera firmware was not changed.

Validation: 65 Python tests and 26 dashboard tests pass, plus TypeScript, lint and production build. Tests cover deadline release without USB, bounds, stale/duplicate requests, active gripper detection, write/release failures, legacy-command locking, exact browser capability checks and missing release confirmation. Physical pulse behavior and the new browser pulse path are not yet verified. These narrow base tests do not enable the shoulder, elbow, wrist or gripper, and do not calibrate the URDF.

### Supervised hold diagnostic (firmware 1.4.12)

The user subsequently verified that the base moves and briefly holds during the pulse, returns after the pulse, and reports all outputs off afterward. That timing is consistent with movement after holding torque is removed; the cause has not been physically established.

The new **Hold last tested angle · up to 3 s** button repeats only the last acknowledged pulse target. It ignores edits to the next pulse target and does not extend the travel window. Run a pulse after a controller restart before using Hold. **Release now** ends the hold early. The firmware acknowledges activation immediately, leaves the serial parser available for release and enforces a non-renewable three-second deadline in its normal update loop. There are no browser keep-alives. Release carries the hold's unique token so an older request cannot stop a newer active session.

Hiding/leaving the page or pressing Release cancels observation and attempts an owned release with a fresh cancellation signal, including when activation acknowledgment was lost. Actual page destruction can prevent browser cleanup; the Pico's deadline remains independent. The browser confirms all seven PWM outputs are off before reporting completion. A missing release acknowledgment prompts physical power removal and never retries the hold. USB is released between exchanges; the legacy motion lock remains active. Software timing is not a hardware emergency stop.

This tests the base only. The user's supply-capacity assumption is recorded without claiming an all-servo load test. [Recovered joint limits and conflicting sources](../../docs/smart-home/recovered-joint-limits.md) are shown as worksheet references; no position bounds were automatically applied. In particular, the saved wrist-bend mapping needs reconciliation.

Validation: 70 Python tests and 31 dashboard tests pass, covering same-target enforcement, independent timeout, early release, ownership, non-extension, release faults, cancellation during activation, legacy compatibility and the browser's bounded observation interval. TypeScript, lint and the production build pass. Actual hold behavior has not been tested on hardware.

Firmware 1.4.12 was installed on September 7 after the user explicitly confirmed servo power was disconnected. The installer verified the Pico's USB identity and CIRCUITPY UID, backed up the four existing files to ignored `build/pico2w-backups/20260907-093625-hold-test/`, disabled autoreload during replacement, and verified copied file hashes. After the soft restart, a read-only `cal_probe` reported `miniarm-base-hold-v1`, `hold_ms: 3000`, all seven PWM outputs off, no fault, no active session and no previous target. COM3 was closed afterward. No movement command was sent. Log: ignored `build/pico2w-hold-test-update.log`. Reconnect the base test and run one supervised pulse at the previously tested angle before using Hold, since restart clears the last-target state.

### Shoulder and base selection (firmware 1.4.13)

The user verified base hold/release on hardware, then requested the next joint. The desk-test panel now selects **Shoulder · channel 14** or **Base · channel 15**, with Shoulder selected initially. Switching clears browser connection/target state and requires a new read-only probe. Switching is disabled during activation, pulse, hold, release and cooldown. Each joint keeps independent first/last target history on the Pico; changing the selector does not erase firmware limits or transfer a base target to the shoulder.

Connect the shoulder with servo power disconnected. Support the forearm so it cannot fall when the shoulder releases, keeping supports clear of the moving joint. Other servos remain unpowered. Explicitly choose the first servo target; 90° is only the unverified recovered reference. For a supervised powered test, use one 400 ms pulse before the hold becomes available. Changes remain limited to 1° per pulse within ±5° of the first target. Holds repeat only that joint's last target for up to three seconds, with early Release. These tests do not measure mechanical travel or establish a model-space neutral.

The controller rejects a new pulse/hold when either calibration session is active or faulted, independently of register readback. All seven outputs must be off before any test begins. Legacy motion is locked after either joint's test. Release and deadlines target only the selected channel; the other joint's hold cannot be released by a command addressed to the wrong joint. A fault in either session prevents both from starting further tests. Wrist bend and other joints remain unavailable.

This shoulder-only preparation was superseded by the installed all-joint update below. Validation: 76 Python tests and 34 dashboard tests pass, along with TypeScript and lint. New checks cover real channel-14 writes, independent targets, global active/fault interlocks, shoulder release/timeout, wrong-joint release, legacy locking, unsupported joints, browser protocol/channel matching and correct output-index checks. Hardware shoulder movement is not yet verified.

### All-joint desk tests

Installed September 7 on the Pico 2 W at COM3. The selector now includes base, shoulder, elbow, wrist rotation, wrist bend, gripper link and gripper. Each selection uses a fresh read-only capability probe, its own explicit first target, 400 ms pulse, and up-to-three-second hold with early release. Switching joints never carries over the previous joint's target. One session at a time is enforced on the Pico, including global fault blocking. No additional firmware revision increment was made.

The installer backed up four Python files to `build/pico2w-backups/20260907-101806-all-joints/`, verified copied hashes and checked all seven joint protocols/channels after restart. Every PWM output read off, with no active session or fault. No movement command was sent. Physical supply disconnection was instructed, not measured by software. COM3 was released. Log: `build/pico2w-all-joints-update.log`.

Validation: 92 Python tests and 41 dashboard tests pass, plus TypeScript, lint and production build. HTTP verification confirms the served arm page contains all seven choices. Tests exercise each channel, request routing, hold output index, release, deadline, independent targets and cross-joint interlocks. Physical testing beyond the previously user-verified base remains pending. These controls do not resolve the wrist model limits or validate combined servo loading.

Refresh `/arm`, select a joint and connect the Pico with servo power off. Choose the first raw target explicitly, support the arm/tool with clearance for movement, then power the servos for one supervised pulse. Check the reported outputs are off afterward; use Hold only after that pulse behaves as expected. Record observations in the worksheet.

### Angle slider

The primary controls are seven always-visible native angle sliders, supporting pointer, touch and arrow keys. One **Connect arm** action selects the Pico and probes all seven joints sequentially through the same selected port, without issuing movement commands. Any fault or incomplete response fails the whole connection; the sliders remain visible with an error instead of disappearing. Saved channel mismatches disable the affected row.

After a restart, each uninitialized slider selects a raw starting angle from 0–180°. Changing its value and releasing the pointer/key sends one 400 ms initialization pulse. Until explicitly adjusted, the displayed thumb is only a reference, the target label is blank, and no angle is sent. Canceling or losing focus during initial selection sends no command. There is no separate starting-angle button. Once initialized, hold and drag to jog; let go to request release. Each drag is limited to three seconds, at 10°/s within the existing ±5° window. The Pico also releases if no target update arrives for 600 ms. One shared operation lock prevents simultaneous joint moves. The browser sends the latest target only, verifies joint/token/sequence acknowledgments and confirms output release. Repeat pulse and hold controls remain under each row's disclosure.

The dashboard is built and locally available. Pico slider code was installed September 7 after the user's go-ahead, with servo power instructed to remain disconnected. Four files were backed up to `build/pico2w-backups/20260907-115306-slider/` and hash-verified after copying. Read-only probes verified slider capabilities on all seven channels, with all outputs off and no active session or fault. COM3 was released and no movement command was sent. Refresh the page, reconnect the Pico and set a starting angle before dragging; the restart clears earlier targets. No physical slider movement has been tested. See the USB protocol document for timing, ownership and fault behavior. Validation before installation: 113 Python tests, 53 dashboard tests, TypeScript, lint and production build passed.

The subsequent all-slider dashboard change requires no new Pico installation. Validation: 56 dashboard tests, TypeScript, lint and production build pass, including sequential whole-arm probes, fault/identity rejection and canceled connections. No physical movement was sent during this UI change.

### Compact motor table and audit

The motor controls now use the NML Hand Exo interface's compact row/column organization: Joint, Jog, Target degrees, Last command degrees, and Status. A shared toolbar contains Connect/Reconnect and Release. Seven slider/input pairs remain visible before connection. References and manual pulse/hold controls are grouped below the table; the calibration worksheet and USB diagnostics remain separate disclosures, with cameras alongside on wide screens. On narrow screens the table scrolls horizontally and cameras follow it.

Typing a target changes only the draft. Go or Enter explicitly commits it: an uninitialized joint receives the existing 400 ms pulse; an initialized joint uses the bounded slider protocol and requests release when the controller acknowledges the target command. It never treats acknowledgment as measured arrival. Blank, nonfinite and out-of-range drafts are rejected without clamping. The global action guard checks before mutating the pending desired target, so a second submission cannot retarget an active numeric move. Matching pointer ownership prevents an unrelated touch from committing an initialization gesture. Release and visibility-loss handling keep the UI in a releasing state until the exchange finishes; cooldown still blocks new motion.

Audit and validation: 64 dashboard tests passed, as did TypeScript, lint and the production build. New tests cover blank-input/zero distinction, clipped controller bounds, numeric Go initialization, arrival acknowledgment and owned release, cancellation, and server rendering of seven labeled slider/input pairs with exactly one top-level Connect and Release. Existing shared-port, fault, timeout, webcam and configuration regressions pass. The layout audit renders the real React component without browser or hardware access; interactive browser behavior and powered motor motion were not exercised. The Pico firmware and the NML_Hand_Exo repository were not modified for this update.

### Jog enable toggle and Home

The next update adds Home and a Jog enabled toggle, defaulting off on connection/reload. Turning it on does not move anything; slider changes then start immediate jogging. Turning it off requests release and prevents more motion. Unchanged values produce no target commands; position-free keep-alives preserve the controller watchdog during a bounded active session. The first explicit slider change now establishes the starting target immediately. Jog sessions still release after three seconds, and hiding the page disables jogging. Numeric drafts still require Go/Enter.

Home uses the six saved model-zero servo references, leaves the gripper unchanged, prevalidates the whole sequence against current test windows and skips unchanged joints. It executes sequentially with release confirmation, not simultaneous pose control. The references are not measured calibration, and released joints may drift. The toolbar and reference disclosure explain this behavior.

Immediate-jog firmware was installed September 7 at 13:10 local time after the user confirmed servo power disconnected. All seven joints passed read-only capability and output-state checks after restart: no faults or active sessions, all PWM outputs off. The four previous files are backed up at `build/pico2w-backups/20260907-131029-immediate-jog/`; installation log: `build/pico2w-immediate-jog-update.log`. No movement commands were sent. Refresh the dashboard and Connect to load the new capabilities; jogging starts disabled. Tests before installation: 122 Python and 69 dashboard tests passed, plus TypeScript, lint and build.

### Graphite presentation

The shared theme now uses graphite surfaces, silver typography and blue interaction accents. `app/presentation.css` styles Homebase and the arm workspace, including the compact motor table, a keyboard-focusable jog switch, a distinct Release button and responsive camera panels. The arm header separates workspace navigation from the test context. Existing event handlers, serial commands, limits and camera behavior are unchanged.

Validation: all 69 dashboard tests, TypeScript, lint and the production build passed. The existing server-rendering audit verifies seven labeled slider/input pairs and the global controls. Interactive browser appearance and powered hardware were not tested during this styling change. The Pico update was subsequently installed as recorded above.

### Connect / Disconnect

The motor toolbar now has one Connect/Disconnect button alongside Home. Disconnect remains available during motion: it immediately disables jogging, cancels the current operation, waits for its release/serial cleanup to settle, then clears the selected port and local targets. The button shows Disconnecting during cleanup, preventing a new connection from racing it. An idle disconnect sends no motion commands. Turning the jog toggle off still requests release without disconnecting. Release errors remain visible after disconnection; disconnecting is not proof of disconnected servo power.

Validation: 70 dashboard tests, TypeScript and lint pass. The new component-handler audit covers idle disconnect, jog and Home cancellation, deferred cleanup, cleared targets and retained release errors using simulated operations. No physical motor commands were issued.

### Follow-up control audit

Revisited the NML Hand Exo motor-row layout and joystick/UDP panels without changing that repository. The arm view now puts control mode, speed limit, test window and automatic release in a compact strip beside the control workflow. The displayed limits are configured command limits, not measured current or velocity; unsupported Dynamixel telemetry/control features were not copied.

Reproduced and fixed rapid slider reversal before React redraw: a 90 → 91 → 90 sequence could incorrectly retain 91 as the pending target because deduplication used a stale rendered draft. The live target now always takes the newest input; the serial wrapper still sends position commands only when that target changes. USB disconnect events now clear an idle selected controller or cancel an active operation and wait for cleanup. Events from unrelated serial devices are ignored.

Validation: 122 Python tests and 70 dashboard tests pass, including the reproduced rapid-input case, selected/unrelated USB disconnects, cancellation during jog/Home and retained release errors. TypeScript and lint pass. SHA-256 checks confirm all four installed Pico files match the tested source. No firmware changes or physical motor commands were made during this audit. Browser interaction and powered motion remain outside this software-only validation.
