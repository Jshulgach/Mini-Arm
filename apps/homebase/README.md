# Homebase

A local smart-home dashboard with adaptive widgets, saved workspaces, device observations, optional AI chat, and a virtual-home playground.

Homebase is maintained independently of Mini-Arm. Robot firmware, ROS packages, CAD, and the Python SDK stay in the sibling Mini-Arm repository. Nothing in this extraction updates or commands hardware.

## Run locally

Use Node **24.20.0 or newer**. From this repository:

```powershell
npm ci
npm run typecheck
npm run lint
npm test
npm run build
npm start
```

Open [Homebase](http://127.0.0.1:5173/canvas). `npm start` serves the built frontend and local API together. `npm run dev` runs the frontend development server only. Set `PORT` for a different port; set `HOMEBASE_DATA_DIR` to use an existing data directory. Keep the same browser origin when migrating to preserve browser layouts.

## Workspaces

- `/canvas`: Overview, Cameras and Workshop, plus custom saved layouts. **Edit canvas** reveals layout controls. Home queries such as “What's offline?”, “Show my cameras” and “Recent activity” use the actual registry.
- `/systems`: Device configuration, opt-in private-LAN TCP checks, observations and notification rules.
- `/playground`: Fictional devices and scenario previews, isolated from real data and hardware.
- `/arm`: Mini-Arm browser integration. Physical calibration and operation remain separate supervised work.
- `/concepts`: Retained UI design studies.

Cameras start paused. Missing checks are distinguished from stale or failed port observations. A reachable port does not prove device identity or normal operation. Home chat cannot operate devices or locate people.

## Optional AI and weather

Weather uses Open-Meteo with Pittsburgh city defaults and a short cache. Optional general AI conversation requires an ignored `.env.local` in this repository:

```dotenv
OPENAI_API_KEY=your-api-key
OPENAI_MODEL=your-available-model-id
```

Restart the service after changing these settings, then enable **Use AI** in chat. Home-registry questions remain local. General/weather AI requests can include the question, eligible conversation history, preferred name and forecast. No camera images or device execution tools are supplied. The configured API is optional; home queries and built-in forecast answers work without it.

Weather overrides: `HOMEBASE_WEATHER_CITY`, `HOMEBASE_WEATHER_LATITUDE`, `HOMEBASE_WEATHER_LONGITUDE`, `HOMEBASE_WEATHER_TIMEZONE`.

## Docker and data

```powershell
docker compose up --build -d
```

Compose binds to `127.0.0.1:5173` and stores the registry in its `homebase-data` volume. Stop another service using that port first. A Docker volume and a local `data/` folder are different stores; migrate data deliberately. Pass optional credentials at runtime, never in the image.

The service is local-only and has no remote-account authentication. Browser layouts live in browser storage; registry and event data live in SQLite under `data/`. Credentials, runtime data, build output and dependencies are ignored by Git. Back up the database with SQLite's backup API or stop the service before copying its files.

## Code and validation

See [architecture](docs/architecture.md) for module responsibilities and data ownership. The shared chat/forecast contracts are validated by both the browser and service. Simulation has no hardware or network adapters.

Tests cover protocol cancellation/release, configuration validation, assistant boundaries, workspace persistence, response contracts, and simulated events. Browser QA covers canvas editing, chat focus, workspace changes, paused previews and playground transitions. Automated checks do not establish physical calibration or hardware safety.

Integration references: [USB protocol](docs/integrations/usb-protocol.md), [recovered joint limits](docs/integrations/recovered-joint-limits.md), and [device integration notes](docs/integrations/device-integration-plan.md). These describe recorded hardware assumptions, not current measured state. See the sibling Mini-Arm repository for firmware and camera flashing instructions.

[Historical implementation notes](docs/history/implementation-notes.md) preserve earlier work and validation records; use this README for current setup.
