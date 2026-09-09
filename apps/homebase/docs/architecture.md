# Homebase architecture

Homebase is the home interface and local service. Mini-Arm owns robot firmware, ROS, the Python SDK and mechanical assets. The Homebase `/arm` page is a client integration, not the firmware owner.

| Boundary           | Source                                                | Responsibility                                               |
| ------------------ | ----------------------------------------------------- | ------------------------------------------------------------ |
| Canvas composition | `app/canvas/page.tsx`                                 | Workspace toolbar, preferences and widget library            |
| Canvas state       | `app/canvas/use-canvas-controller.ts`                 | Persistence, chat requests, cancellation, temporary focus    |
| Widget rendering   | `app/canvas/canvas-widgets.tsx`                       | Chat, forecast, registry, activity and camera content        |
| Widget layout      | `app/canvas/widget-frame.tsx`                         | Resize, reorder, removal and layout presentation             |
| Shared contracts   | `contracts/`                                          | Chat/forecast types, widget IDs, runtime response validation |
| Home runtime       | `scripts/gateway.mjs`, `scripts/serve.mjs`            | HTTP routing, SQLite, registry, observations, monitoring     |
| Assistant          | `scripts/assistant.mjs`, `scripts/home-assistant.mjs` | Forecast retrieval, optional AI, local device answers        |
| Simulation         | `lib/virtual-home.ts`, `app/playground/`              | Fictional events and previews with no device adapters        |
| Integrations       | `lib/arm-*.ts`, camera components                     | Typed robot protocol clients and opt-in camera previews      |

The browser and service both use `contracts/validate-assistant.ts`. Invalid forecast values or unsupported widget instructions are rejected at the response boundary. These contracts describe data and permitted interface focus, not executable device instructions.

The service remains one Node process and the frontend one application. No additional services or repositories are required for these internal boundaries. Canvas controller state remains centralized so workspace switching can cancel pending chat and remount camera previews coherently.

## Data ownership

- `data/`: private SQLite registry, event history and runtime files. Ignored by Git.
- Browser storage: workspace selection/layout and preferences; existing keys are retained during extraction.
- Chat: transient per workspace session. Local home answers are excluded from later AI history.
- Virtual home: page-local simulation, discarded when leaving.
- `.env.local`: optional provider credentials and local settings. Ignored by Git and Docker context.

Keep the same loopback origin to preserve browser layouts when changing the server checkout. Back up SQLite using its backup API or stop the service before copying the database and any sidecar files.

## Deliberate remaining limitations

The registry still starts with this household's inventory and protects those original entries. General-purpose onboarding is separate future work. Live notification rules and virtual scenarios are different models; share their event/rule schema only when a real automation execution layer is implemented. Optional AI has no physical action tools. No general robot orchestration, remote-access authentication, or real-time room occupancy is implemented.

The design gallery is retained as a route. It is not a separate product or repository. The ESP32 camera firmware remains in Mini-Arm until it has an independent release cycle.
