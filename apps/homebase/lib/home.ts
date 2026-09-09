export type Room = string;
export type Device = {
  id: string;
  name: string;
  model: string;
  kind: 'robot' | 'camera' | 'shade' | 'appliance' | 'printer' | 'network';
  room: Room;
  adapter: string;
  endpoint: string;
  notes: string;
  next: string;
};
export const rooms: Room[] = ['Living room', 'Kitchen', 'Unassigned'];
export const devices: Device[] = [
  {
    id: 'vacuum.l70',
    name: 'Eufy L70',
    model: 'L70 Hybrid',
    kind: 'robot',
    room: 'Unassigned',
    adapter: 'Eufy adapter · planned',
    endpoint: '',
    notes:
      'App access, location, battery and movement capabilities are not verified.',
    next: 'Confirm the L70 works in the Eufy app and identify its network address.',
  },
  {
    id: 'robot.miniarm',
    name: 'Mini-Arm',
    model: 'Custom robot arm',
    kind: 'robot',
    room: 'Unassigned',
    adapter: 'USB check & calibration worksheet',
    endpoint: '',
    notes:
      'Pico 2 W on COM3: repaired firmware installed and status queries verified on September 6, 2026 with servo power disconnected. Startup clears all seven servo outputs. Physical calibration, camera alignment and the L70 mount remain unverified. This is a recorded check, not a live connection.',
    next: 'Verify joint mapping, neutral positions and travel before powered motion tests.',
  },
  {
    id: 'camera.gripper',
    name: 'Gripper camera',
    model: 'ESP32-CAM · gripper-mounted',
    kind: 'camera',
    room: 'Unassigned',
    adapter: 'JPEG preview / MJPEG stream',
    endpoint: 'http://192.168.1.189/',
    notes:
      'Camera firmware 1.0.0, JPEG snapshots and continuous MJPEG verified on September 7, 2026 at 192.168.1.189. PSRAM detected. The first image was very dark; lighting, focus, orientation and gripper calibration remain to be checked.',
    next: 'Start the preview and check the lens, lighting, focus and orientation. The address remains configurable if DHCP changes it.',
  },
  {
    id: 'camera.webcam',
    name: 'PC webcam',
    model: 'Webcam on this browsing device',
    kind: 'camera',
    room: 'Unassigned',
    adapter: 'Browser webcam preview',
    endpoint: '',
    notes: 'Use a webcam connected to the computer running this browser. Choose a camera and start its local preview; permission is requested by the browser. This is independent of the gripper camera.',
    next: 'Start the webcam preview and allow camera access. Camera selection becomes more descriptive after permission is granted.',
  },
  {
    id: 'shade.living_room',
    name: 'Living room shade',
    model: 'allen + roth · 2318932',
    kind: 'shade',
    room: 'Living room',
    adapter: 'RF bridge · planned',
    endpoint: '',
    notes:
      'Remote FCC ID 2AVVDAC136-06. Documentation identifies 433.92 MHz OOK. Exact bridge compatibility and position feedback are untested.',
    next: 'Verify a compatible RF bridge with this installed shade.',
  },
  {
    id: 'shade.kitchen',
    name: 'Kitchen shade',
    model: 'allen + roth · 2318939',
    kind: 'shade',
    room: 'Kitchen',
    adapter: 'RF bridge · planned',
    endpoint: '',
    notes:
      'Remote FCC ID 2AVVDAC136-06. Documentation identifies 433.92 MHz OOK. Exact bridge compatibility and position feedback are untested.',
    next: 'Verify a compatible RF bridge with this installed shade.',
  },
  {
    id: 'washer.ge',
    name: 'Washer',
    model: 'GE · GFW550SSNB1WW',
    kind: 'appliance',
    room: 'Unassigned',
    adapter: 'SmartHQ adapter · planned',
    endpoint: '',
    notes:
      'Wi-Fi setup label reported. SmartHQ enrollment and model capabilities are unverified.',
    next: 'Confirm enrollment in SmartHQ before configuring account authorization.',
  },
  {
    id: 'dryer.ge',
    name: 'Dryer',
    model: 'GE · GFD55GSSN0WW',
    kind: 'appliance',
    room: 'Unassigned',
    adapter: 'SmartHQ adapter · planned',
    endpoint: '',
    notes:
      'Wi-Fi setup label reported. Cycle state, remaining time and control capabilities are unverified.',
    next: 'Confirm enrollment in SmartHQ before configuring account authorization.',
  },
  {
    id: 'printer.snapmaker',
    name: 'Snapmaker',
    model: 'Snapmaker 2.0',
    kind: 'printer',
    room: 'Unassigned',
    adapter: 'OctoPrint adapter · planned',
    endpoint: '',
    notes:
      'Reported connected through an OctoPi host. Active toolhead, connection and job state are unknown.',
    next: 'Verify the OctoPi host before reading printer state.',
  },
  {
    id: 'host.octopi',
    name: 'OctoPi',
    model: 'Raspberry Pi · model unknown',
    kind: 'network',
    room: 'Unassigned',
    adapter: 'OctoPrint adapter · planned',
    endpoint: 'http://192.168.1.159',
    notes:
      'Address is tentative. An earlier connection attempt timed out; this does not establish that the device is offline.',
    next: 'Confirm the address in the router device list or on the Pi.',
  },
  {
    id: 'router.main',
    name: 'Home router',
    model: 'Verizon CR1000A',
    kind: 'network',
    room: 'Unassigned',
    adapter: 'Network inventory · planned',
    endpoint: 'http://192.168.1.1',
    notes:
      'Gateway address reported by owner. Physical network connections have not been discovered.',
    next: 'Read the connected-device inventory and identify device addresses.',
  },
  {
    id: 'camera.ring_group',
    name: 'Ring cameras',
    model: 'Device group · models needed',
    kind: 'camera',
    room: 'Unassigned',
    adapter: 'Ring adapter · planned',
    endpoint: '',
    notes:
      'A placeholder group, not a count of individual cameras. Models and locations are still needed.',
    next: 'Add each camera model and location to the inventory.',
  },
];
export const relations = [
  { from: 'camera.gripper', to: 'robot.miniarm', label: 'mounted on' },
  { from: 'robot.miniarm', to: 'vacuum.l70', label: 'mounted on' },
  { from: 'host.octopi', to: 'printer.snapmaker', label: 'controls' },
];
export type DeviceConfig = Pick<Device, 'name' | 'room' | 'endpoint'>;
export type Configs = Record<string, DeviceConfig>;
export const storageKey = 'homebase.device-config.v1';
export function validateConfig(input: unknown, allowedRooms: string[] = rooms): DeviceConfig {
  if (!input || typeof input !== 'object')
    throw new Error('Enter a device name and room.');
  const c = input as Record<string, unknown>;
  if (typeof c.name !== 'string' || !c.name.trim() || c.name.trim().length > 80)
    throw new Error('Use a name between 1 and 80 characters.');
  if (typeof c.room !== 'string' || !allowedRooms.includes(c.room)) throw new Error('Choose a listed room.');
  if (typeof c.endpoint !== 'string' || c.endpoint.length > 300)
    throw new Error('Enter a valid endpoint.');
  const endpoint = c.endpoint.trim();
  if (endpoint) {
    let url: URL;
    try {
      url = new URL(endpoint);
    } catch {
      throw new Error('Use a full http:// or https:// address.');
    }
    if (
      !['http:', 'https:'].includes(url.protocol) ||
      url.username ||
      url.password ||
      url.search ||
      url.hash
    )
      throw new Error(
        'Use an HTTP address without passwords, tokens, query parameters or fragments.',
      );
  }
  return { name: c.name.trim(), room: c.room as Room, endpoint };
}
export function parseConfigs(raw: string | null): Configs {
  if (!raw) return {};
  const parsed: unknown = JSON.parse(raw);
  if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed))
    throw new Error('Saved settings could not be read.');
  const result: Configs = {};
  for (const d of devices)
    if (Object.hasOwn(parsed, d.id))
      result[d.id] = validateConfig((parsed as Record<string, unknown>)[d.id]);
  return result;
}
export function withConfigs(config: Configs): Device[] {
  return devices.map((d) => ({ ...d, ...config[d.id] }));
}
export type Plan = { title: string; steps: string[]; blockers: string[] };
export function previewPlan(command: string): Plan {
  const q = command.toLowerCase();
  if (
    /\b(dock|return|charge)\b/.test(q) &&
    /\b(l70|eufy|vacuum|robot)\b/.test(q)
  )
    return {
      title: 'Return L70 to its dock',
      steps: [
        'Read the L70 state and supported actions.',
        'Check the arm is secured within the base footprint.',
        'Request docking through the Eufy adapter.',
        'Wait for a fresh docked-state observation.',
      ],
      blockers: [
        'Eufy adapter is not connected.',
        'Arm mounting and clearance are unverified.',
      ],
    };
  if (/\b(where|find|locate)\b/.test(q) && /\b(son|child)\b/.test(q))
    return {
      title: 'Locate a household member',
      steps: [
        'Collect recent observations from configured cameras.',
        'Match an explicitly enrolled household identity.',
        'Compare room, timestamp and confidence.',
        'Report the last observation and any uncertainty.',
      ],
      blockers: [
        'No camera observations are available.',
        'No household identity is enrolled.',
      ],
    };
  if (/\b(toy|pick|grab)\b/.test(q))
    return {
      title: 'Move a toy to the couch',
      steps: [
        'Locate the toy and destination in calibrated camera views.',
        'Check reachability, grasp and collision-free motion.',
        'Review the motion plan and hardware interlocks.',
        'Execute through the robot controller and verify placement.',
      ],
      blockers: [
        'Robot model and motion controller are not ready.',
        'No calibrated scene, validated navigation or grasp pipeline is connected.',
      ],
    };
  return {
    title: 'No matching preview',
    steps: [],
    blockers: [
      'This first version has three example plans. A language model is not connected yet. Try one of the suggested requests.',
    ],
  };
}
// Future live adapters run behind an authenticated server gateway.
export interface Observation {
  deviceId: string;
  observedAt: string;
  source: string;
  state: Record<string, unknown>;
}
export interface DeviceAdapter {
  id: string;
  observe(deviceId: string, signal: AbortSignal): Promise<Observation>;
  capabilities(deviceId: string): Promise<string[]>;
}
