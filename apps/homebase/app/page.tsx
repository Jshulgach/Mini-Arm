'use client';
import { useEffect, useRef, useState } from 'react';
import { flushSync } from 'react-dom';
import Link from 'next/link';
import { CameraPreview } from './camera-preview';
import { WebcamPreview } from './webcam-preview';
import {
  ArrowRight,
  Bot,
  Box,
  Camera,
  Check,
  ChevronRight,
  CircleHelp,
  Cpu,
  Eye,
  House,
  Layers,
  LayoutGrid,
  Link2,
  List,
  Maximize2,
  Network,
  Radio,
  Save,
  Search,
  Settings2,
  ShieldCheck,
  Sparkles,
  WashingMachine,
  Wifi,
  X,
} from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Tabs, TabsList, TabsTrigger, TabsContent } from '@/components/ui/tabs';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { Slider } from '@/components/ui/slider';
import {
  validateConfig,
  previewPlan,
  type Device,
  type Plan,
} from '@/lib/home';
import { useDeviceSettings } from '@/lib/settings-store';
import { registerHomeTools, type HomeModelContext } from '@/lib/agent-tools';

const icons = {
  robot: Bot,
  camera: Camera,
  shade: Radio,
  appliance: WashingMachine,
  printer: Box,
  network: Wifi,
};
type View = 'house' | 'graph' | 'devices' | 'tasks';
const examples = [
  'Return the L70 to its dock',
  'Where is my son?',
  'Grab the toy and put it on the couch',
];
function DeviceIcon({ device, size = 20 }: { device: Device; size?: number }) {
  const Icon = icons[device.kind];
  return <Icon size={size} strokeWidth={1.6} />;
}

export default function Home() {
  const [view, setView] = useState<View>('house');
  const { configs, ready, storageError, saveConfigs, inventory:all, rooms, relations } = useDeviceSettings();
  const [notice, setNotice] = useState('');
  const [selected, setSelected] = useState('vacuum.l70');
  const [query, setQuery] = useState('');
  const [room, setRoom] = useState('All rooms');
  const [tilt, setTilt] = useState(false);
  const [zoom, setZoom] = useState(100);
  const [command, setCommand] = useState('');
  const [plan, setPlan] = useState<Plan | null>(null);
  const [engine, setEngine] = useState('API');
  const inventoryRef = useRef(all);
  useEffect(() => {
    inventoryRef.current = all;
  });
  useEffect(() => {
    const context = (document as Document & { modelContext?: HomeModelContext })
      .modelContext;
    if (!context?.registerTool) return;
    return registerHomeTools(
      context,
      () => inventoryRef.current,
      (text, result) => {
        flushSync(() => {
          setCommand(text);
          setPlan(result);
          setView('tasks');
        });
      },
    );
  }, []);
  const active = all.find((d) => d.id === selected) ?? all[0];
  const filtered = all.filter(
    (d) =>
      (room === 'All rooms' || d.room === room) &&
      `${d.name} ${d.model} ${d.kind}`
        .toLowerCase()
        .includes(query.toLowerCase()),
  );
  async function save(device: Device, values: unknown) {
    const next = { ...configs, [device.id]: validateConfig(values, rooms) };
    await saveConfigs(next);
    setNotice('Device details saved. No connection was made by saving.');
  }
  function makePlan(text: string) {
    setCommand(text);
    setPlan(previewPlan(text));
    setView('tasks');
  }
  return (
    <div className="homebase">
      <header className="topbar">
        <Link href="/" className="brand">
          <span className="brand-mark">
            <House size={22} />
          </span>
          <span>
            homebase<span className="brand-dot">.</span>
          </span>
        </Link>
        <div className="home-label">
          MY HOME <ChevronRight size={14} />
          <span>Control workspace</span>
        </div>
        <span className="local-pill">
          <Link href="/systems">Systems & activity <ArrowRight size={14} /></Link>
          <Link href="/canvas">Chat & canvas <ArrowRight size={14} /></Link>
          <Link href="/concepts">Design concepts <ArrowRight size={14} /></Link>
          <span className="status-dot" /> Local workspace
        </span>
      </header>
      <main>
        {storageError && (
          <p className="form-error" role="alert">
            {storageError}
          </p>
        )}
        <div className="page-heading">
          <div>
            <p className="eyebrow">HOME WORKSPACE</p>
            <h1>Your home.</h1>
            <p className="intro">{all.length} inventory entries · {rooms.filter(r=>r!=='Unassigned').length} rooms identified</p>
          </div>
          <div className="setup-badge">
            <span className="amber-dot" /> Setup in progress{' '}
            <span>Device setup & manual previews</span>
          </div>
        </div>
        <Tabs
          value={view}
          onValueChange={(value) => setView(value as View)}
          className="workspace-tabs"
        >
          <div className="view-toolbar">
            <TabsList className="view-tabs">
              <TabsTrigger value="house">
                <House />
                House
              </TabsTrigger>
              <TabsTrigger value="graph">
                <Network />
                Connections
              </TabsTrigger>
              <TabsTrigger value="devices">
                <LayoutGrid />
                Devices
              </TabsTrigger>
              <TabsTrigger value="tasks">
                <Sparkles />
                Tasks
              </TabsTrigger>
            </TabsList>
            <span className="inventory-note">
              Owner-reported inventory <span>·</span> Live states unknown
            </span>
          </div>
          <div className="workspace">
            <section className="main-surface">
              <TabsContent value="house">
                <div className="panel-heading">
                  <div>
                    <h2>House view</h2>
                    <p>Explore rooms and their devices.</p>
                  </div>
                  <Button
                    variant="outline"
                    onClick={() => setTilt(!tilt)}
                    aria-pressed={tilt}
                  >
                    <Layers size={16} />
                    {tilt ? 'Flat view' : 'Tilt view'}
                  </Button>
                </div>
                <div className="house-canvas">
                  <div className="canvas-label">
                    <span className="tiny-cross">+</span> ROOM SCHEMATIC{' '}
                    <span>01</span>
                  </div>
                  <div className={`room-map ${tilt ? 'tilted' : ''}`}>
                    {rooms
                      .filter((r) => r !== 'Unassigned')
                      .map((r, i) => (
                        <section className={`map-room room-${i}`} key={r}>
                          <div className="room-number">0{i + 1}</div>
                          <h3>{r}</h3>
                          <p>
                            {all.filter((d) => d.room === r).length} inventory
                            items
                          </p>
                          <div className="room-devices">
                            {all
                              .filter((d) => d.room === r)
                              .map((d) => (
                                <button
                                  key={d.id}
                                  className={`map-device ${selected === d.id ? 'selected' : ''}`}
                                  onClick={() => setSelected(d.id)}
                                  aria-pressed={selected === d.id}
                                >
                                  <DeviceIcon device={d} />
                                  <span>{d.name}</span>
                                  <CircleHelp size={14} />
                                </button>
                              ))}
                          </div>
                        </section>
                      ))}
                  </div>
                  <div className="canvas-foot">
                    <span>
                      <span className="legend-square" /> Rooms you’ve identified
                    </span>
                    <span>Illustrative layout · not a measured floor plan</span>
                  </div>
                </div>
                <div className="unassigned">
                  <div className="section-label">
                    <h3>
                      Waiting for a room{' '}
                      <span>
                        {all.filter((d) => d.room === 'Unassigned').length}
                      </span>
                    </h3>
                    <span>Select an item to configure it</span>
                  </div>
                  <div className="device-strip">
                    {all
                      .filter((d) => d.room === 'Unassigned')
                      .map((d) => (
                        <button
                          className={`compact-device ${selected === d.id ? 'selected' : ''}`}
                          onClick={() => setSelected(d.id)}
                          key={d.id}
                          aria-pressed={selected === d.id}
                        >
                          <span className={`device-symbol ${d.kind}`}>
                            <DeviceIcon device={d} />
                          </span>
                          <span>
                            {d.name}
                            <small>
                              {d.kind === 'robot'
                                ? 'Robotics'
                                : d.kind === 'camera'
                                  ? 'Vision'
                                  : d.kind === 'network'
                                    ? 'Infrastructure'
                                    : 'Appliances'}
                            </small>
                          </span>
                          <ChevronRight size={14} />
                        </button>
                      ))}
                  </div>
                </div>
              </TabsContent>
              <TabsContent value="graph">
                <div className="panel-heading">
                  <div>
                    <h2>Connections</h2>
                    <p>Reported relationships. Unknown links stay open.</p>
                  </div>
                  <Button variant="outline" onClick={() => setZoom(100)}>
                    <Maximize2 /> Reset
                  </Button>
                </div>
                <div className="graph-canvas">
                  <div
                    className="graph-inner"
                    style={{ transform: `scale(${zoom / 100})` }}
                  >
                    {relations.map((relation, chainIndex) => (
                      <div
                        className={`graph-chain ${chainIndex ? 'second-chain' : ''}`}
                        key={`${relation.from}-${relation.to}-${chainIndex}`}
                      >
                        {[relation.from, relation.to].map((id, i) => {
                          const d = all.find((item) => item.id === id)!;
                          return (
                            <div className="chain-part" key={id}>
                              <button
                                className={`graph-node ${selected === id ? 'selected' : ''}`}
                                onClick={() => setSelected(id)}
                                aria-pressed={selected === id}
                              >
                                <DeviceIcon device={d} size={24} />
                                <strong>{d.name}</strong>
                                <span>{d.model}</span>
                              </button>
                              {i === 0 && (
                                <span className="edge-label">
                                  <span />
                                  {relation.label}
                                  <ArrowRight size={15} />
                                </span>
                              )}
                            </div>
                          );
                        })}
                      </div>
                    ))}
                  </div>
                  <div className="graph-controls">
                    <span>Zoom</span>
                    <Slider
                      aria-label="Graph zoom"
                      min={70}
                      max={125}
                      step={5}
                      value={[zoom]}
                      onValueChange={(v) =>
                        setZoom(Array.isArray(v) ? v[0] : v)
                      }
                    />
                    <span>{zoom}%</span>
                  </div>
                </div>
                <div className="graph-caption">
                  <Link2 size={18} />
                  <p>
                    Lines show owner-reported relationships. Mounts and device
                    links are unverified. Router, Wi-Fi and switch topology has
                    not been discovered.
                  </p>
                </div>
              </TabsContent>
              <TabsContent value="devices">
                <div className="panel-heading">
                  <div>
                    <h2>Device inventory</h2>
                    <p>{all.length} entries, including one camera group.</p>
                  </div>
                  <List size={20} />
                </div>
                <div className="filters">
                  <div className="search-field">
                    <Search size={17} />
                    <Input
                      aria-label="Search devices"
                      placeholder="Search devices or models…"
                      value={query}
                      onChange={(e) => setQuery(e.target.value)}
                    />
                  </div>
                  <Select
                    value={room}
                    onValueChange={(v) => setRoom(v ?? 'All rooms')}
                  >
                    <SelectTrigger aria-label="Filter by room">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {['All rooms', ...rooms].map((r) => (
                        <SelectItem key={r} value={r}>
                          {r}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
                <div className="inventory-list">
                  {filtered.map((d) => (
                    <button
                      key={d.id}
                      onClick={() => setSelected(d.id)}
                      aria-pressed={selected === d.id}
                      className={`inventory-row ${selected === d.id ? 'selected' : ''}`}
                    >
                      <span className={`device-symbol ${d.kind}`}>
                        <DeviceIcon device={d} />
                      </span>
                      <span className="device-name">
                        {d.name}
                        <small>{d.model}</small>
                      </span>
                      <span className="row-room">{d.room}</span>
                      <span className="unknown-tag">{d.id === "camera.webcam" ? "Browser preview" : "Not connected"}</span>
                      <ChevronRight size={16} />
                    </button>
                  ))}
                  {filtered.length === 0 && (
                    <div className="empty-state">
                      <Search />
                      <h3>No matching devices</h3>
                      <p>Try a different name or room.</p>
                      <Button
                        variant="outline"
                        onClick={() => {
                          setRoom('All rooms');
                          setQuery('');
                        }}
                      >
                        Clear filters
                      </Button>
                    </div>
                  )}
                </div>
              </TabsContent>
              <TabsContent value="tasks">
                <div className="panel-heading">
                  <div>
                    <h2>Task workspace</h2>
                    <p>Explore the steps behind a request.</p>
                  </div>
                  <span className="outline-tag">Plan preview only</span>
                </div>
                <div className="task-workspace">
                  <form
                    onSubmit={(e) => {
                      e.preventDefault();
                      if (command.trim()) makePlan(command);
                    }}
                    className="command-form"
                  >
                    <label htmlFor="command">
                      What would you like your home to do?
                    </label>
                    <div className="command-input">
                      <Sparkles size={21} />
                      <Input
                        id="command"
                        value={command}
                        onChange={(e) => setCommand(e.target.value)}
                        placeholder="Return the L70 to its dock…"
                        maxLength={500}
                      />
                      <Button
                        type="submit"
                        disabled={!command.trim()}
                        aria-label="Preview task plan"
                      >
                        <ArrowRight />
                      </Button>
                    </div>
                  </form>
                  <div className="suggestions">
                    {examples.map((s) => (
                      <button key={s} onClick={() => makePlan(s)}>
                        {s}
                        <ArrowRight size={13} />
                      </button>
                    ))}
                  </div>
                  <div className="engine-row">
                    <Cpu size={17} />
                    <span>Future inference provider</span>
                    <Select
                      value={engine}
                      onValueChange={(v) => setEngine(v ?? 'API')}
                    >
                      <SelectTrigger aria-label="Future inference provider">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="API">API</SelectItem>
                        <SelectItem value="Local">Local</SelectItem>
                      </SelectContent>
                    </Select>
                    <span>Not connected</span>
                  </div>
                  {plan ? (
                    <section className="plan-result" aria-live="polite">
                      <div className="section-label">
                        <h3>{plan.title}</h3>
                        <span>Template preview</span>
                      </div>
                      <ol className="plan-steps">
                        {plan.steps.map((step, i) => (
                          <li key={step}>
                            <span>{String(i + 1).padStart(2, '0')}</span>
                            {step}
                          </li>
                        ))}
                      </ol>
                      <div className="blockers">
                        <ShieldCheck size={20} />
                        <div>
                          <strong>Before this can run</strong>
                          {plan.blockers.map((b) => (
                            <p key={b}>{b}</p>
                          ))}
                          <small>
                            No request was sent to a device or model.
                          </small>
                        </div>
                      </div>
                    </section>
                  ) : (
                    <div className="empty-state task-empty">
                      <Sparkles size={34} />
                      <h3>Start with a request.</h3>
                      <p>
                        A preview shows the proposed steps and missing
                        connections. Nothing will move.
                      </p>
                    </div>
                  )}
                </div>
              </TabsContent>
            </section>
            <aside className="inspector" aria-label="Selected device">
              <div className="inspector-heading">
                <span>DEVICE DETAILS</span>
                <Settings2 size={17} />
              </div>
              <div className={`inspector-icon ${active.kind}`}>
                <DeviceIcon device={active} size={30} />
              </div>
              <h2>{active.name}</h2>
              <p className="model-name">{active.model}</p>
              <span className="unknown-tag">
                <span className="amber-dot" />
                {active.id === "camera.webcam" ? "On-demand preview" : "Not connected"}
              </span>
              <Inspector
                key={`${active.id}-${JSON.stringify(configs[active.id])}`}
                device={active}
                all={all}
                ready={ready}
                rooms={rooms}
                relations={relations}
                save={(values) => save(active, values)}
              />
              <div className="next-step">
                <span className="eyebrow">NEXT CONNECTION STEP</span>
                <p>{active.next}</p>
              </div>
              <div className="observation">
                <Eye size={17} />
                <span>
                  {["camera.gripper", "camera.webcam"].includes(active.id) ? "Camera observations appear in the preview" : "No live observation"}
                  <small>Inventory is not continuously monitored</small>
                </span>
              </div>
            </aside>
          </div>
        </Tabs>
        <section className="bottom-band">
          <div>
            <span className="band-icon">
              <Bot size={24} />
            </span>
            <div>
              <h3>Build your connected home</h3>
              <p>
                Configure devices, see connection history, and build local notification rules.
              </p>
            </div>
          </div>
          <Link className="arm-workspace-link" href="/systems">Open Systems <ArrowRight size={16} /></Link>
        </section>
        <footer>
          <span>
            <ShieldCheck size={14} /> Local prototype · controls are not
            connected
          </span>
          <span>
            Built around your home <span className="brand-dot">✦</span>
          </span>
        </footer>
        {notice && (
          <output className="save-notice">
            <Check size={18} />
            <span>{notice}</span>
            <Button
              variant="ghost"
              size="icon"
              aria-label="Dismiss notification"
              onClick={() => setNotice('')}
            >
              <X />
            </Button>
          </output>
        )}
      </main>
    </div>
  );
}

function Inspector({
  device,
  all,
  ready,
  rooms,
  relations,
  save,
}: {
  device: Device;
  all: Device[];
  ready: boolean;
  rooms: string[];
  relations: {from:string;to:string;label:string}[];
  save: (values: unknown) => Promise<void>;
}) {
  const [name, setName] = useState(device.name);
  const [room, setRoom] = useState(device.room);
  const [endpoint, setEndpoint] = useState(device.endpoint);
  const [error, setError] = useState('');
  return (
    <Tabs defaultValue="about" className="inspector-tabs">
      <TabsList variant="line">
        <TabsTrigger value="about">Overview</TabsTrigger>
        <TabsTrigger value="configure">Configure</TabsTrigger>
      </TabsList>
      <TabsContent value="about">
        <dl className="device-facts">
          <div>
            <dt>Room</dt>
            <dd>{device.room}</dd>
          </div>
          <div>
            <dt>State</dt>
            <dd>{device.id === "camera.webcam" ? "Shown in preview" : "Unknown"}</dd>
          </div>
          <div>
            <dt>Last seen</dt>
            <dd>{device.id === "camera.webcam" ? "Session only" : "No observations"}</dd>
          </div>
          <div>
            <dt>Integration</dt>
            <dd>{device.adapter}</dd>
          </div>
        </dl>
        <p className="device-notes">{device.notes}</p>
        {device.id === 'robot.miniarm' && <Link className="arm-workspace-link" href="/arm">Open calibration workspace <ArrowRight size={16} /></Link>}
        {device.id === "camera.gripper" && <CameraPreview address={device.endpoint} />}
        {device.id === "camera.webcam" && <WebcamPreview />}
        {relations
          .filter((r) => r.from === device.id || r.to === device.id)
          .map((r) => (
            <div className="relation" key={`${r.from}-${r.to}`}>
              <Link2 size={16} />
              <span>
                {r.from === device.id
                  ? `${r.label} ${all.find((d) => d.id === r.to)?.name}`
                  : `${all.find((d) => d.id === r.from)?.name} ${r.label} this device`}
                <small>Owner reported · unverified</small>
              </span>
            </div>
          ))}
      </TabsContent>
      <TabsContent value="configure">
        <form
          className="config-form"
          onSubmit={async (e) => {
            e.preventDefault();
            try {
              await save({ name, room, endpoint });
              setError('');
            } catch (err) {
              setError(
                err instanceof Error
                  ? err.message
                  : 'Settings could not be saved.',
              );
            }
          }}
        >
          <label htmlFor="device-name">Display name</label>
          <Input
            id="device-name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            maxLength={80}
            required
          />
          <label id="room-label" htmlFor="device-room">Room</label>
          <Select
            value={room}
            onValueChange={(v) => {
              if (v) setRoom(v);
            }}
          >
            <SelectTrigger id="device-room" aria-labelledby="room-label">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {rooms.map((r) => (
                <SelectItem key={r} value={r}>
                  {r}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          {device.id !== "camera.webcam" && <>
          <label htmlFor="endpoint">Endpoint address</label>
          <Input
            id="endpoint"
            value={endpoint}
            onChange={(e) => setEndpoint(e.target.value)}
            placeholder="http://192.168.1.…"
            maxLength={300}
          />
          </>}
          <p className="field-help">
            {device.id === "camera.webcam" ? "Saved only in this browser. Choose your webcam in Overview; no network address is needed." : "Saved only in this browser. The gripper camera address is contacted when you start its preview. Keep account secrets out of these fields."}
          </p>
          {error && (
            <p className="form-error" role="alert">
              {error}
            </p>
          )}
          <Button type="submit" disabled={!ready}>
            <Save size={16} />
            Save device details
          </Button>
        </form>
      </TabsContent>
    </Tabs>
  );
}
