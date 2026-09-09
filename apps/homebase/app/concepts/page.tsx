'use client';
import Link from 'next/link';
import { useState } from 'react';
import {
  Activity,
  ArrowLeft,
  ArrowRight,
  Bot,
  ChevronRight,
  Command,
  House,
  Layers,
  Lightbulb,
  Network,
  Play,
  Radio,
  ShieldCheck,
  Sun,
  Thermometer,
  WashingMachine,
  Wifi,
  Wind,
  X,
} from 'lucide-react';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';
import { Dialog as IonDialog } from '@base-ui/react/dialog';
import './concepts.css';
import './ion.css';

const concepts = [
  {
    id: 'violet',
    name: 'Violet',
    direction: '01 / Familiar, refined',
    description:
      'Glowing controls, clear status, and a compact device workspace. The closest to your current Homebase.',
    palette: ['#110c1b', '#9c63ed', '#e7cfff'],
    fit: 'Best when you want to keep the current identity and make it more cohesive.',
    tradeoff:
      'Device-first organization still asks you to think about individual systems.',
  },
  {
    id: 'daylight',
    name: 'Daylight',
    direction: '02 / Calm & everyday',
    description:
      'A bright room-first interface, generous spacing, and a small set of useful household actions.',
    palette: ['#f2f5f4', '#2f6c61', '#d8e8e1'],
    fit: 'Best for daily use by everyone in the household.',
    tradeoff:
      'Technical detail belongs in a secondary inspector instead of the main view.',
  },
  {
    id: 'relay',
    name: 'Relay',
    direction: '03 / Operations console',
    description:
      'A precise, dense control surface with faults, command state, and event history at the center.',
    palette: ['#121719', '#f7ab53', '#bcc8cb'],
    fit: 'Best for troubleshooting, equipment monitoring, and advanced automation.',
    tradeoff:
      'More information at once; less relaxed as a family-facing home screen.',
  },
  {
    id: 'atlas',
    name: 'Atlas',
    direction: '04 / Spatial home',
    description:
      'The floor plan becomes the navigation. Choose a space, inspect its systems, and adjust it in context.',
    palette: ['#e9eff6', '#273e66', '#6aa8bd'],
    fit: 'Best for your longer-term interactive house and camera-zone vision.',
    tradeoff:
      'Needs a reliable floor plan; we should keep a list view for small screens.',
  },
  {
    id: 'ion',
    name: 'Ion',
    direction: '05 / Electric & alive',
    description:
      'Blue-neon light on deep navy. Switch from a quiet room control to a layered technical workspace.',
    palette: ['#030912', '#52e9ff', '#408bff'],
    fit: 'Best for a cinematic home interface that can expand into a detailed control workspace.',
    tradeoff:
      'Glow and motion need restraint. Essential text stays sharp, and animation can be paused.',
  },
] as const;
type ConceptId = (typeof concepts)[number]['id'];
const rooms = ['Living room', 'Kitchen', 'Utility', 'Office'];
type Demo = {
  room: string;
  setRoom: (r: string) => void;
  shade: number;
  setShade: (n: number) => void;
  routine: boolean;
  setRoutine: (v: boolean) => void;
  alert: boolean;
  dismiss: () => void;
  notice: string;
  action: (s: string) => void;
};
export default function Concepts() {
  const [concept, setConcept] = useState<ConceptId>('ion');
  const [room, setRoom] = useState('Living room');
  const [shade, setShade] = useState(65);
  const [routine, setRoutine] = useState(true);
  const [alert, setAlert] = useState(true);
  const [notice, setNotice] = useState('');
  const current = concepts.find((c) => c.id === concept)!;
  const demo: Demo = {
    room,
    setRoom,
    shade,
    setShade,
    routine,
    setRoutine,
    alert,
    dismiss: () => {
      setAlert(false);
      setNotice('Sample notification dismissed.');
    },
    notice,
    action: setNotice,
  };
  return (
    <main className="concept-gallery">
      <header className="concept-gallery-heading">
        <div>
          <Link href="/" className="concept-back">
            <ArrowLeft size={15} /> Back to Homebase
          </Link>
          <h1>One home. Five directions.</h1>
          <Link className="concept-back" href="/canvas">Try the live chat &amp; widget canvas <ArrowRight size={15}/></Link>
          <p>Choose a concept, select a room, and try its controls.</p>
        </div>
        <span className="concept-sandbox">
          <ShieldCheck size={16} /> Design study · sample data · no device
          commands
        </span>
      </header>
      <Tabs
        value={concept}
        onValueChange={(v) => {
          setConcept(v as ConceptId);
          setNotice('');
        }}
      >
        <TabsList className="concept-picker" aria-label="Design concepts">
          {concepts.map((c) => (
            <TabsTrigger
              key={c.id}
              value={c.id}
              className={`concept-choice choice-${c.id}`}
            >
              <span className="concept-choice-top">
                <span>{c.direction}</span>
                <span className="concept-swatches" aria-hidden="true">
                  {c.palette.map((p) => (
                    <i key={p} style={{ background: p }} />
                  ))}
                </span>
              </span>
              <strong>{c.name}</strong>
              <span className="concept-choice-description">
                {c.description}
              </span>
            </TabsTrigger>
          ))}
        </TabsList>
        {concepts.map((c) => (
          <TabsContent key={c.id} value={c.id}>
            <div className={`concept-stage concept-${c.id}`}>
              <ConceptCanvas id={c.id} demo={demo} />
            </div>
          </TabsContent>
        ))}
      </Tabs>
      <div className="concept-rationale">
        <div>
          <span>WHY THIS DIRECTION</span>
          <p>{current.fit}</p>
        </div>
        <div>
          <span>TRADEOFF</span>
          <p>{current.tradeoff}</p>
        </div>
        <div>
          <span>TRY THE PROTOTYPE</span>
          <p>
            Switch rooms, adjust the sample shade, toggle the routine, or
            dismiss the sample alert. The house layout is illustrative.
          </p>
        </div>
      </div>
    </main>
  );
}
export function ConceptCanvas({ id, demo }: { id: ConceptId; demo: Demo }) {
  return id === 'violet' ? (
    <Violet demo={demo} />
  ) : id === 'daylight' ? (
    <Daylight demo={demo} />
  ) : id === 'relay' ? (
    <Relay demo={demo} />
  ) : id === 'atlas' ? (
    <Atlas demo={demo} />
  ) : (
    <IonConcept demo={demo} />
  );
}

export function IonConcept({
  demo,
  initialDetail = false,
}: {
  demo: Demo;
  initialDetail?: boolean;
}) {
  const [detailed, setDetailed] = useState(initialDetail);
  const [motion, setMotion] = useState(true);
  const [focused, setFocused] = useState(false);
  const [sceneOpen, setSceneOpen] = useState(false);
  const [previewCount, setPreviewCount] = useState(0);
  return (
    <div
      className={`ion-shell ${detailed ? 'ion-detailed' : 'ion-simple'} ${motion ? '' : 'ion-still'} ${focused ? 'ion-focused' : ''}`}
      onPointerMove={(e) => {
        if (!motion || e.pointerType !== 'mouse') return;
        const rect = e.currentTarget.getBoundingClientRect();
        e.currentTarget.style.setProperty(
          '--ion-x',
          `${e.clientX - rect.left}px`,
        );
        e.currentTarget.style.setProperty(
          '--ion-y',
          `${e.clientY - rect.top}px`,
        );
      }}
    >
      <header className="ion-topbar">
        <div className="ion-wordmark">
          <span className="ion-mark">
            <Command size={24} />
          </span>
          <div>
            HOMEBASE <b>ION</b>
            <small>PERSONAL HOME INTERFACE</small>
          </div>
        </div>
        <div className="ion-toolbar">
          <fieldset className="ion-density" aria-label="Interface detail">
            <button aria-pressed={!detailed} onClick={() => setDetailed(false)}>
              Simple
            </button>
            <button aria-pressed={detailed} onClick={() => setDetailed(true)}>
              Detailed
            </button>
          </fieldset>
          <button
            className="ion-motion"
            aria-pressed={motion}
            onClick={() => setMotion(!motion)}
          >
            Motion {motion ? 'on' : 'off'}
          </button>
        </div>
      </header>
      <div className="ion-heading">
        <div>
          <p className="c-kicker">
            MY HOME / {detailed ? 'SYSTEM WORKSPACE' : 'ROOM CONTROL'}
          </p>
          <h2 key={demo.room} className="ion-room-arrival">
            {demo.room}
            <span>.</span>
          </h2>
        </div>
        <span className="ion-demo">
          <i /> SAMPLE DATA
        </span>
      </div>
      <RoomButtons demo={demo} />
      <div className="ion-workspace">
        {detailed && (
          <details className="ion-systems ion-panel ion-fold" open>
            <summary className="ion-panel-title">
              <Network size={16} />
              <h3>Connections</h3>
              <span>DEMO</span>
            </summary>
            <div className="ion-fold-body">
              <div
                className="ion-network"
                aria-label="Illustrative connections from the home gateway to four rooms"
              >
                <div className="ion-hub">
                  <House size={21} />
                  <strong>Home gateway</strong>
                </div>
                <div className="ion-network-branches" aria-hidden="true" />
                {rooms.map((room, i) => (
                  <button
                    key={room}
                    aria-pressed={demo.room === room}
                    onClick={() => demo.setRoom(room)}
                  >
                    <span className="ion-node-dot" />
                    <span>{room}</span>
                    <small>0{i + 1}</small>
                  </button>
                ))}
              </div>
              <div className="ion-system-list">
                <div>
                  <Wifi size={16} />
                  <span>Home router</span>
                  <b>Available</b>
                </div>
                <div>
                  <Bot size={16} />
                  <span>Mini-Arm</span>
                  <b>Disconnected</b>
                </div>
                <div className="ion-warning">
                  <Network size={16} />
                  <span>OctoPi</span>
                  <b>Unreachable</b>
                </div>
              </div>
              <p className="ion-caption">
                Illustrative topology · connection state is simulated.
              </p>
            </div>
          </details>
        )}
        <section
          className="ion-center ion-panel"
          aria-label="Sample room shade control"
        >
          <div className="ion-panel-title">
            <Radio size={16} />
            <h3>Room atmosphere</h3>
            <button
              className="ion-focus-button"
              aria-pressed={focused}
              onClick={() => setFocused(!focused)}
            >
              {focused ? 'Show all panels' : 'Focus shade'}
              <Layers size={14} />
            </button>
          </div>
          <div className="ion-dial">
            <svg viewBox="0 0 320 320" aria-hidden="true">
              <circle className="ion-ring-outer" cx="160" cy="160" r="148" />
              <circle className="ion-ring-ticks" cx="160" cy="160" r="137" />
              <circle className="ion-ring-track" cx="160" cy="160" r="119" />
              <circle
                className="ion-ring-value"
                cx="160"
                cy="160"
                r="119"
                pathLength="100"
                strokeDasharray={`${demo.shade} 100`}
                transform="rotate(-90 160 160)"
              />
              <circle className="ion-ring-inner" cx="160" cy="160" r="100" />
            </svg>
            <div className="ion-dial-label">
              <span>SHADE OPENING</span>
              <strong>
                {demo.shade}
                <small>%</small>
              </strong>
              <span>{demo.room}</span>
            </div>
          </div>
          <Shade demo={demo} />
          <fieldset className="ion-presets" aria-label="Sample shade presets">
            {[0, 50, 100].map((value) => (
              <button
                key={value}
                aria-pressed={demo.shade === value}
                onClick={() => demo.setShade(value)}
              >
                {value === 0 ? 'Close' : value === 100 ? 'Open' : 'Halfway'}
                <span>{value}%</span>
              </button>
            ))}
          </fieldset>
        </section>
        <aside className="ion-actions">
          <details className="ion-panel ion-fold" open>
            <summary className="ion-panel-title">
              <Sun size={16} />
              <h3>Evening routine</h3>
            </summary>
            <div className="ion-fold-body">
              <Routine demo={demo} />
              <button className="c-primary" onClick={() => setSceneOpen(true)}>
                <Play size={16} /> Preview evening <ArrowRight size={16} />
              </button>
            </div>
          </details>
          <div key={String(demo.alert)} className="ion-alert-arrival">
            <Alert demo={demo} />
          </div>
          {detailed ? (
            <details className="ion-panel ion-signal ion-fold" open>
              <summary className="ion-panel-title">
                <Activity size={16} />
                <h3>Connection timing</h3>
              </summary>
              <div className="ion-fold-body">
                <div className="ion-signal-reading">
                  <strong>
                    24 <small>ms</small>
                  </strong>
                  <span>LAST SAMPLE</span>
                </div>
                <p className="sr-only">
                  Simulated connection latency over twelve samples, ranging from
                  18 to 42 milliseconds.
                </p>
                <svg viewBox="0 0 300 78" aria-hidden="true">
                  <path
                    className="ion-chart-grid"
                    d="M0 20H300 M0 40H300 M0 60H300"
                  />
                  <polyline points="0,52 27,44 54,52 81,56 108,42 135,48 162,8 189,40 216,46 243,50 270,44 300,44" />
                </svg>
                <p className="ion-caption">
                  Illustrative trace · no active monitoring
                </p>
              </div>
            </details>
          ) : (
            <div className="ion-hint">
              <Layers size={20} />
              <p>
                Need the whole picture?
                <span>
                  Detailed view adds connections, timing, and recent activity.
                </span>
              </p>
            </div>
          )}
        </aside>
      </div>
      {detailed && (
        <details className="ion-panel ion-events ion-fold" open>
          <summary className="ion-panel-title">
            <Activity size={16} />
            <h3>Recent activity</h3>
            <span>3 SAMPLE EVENTS</span>
          </summary>
          <div className="ion-fold-body">
            <Events />
          </div>
        </details>
      )}
      <div
        key={previewCount}
        className={previewCount ? 'ion-feedback-arrival' : ''}
      >
        <Feedback demo={demo} />
      </div>
      <IonDialog.Root open={sceneOpen} onOpenChange={setSceneOpen}>
        <IonDialog.Portal>
          <IonDialog.Backdrop
            className={`ion-scene-overlay ${motion ? '' : 'ion-no-motion'}`}
          />
          <IonDialog.Popup
            className={`ion-scene-window ${motion ? '' : 'ion-no-motion'}`}
          >
            <div className="ion-scene-header">
              <Sun size={22} />
              <IonDialog.Title>Evening scene</IonDialog.Title>
              <IonDialog.Close aria-label="Close scene preview">
                <X size={20} />
              </IonDialog.Close>
            </div>
            <IonDialog.Description>
              Preview the sequence for {demo.room}. This only changes the sample
              shade in this design study.
            </IonDialog.Description>
            <ol className="ion-scene-steps">
              <li>
                <span>01</span>
                <div>
                  Set the shade to 20%
                  <small>Current sample position: {demo.shade}%</small>
                </div>
              </li>
              <li>
                <span>02</span>
                <div>
                  Keep your routine preference
                  <small>
                    {demo.routine
                      ? 'Evening wind-down enabled'
                      : 'Evening wind-down paused'}
                  </small>
                </div>
              </li>
              <li>
                <span>03</span>
                <div>
                  Show the result
                  <small>No device commands or saved settings</small>
                </div>
              </li>
            </ol>
            <button
              className="ion-scene-apply"
              onClick={() => {
                demo.setShade(20);
                demo.action(
                  `${demo.room}: sample evening scene applied at 20%. No devices were changed.`,
                );
                setPreviewCount((n) => n + 1);
                setSceneOpen(false);
              }}
            >
              Try sample scene <ArrowRight size={16} />
            </button>
          </IonDialog.Popup>
        </IonDialog.Portal>
      </IonDialog.Root>
      <details className="ion-reference">
        <summary>Design references &amp; palette</summary>
        <div className="ion-reference-content">
          <div>
            <p>
              Glass-like layers and focused controls inspired by{' '}
              <a
                href="https://www.experienceperception.com/work/iron-man-2/"
                target="_blank"
                rel="noreferrer"
              >
                Perception’s Iron Man 2 interfaces ↗
              </a>
              ; layered technical displays informed by{' '}
              <a
                href="https://territorystudio.com/project/marvels-avengers-age-of-ultron/"
                target="_blank"
                rel="noreferrer"
              >
                Territory’s Avengers screen graphics ↗
              </a>
              . This is an original Homebase interpretation.
            </p>
            <p>
              Glow stays on edges and active controls. Slow ring motion is
              decorative; it does not indicate live monitoring. Reduced-motion
              preferences are respected.
            </p>
          </div>
          <div className="ion-palette">
            {[
              ['Midnight', '#030912'],
              ['Panel', '#081a29'],
              ['Cyan', '#52e9ff'],
              ['Electric blue', '#408bff'],
              ['Ice text', '#d9f7ff'],
              ['Alert amber', '#ffbf69'],
            ].map(([name, color]) => (
              <span key={color}>
                <i style={{ background: color }} />
                <span>
                  {name}
                  <code>{color}</code>
                </span>
              </span>
            ))}
          </div>
        </div>
      </details>
    </div>
  );
}
function Mark({ name = 'homebase' }: { name?: string }) {
  return (
    <span className="c-brand">
      <House size={21} />
      {name}
      <i />
    </span>
  );
}
function RoomButtons({
  demo,
  vertical = false,
}: {
  demo: Demo;
  vertical?: boolean;
}) {
  return (
    <div
      className={`c-rooms ${vertical ? 'vertical' : ''}`}
      aria-label="Sample rooms"
    >
      {rooms.map((r, i) => (
        <button
          key={r}
          onClick={() => demo.setRoom(r)}
          aria-pressed={demo.room === r}
        >
          <span>{vertical ? String(i + 1).padStart(2, '0') : null}</span>
          {r}
          {vertical && <ChevronRight size={14} />}
        </button>
      ))}
    </div>
  );
}
function Shade({ demo }: { demo: Demo }) {
  return (
    <div className="c-shade">
      <div>
        <span>Blackout shade</span>
        <strong>
          {demo.shade}
          <small>%</small>
        </strong>
      </div>
      <label>
        <span className="sr-only">Sample shade opening</span>
        <input
          type="range"
          min="0"
          max="100"
          value={demo.shade}
          onChange={(e) => demo.setShade(Number(e.target.value))}
        />
      </label>
      <div className="c-range-labels">
        <span>Closed</span>
        <span>Open</span>
      </div>
    </div>
  );
}
function Routine({ demo }: { demo: Demo }) {
  return (
    <label className="c-routine">
      <span>
        <strong>Evening wind-down</strong>
        <small>At sunset → close living room shade</small>
      </span>
      <input
        type="checkbox"
        role="switch"
        aria-checked={demo.routine}
        aria-label="Sample evening routine"
        checked={demo.routine}
        onChange={(e) => demo.setRoutine(e.target.checked)}
      />
    </label>
  );
}
function Alert({ demo }: { demo: Demo }) {
  return demo.alert ? (
    <div className="c-alert">
      <span className="c-alert-icon">
        <Wifi size={18} />
      </span>
      <div>
        <strong>One connection needs attention</strong>
        <p>OctoPi has not reported recently.</p>
      </div>
      <button
        onClick={demo.dismiss}
        aria-label="Dismiss sample connection alert"
      >
        <X size={17} />
      </button>
    </div>
  ) : (
    <div className="c-alert dismissed">
      <ShieldCheck size={17} />
      <span>Sample alert dismissed</span>
    </div>
  );
}
function Feedback({ demo }: { demo: Demo }) {
  return (
    <output className="c-feedback" aria-live="polite">
      {demo.notice || 'Interactive preview · changes stay in this design study'}
    </output>
  );
}
function Events() {
  return (
    <div className="c-events">
      <div>
        <time>18:42</time>
        <span>Living room scene updated</span>
        <small>Scene</small>
      </div>
      <div>
        <time>18:37</time>
        <span>Washer cycle completed</span>
        <small>Appliance</small>
      </div>
      <div>
        <time>18:31</time>
        <span>OctoPi connection interrupted</span>
        <small>Network</small>
      </div>
    </div>
  );
}
function Violet({ demo }: { demo: Demo }) {
  return (
    <>
      <div className="c-topbar">
        <Mark />
        <span className="c-breadcrumb">
          MY HOME <ChevronRight size={13} /> Overview
        </span>
        <span className="c-top-note">PREVIEW / 18:45</span>
      </div>
      <div className="v-content">
        <div className="v-heading">
          <div>
            <p className="c-kicker">YOUR HOME, IN VIEW</p>
            <h2>
              Everything in its place<span>.</span>
            </h2>
            <p>A quieter evening starts here.</p>
          </div>
          <button
            className="c-primary"
            onClick={() =>
              demo.action(
                'Sample evening scene previewed. No devices were changed.',
              )
            }
          >
            <Play size={15} /> Preview evening
          </button>
        </div>
        <div className="v-metrics">
          <div>
            <House />
            <strong>04</strong>
            <span>Spaces</span>
          </div>
          <div>
            <Network />
            <strong>
              11<span>/12</span>
            </strong>
            <span>Systems available</span>
          </div>
          <div>
            <Thermometer />
            <strong>
              21.4<span>°</span>
            </strong>
            <span>Living room</span>
          </div>
          <div>
            <Activity />
            <strong>03</strong>
            <span>Routines enabled</span>
          </div>
        </div>
        <div className="v-workspace">
          <section className="c-panel">
            <div className="c-panel-heading">
              <h3>Your spaces</h3>
              <span>Room controls</span>
            </div>
            <RoomButtons demo={demo} />
            <div className="v-room-detail">
              <div className="v-room-title">
                <span className="v-device-icon">
                  <Radio size={25} />
                </span>
                <div>
                  <p className="c-kicker">{demo.room.toUpperCase()}</p>
                  <h3>Set the atmosphere</h3>
                  <p>Light, shade, and the systems around you.</p>
                </div>
              </div>
              <Shade demo={demo} />
            </div>
            <div className="v-device-table">
              <div>
                <span>
                  <WashingMachine size={17} /> Washer
                </span>
                <strong>Cycle complete</strong>
                <span>Utility</span>
              </div>
              <div>
                <span>
                  <Bot size={17} /> Mini-Arm
                </span>
                <strong>Disconnected</strong>
                <span>Desk</span>
              </div>
              <div>
                <span>
                  <Network size={17} /> Home router
                </span>
                <strong>Available</strong>
                <span>Network</span>
              </div>
            </div>
          </section>
          <aside>
            <section className="c-panel">
              <div className="c-panel-heading">
                <h3>Routines</h3>
                <span>01 selected</span>
              </div>
              <Routine demo={demo} />
            </section>
            <Alert demo={demo} />
            <section className="c-panel">
              <div className="c-panel-heading">
                <h3>Recent activity</h3>
                <Activity size={16} />
              </div>
              <Events />
            </section>
          </aside>
        </div>
        <Feedback demo={demo} />
      </div>
    </>
  );
}
function Daylight({ demo }: { demo: Demo }) {
  return (
    <div className="d-layout">
      <aside className="d-sidebar">
        <Mark />
        <div className="d-nav-heading">YOUR SPACES</div>
        <RoomButtons demo={demo} vertical />
        <div className="d-sidebar-bottom">
          <div className="d-avatar">J</div>
          <div>
            <strong>My home</strong>
            <small>Local workspace</small>
          </div>
          <ShieldCheck size={17} />
        </div>
      </aside>
      <div className="d-main">
        <div className="d-topline">
          <span>Tuesday, September 8</span>
          <span>
            Design preview <Sun size={17} />
          </span>
        </div>
        <div className="d-heading">
          <p className="c-kicker">GOOD EVENING</p>
          <h2>
            Make yourself
            <br />
            at home.
          </h2>
          <p>One place for the little things that keep your home running.</p>
        </div>
        <div className="d-focus-grid">
          <section className="d-room-card">
            <div className="d-room-meta">
              <span>
                <House size={16} />
                {demo.room}
              </span>
              <span>21.4°</span>
            </div>
            <h3>
              A softer
              <br />
              end to the day.
            </h3>
            <p>Your room, just the way you like it.</p>
            <Shade demo={demo} />
          </section>
          <section className="d-quick-card">
            <p className="c-kicker">A LITTLE HELP</p>
            <h3>
              Ready for
              <br />
              the evening?
            </h3>
            <p>Dim the room. Close the shade. Keep the essentials in view.</p>
            <button
              className="c-primary"
              onClick={() =>
                demo.action('Evening scene selected in this preview only.')
              }
            >
              <Sun size={17} /> Try evening scene <ArrowRight size={16} />
            </button>
            <Routine demo={demo} />
          </section>
        </div>
        <div className="d-section-title">
          <h3>Around the house</h3>
          <span>Sample household status</span>
        </div>
        <div className="d-small-cards">
          <article>
            <WashingMachine />
            <span>Laundry</span>
            <strong>All finished.</strong>
            <p>The washer is ready to unload.</p>
          </article>
          <article>
            <Wind />
            <span>Room comfort</span>
            <strong>Just right.</strong>
            <p>21.4° in the living room.</p>
          </article>
          <article>
            <Bot />
            <span>Robot workspace</span>
            <strong>Taking a break.</strong>
            <p>Mini-Arm is disconnected.</p>
          </article>
        </div>
        <Alert demo={demo} />
        <Feedback demo={demo} />
      </div>
    </div>
  );
}
function Relay({ demo }: { demo: Demo }) {
  const [filter, setFilter] = useState('All systems');
  return (
    <>
      <div className="r-topbar">
        <Mark name="HOMEBASE / RELAY" />
        <span className="r-mode">OPERATOR VIEW</span>
        <span>LOCAL / DEMO</span>
      </div>
      <div className="r-layout">
        <aside className="r-rail">
          <Command size={23} />
          <Activity size={23} />
          <Network size={23} />
          <Layers size={23} />
          <span>
            HB
            <br />
            01
          </span>
        </aside>
        <div className="r-main">
          <div className="r-heading">
            <div>
              <p className="c-kicker">SYSTEM OPERATIONS</p>
              <h2>
                House status<span> / 001</span>
              </h2>
            </div>
            <span className="r-attention">
              {demo.alert ? '01 ATTENTION' : '00 ATTENTION'}
            </span>
          </div>
          <div className="r-stats">
            <div>
              <span>AVAILABLE</span>
              <strong>
                11<small>/12</small>
              </strong>
            </div>
            <div>
              <span>EVENTS / HOUR</span>
              <strong>028</strong>
            </div>
            <div>
              <span>AUTOMATION MODE</span>
              <strong>{demo.routine ? 'ARMED' : 'PAUSED'}</strong>
            </div>
            <div>
              <span>DATA SOURCE</span>
              <strong>SIMULATED</strong>
            </div>
          </div>
          <div className="r-controls">
            <label>
              Systems
              <select
                value={filter}
                onChange={(e) => setFilter(e.target.value)}
              >
                <option>All systems</option>
                <option>Needs attention</option>
              </select>
            </label>
            <span>STATE / LAST OBSERVATION / ACTION</span>
          </div>
          <section className="r-table-wrap">
            <table className="r-table">
              <thead>
                <tr>
                  <th>System</th>
                  <th>Area</th>
                  <th>State</th>
                  <th>Age</th>
                  <th>Inspect</th>
                </tr>
              </thead>
              <tbody>
                {(filter === 'All systems'
                  ? [
                      ['01', 'Home router', 'Network', 'AVAILABLE', '2 s'],
                      [
                        '02',
                        'Living room shade',
                        'Living room',
                        `${demo.shade}% OPEN`,
                        '4 s',
                      ],
                      ['03', 'Washer', 'Utility', 'COMPLETE', '8 min'],
                      ['04', 'OctoPi', 'Utility', 'UNREACHABLE', '14 min'],
                      ['05', 'Mini-Arm', 'Desk', 'DISCONNECTED', 'Manual'],
                    ]
                  : [['04', 'OctoPi', 'Utility', 'UNREACHABLE', '14 min']]
                ).map((row) => (
                  <tr
                    key={row[0]}
                    className={row[3] === 'UNREACHABLE' ? 'r-warning' : ''}
                  >
                    <th>
                      <small>{row[0]}</small>
                      {row[1]}
                    </th>
                    <td>{row[2]}</td>
                    <td>
                      <span>{row[3]}</span>
                    </td>
                    <td>{row[4]}</td>
                    <td>
                      <button
                        aria-label={`Inspect sample ${row[1]}`}
                        onClick={() =>
                          demo.action(
                            `${row[1]} selected. This concept would open its connection history and capabilities.`,
                          )
                        }
                      >
                        <ArrowRight size={16} />
                      </button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </section>
          <div className="r-lower">
            <section>
              <div className="r-section-label">
                <span>DIRECT ADJUSTMENT</span>
                <Radio size={17} />
              </div>
              <RoomButtons demo={demo} />
              <Shade demo={demo} />
            </section>
            <section>
              <div className="r-section-label">
                <span>AUTOMATION POLICY</span>
                <ShieldCheck size={17} />
              </div>
              <Routine demo={demo} />
              <Alert demo={demo} />
            </section>
          </div>
          <section className="r-log">
            <div className="r-section-label">
              <span>EVENT STREAM / SAMPLE</span>
              <span>3 RECORDS</span>
            </div>
            <Events />
          </section>
          <Feedback demo={demo} />
        </div>
      </div>
    </>
  );
}
function Atlas({ demo }: { demo: Demo }) {
  return (
    <>
      <div className="a-topbar">
        <Mark />
        <span>
          My home <ChevronRight size={14} /> Ground floor
        </span>
        <span className="a-view-badge">
          <Layers size={15} /> Spatial view
        </span>
      </div>
      <div className="a-heading">
        <div>
          <p className="c-kicker">A PLACE FOR EVERYTHING</p>
          <h2>Your home, connected.</h2>
        </div>
        <span>Select a room to explore its systems.</span>
      </div>
      <div className="a-layout">
        <section className="a-map-panel">
          <div className="a-map-title">
            <span>GROUND FLOOR / CONCEPT PLAN</span>
            <span>N ↑</span>
          </div>
          <div className="a-floorplan" aria-label="Illustrative room selection">
            <button
              className="a-living"
              aria-pressed={demo.room === 'Living room'}
              onClick={() => demo.setRoom('Living room')}
            >
              <span className="a-room-index">01</span>
              <House size={28} />
              <strong>Living room</strong>
              <span>Shade · 21.4°</span>
              <i>{demo.shade}% open</i>
            </button>
            <button
              className="a-kitchen"
              aria-pressed={demo.room === 'Kitchen'}
              onClick={() => demo.setRoom('Kitchen')}
            >
              <span className="a-room-index">02</span>
              <Lightbulb size={25} />
              <strong>Kitchen</strong>
              <span>2 systems</span>
            </button>
            <button
              className="a-office"
              aria-pressed={demo.room === 'Office'}
              onClick={() => demo.setRoom('Office')}
            >
              <span className="a-room-index">03</span>
              <Bot size={25} />
              <strong>Office</strong>
              <span>Robot workspace</span>
            </button>
            <button
              className="a-utility"
              aria-pressed={demo.room === 'Utility'}
              onClick={() => demo.setRoom('Utility')}
            >
              <span className="a-room-index">04</span>
              <WashingMachine size={25} />
              <strong>Utility</strong>
              <span>Laundry · OctoPi</span>
              <i className="a-attention">Connection alert</i>
            </button>
            <div className="a-hall">
              <span>ENTRY & HALL</span>
              <ArrowRight size={22} />
            </div>
          </div>
          <div className="a-map-footer">
            <span>
              <i /> Selected room
            </span>
            <span>Illustrative layout, not your measured house</span>
          </div>
          <div className="a-context-row">
            <div>
              <Thermometer />
              <strong>21.4°</strong>
              <span>Indoor temperature</span>
            </div>
            <div>
              <Network />
              <strong>11 / 12</strong>
              <span>Systems available</span>
            </div>
            <div>
              <Activity />
              <strong>3</strong>
              <span>Routines enabled</span>
            </div>
          </div>
        </section>
        <aside className="a-inspector">
          <p className="c-kicker">SELECTED SPACE</p>
          <h3>{demo.room}</h3>
          <p className="a-inspector-intro">
            Controls and recent observations, right where they belong.
          </p>
          <div className="a-inspector-tabs">
            <span>Controls</span>
            <small>Sample room inspector</small>
          </div>
          <Shade demo={demo} />
          <Routine demo={demo} />
          <button
            className="c-primary"
            onClick={() =>
              demo.action(
                `${demo.room}: sample scene previewed. The room selection would also filter cameras and history.`,
              )
            }
          >
            Preview room scene <ArrowRight size={16} />
          </button>
          <Alert demo={demo} />
          <div className="a-camera-placeholder">
            <House size={24} />
            <span>Room camera</span>
            <small>A real view would appear here after connection.</small>
          </div>
        </aside>
      </div>
      <Feedback demo={demo} />
    </>
  );
}
