'use client';
import Link from 'next/link';
import {
  useEffect,
  useRef,
  useState,
  type CSSProperties,
  type ReactNode,
  type SetStateAction,
} from 'react';
import {
  ArrowLeft,
  ArrowRight,
  CloudSun,
  CornerDownLeft,
  Grip,
  House,
  MessageCircle,
  MoveLeft,
  MoveRight,
  RotateCcw,
  Send,
  Settings2,
  Sparkles,
  Wind,
  Activity,
  Camera,
  Plus,
  X,
  CloudRain,
  CloudSnow,
  CloudLightning,
  Sun,
} from 'lucide-react';
import { Dialog as LibraryDialog } from '@base-ui/react/dialog';
import { WebcamPreview } from '../webcam-preview';
import { CameraPreview } from '../camera-preview';
import { useGateway, healthLabel } from '@/lib/gateway';
import {
  defaultLayout,
  resizeWidget,
  widgetCatalog,
  widgetIds,
  addWidget,
  type WidgetId,
  type WidgetLayout,
} from '@/lib/canvas-layout';
import {
  defaultWorkspaces,
  parseWorkspaces,
  updateWorkspaceLayout,
  workspaceKey,
  presetIds,
} from '@/lib/canvas-workspaces';
import './canvas.css';
const widgetIcons = {
  chat: MessageCircle,
  weather: CloudSun,
  home: House,
  activity: Activity,
  webcam: Camera,
  gripper: Camera,
};
function ForecastIcon({ code, size = 42 }: { code: number; size?: number }) {
  const Icon =
    code >= 95
      ? CloudLightning
      : (code >= 71 && code <= 77) || (code >= 85 && code <= 86)
        ? CloudSnow
        : code >= 51
          ? CloudRain
          : code === 0
            ? Sun
            : CloudSun;
  return <Icon size={size} />;
}

type Day = {
  date: string;
  label: string;
  code: number;
  condition: string;
  high: number;
  low: number;
  rain: number;
  wind: number;
};
type Forecast = {
  city: string;
  timezone: string;
  units: string;
  windUnits: string;
  fetchedAt: string;
  source: string;
  sourceUrl: string;
  days: Day[];
};
type HomeResult = {
  title: string;
  asOf: string;
  devices: {
    id: string;
    name: string;
    room: string;
    label: string;
    state: string;
    monitored: boolean;
    observedAt: string | null;
    source: string;
    lastResult: string | null;
  }[];
  events: { id: string; at: string; source: string; message: string }[];
  focusWidgets: WidgetId[];
};
type Reply = {
  homeResult?: HomeResult;
  text: string;
  mode: string;
  notice?: string;
  weather?: Forecast;
  day?: number;
  focus?: 'weather';
};
type Message = {
  homeResult?: HomeResult;
  role: 'user' | 'assistant';
  text: string;
  mode?: string;
  weather?: Forecast;
  day?: number;
};
type Preferences = { name: string; units: 'fahrenheit' | 'celsius' };
const layoutKey = 'homebase.canvas.layout.v1',
  preferenceKey = 'homebase.canvas.preferences.v1';
function forecastLabel(item: Day, timezone: string, asOf: string) {
  const today = new Intl.DateTimeFormat('en-CA', {
    timeZone: timezone,
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
  }).format(new Date(asOf));
  const offset =
    (Date.parse(`${item.date}T12:00:00Z`) - Date.parse(`${today}T12:00:00Z`)) /
    86400000;
  return offset === 0
    ? 'Today'
    : offset === 1
      ? 'Tomorrow'
      : offset === 2
        ? 'Day after tomorrow'
        : item.date;
}
async function request<T>(
  path: string,
  body: unknown,
  signal: AbortSignal,
): Promise<T> {
  const response = await fetch(`/api/${path}`, {
    method: body === undefined ? 'GET' : 'POST',
    headers:
      body === undefined ? undefined : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal,
  }).catch((error: unknown) => {
    if (signal.aborted) throw error;
    throw new Error(
      'Homebase is unreachable. Check that the local service is running, then try again.',
    );
  });
  const value = await response.json().catch(() => ({
    error: 'Start the updated Homebase service to use chat and weather.',
  }));
  if (!response.ok)
    throw new Error(
      value &&
        typeof value === 'object' &&
        'error' in value &&
        typeof value.error === 'string'
        ? value.error
        : 'Request failed.',
    );
  return value as T;
}
export default function Canvas() {
  const [workspaces, setWorkspaces] = useState(defaultWorkspaces);
  const activeWorkspace = workspaces.items.find(
    (w) => w.id === workspaces.active,
  )!;
  const layout = activeWorkspace.layout;
  function setLayout(action: SetStateAction<WidgetLayout[]>) {
    setWorkspaces((state) => {
      const current = state.items.find((w) => w.id === state.active)!.layout;
      return updateWorkspaceLayout(
        state,
        typeof action === 'function' ? action(current) : action,
      );
    });
  }
  const [workspaceEditor, setWorkspaceEditor] = useState(false),
    [workspaceName, setWorkspaceName] = useState('');
  const [focusedWidgets, setFocusedWidgets] = useState<WidgetId[]>([]);
  const [focusedDeviceIds, setFocusedDeviceIds] = useState<string[] | null>(
    null,
  );
  const [preferences, setPreferences] = useState<Preferences>({
    name: '',
    units: 'fahrenheit',
  });
  const [editing, setEditing] = useState(false);
  const [ready, setReady] = useState(false),
    [storageError, setStorageError] = useState('');
  const [status, setStatus] = useState<{
    city: string;
    aiReady: boolean;
    model: string | null;
  } | null>(null);
  const [weather, setWeather] = useState<Forecast | null>(null),
    [weatherError, setWeatherError] = useState(''),
    [weatherLoading, setWeatherLoading] = useState(false);
  const [day, setDay] = useState(1),
    [attention, setAttention] = useState(false),
    [input, setInput] = useState(''),
    [messages, setMessages] = useState<Message[]>([]),
    [busy, setBusy] = useState(false),
    [useAI, setUseAI] = useState(false);
  const [refresh, setRefresh] = useState(0),
    [settings, setSettings] = useState(false);
  const [libraryOpen, setLibraryOpen] = useState(false),
    [libraryNotice, setLibraryNotice] = useState('');
  const weatherVisible = layout.some((widget) => widget.id === 'weather');
  const { data: home, error: homeError } = useGateway();
  const board = useRef<HTMLDivElement>(null),
    weatherCard = useRef<HTMLElement>(null),
    transcript = useRef<HTMLDivElement>(null),
    chatController = useRef<AbortController | null>(null);
  const drag = useRef<{
    id: WidgetId;
    x: number;
    y: number;
    original: WidgetLayout;
    gridWidth: number;
  } | null>(null);
  useEffect(() => {
    const lifecycle = new AbortController();
    queueMicrotask(() => {
      if (lifecycle.signal.aborted) return;
      try {
        let legacy: unknown = null;
        try {
          legacy = JSON.parse(localStorage.getItem(layoutKey) || 'null');
        } catch {
          /* An old damaged layout must not discard valid workspaces. */
        }
        let saved: unknown = null;
        try {
          saved = JSON.parse(localStorage.getItem(workspaceKey) || 'null');
        } catch {
          /* Recover the original layout. */
        }
        setWorkspaces(parseWorkspaces(saved, legacy));
        const p = JSON.parse(localStorage.getItem(preferenceKey) || 'null');
        if (
          p &&
          typeof p.name === 'string' &&
          ['fahrenheit', 'celsius'].includes(p.units)
        )
          setPreferences({ name: p.name.slice(0, 40), units: p.units });
      } catch {
        setStorageError(
          'Saved layout could not be read. Using the default layout.',
        );
      }
      setReady(true);
    });
    return () => {
      lifecycle.abort();
      chatController.current?.abort();
    };
  }, []);
  useEffect(() => {
    if (!ready) return;
    try {
      localStorage.setItem(workspaceKey, JSON.stringify(workspaces));
      localStorage.setItem(preferenceKey, JSON.stringify(preferences));
    } catch {
      queueMicrotask(() =>
        setStorageError(
          'Browser storage is unavailable. Changes last for this session only.',
        ),
      );
    }
  }, [workspaces, preferences, ready]);
  useEffect(() => {
    const controller = new AbortController();
    request<typeof status>('assistant', undefined, controller.signal)
      .then(setStatus)
      .catch(() => {});
    return () => controller.abort();
  }, []);
  useEffect(() => {
    if (!ready || !weatherVisible) return;
    const controller = new AbortController();
    queueMicrotask(() => {
      if (controller.signal.aborted) return;
      setWeatherLoading(true);
      setWeatherError('');
      setWeather(null);
    });
    request<Forecast>(
      'weather',
      { units: preferences.units },
      AbortSignal.any([controller.signal, AbortSignal.timeout(15000)]),
    )
      .then((value) => {
        if (!controller.signal.aborted) setWeather(value);
      })
      .catch((error) => {
        if (!controller.signal.aborted) setWeatherError(error.message);
      })
      .finally(() => {
        if (!controller.signal.aborted) setWeatherLoading(false);
      });
    return () => controller.abort();
  }, [ready, preferences.units, refresh, weatherVisible]);
  useEffect(() => {
    if (transcript.current)
      transcript.current.scrollTop = transcript.current.scrollHeight;
  }, [messages, busy]);
  useEffect(() => {
    if (attention) {
      weatherCard.current?.scrollIntoView({
        behavior: matchMedia('(prefers-reduced-motion: reduce)').matches
          ? 'instant'
          : 'smooth',
        block: 'nearest',
      });
    } else {
      // Shrinking a focused forecast must not retain its old scroll offset.
      weatherCard.current?.querySelector('.canvas-widget-body')?.scrollTo(0, 0);
    }
  }, [attention]);
  function leaveWorkspace() {
    chatController.current?.abort();
    chatController.current = null;
    drag.current = null;
    setBusy(false);
    setMessages([]);
    setInput('');
    setAttention(false);
    setFocusedWidgets([]);
    setFocusedDeviceIds(null);
    setLibraryNotice('');
    setLibraryOpen(false);
    setWorkspaceEditor(false);
  }
  function switchWorkspace(id: string) {
    if (id === workspaces.active) return;
    leaveWorkspace();
    setWorkspaces((state) => ({ ...state, active: id }));
  }
  function saveWorkspace(copy: boolean) {
    const name = workspaceName.trim();
    if (!name || (copy && workspaces.items.length >= 8)) return;
    if (copy) {
      const id = `custom-${crypto.randomUUID()}`;
      leaveWorkspace();
      setWorkspaces((state) => ({
        ...state,
        active: id,
        items: [
          ...state.items,
          {
            id,
            name,
            layout: state.items
              .find((w) => w.id === state.active)!
              .layout.map((w) => ({ ...w })),
          },
        ],
      }));
    } else {
      setWorkspaces((state) => ({
        ...state,
        items: state.items.map((w) =>
          w.id === state.active ? { ...w, name } : w,
        ),
      }));
      setWorkspaceEditor(false);
    }
    setWorkspaceName('');
  }
  function revealHome(result: HomeResult) {
    const targets = result.focusWidgets.filter((id) => widgetIds.includes(id));
    setLayout((items) =>
      targets.reduce((next, id) => addWidget(next, id), items),
    );
    setAttention(false);
    setFocusedWidgets(targets);
    setFocusedDeviceIds(
      targets.includes('home') ? result.devices.map((d) => d.id) : null,
    );
    setLibraryNotice(
      'Relevant widgets are in focus. Camera previews stay paused until you start them.',
    );
  }
  useEffect(() => {
    if (focusedWidgets.length)
      board.current
        ?.querySelector(`[data-widget="${focusedWidgets[0]}"]`)
        ?.scrollIntoView({
          block: 'nearest',
          behavior: matchMedia('(prefers-reduced-motion: reduce)').matches
            ? 'instant'
            : 'smooth',
        });
  }, [focusedWidgets]);
  function change(id: WidgetId, patch: Partial<WidgetLayout>) {
    setLayout((items) => {
      const current = items.find((item) => item.id === id)!;
      const next = { ...current, ...patch };
      return next.width === current.width && next.height === current.height
        ? items
        : items.map((item) => (item.id === id ? next : item));
    });
    if (id === 'weather') setAttention(false);
    setFocusedWidgets((ids) => ids.filter((value) => value !== id));
  }
  function move(id: WidgetId, direction: number) {
    setLayout((items) => {
      const next = [...items],
        index = next.findIndex((v) => v.id === id),
        target = index + direction;
      if (target >= 0 && target < next.length)
        [next[index], next[target]] = [next[target], next[index]];
      return next;
    });
  }
  function add(id: WidgetId) {
    setLayout((items) => addWidget(items, id));
    setLibraryNotice(`${widgetCatalog[id].title} added to your canvas.`);
  }
  function remove(id: WidgetId) {
    if (id === 'chat') {
      chatController.current?.abort();
      chatController.current = null;
      setBusy(false);
    }
    if (id === 'weather') setAttention(false);
    setFocusedWidgets((ids) => ids.filter((value) => value !== id));
    setLayout((items) => items.filter((item) => item.id !== id));
    setLibraryNotice(
      `${widgetCatalog[id].title} removed from the canvas. Add it back from the library anytime.`,
    );
  }
  async function ask(question = input) {
    if (busy || !question.trim()) return;
    setInput('');
    setBusy(true);
    const history = messages
      .filter(
        (message, index) =>
          !message.homeResult && !messages[index + 1]?.homeResult,
      )
      .slice(-8)
      .map(({ role, text }) => ({ role, text: text.slice(0, 3000) }));
    setMessages((items) => [
      ...items.slice(-29),
      { role: 'user', text: question },
    ]);
    const controller = new AbortController();
    chatController.current = controller;
    try {
      const result = await request<Reply>(
        'assistant',
        {
          message: question,
          units: preferences.units,
          name: preferences.name,
          useAI,
          history,
          context: messages.at(-1)?.weather ? 'weather' : undefined,
        },
        AbortSignal.any([controller.signal, AbortSignal.timeout(40000)]),
      );
      if (controller.signal.aborted) return;
      setMessages((items) => [
        ...items,
        {
          role: 'assistant',
          text: result.text + (result.notice ? `\n\n${result.notice}` : ''),
          mode: result.mode,
          weather: result.weather,
          day: result.day,
          homeResult: result.homeResult,
        },
      ]);
      if (result.homeResult) revealHome(result.homeResult);
      if (result.focus === 'weather' && result.weather) {
        setFocusedWidgets([]);
        setFocusedDeviceIds(null);
        setLayout((items) => addWidget(items, 'weather'));
        setWeather(result.weather);
        setWeatherError('');
        setDay(result.day ?? 1);
        setAttention(true);
      }
    } catch (error) {
      if (!controller.signal.aborted)
        setMessages((items) => [
          ...items,
          {
            role: 'assistant',
            text:
              error instanceof Error
                ? error.message
                : 'Could not complete the request. Please try again.',
            mode: 'Request failed',
          },
        ]);
    } finally {
      if (!controller.signal.aborted) setBusy(false);
      if (chatController.current === controller) chatController.current = null;
    }
  }
  const selectedDay = weather?.days[day];
  function contents(id: WidgetId): ReactNode {
    if (id === 'webcam')
      return (
        <div className="canvas-camera-preview">
          <WebcamPreview compact />
        </div>
      );
    if (id === 'gripper') {
      const address = home?.devices.find(
        (device) => device.id === 'camera.gripper',
      )?.endpoint;
      return (
        <div className="canvas-camera-preview">
          {address ? (
            <CameraPreview address={address} compact />
          ) : (
            <p>
              Set the gripper camera’s address in{' '}
              <Link href="/systems">Systems</Link> to enable its preview.
            </p>
          )}
        </div>
      );
    }
    if (id === 'activity')
      return (
        <div className="canvas-activity">
          {!home ? (
            <p>{homeError || 'Loading recent activity…'}</p>
          ) : !home.events.length ? (
            <p>No events recorded yet.</p>
          ) : (
            <ol>
              {home.events.slice(0, 12).map((event) => (
                <li key={event.id}>
                  <time dateTime={event.at}>
                    {new Date(event.at).toLocaleString([], {
                      month: 'short',
                      day: 'numeric',
                      hour: 'numeric',
                      minute: '2-digit',
                    })}
                  </time>
                  <p>{event.message}</p>
                  <small>{event.source}</small>
                </li>
              ))}
            </ol>
          )}
          <Link href="/systems">
            All activity <ArrowRight size={15} />
          </Link>
        </div>
      );
    if (id === 'chat')
      return (
        <>
          <div
            className="canvas-chat-history"
            ref={transcript}
            role="log"
            aria-label="Homebase conversation"
            aria-live="polite"
          >
            {!messages.length && (
              <div className="canvas-chat-welcome">
                <Sparkles size={26} />
                <h2>Ask. Bring it into view.</h2>
                <p>
                  Ask about devices, cameras, recent activity, or the forecast.
                  The relevant widgets will come into focus.
                </p>
                <button
                  onClick={() =>
                    void ask('What’s the weather going to be like tomorrow?')
                  }
                >
                  What’s tomorrow looking like? <ArrowRight size={15} />
                </button>
                <button onClick={() => void ask('What’s offline?')}>
                  What’s offline?
                </button>
                <button onClick={() => void ask('Show my cameras')}>
                  Show my cameras <Camera size={15} />
                </button>
              </div>
            )}
            {messages.map((message, index) => (
              <div key={index} className={`canvas-message ${message.role}`}>
                <span>
                  {message.role === 'user' ? 'You' : message.mode || 'Homebase'}
                </span>
                <p>{message.text}</p>
                {message.homeResult && (
                  <div className="canvas-home-answer">
                    <strong>{message.homeResult.title}</strong>
                    <small>
                      Registry snapshot ·{' '}
                      {new Date(message.homeResult.asOf).toLocaleString()}
                    </small>
                    <ul>
                      {message.homeResult.devices.map((device) => (
                        <li key={device.id}>
                          <b>{device.name}</b>
                          <span>
                            {device.room} · {device.label}
                          </span>
                          <small>
                            {device.observedAt
                              ? `${device.lastResult} · ${new Date(device.observedAt).toLocaleString()} · ${device.source}`
                              : 'No matching observation recorded'}
                            {device.monitored
                              ? ' · Monitoring on'
                              : ' · Monitoring off'}
                          </small>
                        </li>
                      ))}
                    </ul>
                    {message.homeResult.events.map((event) => (
                      <p key={event.id}>
                        {event.message}
                        <small>
                          {new Date(event.at).toLocaleString()} · {event.source}
                        </small>
                      </p>
                    ))}
                    <button onClick={() => revealHome(message.homeResult!)}>
                      Show related widgets <ArrowRight size={14} />
                    </button>
                  </div>
                )}
                {message.weather && (
                  <button
                    className="canvas-chat-result"
                    onClick={() => {
                      setLayout((items) => addWidget(items, 'weather'));
                      setWeather(message.weather!);
                      setDay(message.day ?? 1);
                      setAttention(true);
                    }}
                  >
                    <CloudSun size={21} />
                    <span>
                      {message.weather.city} ·{' '}
                      {message.weather.days[message.day ?? 1].label}
                      <strong>
                        {message.weather.days[message.day ?? 1].high}
                        {message.weather.units} /{' '}
                        {message.weather.days[message.day ?? 1].low}
                        {message.weather.units}
                      </strong>
                      <small>
                        {message.weather.days[message.day ?? 1].rain}%
                        precipitation chance · Open-Meteo
                      </small>
                    </span>
                    <ArrowRight size={17} />
                  </button>
                )}
              </div>
            ))}
            {busy && <p className="canvas-thinking">Preparing your answer…</p>}
          </div>
          <form
            className="canvas-chat-form"
            onSubmit={(e) => {
              e.preventDefault();
              void ask();
            }}
          >
            <label className="sr-only" htmlFor="canvas-question">
              Ask Homebase
            </label>
            <textarea
              id="canvas-question"
              maxLength={1200}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask about your home…"
              rows={2}
              onKeyDown={(e) => {
                if (
                  e.key === 'Enter' &&
                  !e.shiftKey &&
                  !e.nativeEvent.isComposing
                ) {
                  e.preventDefault();
                  void ask();
                }
              }}
            />
            <button
              type="submit"
              aria-label="Send question"
              disabled={busy || !input.trim()}
            >
              <Send size={19} />
            </button>
          </form>
          <div className="canvas-chat-footer">
            <label>
              <input
                type="checkbox"
                checked={useAI}
                disabled={!status?.aiReady || busy}
                onChange={(e) => setUseAI(e.target.checked)}
              />{' '}
              Use AI {status?.aiReady ? '' : '· not configured'}
            </label>
            <span>
              {useAI
                ? 'General chat uses OpenAI · device answers stay local'
                : 'History in this workspace session'}
            </span>
          </div>
        </>
      );
    if (id === 'weather')
      return (
        <div className="canvas-weather">
          <div className="canvas-weather-location">
            <span>
              {weather?.city || status?.city || 'Pittsburgh'}{' '}
              {weather && (
                <a
                  className="canvas-compact-source"
                  href="https://open-meteo.com/"
                  target="_blank"
                  rel="noreferrer"
                >
                  Open-Meteo ↗
                </a>
              )}
            </span>
            <button
              disabled={weatherLoading || busy}
              onClick={() => setRefresh((n) => n + 1)}
              aria-label="Refresh forecast"
            >
              <RotateCcw size={15} />
            </button>
          </div>
          {weatherLoading && !weather && (
            <p className="canvas-empty">Loading the home forecast…</p>
          )}
          {weatherError && (
            <div role="alert" className="canvas-empty">
              <p>{weatherError}</p>
              <button onClick={() => setRefresh((n) => n + 1)}>
                Try again
              </button>
            </div>
          )}
          {selectedDay && weather && (
            <>
              <div className="canvas-weather-main">
                <ForecastIcon code={selectedDay.code} />
                <div>
                  <span>
                    {forecastLabel(
                      selectedDay,
                      weather.timezone,
                      home?.serverTime || weather.fetchedAt,
                    )}{' '}
                    · {selectedDay.date}
                  </span>
                  <strong>
                    {selectedDay.high}
                    <small>{weather.units}</small>
                  </strong>
                  <p>{selectedDay.condition}</p>
                </div>
              </div>
              <div className="canvas-weather-stats">
                <span>
                  Low{' '}
                  <b>
                    {selectedDay.low}
                    {weather.units}
                  </b>
                </span>
                <span>
                  Precipitation <b>{selectedDay.rain}%</b>
                </span>
                <span>
                  <Wind size={14} /> Wind <b>{selectedDay.wind} mph</b>
                </span>
              </div>
              <button
                className="canvas-weather-expand"
                onClick={() => setAttention(true)}
              >
                Show full forecast <ArrowRight size={14} />
              </button>
              <div className="canvas-forecast-days" aria-label="Forecast day">
                {weather.days.map((item, i) => (
                  <button
                    key={item.date}
                    aria-pressed={day === i}
                    onClick={() => setDay(i)}
                  >
                    <span>
                      {forecastLabel(
                        item,
                        weather.timezone,
                        home?.serverTime || weather.fetchedAt,
                      )}
                    </span>
                    <strong>
                      {item.high}° <small>{item.low}°</small>
                    </strong>
                    <span>{item.rain}% precip.</span>
                  </button>
                ))}
              </div>
              <p className="canvas-source">
                <a
                  href="https://open-meteo.com/"
                  target="_blank"
                  rel="noreferrer"
                >
                  Forecast: Open-Meteo ↗
                </a>
                <span>
                  {Date.parse(home?.serverTime || weather.fetchedAt) -
                    Date.parse(weather.fetchedAt) >
                  900000
                    ? 'Cached forecast · retrieved '
                    : 'Retrieved '}
                  {new Date(weather.fetchedAt).toLocaleString([], {
                    timeZone: weather.timezone,
                    month: 'short',
                    day: 'numeric',
                    hour: 'numeric',
                    minute: '2-digit',
                  })}{' '}
                  · {weather.timezone}
                </span>
              </p>
            </>
          )}
          {attention && (
            <button
              className="canvas-return"
              onClick={() => setAttention(false)}
            >
              <CornerDownLeft size={15} /> Return to my layout
            </button>
          )}
        </div>
      );
    return (
      <div className="canvas-home-status">
        {!home ? (
          <p>{homeError || 'Loading your home registry…'}</p>
        ) : (
          <>
            <div className="canvas-home-count">
              <strong>{home.devices.length}</strong>
              <span>registered devices</span>
            </div>
            <p>
              {home.rooms.filter((room) => room !== 'Unassigned').length} rooms
              · {home.devices.filter((d) => d.monitor).length} monitored
              endpoints
            </p>
            {homeError && (
              <p role="alert">
                Gateway unavailable. Showing the last registry snapshot.
              </p>
            )}
            {focusedDeviceIds?.length === 0 && (
              <p>No devices matched this answer.</p>
            )}
            <div className="canvas-device-status">
              {home.devices
                .filter(
                  (d) =>
                    focusedDeviceIds === null ||
                    focusedDeviceIds.includes(d.id),
                )
                .map((d) => (
                  <div key={d.id}>
                    <span>{d.name}</span>
                    <small>
                      {healthLabel(d, home.health[d.id], home.serverTime)}
                      {home.health[d.id]?.endpoint === d.endpoint && (
                        <span className="canvas-observed">
                          {new Date(
                            home.health[d.id].observedAt,
                          ).toLocaleString()}{' '}
                          · {home.health[d.id].source}
                        </span>
                      )}
                    </small>
                  </div>
                ))}
            </div>
            <Link href="/systems">
              Open systems &amp; activity <ArrowRight size={15} />
            </Link>
          </>
        )}
      </div>
    );
  }
  return (
    <main
      className={`adaptive-canvas ${editing ? 'canvas-editing' : 'canvas-viewing'}`}
      aria-busy={!ready}
      inert={!ready}
    >
      <header className="canvas-top">
        <Link href="/" className="canvas-brand">
          <House size={24} /> HOMEBASE <span>CANVAS</span>
        </Link>
        <nav>
          <Link href="/playground">Virtual home</Link>
          <Link href="/systems">Systems</Link>
          <Link href="/">
            <ArrowLeft size={14} /> Overview
          </Link>
        </nav>
      </header>
      <div className="canvas-heading">
        <div>
          <p>YOUR HOME, IN CONTEXT</p>
          <h1>
            Your home, at a glance<span>.</span>
          </h1>
        </div>
        <div>
          <button
            aria-pressed={editing}
            onClick={() => {
              setEditing((value) => !value);
              setWorkspaceEditor(false);
              setLibraryOpen(false);
              drag.current = null;
            }}
          >
            <Settings2 size={16} /> {editing ? 'Done editing' : 'Edit canvas'}
          </button>
          {editing && (
            <button
              className="canvas-add-widget"
              onClick={() => {
                setLibraryNotice('');
                setLibraryOpen(true);
              }}
            >
              <Plus size={17} /> Add widget
            </button>
          )}
          <button
            aria-expanded={settings}
            onClick={() => setSettings(!settings)}
          >
            <Settings2 size={16} /> Preferences
          </button>
          {editing && (
            <button
              onClick={() => {
                setLayout(
                  (
                    defaultWorkspaces().items.find(
                      (w) => w.id === workspaces.active,
                    )?.layout ?? defaultLayout
                  ).map((v) => ({ ...v })),
                );
                setFocusedWidgets([]);
                setFocusedDeviceIds(null);
                setAttention(false);
              }}
            >
              <RotateCcw size={16} /> Reset layout
            </button>
          )}
        </div>
      </div>
      <section className="canvas-workspaces" aria-label="Saved workspaces">
        <nav className="canvas-workspace-tabs" aria-label="Choose workspace">
          {workspaces.items.map((workspace) => (
            <button
              key={workspace.id}
              aria-pressed={workspace.id === workspaces.active}
              onClick={() => switchWorkspace(workspace.id)}
            >
              {workspace.name}
            </button>
          ))}
        </nav>
        {editing && (
          <button
            aria-expanded={workspaceEditor}
            onClick={() => {
              setWorkspaceName(activeWorkspace.name);
              setWorkspaceEditor((v) => !v);
            }}
          >
            Manage workspace
          </button>
        )}
        {editing && <small>Saved automatically in this browser</small>}
      </section>
      {editing && workspaceEditor && (
        <form
          className="canvas-workspace-editor"
          onSubmit={(event) => {
            event.preventDefault();
            saveWorkspace(true);
          }}
        >
          <label>
            Workspace name
            <input
              value={workspaceName}
              maxLength={30}
              onChange={(e) => setWorkspaceName(e.target.value)}
            />
          </label>
          <button
            type="submit"
            disabled={!workspaceName.trim() || workspaces.items.length >= 8}
          >
            Save as new workspace
          </button>
          <button
            type="button"
            disabled={!workspaceName.trim()}
            onClick={() => saveWorkspace(false)}
          >
            Rename current
          </button>
          <button
            type="button"
            disabled={presetIds.includes(workspaces.active)}
            onClick={() => {
              const id = workspaces.active;
              leaveWorkspace();
              setWorkspaces((state) => ({
                ...state,
                active: 'overview',
                items: state.items.filter((w) => w.id !== id),
              }));
            }}
          >
            Delete current
          </button>
          <p>
            Up to eight workspaces. Overview, Cameras and Workshop can be
            renamed but cannot be deleted. Switching clears this chat session
            and stops camera previews.
          </p>
        </form>
      )}
      {focusedWidgets.length > 0 && (
        <output className="canvas-focus-bar">
          Showing related widgets{' '}
          <button
            onClick={() => {
              setFocusedWidgets([]);
              setFocusedDeviceIds(null);
            }}
          >
            Return to my layout
          </button>
        </output>
      )}
      {!editing && (
        <section className="canvas-now-summary" aria-label="Home summary">
          <span>
            <House size={16} />{' '}
            {home
              ? `${home.devices.length} registered devices`
              : 'Loading home registry…'}
          </span>
          {home && (
            <span>
              {home.devices.filter((device) => device.monitor).length} monitored
              · {home.devices.filter((device) => !device.monitor).length}{' '}
              monitoring off
            </span>
          )}
          <Link href="/systems">
            {homeError
              ? 'Gateway unavailable · view last snapshot'
              : 'Review home status'}{' '}
            <ArrowRight size={14} />
          </Link>
        </section>
      )}
      {editing && (
        <p className="canvas-instructions">
          Drag a card’s corner to resize, or use its size buttons. Ask about
          your home to bring relevant widgets into focus.
        </p>
      )}
      {settings && (
        <section className="canvas-preferences">
          <label>
            Your first name{' '}
            <input
              maxLength={40}
              value={preferences.name}
              onChange={(e) =>
                setPreferences((p) => ({ ...p, name: e.target.value }))
              }
            />
          </label>
          <label>
            Temperature{' '}
            <select
              disabled={busy}
              value={preferences.units}
              onChange={(e) =>
                setPreferences((p) => ({
                  ...p,
                  units: e.target.value as Preferences['units'],
                }))
              }
            >
              <option value="fahrenheit">Fahrenheit</option>
              <option value="celsius">Celsius</option>
            </select>
          </label>
          <p>
            Home city: {status?.city || 'Pittsburgh'}. Weather uses city-level
            coordinates. If enabled, AI receives your question, recent chat,
            first name and forecast. No cameras or device controls are connected
            to chat.
          </p>
          {!status?.aiReady && (
            <p>
              General AI replies need OPENAI_API_KEY and OPENAI_MODEL in the
              service’s .env.local file. Forecast questions work without them.
            </p>
          )}
        </section>
      )}
      {storageError && <output>{storageError}</output>}
      {!libraryOpen && libraryNotice && (
        <output className="canvas-library-notice">{libraryNotice}</output>
      )}
      {!layout.length && (
        <section className="canvas-empty-board">
          <Plus size={30} />
          <h2>Your canvas, your choice.</h2>
          <p>
            Add a widget to begin. Your home devices and settings are unchanged.
          </p>
          <button onClick={() => setLibraryOpen(true)}>
            Add your first widget
          </button>
        </section>
      )}
      <LibraryDialog.Root open={libraryOpen} onOpenChange={setLibraryOpen}>
        <LibraryDialog.Portal>
          <LibraryDialog.Backdrop className="canvas-library-backdrop" />
          <LibraryDialog.Popup className="canvas-library-dialog">
            <header>
              <div>
                <LibraryDialog.Title>Widget library</LibraryDialog.Title>
                <LibraryDialog.Description>
                  Choose what belongs on your canvas. Resize and arrange it
                  afterward.
                </LibraryDialog.Description>
              </div>
              <LibraryDialog.Close aria-label="Close widget library">
                <X size={22} />
              </LibraryDialog.Close>
            </header>
            <div className="canvas-library-grid">
              {widgetIds.map((id) => {
                const item = widgetCatalog[id],
                  Icon = widgetIcons[id],
                  added = layout.some((widget) => widget.id === id);
                return (
                  <article key={id}>
                    <Icon size={24} />
                    <span>{item.category}</span>
                    <h3>{item.title}</h3>
                    <p>{item.description}</p>
                    <button
                      disabled={added}
                      onClick={() => add(id)}
                      aria-label={
                        added
                          ? `${item.title} already added`
                          : `Add ${item.title}`
                      }
                    >
                      {added ? (
                        'Added to canvas'
                      ) : (
                        <>
                          <Plus size={15} /> Add widget
                        </>
                      )}
                    </button>
                  </article>
                );
              })}
            </div>
            <output aria-live="polite">
              {libraryNotice ||
                'Camera previews stay stopped until you start them.'}
            </output>
            <LibraryDialog.Close className="canvas-library-done">
              Done
            </LibraryDialog.Close>
          </LibraryDialog.Popup>
        </LibraryDialog.Portal>
      </LibraryDialog.Root>
      <div className="canvas-board" ref={board} key={workspaces.active}>
        {layout.map((widget, index) => {
          const expanded =
            (widget.id === 'weather' && attention) ||
            focusedWidgets.includes(widget.id);
          const Icon = widgetIcons[widget.id];
          return (
            <section
              key={widget.id}
              ref={widget.id === 'weather' ? weatherCard : undefined}
              data-widget={widget.id}
              className={`canvas-widget widget-${widget.id} ${expanded ? 'canvas-attention' : ''}`}
              data-height={expanded ? 4 : widget.height}
              style={
                {
                  '--widget-width': expanded
                    ? Math.max(6, widget.width)
                    : widget.width,
                  '--widget-height': expanded ? 4 : widget.height,
                } as CSSProperties
              }
              aria-label={`${widget.id} widget`}
            >
              <header className="canvas-widget-header">
                <h2>
                  <Icon size={17} />
                  {widgetCatalog[widget.id].title}
                </h2>
                {editing && (
                  <div className="canvas-widget-tools">
                    <button
                      disabled={index === 0}
                      onClick={() => move(widget.id, -1)}
                      aria-label={`Move ${widget.id} earlier`}
                    >
                      <MoveLeft size={14} />
                    </button>
                    <button
                      disabled={index === layout.length - 1}
                      onClick={() => move(widget.id, 1)}
                      aria-label={`Move ${widget.id} later`}
                    >
                      <MoveRight size={14} />
                    </button>
                    <details>
                      <summary aria-label={`Size ${widget.id} widget`}>
                        <Grip size={16} />
                      </summary>
                      <div className="canvas-size-menu">
                        <strong>Width</strong>
                        {([3, 6, 12] as const).map((width, i) => (
                          <button
                            key={width}
                            aria-pressed={widget.width === width}
                            onClick={() => change(widget.id, { width })}
                          >
                            {['Compact', 'Wide', 'Full'][i]}
                          </button>
                        ))}
                        <strong>Height</strong>
                        {([2, 3, 4] as const).map((height, i) => (
                          <button
                            key={height}
                            aria-pressed={widget.height === height}
                            onClick={() => change(widget.id, { height })}
                          >
                            {['Short', 'Medium', 'Tall'][i]}
                          </button>
                        ))}
                      </div>
                    </details>
                    <button
                      onClick={() => remove(widget.id)}
                      aria-label={`Remove ${widgetCatalog[widget.id].title}`}
                    >
                      <X size={14} />
                    </button>
                  </div>
                )}
              </header>
              <div className="canvas-widget-body">{contents(widget.id)}</div>
              {editing && (
                <button
                  className="canvas-resize"
                  aria-label={`Resize ${widget.id}: use arrow keys or drag`}
                  title="Drag to resize · arrow keys also work"
                  onPointerDown={(e) => {
                    if (e.button !== 0) return;
                    e.currentTarget.setPointerCapture(e.pointerId);
                    drag.current = {
                      id: widget.id,
                      x: e.clientX,
                      y: e.clientY,
                      original: {
                        ...widget,
                        width: expanded
                          ? (Math.max(6, widget.width) as WidgetLayout['width'])
                          : widget.width,
                        height: expanded ? 4 : widget.height,
                      },
                      gridWidth: board.current?.clientWidth || 1000,
                    };
                    if (expanded) change(widget.id, drag.current.original);
                  }}
                  onPointerMove={(e) => {
                    const start = drag.current;
                    if (
                      !start ||
                      start.id !== widget.id ||
                      !e.currentTarget.hasPointerCapture(e.pointerId)
                    )
                      return;
                    const next = resizeWidget(
                      start.original,
                      e.clientX - start.x,
                      e.clientY - start.y,
                      start.gridWidth,
                    );
                    change(widget.id, next);
                  }}
                  onPointerUp={(e) => {
                    drag.current = null;
                    if (e.currentTarget.hasPointerCapture(e.pointerId))
                      e.currentTarget.releasePointerCapture(e.pointerId);
                  }}
                  onPointerCancel={() => {
                    const start = drag.current;
                    if (start) change(start.id, start.original);
                    drag.current = null;
                  }}
                  onLostPointerCapture={() => {
                    drag.current = null;
                  }}
                  onKeyDown={(e) => {
                    const offsets: Record<string, [number, number]> = {
                      ArrowRight: [1, 0],
                      ArrowLeft: [-1, 0],
                      ArrowDown: [0, 1],
                      ArrowUp: [0, -1],
                    };
                    const offset = offsets[e.key];
                    if (offset) {
                      e.preventDefault();
                      change(
                        widget.id,
                        resizeWidget(
                          widget,
                          offset[0] *
                            Math.max(
                              80,
                              (board.current?.clientWidth || 1000) / 4,
                            ),
                          offset[1] * 120,
                          board.current?.clientWidth || 1000,
                        ),
                      );
                    }
                  }}
                >
                  <Grip size={16} />
                </button>
              )}
            </section>
          );
        })}
      </div>
      <footer className="canvas-footnote">
        Layout and preferences save in this browser. Forecast answers use
        provider data; AI is optional. No device actions.
      </footer>
    </main>
  );
}
