'use client';
import { useEffect, useRef, useState, type SetStateAction } from 'react';
import type {
  Forecast,
  Reply,
  Message,
  Preferences,
  HomeResult,
} from '@/contracts/assistant';
import { useGateway } from '@/lib/gateway';
import {
  addWidget,
  widgetIds,
  widgetCatalog,
  type WidgetId,
  type WidgetLayout,
} from '@/lib/canvas-layout';
import {
  defaultWorkspaces,
  parseWorkspaces,
  updateWorkspaceLayout,
  workspaceKey,
} from '@/lib/canvas-workspaces';
import { request } from '@/lib/canvas-client';
const layoutKey = 'homebase.canvas.layout.v1',
  preferenceKey = 'homebase.canvas.preferences.v1';

export function useCanvasController() {
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

  function setDrag(value: typeof drag.current) {
    drag.current = value;
  }
  return {
    setDrag,
    workspaces,
    setWorkspaces,
    activeWorkspace,
    layout,
    setLayout,
    workspaceEditor,
    setWorkspaceEditor,
    workspaceName,
    setWorkspaceName,
    focusedWidgets,
    setFocusedWidgets,
    focusedDeviceIds,
    setFocusedDeviceIds,
    preferences,
    setPreferences,
    editing,
    setEditing,
    ready,
    setReady,
    storageError,
    setStorageError,
    status,
    setStatus,
    weather,
    setWeather,
    weatherError,
    setWeatherError,
    weatherLoading,
    setWeatherLoading,
    day,
    setDay,
    attention,
    setAttention,
    input,
    setInput,
    messages,
    setMessages,
    busy,
    setBusy,
    useAI,
    setUseAI,
    refresh,
    setRefresh,
    settings,
    setSettings,
    libraryOpen,
    setLibraryOpen,
    libraryNotice,
    setLibraryNotice,
    weatherVisible,
    home,
    homeError,
    board,
    weatherCard,
    transcript,
    chatController,
    drag,
    leaveWorkspace,
    switchWorkspace,
    saveWorkspace,
    revealHome,
    change,
    move,
    add,
    remove,
    ask,
  };
}
export type CanvasController = ReturnType<typeof useCanvasController>;
