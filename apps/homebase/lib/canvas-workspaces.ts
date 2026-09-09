import {
  defaultLayout,
  parseLayout,
  type WidgetLayout,
} from './canvas-layout.ts';

export type Workspace = { id: string; name: string; layout: WidgetLayout[] };
export type WorkspaceState = { version: 1; active: string; items: Workspace[] };
export const workspaceKey = 'homebase.canvas.workspaces.v1';
export const presetIds = ['overview', 'cameras', 'workshop'];
export function defaultWorkspaces(
  legacy: unknown = defaultLayout,
): WorkspaceState {
  return {
    version: 1,
    active: 'overview',
    items: [
      { id: 'overview', name: 'Overview', layout: parseLayout(legacy) },
      {
        id: 'cameras',
        name: 'Cameras',
        layout: [
          { id: 'chat', width: 6, height: 4 },
          { id: 'webcam', width: 6, height: 4 },
          { id: 'gripper', width: 6, height: 4 },
        ],
      },
      {
        id: 'workshop',
        name: 'Workshop',
        layout: [
          { id: 'chat', width: 6, height: 4 },
          { id: 'home', width: 6, height: 3 },
          { id: 'activity', width: 6, height: 3 },
          { id: 'gripper', width: 6, height: 4 },
        ],
      },
    ],
  };
}
export function parseWorkspaces(
  value: unknown,
  legacy?: unknown,
): WorkspaceState {
  const fallback = () => defaultWorkspaces(legacy);
  if (!value || typeof value !== 'object') return fallback();
  const v = value as WorkspaceState;
  if (
    v.version !== 1 ||
    !Array.isArray(v.items) ||
    v.items.length < 3 ||
    v.items.length > 8 ||
    v.items.some(
      (w) =>
        !w ||
        typeof w.id !== 'string' ||
        !/^[a-z0-9-]{1,60}$/.test(w.id) ||
        typeof w.name !== 'string' ||
        !w.name.trim() ||
        w.name.length > 30,
    ) ||
    new Set(v.items.map((w) => w.id)).size !== v.items.length ||
    !presetIds.every((id) => v.items.some((w) => w.id === id))
  )
    return fallback();
  const items = v.items.map((w) => ({
    id: w.id,
    name: w.name.trim(),
    layout: parseLayout(w.layout),
  }));
  return {
    version: 1,
    active: items.some((w) => w.id === v.active) ? v.active : 'overview',
    items,
  };
}
export function updateWorkspaceLayout(
  state: WorkspaceState,
  layout: WidgetLayout[],
): WorkspaceState {
  return {
    ...state,
    items: state.items.map((w) =>
      w.id === state.active ? { ...w, layout } : w,
    ),
  };
}
