export const widgetIds = [
  'chat',
  'weather',
  'home',
  'activity',
  'webcam',
  'gripper',
] as const;
export type WidgetId = (typeof widgetIds)[number];
export type WidgetLayout = {
  id: WidgetId;
  width: 3 | 6 | 12;
  height: 2 | 3 | 4;
};
export const defaultLayout: WidgetLayout[] = [
  { id: 'chat', width: 6, height: 4 },
  { id: 'weather', width: 3, height: 2 },
  { id: 'home', width: 3, height: 2 },
];
export const widgetCatalog: Record<
  WidgetId,
  {
    title: string;
    description: string;
    category: string;
    width: 3 | 6 | 12;
    height: 2 | 3 | 4;
  }
> = {
  chat: {
    title: 'Ask Homebase',
    description: 'Ask about devices, cameras, activity and the home forecast.',
    category: 'Everyday',
    width: 6,
    height: 4,
  },
  weather: {
    title: 'Weather',
    description: 'A compact home forecast that expands when you need it.',
    category: 'Everyday',
    width: 3,
    height: 2,
  },
  home: {
    title: 'Home status',
    description: 'Registered devices and their latest connection observations.',
    category: 'Everyday',
    width: 3,
    height: 2,
  },
  activity: {
    title: 'Recent activity',
    description: 'Recent events from your local home gateway.',
    category: 'Everyday',
    width: 6,
    height: 3,
  },
  webcam: {
    title: 'PC webcam',
    description: 'A local webcam preview. Starts only when you choose Start.',
    category: 'Cameras',
    width: 6,
    height: 4,
  },
  gripper: {
    title: 'Gripper camera',
    description:
      'ESP32 camera snapshots using its saved device address. Starts paused.',
    category: 'Cameras',
    width: 6,
    height: 4,
  },
};
export function addWidget(
  layout: WidgetLayout[],
  id: WidgetId,
): WidgetLayout[] {
  if (layout.some((widget) => widget.id === id)) return layout;
  const { width, height } = widgetCatalog[id];
  return [...layout, { id, width, height }];
}
export function parseLayout(value: unknown): WidgetLayout[] {
  if (
    !Array.isArray(value) ||
    value.length > widgetIds.length ||
    new Set(value.map((v) => v?.id)).size !== value.length
  )
    return defaultLayout.map((v) => ({ ...v }));
  if (
    value.some(
      (v) =>
        !widgetIds.includes(v?.id) ||
        ![3, 6, 12].includes(v.width) ||
        ![2, 3, 4].includes(v.height),
    )
  )
    return defaultLayout.map((v) => ({ ...v }));
  return value.map(({ id, width, height }) => ({ id, width, height }));
}
export function resizeWidget(
  value: WidgetLayout,
  dx: number,
  dy: number,
  gridWidth: number,
): WidgetLayout {
  const widths = [3, 6, 12] as const,
    heights = [2, 3, 4] as const;
  const step = Math.max(80, gridWidth / 4);
  return {
    ...value,
    width:
      widths[
        Math.max(
          0,
          Math.min(2, widths.indexOf(value.width) + Math.round(dx / step)),
        )
      ],
    height:
      heights[
        Math.max(
          0,
          Math.min(2, heights.indexOf(value.height) + Math.round(dy / 120)),
        )
      ],
  };
}
