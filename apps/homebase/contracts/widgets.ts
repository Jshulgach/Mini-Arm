export const widgetIds = [
  'chat',
  'weather',
  'home',
  'activity',
  'webcam',
  'gripper',
] as const;
export type WidgetId = (typeof widgetIds)[number];
