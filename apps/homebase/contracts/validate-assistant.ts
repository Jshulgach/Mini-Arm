import type { Forecast, Reply } from './assistant.ts';
import { widgetIds } from './widgets.ts';
function invalid(): never {
  throw Object.assign(
    new Error('The assistant returned an invalid response. Please retry.'),
    { status: 502 },
  );
}
function object(value: unknown): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    return invalid();
  return value as Record<string, unknown>;
}
function strings(value: Record<string, unknown>, keys: string[]) {
  if (keys.some((key) => typeof value[key] !== 'string')) invalid();
}
function timestamp(value: unknown) {
  if (typeof value !== 'string' || !Number.isFinite(Date.parse(value)))
    invalid();
}
export function parseForecast(value: unknown): Forecast {
  const data = object(value);
  strings(data, [
    'city',
    'timezone',
    'units',
    'windUnits',
    'fetchedAt',
    'source',
    'sourceUrl',
  ]);
  timestamp(data.fetchedAt);
  if (
    !['°F', '°C'].includes(data.units as string) ||
    data.windUnits !== 'mph' ||
    !Array.isArray(data.days) ||
    data.days.length !== 3
  )
    invalid();
  for (const raw of data.days) {
    const day = object(raw);
    strings(day, ['date', 'label', 'condition']);
    timestamp(day.date);
    if (
      ['code', 'high', 'low', 'rain', 'wind'].some(
        (key) => typeof day[key] !== 'number' || !Number.isFinite(day[key]),
      )
    )
      invalid();
    if (
      (day.low as number) > (day.high as number) ||
      (day.rain as number) < 0 ||
      (day.rain as number) > 100 ||
      (day.wind as number) < 0
    )
      invalid();
  }
  return value as Forecast;
}
export function parseAssistantReply(value: unknown): Reply {
  const data = object(value);
  strings(data, ['text', 'mode']);
  if (data.notice !== undefined && typeof data.notice !== 'string') invalid();
  if (data.weather !== undefined) parseForecast(data.weather);
  if (
    data.day !== undefined &&
    (!Number.isInteger(data.day) ||
      Number(data.day) < 0 ||
      Number(data.day) > 2)
  )
    invalid();
  if (
    data.focus !== undefined &&
    (data.focus !== 'weather' || data.weather === undefined)
  )
    invalid();
  if (data.homeResult !== undefined) {
    const home = object(data.homeResult);
    strings(home, ['title', 'asOf']);
    timestamp(home.asOf);
    if (
      !Array.isArray(home.focusWidgets) ||
      home.focusWidgets.some((id) => !widgetIds.includes(id)) ||
      !Array.isArray(home.devices) ||
      !Array.isArray(home.events)
    )
      invalid();
    for (const raw of home.devices) {
      const device = object(raw);
      strings(device, ['id', 'name', 'room', 'label', 'state', 'source']);
      if (
        typeof device.monitored !== 'boolean' ||
        (device.lastResult !== null && typeof device.lastResult !== 'string')
      )
        invalid();
      if (device.observedAt !== null) timestamp(device.observedAt);
    }
    for (const raw of home.events) {
      const event = object(raw);
      strings(event, ['id', 'at', 'source', 'message']);
      timestamp(event.at);
    }
  }
  return value as Reply;
}
