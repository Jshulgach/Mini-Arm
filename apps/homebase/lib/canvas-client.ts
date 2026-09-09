import type { Day } from '../contracts/assistant.ts';
import {
  parseAssistantReply,
  parseForecast,
} from '../contracts/validate-assistant.ts';
export function forecastLabel(item: Day, timezone: string, asOf: string) {
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
export async function request<T>(
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
  if (path === 'assistant' && body !== undefined)
    return parseAssistantReply(value) as T;
  if (path === 'weather') return parseForecast(value) as T;
  return value as T;
}
