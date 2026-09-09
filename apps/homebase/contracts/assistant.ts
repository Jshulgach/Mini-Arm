import type { WidgetId } from './widgets.ts';
export type Day = {
  date: string;
  label: string;
  code: number;
  condition: string;
  high: number;
  low: number;
  rain: number;
  wind: number;
};
export type Forecast = {
  city: string;
  timezone: string;
  units: string;
  windUnits: string;
  fetchedAt: string;
  source: string;
  sourceUrl: string;
  days: Day[];
};
export type HomeResult = {
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
export type Reply = {
  homeResult?: HomeResult;
  text: string;
  mode: string;
  notice?: string;
  weather?: Forecast;
  day?: number;
  focus?: 'weather';
};
export type Message = {
  homeResult?: HomeResult;
  role: 'user' | 'assistant';
  text: string;
  mode?: string;
  weather?: Forecast;
  day?: number;
};
export type Preferences = { name: string; units: 'fahrenheit' | 'celsius' };
