import test from 'node:test';
import assert from 'node:assert/strict';
import {
  parseAssistantReply,
  parseForecast,
} from '../contracts/validate-assistant.ts';
import { createAssistant } from '../scripts/assistant.mjs';
import { initialConfiguration } from '../scripts/gateway.mjs';
test('the shared contract accepts actual local home answers and rejects unsupported widget instructions', async () => {
  const assistant = createAssistant({ env: {} });
  const reply = await assistant.ask(
    { message: 'Show my cameras' },
    { ...initialConfiguration(), health: {}, events: [] },
  );
  assert.equal(parseAssistantReply(reply), reply);
  assert.throws(
    () =>
      parseAssistantReply({
        ...reply,
        homeResult: { ...reply.homeResult, focusWidgets: ['execute-motor'] },
      }),
    /invalid response/,
  );
  assert.throws(
    () =>
      parseAssistantReply({
        ...reply,
        homeResult: {
          ...reply.homeResult,
          devices: [{ ...reply.homeResult.devices[0], monitored: 'yes' }],
        },
      }),
    /invalid response/,
  );
  assert.throws(
    () =>
      parseAssistantReply({
        text: 'Hello',
        mode: 'Homebase',
        focus: 'weather',
      }),
    /invalid response/,
  );
});
test('the shared forecast contract rejects malformed numeric values, units and dates before rendering', () => {
  const forecast = {
    city: 'Pittsburgh',
    timezone: 'America/New_York',
    units: '°F',
    windUnits: 'mph',
    fetchedAt: '2026-09-09T00:00:00Z',
    source: 'Open-Meteo',
    sourceUrl: 'https://open-meteo.com/',
    days: [0, 1, 2].map((i) => ({
      date: `2026-09-${String(9 + i).padStart(2, '0')}`,
      label: 'Day',
      condition: 'Clear',
      code: 0,
      high: 75,
      low: 60,
      rain: 0,
      wind: 5,
    })),
  };
  assert.equal(parseForecast(forecast), forecast);
  for (const patch of [
    { high: null },
    { rain: 101 },
    { wind: -1 },
    { low: 100 },
    { date: 'bad' },
  ])
    assert.throws(
      () =>
        parseForecast({
          ...forecast,
          days: [{ ...forecast.days[0], ...patch }, ...forecast.days.slice(1)],
        }),
      /invalid response/,
    );
  assert.throws(
    () => parseForecast({ ...forecast, units: 'K' }),
    /invalid response/,
  );
  assert.equal(
    parseAssistantReply({
      text: 'Forecast',
      mode: 'Forecast assistant',
      weather: forecast,
      day: 1,
      focus: 'weather',
    }).day,
    1,
  );
});
