import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createAssistant,
  normalizeForecast,
  localDate,
  weatherIntent,
} from '../scripts/assistant.mjs';
import {
  parseLayout,
  resizeWidget,
  defaultLayout,
  addWidget,
} from '../lib/canvas-layout.ts';
const time = Date.parse('2026-09-09T02:00:00Z'); // Still September 8 at home.
test('widget library preserves subsets and empty canvases, adds once, and migrates the original layout', () => {
  assert.deepEqual(parseLayout([]), []);
  assert.deepEqual(parseLayout(defaultLayout), defaultLayout);
  const subset = [{ id: 'webcam', width: 6, height: 4 }];
  assert.deepEqual(parseLayout(subset), subset);
  assert.equal(addWidget(subset, 'webcam'), subset);
  const restored = addWidget(subset, 'weather');
  assert.equal(restored.length, 2);
  assert.equal(restored[1].id, 'weather');
  assert.deepEqual(
    parseLayout([{ id: 'unknown', width: 3, height: 2 }]),
    defaultLayout,
  );
});
function fixture() {
  return {
    daily_units: { temperature_2m_max: '°F', wind_speed_10m_max: 'mp/h' },
    daily: {
      time: ['2026-09-08', '2026-09-09', '2026-09-10'],
      weather_code: [0, 61, 3],
      temperature_2m_max: [75, 68, 70],
      temperature_2m_min: [55, 50, 53],
      precipitation_probability_max: [0, 70, 15],
      wind_speed_10m_max: [5, 12, 7],
    },
  };
}
const context = {
  now: time,
  timezone: 'America/New_York',
  city: 'Pittsburgh',
  units: 'fahrenheit',
};
test('forecast dates follow home timezone and missing values never become zero', () => {
  assert.equal(localDate(time, 'America/New_York'), '2026-09-08');
  const result = normalizeForecast(fixture(), context);
  assert.equal(result.days[1].date, '2026-09-09');
  assert.equal(result.days[1].rain, 70);
  for (const field of [
    'weather_code',
    'temperature_2m_max',
    'precipitation_probability_max',
  ]) {
    const input = fixture();
    input.daily[field][1] = null;
    assert.throws(() => normalizeForecast(input, context), /incomplete/);
  }
  const wrong = fixture();
  wrong.daily_units.temperature_2m_max = '°C';
  assert.throws(() => normalizeForecast(wrong, context), /units/);
  assert.throws(
    () =>
      normalizeForecast(fixture(), { ...context, now: time + 86400000 * 5 }),
    /incomplete/,
  );
});
test('weather requests coalesce, use fixed provider URLs and return focus plus source data', async () => {
  let calls = 0;
  const assistant = createAssistant({
    now: () => time,
    env: {},
    fetcher: async (url) => {
      calls++;
      assert.equal(new URL(url).hostname, 'api.open-meteo.com');
      await new Promise((resolve) => setTimeout(resolve, 5));
      return { ok: true, json: async () => fixture() };
    },
  });
  const [a, b] = await Promise.all([
    assistant.forecast(),
    assistant.ask({ message: 'weather tomorrow', name: 'Jonathan' }),
  ]);
  assert.equal(calls, 1);
  assert.equal(b.focus, 'weather');
  assert.equal(b.day, 1);
  assert.match(b.text, /Jonathan.*high of 68°F/);
  assert.match(b.text, /umbrella/);
  assert.equal(b.weather.fetchedAt, a.fetchedAt);
  const follow = await assistant.ask({
    message: 'What about today?',
    context: 'weather',
  });
  assert.equal(follow.day, 0);
  assert.equal(
    (await assistant.ask({ message: 'move the robot' })).focus,
    undefined,
  );
  assert.equal(
    (await assistant.ask({ message: 'forecast next week' })).weather,
    undefined,
  );
  await assert.rejects(() => assistant.forecast('kelvin'), /Fahrenheit/);
  assert.equal(weatherIntent('rain the day after tomorrow'), 2);
});
test('provider failure is explicit and optional AI does not run unless requested', async () => {
  const bad = createAssistant({
    env: {},
    now: () => time,
    fetcher: async () => {
      throw new Error('network');
    },
  });
  await assert.rejects(
    () => bad.ask({ message: 'weather tomorrow' }),
    /Could not reach/,
  );
  let aiCalls = 0;
  const assistant = createAssistant({
    now: () => time,
    env: { OPENAI_API_KEY: 'test-secret', OPENAI_MODEL: 'test-model' },
    fetcher: async (url, options) => {
      if (url instanceof URL && url.hostname === 'api.open-meteo.com')
        return { ok: true, json: async () => fixture() };
      aiCalls++;
      const request = JSON.parse(options.body);
      assert.equal(request.store, false);
      assert.equal(request.model, 'test-model');
      assert.equal(request.tools, undefined);
      return {
        ok: true,
        json: async () => ({
          status: 'completed',
          output: [
            {
              type: 'message',
              content: [{ type: 'output_text', text: 'A rainy day at home.' }],
            },
          ],
        }),
      };
    },
  });
  assert.ok(!JSON.stringify(assistant.status()).includes('test-secret'));
  await assistant.ask({ message: 'weather tomorrow' });
  assert.equal(aiCalls, 0);
  const answer = await assistant.ask({
    message: 'weather tomorrow',
    useAI: true,
    history: [],
  });
  assert.equal(aiCalls, 1);
  assert.equal(answer.mode, 'OpenAI');
  assert.equal(answer.weather.days[1].high, 68);
  await assert.rejects(
    () => assistant.ask({ message: 'another question', useAI: true }),
    /wait/,
  );
});
test('layout rejects corrupt persistence and resize snaps without exceeding supported sizes', () => {
  assert.deepEqual(
    parseLayout([{ id: 'chat', width: 1000, height: -1 }]),
    defaultLayout,
  );
  assert.deepEqual(
    parseLayout([...defaultLayout.slice(0, 2), defaultLayout[0]]),
    defaultLayout,
  );
  assert.deepEqual(
    resizeWidget({ id: 'weather', width: 3, height: 2 }, 600, 300, 1200),
    { id: 'weather', width: 12, height: 4 },
  );
  assert.deepEqual(
    resizeWidget({ id: 'weather', width: 3, height: 2 }, -900, -900, 1200),
    { id: 'weather', width: 3, height: 2 },
  );
  const source = structuredClone(defaultLayout),
    copy = parseLayout(source);
  copy[0].width = 12;
  assert.equal(source[0].width, 6);
});
