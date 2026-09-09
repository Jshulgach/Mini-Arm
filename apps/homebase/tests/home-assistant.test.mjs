import test from 'node:test';
import assert from 'node:assert/strict';
import { Readable } from 'node:stream';
import { deviceObservation, homeAnswer } from '../scripts/home-assistant.mjs';
import { createAssistant } from '../scripts/assistant.mjs';
import { createGateway, handleGateway } from '../scripts/gateway.mjs';
import {
  defaultWorkspaces,
  parseWorkspaces,
  updateWorkspaceLayout,
} from '../lib/canvas-workspaces.ts';

const now = Date.parse('2026-09-09T03:00:00Z');
const device = {
  id: 'host.octopi',
  name: 'OctoPi',
  kind: 'network',
  room: 'Workshop',
  monitor: false,
  endpoint: 'http://192.168.1.159',
};
const observation = {
  state: 'unreachable',
  observedAt: new Date(now - 1000).toISOString(),
  source: 'TCP port check',
  endpoint: device.endpoint,
};
function fixture() {
  return {
    devices: [
      device,
      {
        ...device,
        id: 'camera.webcam',
        name: 'PC webcam',
        kind: 'camera',
        endpoint: '',
      },
      {
        ...device,
        id: 'camera.gripper',
        name: 'Gripper camera',
        kind: 'camera',
        endpoint: 'http://192.168.1.189',
      },
    ],
    rooms: ['Workshop'],
    health: {},
    events: [
      {
        id: 'event',
        at: new Date(now).toISOString(),
        source: 'gateway',
        message: 'Gateway started',
      },
    ],
  };
}
test('home answers distinguish current failures, stale observations, missing checks and endpoint changes', () => {
  assert.equal(deviceObservation(device, undefined, now).state, 'unmonitored');
  assert.equal(
    deviceObservation({ ...device, monitor: true }, undefined, now).state,
    'awaiting',
  );
  assert.equal(
    deviceObservation(device, observation, now).state,
    'unreachable',
  );
  const stale = deviceObservation(
    device,
    { ...observation, observedAt: new Date(now - 90001).toISOString() },
    now,
  );
  assert.equal(stale.state, 'stale');
  assert.equal(stale.lastResult, 'Port unreachable');
  for (const health of [
    { ...observation, endpoint: 'http://192.168.1.160' },
    { ...observation, observedAt: 'invalid' },
    { ...observation, observedAt: new Date(now + 1).toISOString() },
  ])
    assert.equal(deviceObservation(device, health, now).state, 'unmonitored');
  const home = fixture();
  let answer = homeAnswer("What's offline?", home, now);
  assert.match(answer.text, /does not mean they are all online/);
  assert.equal(answer.homeResult.devices.length, 0);
  home.health[device.id] = observation;
  answer = homeAnswer("What's offline?", home, now);
  assert.equal(answer.homeResult.devices.length, 1);
  assert.equal(answer.homeResult.devices[0].observedAt, observation.observedAt);
  answer = homeAnswer("What's offline?", home, now + 100000);
  assert.equal(answer.homeResult.devices.length, 0);
  assert.match(answer.text, /1 stale/);
});
test('home chat selects rooms and cameras without probing, sending data to AI or claiming camera perception', async () => {
  const assistant = createAssistant({
    now: () => now,
    env: { OPENAI_API_KEY: 'test', OPENAI_MODEL: 'test' },
    fetcher: () => assert.fail('Home answers must remain local'),
  });
  const home = fixture();
  const cameras = await assistant.ask(
    { message: 'Show my cameras', useAI: true },
    home,
  );
  assert.deepEqual(cameras.homeResult.focusWidgets, [
    'home',
    'webcam',
    'gripper',
  ]);
  assert.equal(cameras.homeResult.devices.length, 2);
  assert.match(cameras.text, /paused/);
  assert.ok(cameras.homeResult.devices.every((d) => !('endpoint' in d)));
  assert.equal(
    (await assistant.ask({ message: 'What is in the Workshop?' }, home))
      .homeResult.devices.length,
    3,
  );
  assert.equal(
    (await assistant.ask({ message: 'How is OctoPi?' }, home)).homeResult
      .devices.length,
    1,
  );
  assert.equal(
    (await assistant.ask({ message: 'Recent activity' }, home)).homeResult
      .events.length,
    1,
  );
  assert.match(
    (await assistant.ask({ message: 'Start my camera' }, home)).text,
    /cannot operate/,
  );
  assert.match(
    (await assistant.ask({ message: 'Home status' })).text,
    /unavailable/,
  );
});
test('assistant API uses the server registry and ignores a forged browser snapshot', async () => {
  let calls = 0;
  const gateway = createGateway(':memory:', {
    dial: async () => {
      calls++;
      return false;
    },
  });
  try {
    const assistant = createAssistant({
      env: {},
      now: () => now,
      fetcher: () => assert.fail('No external calls'),
    });
    const req = Readable.from([
      Buffer.from(
        JSON.stringify({
          message: "What's offline?",
          home: { devices: [device], health: { [device.id]: observation } },
        }),
      ),
    ]);
    req.method = 'POST';
    req.url = '/api/assistant';
    req.headers = {
      host: '127.0.0.1:5173',
      'content-type': 'application/json',
    };
    const res = {
      setHeader() {},
      writeHead(status) {
        this.status = status;
      },
      end(value) {
        this.value = JSON.parse(value);
      },
    };
    await handleGateway(req, res, gateway, assistant);
    assert.equal(res.status, 200);
    assert.equal(res.value.homeResult.devices.length, 0);
    assert.equal(calls, 0);
  } finally {
    await gateway.close();
  }
});
test('workspaces migrate legacy layouts and preserve independent empty and resized canvases', () => {
  let state = parseWorkspaces(null, []);
  assert.deepEqual(state.items[0].layout, []);
  const cameras = structuredClone(state.items[1].layout);
  state = updateWorkspaceLayout(state, [
    { id: 'weather', width: 12, height: 3 },
  ]);
  assert.deepEqual(state.items[1].layout, cameras);
  state = { ...state, active: 'cameras' };
  state = updateWorkspaceLayout(state, []);
  const saved = parseWorkspaces(JSON.parse(JSON.stringify(state)));
  assert.equal(saved.active, 'cameras');
  assert.deepEqual(saved.items[1].layout, []);
  assert.equal(saved.items[0].layout[0].width, 12);
  assert.equal(
    parseWorkspaces({ ...state, active: 'missing' }).active,
    'overview',
  );
  for (const bad of [
    null,
    { version: 2, items: [] },
    { ...state, items: [...state.items, state.items[0]] },
    { ...state, items: [{ id: 'bad', name: 'Bad', layout: [] }] },
  ])
    assert.equal(parseWorkspaces(bad).items.length, 3);
  const a = defaultWorkspaces(),
    b = defaultWorkspaces();
  a.items[0].layout[0].width = 12;
  assert.equal(b.items[0].layout[0].width, 6);
});
