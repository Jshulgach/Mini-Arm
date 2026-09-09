import test from 'node:test';
import assert from 'node:assert/strict';
import {
  initialVirtualHome,
  simulateEvent,
  rules,
} from '../lib/virtual-home.ts';
const enabled = rules.map((rule) => rule.id);
test('virtual events update only simulated state and record the matching rule effects', () => {
  const original = initialVirtualHome();
  const snapshot = structuredClone(original);
  const next = simulateEvent(original, 'laundry', enabled);
  assert.deepEqual(original, snapshot);
  assert.equal(next.devices.washer, 'Complete');
  assert.equal(next.devices.hub, 'Reachable');
  assert.equal(next.focus, 'washer');
  assert.equal(next.notifications.length, 1);
  assert.deepEqual(
    next.runs[0].decisions.filter((d) => d.matched).map((d) => d.id),
    ['laundry-notice'],
  );
});
test('repeated states do not repeat notifications, and recovery allows a subsequent disconnect', () => {
  let state = simulateEvent(initialVirtualHome(), 'disconnect', enabled);
  state = simulateEvent(state, 'disconnect', enabled);
  assert.equal(state.notifications.length, 1);
  assert.equal(state.runs[0].changed, false);
  assert.equal(state.focus, null);
  assert.ok(
    state.runs[0].decisions.every((d) => !d.matched && d.effects.length === 0),
  );
  state = simulateEvent(state, 'recover', enabled);
  assert.equal(state.devices.hub, 'Reachable');
  state = simulateEvent(state, 'disconnect', enabled);
  assert.equal(state.notifications.length, 3);
});
test('disabled rules still allow the event but produce no effects or retroactive changes', () => {
  const disabled = simulateEvent(initialVirtualHome(), 'laundry', []);
  assert.equal(disabled.devices.washer, 'Complete');
  assert.equal(disabled.notifications.length, 0);
  assert.equal(disabled.focus, null);
  const next = simulateEvent(disabled, 'laundry', enabled);
  assert.equal(next.notifications.length, 0);
  assert.match(next.runs[0].decisions[0].reason, /duplicate/);
  assert.match(disabled.runs[0].decisions[0].reason, /Disabled/);
});
test('arrival proposes a physical action without changing a shade or emitting a device command', () => {
  const state = simulateEvent(initialVirtualHome(), 'arrival', enabled);
  assert.deepEqual(Object.keys(state.devices).sort(), [
    'entry',
    'hub',
    'washer',
  ]);
  const effects = state.runs[0].decisions.flatMap((d) => d.effects);
  assert.deepEqual(
    effects.map((e) => e.kind),
    ['focus', 'proposal'],
  );
  assert.ok(
    effects.every((e) => Object.keys(e).sort().join(',') === 'kind,text'),
  );
  assert.equal(state.notifications.length, 0);
});
test('virtual history is bounded, snapshots stay independent, and unknown events are rejected', () => {
  let state = initialVirtualHome();
  for (let i = 0; i < 50; i++)
    state = simulateEvent(state, i % 2 ? 'recover' : 'disconnect', enabled);
  assert.equal(state.runs.length, 20);
  assert.equal(state.notifications.length, 12);
  assert.equal(state.sequence, 50);
  assert.equal(initialVirtualHome().sequence, 0);
  assert.throws(() => simulateEvent(state, 'unknown', enabled), /Unknown/);
});
