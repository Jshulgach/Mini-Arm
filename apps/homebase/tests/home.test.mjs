import test from 'node:test';
import assert from 'node:assert/strict';
import {
  devices,
  validateConfig,
  parseConfigs,
  withConfigs,
  previewPlan,
} from '../lib/home.ts';

test('configuration round-trip changes labels and room without overriding adapter or control state', () => {
  const config = validateConfig({
    name: '  Workshop arm  ',
    room: 'Living room',
    endpoint: 'http://192.168.1.20',
    adapter: 'evil',
    connected: true,
  });
  const restored = withConfigs(
    parseConfigs(
      JSON.stringify({ 'robot.miniarm': config, 'invented.device': config }),
    ),
  );
  const arm = restored.find((d) => d.id === 'robot.miniarm');
  assert.equal(arm.name, 'Workshop arm');
  assert.equal(arm.room, 'Living room');
  assert.equal(arm.adapter, devices.find((d) => d.id === arm.id).adapter);
  assert.equal(arm.connected, undefined);
  assert.equal(restored.length, devices.length);
});
test('malformed, secret-bearing and executable endpoint settings are rejected', () => {
  for (const endpoint of [
    'javascript:alert(1)',
    'file:///C:/private',
    'http://user:password@192.168.1.2',
    'https://home.local/?token=secret',
    'http://home/#key',
    '192.168.1.2',
  ]) {
    assert.throws(() =>
      validateConfig({ name: 'Camera', room: 'Kitchen', endpoint }),
    );
  }
  for (const raw of [
    'null',
    '[]',
    '{broken',
    '{"vacuum.l70":{"name":"x","room":"Invented","endpoint":""}}',
  ])
    assert.throws(() => parseConfigs(raw));
});
test('plans preserve unknowns and never return fabricated observations or executable commands', () => {
  for (const command of [
    'Return the L70 to its dock',
    'Where is my son?',
    'Grab the toy and put it on the couch',
  ]) {
    const plan = previewPlan(command);
    assert.ok(plan.steps.length);
    assert.ok(plan.blockers.length);
    assert.equal(plan.observation, undefined);
    assert.equal(plan.execute, undefined);
  }
  assert.equal(previewPlan('Turn on the garden lights').steps.length, 0);
  assert.equal(previewPlan('').steps.length, 0);
});
