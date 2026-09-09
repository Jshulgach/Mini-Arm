import test from 'node:test';
import assert from 'node:assert/strict';
import { checkArmConnection, parseArmReport, picoFilter, probeBaseTest, pulseBaseTest, parseBaseTestStatus, holdBaseTest, validateHoldStatus, moveJointTarget } from '../lib/arm-serial.ts';

const report = 'Testing 123\n=== Robot Info ===\nJoint states: [0, 0, 0, 0, 0, 0]\nServo readings: [None, None, None, None, None, None]\nSimulated hardware: False\n';
function fakePort(respond = true, payload = report) {
  let input;
  const port = {
    opened: 0, closed: 0, writes: [], canceled: 0,
    getInfo: () => picoFilter,
    async open() { this.opened++; },
    async close() { assert.equal(this.readable.locked, false); assert.equal(this.writable.locked, false); this.closed++; },
    readable: new ReadableStream({ start(controller) { input = controller; }, cancel() { port.canceled++; } }),
    writable: new WritableStream({ write(bytes) {
      const command = new TextDecoder().decode(bytes); port.writes.push(command);
      if (!respond) return;
      const token = command.match(/^sync:(.*)-b;/)[1];
      const reply = `stale unrelated data\n@sync:wrong-b\nignored\n@sync:${token}-b\r\n${payload}@sync:${token}-e\r\n`;
      // Fragment every marker and line to exercise the stream parser.
      for (let i = 0; i < reply.length; i += 7) input.enqueue(new TextEncoder().encode(reply.slice(i, i + 7)));
    } }),
    input: () => input,
  };
  return port;
}
test('USB check accepts matching fences only, sends read-only commands and releases both streams', async () => {
  const port = fakePort(); const result = await checkArmConnection(port, new AbortController().signal);
  assert.deepEqual(result.jointCommands, [0, 0, 0, 0, 0, 0]);
  assert.deepEqual(result.servoCommands, [null, null, null, null, null, null]);
  assert.equal(result.transcript.includes('ignored'), false);
  assert.match(port.writes[0], /^sync:[a-f0-9-]+-b;test;info;sync:[a-f0-9-]+-e;$/);
  assert.equal(port.opened, 1); assert.equal(port.closed, 1);
});
test('no reply times out and releases the port without retrying', async () => {
  const port = fakePort(false);
  await assert.rejects(checkArmConnection(port, new AbortController().signal, 20), /timeout/);
  assert.equal(port.writes.length, 1); assert.equal(port.closed, 1);
});
test('cancel before opening and while awaiting data both release resources', async () => {
  const before = new AbortController(); before.abort(); const unopened = fakePort();
  await assert.rejects(checkArmConnection(unopened, before.signal), { name: 'AbortError' });
  assert.equal(unopened.opened, 0);
  const during = new AbortController(); const port = fakePort(false);
  const checking = checkArmConnection(port, during.signal);
  await new Promise(resolve => setTimeout(resolve, 0)); during.abort();
  await assert.rejects(checking, { name: 'AbortError' }); assert.equal(port.closed, 1);
});
test('cancel during a delayed port open closes it before any command', async () => {
  const controller = new AbortController(); const port = fakePort(); let release;
  port.open = () => new Promise(resolve => { release = resolve; });
  const checking = checkArmConnection(port, controller.signal); controller.abort(); release();
  await assert.rejects(checking, { name: 'AbortError' });
  assert.equal(port.closed, 1); assert.deepEqual(port.writes, []);
});
test('rejects the wrong USB device without opening', async () => {
  const port = fakePort(); port.getInfo = () => ({ usbVendorId: 123, usbProductId: 456 });
  await assert.rejects(checkArmConnection(port, new AbortController().signal), /Select/);
  assert.equal(port.opened, 0);
});
test('physical disconnect and excessive output fail closed', async () => {
  for (const oversized of [false, true]) {
    const port = fakePort(false); const checking = checkArmConnection(port, new AbortController().signal);
    await new Promise(resolve => setTimeout(resolve, 0));
    if (oversized) port.input().enqueue(new Uint8Array(32769)); else port.input().close();
    await assert.rejects(checking, oversized ? /32 KB/ : /closed/);
    assert.equal(port.closed, 1);
  }
});
test('rejects simulated or malformed status and keeps disabled outputs distinct from zero', () => {
  assert.throws(() => parseArmReport(report.replace('False', 'True')), /physical/);
  assert.throws(() => parseArmReport(report.replace('0, 0, 0, 0, 0, 0', '0, NaN, 0, 0, 0, 0')), /Invalid/);
  assert.throws(() => parseArmReport(report.replace('0, 0, 0, 0, 0, 0', '0, 0')), /six/);
  assert.equal(parseArmReport(report.replace('None, None', '0, 90')).servoCommands[0], 0);
});

const capabilities = { ok: true, protocol: 'miniarm-base-pulse-v1', channel: 15, duration_ms: 400,
  max_step: 1, window: 5, center: null, last: null, active: false, fault: null, locked: false, outputs: Array(7).fill(null) };
const calReply = value => '@cal:' + JSON.stringify(value) + '\n';
test('numeric first target sends exactly one explicit initialization pulse and confirms output release', async () => {
  const complete = {...capabilities, last:80, center:80, locked:true};
  const port = fakePort(true, calReply(complete));
  const result = await moveJointTarget(port, new AbortController().signal, capabilities, 80, () => assert.fail('Initialization does not start a jog session'));
  assert.equal(result.last, 80); assert.equal(port.writes.length, 1); assert.equal(port.closed, 1);
  assert.match(port.writes[0], /cal_pulse:[a-f0-9-]+:80:none;/);
});
test('base probe is read-only and requires the exact firmware capabilities', async () => {
  const port = fakePort(true, calReply(capabilities));
  assert.deepEqual(await probeBaseTest(port, new AbortController().signal), capabilities);
  assert.match(port.writes[0], /-b;cal_probe;sync:/);
  assert.equal(port.closed, 1);
  for (const value of [{ ...capabilities, protocol: 'old' }, { ...capabilities, duration_ms: 4000 }, { ...capabilities, outputs: [null] },
    { ...capabilities, last: 90 }, { ...capabilities, center: 90, last: 100, locked: true }]) {
    assert.throws(() => parseBaseTestStatus(calReply(value)), /Unexpected/);
  }
  assert.throws(() => parseBaseTestStatus('Unknown command'), /does not expose/);
});
test('explicit pulse includes previous target and requires confirmed output release', async () => {
  const complete = { ...capabilities, center: 90, last: 90, locked: true };
  const port = fakePort(true, calReply(complete));
  assert.equal((await pulseBaseTest(port, new AbortController().signal, 90, capabilities)).last, 90);
  assert.match(port.writes[0], /cal_pulse:[a-f0-9-]+:90:none;/);
  assert.equal(port.closed, 1);
  const bad = fakePort(true, calReply({ ...complete, outputs: [90, ...Array(6).fill(null)] }));
  await assert.rejects(pulseBaseTest(bad, new AbortController().signal, 90, capabilities), /not confirmed/);
  assert.equal(bad.writes.length, 1); assert.equal(bad.closed, 1);
});
test('invalid or excessive target changes and active outputs never open the port', async () => {
  for (const [angle, previous] of [[NaN, capabilities], [-1, capabilities], [181, capabilities],
    [92, { ...capabilities, last: 90, center: 90, locked: true }],
    [96, { ...capabilities, last: 95, center: 90, locked: true }],
    [90, { ...capabilities, outputs: [null, null, null, null, null, null, 0] }]]) {
    const port = fakePort();
    await assert.rejects(pulseBaseTest(port, new AbortController().signal, angle, previous));
    assert.equal(port.opened, 0);
  }
});
test('rejected pulse response is surfaced without retrying', async () => {
  const port = fakePort(true, calReply({ ok: false, error: 'Previous target changed' }));
  await assert.rejects(pulseBaseTest(port, new AbortController().signal, 90, capabilities), /Previous target/);
  assert.equal(port.writes.length, 1); assert.equal(port.closed, 1);
});

const holdReady = { ...capabilities, center: 90, last: 90, locked: true,
  hold_protocol: 'miniarm-base-hold-v1', hold_ms: 3000, mode: null, active_token: null };
function holdPort({ cancelStart, rejectStart = false, failRelease = false, joint = 'base', wrongOutput = false } = {}) {
  const prefix = joint === 'base' ? 'cal' : joint.replaceAll('-', '_');
  const index = ['base', 'shoulder', 'elbow', 'wrist-rotation', 'wrist-bend', 'gripper-link', 'gripper'].indexOf(joint);
  const ready = { ...holdReady, protocol: `miniarm-${joint}-pulse-v1`, hold_protocol: `miniarm-${joint}-hold-v1`, channel: [15,14,13,12,2,1,0][index] };
  const port = { writes: [], opened: 0, closed: 0, readable: null, writable: null,
    getInfo: () => picoFilter,
    async open() {
      this.opened++; let input;
      this.readable = new ReadableStream({ start(controller) { input = controller; } });
      this.writable = new WritableStream({ write(bytes) {
        const command = new TextDecoder().decode(bytes); port.writes.push(command);
        const fence = command.match(/^sync:(.*)-b;/)[1];
        const hold = command.match(new RegExp(`;${prefix}_hold:([a-f0-9-]+):90;`));
        const release = command.match(new RegExp(`;${prefix}_release:([a-f0-9-]+);`));
        assert.ok(hold || release, command);
        let response = { ...ready };
        if (hold) {
          if (cancelStart) { cancelStart.abort(); return; }
          response = rejectStart ? { ok: false, error: 'Start rejected' } :
            { ...ready, active: true, mode: 'hold', active_token: hold[1], outputs: Array.from({length: 7}, (_, i) => i === (wrongOutput ? (index + 1) % 7 : index) ? 90 : null) };
        } else if (failRelease) response = { ok: false, error: 'I2C release failed' };
        input.enqueue(new TextEncoder().encode(`@sync:${fence}-b\n${calReply(response)}@sync:${fence}-e\n`));
      } });
    },
    async close() { assert.equal(this.readable.locked, false); assert.equal(this.writable.locked, false); this.closed++; },
  };
  return port;
}
test('hold capability checks reject old firmware or an untested target before opening USB', async () => {
  assert.deepEqual(validateHoldStatus(holdReady), holdReady);
  assert.throws(() => validateHoldStatus(capabilities), /hold capability/);
  assert.throws(() => validateHoldStatus({ ...holdReady, hold_ms: 30000 }), /hold capability/);
  const port = holdPort();
  await assert.rejects(holdBaseTest(port, new AbortController().signal, { ...holdReady, center: null, last: null }, () => {}), /pulse first/);
  assert.equal(port.opened, 0);
});
test('Release uses the active token and a fresh signal after user cancellation', async () => {
  const port = holdPort(); const stop = new AbortController();
  const result = await holdBaseTest(port, stop.signal, holdReady, status => { assert.equal(status.active, true); stop.abort(); });
  assert.equal(result.active, false); assert.deepEqual(result.outputs, Array(7).fill(null));
  const token = port.writes[0].match(/cal_hold:([a-f0-9-]+):90/)[1];
  assert.ok(port.writes[1].includes(`cal_release:${token};`));
  assert.equal(port.writes.length, 2); assert.equal(port.closed, 2);
});
test('cancellation during hold activation still attempts an owned release', async () => {
  const stop = new AbortController(); const port = holdPort({ cancelStart: stop });
  const result = await holdBaseTest(port, stop.signal, holdReady, () => assert.fail('Canceled start must not report holding'));
  assert.equal(result.active, false); assert.equal(port.writes.length, 2); assert.equal(port.closed, 2);
});
test('hold activation errors still release, while release failures are never called success', async () => {
  const rejected = holdPort({ rejectStart: true });
  await assert.rejects(holdBaseTest(rejected, new AbortController().signal, holdReady, () => {}), /subsequently confirmed off/);
  assert.equal(rejected.writes.length, 2);
  const port = holdPort({ failRelease: true }); const stop = new AbortController();
  await assert.rejects(holdBaseTest(port, stop.signal, holdReady, () => stop.abort()), /release was not confirmed/);
  assert.equal(port.writes.length, 2);
});
test('browser sends release after its bounded observation interval without renewing hold', async () => {
  const port = holdPort(); const before = performance.now();
  const result = await holdBaseTest(port, new AbortController().signal, holdReady, () => {});
  assert.ok(performance.now() - before >= 2900);
  assert.equal(result.active, false); assert.equal(port.writes.length, 2);
});

const shoulderReady = { ...holdReady, protocol: 'miniarm-shoulder-pulse-v1', channel: 14, hold_protocol: 'miniarm-shoulder-hold-v1' };
for (const [index, joint] of ['base', 'shoulder', 'elbow', 'wrist-rotation', 'wrist-bend', 'gripper-link', 'gripper'].entries()) {
  test(`${joint} verifies identity, pulse routing, hold output and owned release`, async () => {
    const channel = [15,14,13,12,2,1,0][index];
    const prefix = joint === 'base' ? 'cal' : joint.replaceAll('-', '_');
    const ready = { ...holdReady, protocol: `miniarm-${joint}-pulse-v1`, hold_protocol: `miniarm-${joint}-hold-v1`, channel };
    const probe = fakePort(true, calReply(ready));
    assert.equal((await probeBaseTest(probe, new AbortController().signal, joint)).channel, channel);
    assert.ok(probe.writes[0].includes(`;${prefix}_probe;`));
    assert.throws(() => parseBaseTestStatus(calReply({...ready, channel: 99}), joint), /Unexpected/);
    const pulse = fakePort(true, calReply(ready));
    await pulseBaseTest(pulse, new AbortController().signal, 90, ready);
    assert.ok(pulse.writes[0].includes(`;${prefix}_pulse:`));
    const port = holdPort({ joint }); const stop = new AbortController();
    await holdBaseTest(port, stop.signal, ready, status => {
      assert.deepEqual(status.outputs, Array.from({length: 7}, (_, i) => i === index ? 90 : null)); stop.abort();
    });
    const token = port.writes[0].match(new RegExp(`${prefix}_hold:([a-f0-9-]+):90`))[1];
    assert.ok(port.writes[1].includes(`;${prefix}_release:${token};`));
    const wrong = holdPort({joint, wrongOutput: true});
    await assert.rejects(holdBaseTest(wrong, new AbortController().signal, ready, () => assert.fail('Wrong output accepted')), /activation was not confirmed/);
    assert.equal(wrong.writes.length, 2);
  });
}
test('shoulder probe requires shoulder identity and never accepts a base response', async () => {
  const port = fakePort(true, calReply(shoulderReady));
  assert.equal((await probeBaseTest(port, new AbortController().signal, 'shoulder')).channel, 14);
  assert.match(port.writes[0], /shoulder_probe;/);
  const wrong = fakePort(true, calReply(holdReady));
  await assert.rejects(probeBaseTest(wrong, new AbortController().signal, 'shoulder'), /Unexpected/);
});
test('shoulder pulse uses its own command and rejects a mismatched channel before writing', async () => {
  const port = fakePort(true, calReply(shoulderReady));
  await pulseBaseTest(port, new AbortController().signal, 90, shoulderReady);
  assert.match(port.writes[0], /shoulder_pulse:[a-f0-9-]+:90:90;/);
  const wrong = fakePort();
  await assert.rejects(pulseBaseTest(wrong, new AbortController().signal, 90, { ...shoulderReady, channel: 15 }), /Unsupported/);
  assert.equal(wrong.opened, 0);
});
test('shoulder hold checks output index one and releases through its own command', async () => {
  const port = holdPort({ joint: 'shoulder' }); const stop = new AbortController();
  const result = await holdBaseTest(port, stop.signal, shoulderReady, status => {
    assert.equal(status.outputs[0], null); assert.equal(status.outputs[1], 90); stop.abort();
  });
  assert.equal(result.active, false);
  assert.match(port.writes[1], /shoulder_release:/);
  const wrong = holdPort({ joint: 'shoulder', wrongOutput: true });
  await assert.rejects(holdBaseTest(wrong, new AbortController().signal, shoulderReady, () => assert.fail('Wrong output must not be accepted')), /activation was not confirmed/);
  assert.match(wrong.writes[1], /shoulder_release:/);
});
