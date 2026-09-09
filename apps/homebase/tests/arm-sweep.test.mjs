import test from 'node:test';
import assert from 'node:assert/strict';
import { picoFilter, sweepJoint, validateSweepStatus, probeAllJoints, moveJointTarget } from '../lib/arm-serial.ts';

const joints = ['base', 'shoulder', 'elbow', 'wrist-rotation', 'wrist-bend', 'gripper-link', 'gripper'];
function ready(joint = 'base') {
  return {ok:true, protocol:`miniarm-${joint}-pulse-v1`, channel:[15,14,13,12,2,1,0][joints.indexOf(joint)],
    duration_ms:400, max_step:1, window:5, center:90, last:90, active:false, locked:true, fault:null,
    outputs:Array(7).fill(null), mode:null, active_token:null, sweep_protocol:`miniarm-${joint}-sweep-v1`,
    sweep_ms:3000, sweep_watchdog_ms:600, sweep_rate:10, sweep_sequence:0, sweep_target:null};
}
function portFor(joint = 'base', {cancelStart, cancelTarget, badSequence, badOutput, failRelease, rejectStart, reachTarget = false} = {}) {
  const prefix = joint === 'base' ? 'cal' : joint.replaceAll('-', '_');
  const index = joints.indexOf(joint);
  let last = 90;
  const port = {writes:[], closed:0, getInfo:() => picoFilter,
    async open() {
      let input;
      this.readable = new ReadableStream({start(c) { input = c; }});
      this.writable = new WritableStream({write(bytes) {
        const command = new TextDecoder().decode(bytes); port.writes.push(command);
        const fence = command.match(/^sync:(.*)-b;/)[1];
        const [name, token, sequence, target] = command.split(';')[1].split(':');
        if (reachTarget && name === prefix + '_target') last = Number(target);
        let result = {...ready(joint), last};
        if (name === prefix + '_release') {
          if (failRelease) result = {ok:false, error:'release failed'};
        } else if (name === prefix + '_sweep' || name === prefix + '_target') {
          if (name.endsWith('_sweep') && cancelStart) { cancelStart.abort(); return; }
          if (name.endsWith('_target') && cancelTarget) { cancelTarget.abort(); return; }
          if (name.endsWith('_sweep') && rejectStart) result = {ok:false, error:'Start rejected'};
          else result = {...result, active:true, mode:'sweep', active_token:token,
            sweep_target:target === undefined ? 90 : Number(target),
            sweep_sequence:target === undefined ? 0 : Number(sequence) + (badSequence ? 1 : 0),
            outputs:Array.from({length:7}, (_, i) => i === (badOutput ? (index+1)%7 : index) ? last : null)};
        } else assert.fail('Unexpected command: ' + name);
        input.enqueue(new TextEncoder().encode(`@sync:${fence}-b\n@cal:${JSON.stringify(result)}\n@sync:${fence}-e\n`));
      }});
    },
    async close() { assert.equal(this.readable.locked, false); assert.equal(this.writable.locked, false); this.closed++; },
  };
  return port;
}

test('numeric Go uses bounded jog and releases when the target command is acknowledged', async () => {
  const port = portFor('elbow', {reachTarget:true}); const seen = [];
  const result = await moveJointTarget(port, new AbortController().signal, ready('elbow'), 94.5, s => seen.push(s.last));
  assert.deepEqual(seen, [90, 94.5]); assert.equal(result.last, 94.5);
  assert.equal(result.active, false); assert.deepEqual(result.outputs, Array(7).fill(null));
  assert.equal(port.writes.length, 3); assert.match(port.writes[1], /elbow_target:[a-f0-9-]+:1:94.5;/);
  assert.match(port.writes[2], /elbow_release:/);
});

test('numeric Go cancellation releases, while invalid targets never open USB', async () => {
  const stop = new AbortController(); const port = portFor();
  await moveJointTarget(port, stop.signal, ready(), 94, () => stop.abort());
  assert.equal(port.writes.length, 2); assert.match(port.writes[1], /cal_release:/);
  for (const angle of [NaN, Infinity, -1, 181, 96]) {
    const untouched = portFor();
    await assert.rejects(moveJointTarget(untouched, new AbortController().signal, ready(), angle, () => {}), /range/);
    assert.equal(untouched.writes.length, 0);
  }
});

test('numeric Go to the existing target releases without sending an extra target update', async () => {
  const port = portFor();
  await moveJointTarget(port, new AbortController().signal, ready(), 90, () => {});
  assert.equal(port.writes.length, 2); assert.match(port.writes[1], /cal_release:/);
});

function wholeArmPort({badJoint, abortAfter, stop, malformed = false} = {}) {
  let opened = false;
  const port = {writes: [], closed: 0, getInfo: () => picoFilter,
    async open() {
      assert.equal(opened, false, 'Shared port must never open concurrently'); opened = true;
      let input;
      this.readable = new ReadableStream({start(c) { input = c; }});
      this.writable = new WritableStream({write(bytes) {
        const command = new TextDecoder().decode(bytes); port.writes.push(command);
        const fence = command.match(/^sync:(.*)-b;/)[1];
        const probe = command.split(';')[1];
        assert.match(probe, /^(cal|shoulder|elbow|wrist_rotation|wrist_bend|gripper_link|gripper)_probe$/);
        const prefix = probe.slice(0, -6);
        const joint = prefix === 'cal' ? 'base' : prefix.replaceAll('_', '-');
        const state = {...ready(joint), last:null, center:null, locked:false};
        if (joint === badJoint) {
          if (malformed) state.channel = 99;
          else state.fault = 'Bus fault';
        }
        input.enqueue(new TextEncoder().encode(`@sync:${fence}-b\n@cal:${JSON.stringify(state)}\n@sync:${fence}-e\n`));
      }});
    },
    async close() {
      assert.equal(this.readable.locked, false); assert.equal(this.writable.locked, false);
      opened = false; this.closed++;
      if (this.closed === abortAfter) stop.abort();
    },
  };
  return port;
}

test('one selected port checks every joint sequentially without initializing any target', async () => {
  const port = wholeArmPort();
  const states = await probeAllJoints(port, new AbortController().signal);
  assert.deepEqual(Object.keys(states), joints);
  assert.equal(port.writes.length, 7); assert.equal(port.closed, 7);
  for (const joint of joints) {
    assert.equal(states[joint].protocol, `miniarm-${joint}-pulse-v1`);
    assert.equal(states[joint].last, null); assert.equal(states[joint].center, null);
    assert.deepEqual(states[joint].outputs, Array(7).fill(null));
  }
});

test('a fault or mismatched joint aborts whole-arm connection instead of returning partial controls', async () => {
  for (const malformed of [false, true]) {
    const port = wholeArmPort({badJoint: 'elbow', malformed});
    await assert.rejects(probeAllJoints(port, new AbortController().signal), malformed ? /Unexpected/ : /fault/);
    assert.equal(port.writes.length, 3); assert.equal(port.closed, 3);
  }
});

test('canceling a whole-arm check releases the port and never probes subsequent joints', async () => {
  const stop = new AbortController(); const port = wholeArmPort({abortAfter:2, stop});
  await assert.rejects(probeAllJoints(port, stop.signal), {name:'AbortError'});
  assert.equal(port.writes.length, 2); assert.equal(port.closed, 2);
});

for (const joint of joints) test(`${joint} slider sends latest target and releases its token on let-go`, async () => {
  const port = portFor(joint); const stop = new AbortController(); let desired = 91;
  const result = await sweepJoint(port, stop.signal, ready(joint), () => desired, status => {
    if (status.sweep_sequence === 0) desired = 94; // supersedes the older target
    else stop.abort();
  });
  assert.equal(result.active, false);
  assert.equal(port.writes.length, 3); assert.equal(port.closed, 3);
  const prefix = joint === 'base' ? 'cal' : joint.replaceAll('-', '_');
  const token = port.writes[0].match(new RegExp(`${prefix}_sweep:([a-f0-9-]+):90`))[1];
  assert.ok(port.writes[1].includes(`${prefix}_target:${token}:1:94;`));
  assert.ok(port.writes[2].includes(`${prefix}_release:${token};`));
});

test('cancel during start or target acknowledgment always attempts release without retry', async () => {
  for (const stage of ['cancelStart', 'cancelTarget']) {
    const stop = new AbortController(); const port = portFor('base', {[stage]:stop});
    const result = await sweepJoint(port, stop.signal, ready(), () => 94, () => {});
    assert.equal(result.active, false);
    assert.ok(port.writes.at(-1).includes('cal_release:'));
    assert.equal(port.writes.length, stage === 'cancelStart' ? 2 : 3);
  }
});

test('bad capability and uninitialized state never open USB', async () => {
  for (const status of [{...ready(), sweep_watchdog_ms:6000}, {...ready(), sweep_rate:100}, {...ready(), sweep_protocol:'old'},
    {...ready(), last:null, center:null}, {...ready(), outputs:[90,...Array(6).fill(null)]}]) {
    const port = portFor();
    await assert.rejects(sweepJoint(port, new AbortController().signal, status, () => 94, () => {}));
    assert.equal(port.writes.length, 0);
  }
  assert.throws(() => validateSweepStatus({...ready(), sweep_target:96}), /capability/);
});

test('bad acknowledgment, wrong output and rejected start fail closed after release', async () => {
  for (const option of ['badSequence', 'badOutput', 'rejectStart']) {
    const port = portFor('base', {[option]:true});
    await assert.rejects(sweepJoint(port, new AbortController().signal, ready(), () => 94, () => {}), /subsequently confirmed off/);
    assert.ok(port.writes.at(-1).includes('cal_release:'));
    assert.ok(port.writes.length <= 3);
  }
});

test('out-of-range targets are never transmitted and release failure is surfaced', async () => {
  for (const target of [NaN, Infinity, -1, 181, 96]) {
    const port = portFor();
    await assert.rejects(sweepJoint(port, new AbortController().signal, ready(), () => target, () => {}), /outside/);
    assert.equal(port.writes.length, 2); assert.ok(port.writes[1].includes('cal_release:'));
  }
  const stop = new AbortController(); const port = portFor('base', {failRelease:true});
  await assert.rejects(sweepJoint(port, stop.signal, ready(), () => 94, () => stop.abort()), /release was not confirmed/);
});

test('browser ends a sustained drag without renewing its absolute limit', async () => {
  const port = portFor(); const before = performance.now();
  await sweepJoint(port, new AbortController().signal, ready(), () => 94, () => {});
  assert.ok(performance.now() - before >= 2700);
  assert.equal(port.writes.filter(v => v.includes('cal_sweep:')).length, 1);
  assert.ok(port.writes.at(-1).includes('cal_release:'));
  assert.ok(port.writes.length < 35);
});
