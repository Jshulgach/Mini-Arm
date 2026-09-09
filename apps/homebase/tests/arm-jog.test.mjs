import test from 'node:test';
import assert from 'node:assert/strict';
import { jogBounds, parseJogTarget, formatJogAngle } from '../lib/arm-jog.ts';

test('blank or malformed target drafts never become zero-degree commands', () => {
  for (const value of [undefined, '', ' ', 'NaN', 'Infinity', '-Infinity', '1e999', 'no', '90 degrees']) {
    assert.equal(parseJogTarget(value), null);
  }
  assert.equal(parseJogTarget('0'), 0);
  assert.equal(parseJogTarget('90.5'), 90.5);
});

test('target inputs use controller window clipped to the servo range, without clamping drafts', () => {
  assert.deepEqual(jogBounds(), {min:0, max:180});
  for (const [center, min, max] of [[90,85,95], [0,0,5], [180,175,180], [2,0,7], [178,173,180]]) {
    const status = {center};
    assert.deepEqual(jogBounds(status), {min, max});
    assert.equal(parseJogTarget(String(min), status), min);
    assert.equal(parseJogTarget(String(max), status), max);
    assert.equal(parseJogTarget(String(min - .1), status), null);
    assert.equal(parseJogTarget(String(max + .1), status), null);
  }
});

test('command formatting preserves sub-degree values without long floating-point strings', () => {
  assert.equal(formatJogAngle(90.123456), '90.123');
  assert.equal(formatJogAngle(0), '0');
  assert.equal(formatJogAngle(180), '180');
});
