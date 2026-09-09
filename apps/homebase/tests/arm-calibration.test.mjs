import test from 'node:test';
import assert from 'node:assert/strict';
import { newCalibration, validateCalibration, exportCalibration, recoveredJoints } from '../lib/arm-calibration.ts';

test('recovered wiring does not imply measured calibration', () => {
  const value = newCalibration();
  assert.deepEqual(value.joints.map(j => j.channel), [15, 14, 13, 12, 2, 1, 0]);
  for (const joint of value.joints) {
    assert.equal(joint.neutral, null); assert.equal(joint.minimum, null);
    assert.equal(joint.maximum, null); assert.equal(joint.mappingChecked, false);
    assert.equal(joint.direction, null);
  }
  assert.deepEqual(validateCalibration(value), value);
  assert.equal(recoveredJoints[2].neutral, 45);
});
test('rejects duplicate channels, incomplete joints and malformed imported values', () => {
  for (const mutate of [
    v => v.joints[0].channel = 14,
    v => v.joints[0].channel = 1.5,
    v => v.joints[0].channel = 16,
    v => v.joints[0].neutral = NaN,
    v => v.joints[0].neutral = '90',
    v => v.joints[0].minimum = -1,
    v => v.joints[0].maximum = Infinity,
    v => v.joints[0].direction = 0,
    v => v.joints[0].mappingChecked = 'yes',
    v => v.joints[0].notes = 'x'.repeat(1001),
    v => delete v.joints[0].minimum,
    v => v.joints[0].id = 'shoulder',
    v => v.joints.pop(),
  ]) {
    const v = newCalibration(); mutate(v); assert.throws(() => validateCalibration(v));
  }
});
test('validates ordered measured limits and neutral without requiring invented measurements', () => {
  const v = newCalibration(); const joint = v.joints[0];
  joint.minimum = 30; joint.maximum = 120; joint.neutral = 90;
  assert.deepEqual(validateCalibration(v), v);
  joint.neutral = 10; assert.throws(() => validateCalibration(v), /neutral/);
  joint.neutral = null; assert.doesNotThrow(() => validateCalibration(v));
  joint.maximum = 30; assert.throws(() => validateCalibration(v), /minimum/);
});
test('export/import preserves manual notes, strips unknown configuration and labels its limits', () => {
  const v = newCalibration(); v.joints[0].notes = 'Observed from above';
  v.joints[0].mappingChecked = true; v.execute = 'home';
  const exported = JSON.parse(exportCalibration(v));
  assert.match(exported.purpose, /Not applied to firmware/);
  assert.equal(exported.execute, undefined);
  assert.deepEqual(validateCalibration(exported), { version: 1, joints: v.joints });
});
