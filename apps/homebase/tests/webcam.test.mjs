import test from 'node:test';
import assert from 'node:assert/strict';
import { WebcamSession, webcamConstraints, webcamError } from '../lib/webcam.ts';

function fakeStream() {
  const tracks = [{ stopped: 0, stop() { this.stopped++; } }, { stopped: 0, stop() { this.stopped++; } }];
  return { getTracks: () => tracks, tracks };
}
function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

test('stop after opening releases every track and is idempotent', async () => {
  const session = new WebcamSession(); const stream = fakeStream();
  assert.equal(await session.start(async () => stream), stream);
  session.stop(); session.stop();
  assert.deepEqual(stream.tracks.map(t => t.stopped), [1, 1]);
  assert.equal(session.isCurrent(stream), false);
});
test('cancel or unmount during a permission prompt stops a late stream', async () => {
  const session = new WebcamSession(); const pending = deferred(); const stream = fakeStream();
  const start = session.start(() => pending.promise);
  session.stop(); pending.resolve(stream);
  assert.equal(await start, null);
  assert.deepEqual(stream.tracks.map(t => t.stopped), [1, 1]);
});
test('a slow old camera request cannot replace a newer stream', async () => {
  const session = new WebcamSession(); const pending = deferred(); const old = fakeStream(); const current = fakeStream();
  const first = session.start(() => pending.promise);
  await session.start(async () => current); pending.resolve(old);
  assert.equal(await first, null);
  assert.equal(session.isCurrent(current), true);
  assert.deepEqual(old.tracks.map(t => t.stopped), [1, 1]);
  assert.deepEqual(current.tracks.map(t => t.stopped), [0, 0]);
  session.stop();
});
test('superseded permission errors are ignored and active errors propagate', async () => {
  const session = new WebcamSession(); const pending = deferred();
  const first = session.start(() => pending.promise); session.stop(); pending.reject(new Error('cancelled'));
  assert.equal(await first, null);
  await assert.rejects(session.start(async () => { throw new Error('denied'); }), /denied/);
});
test('switching cameras releases the old stream before acquiring another', async () => {
  const session = new WebcamSession(); const old = fakeStream(); const next = fakeStream();
  await session.start(async () => old);
  await session.start(async () => { assert.deepEqual(old.tracks.map(t => t.stopped), [1, 1]); return next; });
  session.stop();
});
test('capture requests video only and selects a specific camera explicitly', () => {
  const constraints = webcamConstraints('camera-id', '1080');
  assert.equal(constraints.audio, false);
  assert.deepEqual(constraints.video.deviceId, { exact: 'camera-id' });
  assert.equal(constraints.video.width.ideal, 1920);
  assert.equal(webcamConstraints('', '720').video.deviceId, undefined);
  assert.match(webcamError({ name: 'NotAllowedError' }), /Allow camera access/);
});
