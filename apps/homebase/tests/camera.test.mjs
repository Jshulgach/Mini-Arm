import test from 'node:test';
import assert from 'node:assert/strict';
import { cameraUrls } from '../lib/camera.ts';

test('legacy MJPEG URL also resolves the camera snapshot port', () => {
  const urls = cameraUrls('http://192.168.1.185:81/stream');
  assert.equal(urls.snapshot, 'http://192.168.1.185/capture');
  assert.equal(urls.stream, 'http://192.168.1.185:81/stream');
});
test('camera endpoint rejects credentials and executable schemes', () => {
  for (const address of ['javascript:alert(1)', 'file:///a', 'http://u:p@camera/', 'http://camera/?token=x', 'http://camera/control'])
    assert.throws(() => cameraUrls(address));
});
