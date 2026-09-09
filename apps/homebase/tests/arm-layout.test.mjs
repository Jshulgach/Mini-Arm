import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { createRequire } from 'node:module';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { mkdir, mkdtemp, writeFile, unlink, rmdir } from 'node:fs/promises';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';

test('disconnected motor table renders all seven labelled slider/input pairs and the global controls', async () => {
  const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
  const bundle = await build({absWorkingDir:root, entryPoints:['app/arm/base-test.tsx'], bundle:true,
    write:false, platform:'node', format:'cjs', packages:'external', alias:{'@':root}, jsx:'automatic', logLevel:'silent'});
  // Execute the real component's server render with installed React. No browser,
  // serial device, camera, or fake motion service is used by this layout check.
  const cache = path.join(root, 'node_modules', '.cache');
  await mkdir(cache, {recursive:true});
  const directory = await mkdtemp(path.join(cache, 'arm-layout-'));
  const filename = path.join(directory, 'render.cjs');
  const localRequire = createRequire(import.meta.url);
  const channels = {base:15, shoulder:14, elbow:13, 'wrist-rotation':12, 'wrist-bend':2, 'gripper-link':1, gripper:0};
  let html;
  try {
    await writeFile(filename, bundle.outputFiles[0].text);
    const { BaseTest } = localRequire(filename);
    html = renderToStaticMarkup(React.createElement(BaseTest, {channels}));
  } finally {
    delete localRequire.cache[filename];
    await unlink(filename); await rmdir(directory);
  }
  const ranges = html.match(/<input(?=[^>]*type="range")[^>]*>/g) ?? [];
  const numbers = html.match(/<input(?=[^>]*type="number")[^>]*>/g) ?? [];
  assert.equal(ranges.length, 7); assert.equal(numbers.length, 7);
  assert.ok([...ranges, ...numbers].every(tag => tag.includes('disabled=""')));
  for (const joint of Object.keys(channels)) assert.ok(html.includes(`id="jog-${joint}"`) && html.includes(`for="jog-${joint}"`));
  assert.equal((html.match(/>Connect</g) ?? []).length, 1);
  assert.equal((html.match(/>Release</g) ?? []).length, 0);
  assert.ok(!html.includes('Reconnect arm'));
  assert.equal((html.match(/>Home</g) ?? []).length, 1);
  assert.ok(html.includes('role="switch"') && html.includes('aria-checked="false"'));
  assert.equal((html.match(/<th scope="row"/g) ?? []).length, 7);
  assert.ok(html.includes('Last command °') && html.includes('Target °'));
  assert.equal((html.match(/Reference &amp; manual tests/g) ?? []).length, 0);
  assert.ok(!html.includes('NaN') && !html.includes('undefined'));
});
