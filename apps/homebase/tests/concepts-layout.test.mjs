import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { createRequire } from 'node:module';
import { mkdir, mkdtemp, writeFile, unlink, rmdir } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';

test('Five concept layouts render sample controls without loading device integrations', async () => {
  const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
  const bundle = await build({
    absWorkingDir: root,
    entryPoints: ['app/concepts/page.tsx'],
    bundle: true,
    write: false,
    metafile: true,
    platform: 'node',
    format: 'cjs',
    packages: 'external',
    alias: { '@': root },
    jsx: 'automatic',
    logLevel: 'silent',
    plugins: [
      {
        name: 'concept-render',
        setup(b) {
          b.onResolve({ filter: /^next\/link$/ }, () => ({
            path: 'link',
            namespace: 'fixture',
          }));
          b.onLoad({ filter: /.*/, namespace: 'fixture' }, () => ({
            contents: "export default 'a';",
            loader: 'js',
          }));
          b.onLoad({ filter: /\.css$/ }, () => ({
            contents: '',
            loader: 'js',
          }));
        },
      },
    ],
  });
  for (const input of Object.keys(bundle.metafile.inputs)) {
    assert.ok(
      !/lib\/(gateway|serial|camera|settings-store)/.test(input),
      input,
    );
  }
  const cache = path.join(root, 'node_modules', '.cache');
  await mkdir(cache, { recursive: true });
  const directory = await mkdtemp(path.join(cache, 'concepts-layout-'));
  const filename = path.join(directory, 'render.cjs');
  const require = createRequire(import.meta.url);
  const noop = () => {};
  const demo = {
    room: 'Living room',
    shade: 65,
    routine: true,
    alert: true,
    notice: '',
    setRoom: noop,
    setShade: noop,
    setRoutine: noop,
    dismiss: noop,
    action: noop,
  };
  try {
    await writeFile(filename, bundle.outputFiles[0].text);
    const { default: Page, ConceptCanvas, IonConcept } = require(filename);
    const gallery = renderToStaticMarkup(React.createElement(Page));
    for (const name of [
      'Violet',
      'Daylight',
      'Relay',
      'Atlas',
      'Ion',
      'no device commands',
    ])
      assert.ok(gallery.includes(name), name);
    for (const id of ['violet', 'daylight', 'relay', 'atlas', 'ion']) {
      const html = renderToStaticMarkup(
        React.createElement(ConceptCanvas, { id, demo }),
      );
      assert.ok(html.includes('Sample shade opening'), id);
      assert.ok(html.includes('value="65"'), id);
      assert.ok(html.includes('aria-checked="true"'), id);
      assert.ok(html.includes('Dismiss sample connection alert'), id);
      assert.ok(
        !/<(?:img|video|iframe)\b/.test(html),
        'No live camera elements',
      );
      const changed = renderToStaticMarkup(
        React.createElement(ConceptCanvas, {
          id,
          demo: {
            ...demo,
            shade: 20,
            routine: false,
            alert: false,
            room: 'Kitchen',
          },
        }),
      );
      assert.ok(changed.includes('value="20"'), id);
      assert.ok(changed.includes('aria-checked="false"'), id);
      assert.ok(changed.includes('Sample alert dismissed'), id);
      assert.ok(changed.includes('Kitchen'), id);
    }
    const simple = renderToStaticMarkup(
      React.createElement(IonConcept, { demo }),
    );
    const detailed = renderToStaticMarkup(
      React.createElement(IonConcept, { demo, initialDetail: true }),
    );
    for (const html of [simple, detailed]) {
      for (const label of [
        'Interface detail',
        'Motion on',
        'Sample shade presets',
        'Design references',
        'stroke-dasharray="65 100"',
      ])
        assert.ok(html.includes(label), label);
    }
    assert.ok(!simple.includes('Connection timing'));
    for (const label of [
      'Connection timing',
      'Home gateway',
      'Recent activity',
      'no active monitoring',
    ])
      assert.ok(detailed.includes(label), label);
  } finally {
    delete require.cache[filename];
    await unlink(filename);
    await rmdir(directory);
  }
});
