import { createServer } from 'node:http';
import { readFile, realpath, stat } from 'node:fs/promises';
import { extname, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createGateway, handleGateway } from './gateway.mjs';
import { createAssistant } from './assistant.mjs';
import { existsSync } from 'node:fs';

const envFile=fileURLToPath(new URL('../.env.local',import.meta.url));
if(existsSync(envFile))process.loadEnvFile(envFile);
const assistant=createAssistant();

const gateway = createGateway(resolve(process.env.HOMEBASE_DATA_DIR ?? fileURLToPath(new URL('../data/', import.meta.url)), 'homebase.sqlite'));
let polling = false;
const monitor = setInterval(async () => {
  if (polling) return;
  polling = true;
  try { await gateway.poll(); } catch(error) { console.error('Monitoring failed:', error.message); }
  finally { polling = false; }
}, 30000);

const root = await realpath(
  fileURLToPath(new URL('../dist/client/', import.meta.url)),
).catch(() => {
  console.error('Build Homebase first with npm run build.');
  process.exit(1);
});
const mime = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.json': 'application/json',
  '.txt': 'text/plain; charset=utf-8',
  '.woff2': 'font/woff2',
  '.ico': 'image/x-icon',
};
const server = createServer(async (req, res) => {
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('Cache-Control', 'no-cache');
  if (await handleGateway(req, res, gateway, assistant)) return;
  if (!['GET', 'HEAD'].includes(req.method ?? '')) {
    res.writeHead(405, { Allow: 'GET, HEAD' });
    res.end();
    return;
  }
  try {
    const pathname = decodeURIComponent(
      new URL(req.url, 'http://localhost').pathname,
    );
    // Reject Windows path syntax, dotfiles and traversal; follow no path outside the export.
    if (
      pathname.includes('\\') ||
      pathname.includes(':') ||
      pathname.includes('\0') ||
      pathname.split('/').some((p) => p.startsWith('.'))
    ) {
      res.writeHead(404);
      res.end();
      return;
    }
    const relative = pathname === '/' ? '/index.html' : pathname.replace(/\/$/, '');
    const candidates = [relative];
    if (!extname(relative)) candidates.push(`${relative}.html`);
    let target;
    for (const candidate of candidates) {
      const resolved = await realpath(resolve(root, `.${candidate}`)).catch(() => null);
      if (resolved?.startsWith(root + sep) && (await stat(resolved)).isFile()) {
        target = resolved;
        break;
      }
    }
    if (!target) {
      res.writeHead(404);
      res.end();
      return;
    }
    const body = await readFile(target);
    res.writeHead(200, {
      'Content-Type': mime[extname(target)] ?? 'application/octet-stream',
      'Content-Length': body.length,
    });
    res.end(req.method === 'HEAD' ? undefined : body);
  } catch {
    res.writeHead(404);
    res.end();
  }
});
const port = Number(process.env.PORT ?? 5173);
if (!Number.isInteger(port) || port < 1 || port > 65535)
  throw new Error('PORT must be between 1 and 65535.');
server.on('error', (error) => {
  console.error(error.message);
  clearInterval(monitor);
  void gateway.close();
  process.exitCode = 1;
});
const listenHost = process.env.HOMEBASE_LISTEN_HOST ?? '127.0.0.1';
if (!['127.0.0.1', '0.0.0.0'].includes(listenHost)) throw new Error('Unsupported listen host.');
server.listen(port, listenHost, () =>
  console.log(`Homebase: http://127.0.0.1:${port}/`),
);
for (const signal of ['SIGINT', 'SIGTERM'])
  process.on(signal, () => { clearInterval(monitor); server.close(() => void gateway.close()); });
