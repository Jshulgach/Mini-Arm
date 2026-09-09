export function cameraUrls(address: string) {
  const url = new URL(address);
  if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash)
    throw new Error('Use a camera HTTP address without credentials or query parameters.');
  if (!['/', '/capture', '/stream'].includes(url.pathname))
    throw new Error('Use the camera base address, /capture, or /stream endpoint.');
  const stream = new URL(url);
  const base = new URL(url);
  if (url.pathname === '/stream' && url.port === '81') base.port = '80';
  base.pathname = '/';
  if (url.pathname !== '/stream') stream.port = '81';
  stream.pathname = '/stream';
  return { snapshot: new URL('/capture', base).href, stream: stream.href, page: base.href };
}
