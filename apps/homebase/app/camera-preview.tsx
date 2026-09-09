 'use client';
import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { cameraUrls } from '@/lib/camera';

export function CameraPreview({ address, compact = false }: { address: string; compact?: boolean }) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [running, setRunning] = useState(false);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [fps, setFps] = useState(3);
  const [lastFrame, setLastFrame] = useState<number | null>(null);
  const [error, setError] = useState('');
  let urls: ReturnType<typeof cameraUrls> | undefined;
  let configError = '';
  try { if (address) urls = cameraUrls(address); }
  catch (err) { configError = err instanceof Error ? err.message : 'Invalid camera address'; }
  const snapshot = urls?.snapshot;

  useEffect(() => {
    const pause = () => { if (document.hidden) setRunning(false); };
    document.addEventListener('visibilitychange', pause);
    return () => document.removeEventListener('visibilitychange', pause);
  }, []);

  useEffect(() => {
    if (!running || !snapshot) return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout>;
    let watchdog: ReturnType<typeof setTimeout>;
    let pending: HTMLImageElement | undefined;
    const fail = () => {
      if (cancelled) return;
      clearTimeout(watchdog);
      setError('No fresh frame. Check camera power, Wi-Fi and address; allow local-network access if your browser asks.');
      setRunning(false);
    };
    const next = () => {
      if (cancelled) return;
      const frame = new Image();
      pending = frame;
      frame.referrerPolicy = 'no-referrer';
      frame.onload = () => {
        if (cancelled) return;
        clearTimeout(watchdog);
        const target = canvas.current;
        const context = target?.getContext('2d');
        if (!target || !context) { fail(); return; }
        target.width = frame.naturalWidth;
        target.height = frame.naturalHeight;
        context.drawImage(frame, 0, 0);
        setLastFrame(Date.now());
        setError('');
        timer = setTimeout(next, 1000 / fps);
      };
      frame.onerror = fail;
      const url = new URL(snapshot);
      url.searchParams.set('t', String(Date.now()));
      watchdog = setTimeout(fail, 4000);
      frame.src = url.href;
    };
    next();
    return () => {
      cancelled = true;
      clearTimeout(timer); clearTimeout(watchdog);
      if (pending) { pending.onload = null; pending.onerror = null; pending.src = ''; }
    };
  }, [running, snapshot, fps]);

  return <section className="camera-preview" aria-label="Gripper camera preview">
    <h3>Gripper view</h3>
    <div className="camera-surface">
      <canvas ref={canvas} aria-label="Most recently received gripper camera image" hidden={!lastFrame} />
      {!lastFrame && <p>{running ? 'Waiting for a camera frame…' : 'Your camera view will appear here.'}</p>}
    </div>
    <p className="camera-state" aria-live="polite">
      {error ? 'Feed unavailable · displayed image may be stale' : running ? (lastFrame ? 'Refreshing camera frames' : 'Connecting…') : 'Preview paused'}
      {lastFrame && <small>Last frame received {new Date(lastFrame).toLocaleTimeString()}</small>}
    </p>
    <div className="camera-controls">
      <Button disabled={!urls} onClick={() => { setError(''); setRunning(!running); }}>
        {running ? 'Pause preview' : error ? 'Retry preview' : 'Start preview'}
      </Button>
    </div>
    {!address && <p>Set the camera address in Configure, then start the preview.</p>}
    {(error || configError) && <p className="form-error" role="alert">{error || configError}</p>}
    <details className="camera-options" open={!compact}>
    <summary>Gripper camera settings</summary>
    <div className="camera-controls"><label>Refresh rate <select value={fps} onChange={e => setFps(Number(e.target.value))}>
      <option value={1}>Up to 1 fps</option><option value={3}>Up to 3 fps</option><option value={5}>Up to 5 fps</option>
    </select></label></div>
    {urls && <p><a href={urls.page} target="_blank" rel="noreferrer">Camera page</a> · <a href={urls.stream} target="_blank" rel="noreferrer">Continuous MJPEG stream</a></p>}
    {urls && <div className="camera-settings-panel">
      <Button variant="outline" onClick={() => setSettingsOpen(!settingsOpen)} aria-expanded={settingsOpen}>
        {settingsOpen ? 'Close image settings' : 'Image settings'}
      </Button>
      {settingsOpen && <>
        <p className="field-help">Adjust brightness, exposure and color. Changes last until the camera restarts.</p>
        <iframe title="Gripper camera image settings" src={new URL('/settings', urls.page).href}
          className="camera-settings-frame" referrerPolicy="no-referrer" />
        <a href={new URL('/settings', urls.page).href} target="_blank" rel="noreferrer">Open settings on the camera</a>
      </>}
    </div>}
    <p className="field-help">Images load directly from the camera. Hiding the tab pauses the preview.</p>
    </details>
  </section>;
}
