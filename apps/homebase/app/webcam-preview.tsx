 'use client';
import { useCallback, useEffect, useRef, useState, useSyncExternalStore } from 'react';
import { Button } from '@/components/ui/button';
import { WebcamSession, webcamConstraints, webcamError } from '@/lib/webcam';

const subscribeToCapabilities = () => () => {};
const webcamCapability = () => window.isSecureContext && typeof navigator.mediaDevices?.getUserMedia === 'function' ? 'supported' : 'unsupported';
const serverCapability = () => 'loading';
const unsupportedMessage = 'This browser does not expose webcam access here. Open Homebase at http://127.0.0.1:5173 in Chrome or Edge on this PC.';

export function WebcamPreview({ compact = false }: { compact?: boolean }) {
  const video = useRef<HTMLVideoElement>(null);
  const [session] = useState(() => new WebcamSession());
  const [cameras, setCameras] = useState<MediaDeviceInfo[]>([]);
  const [deviceId, setDeviceId] = useState('');
  const [resolution, setResolution] = useState('720');
  const [mirror, setMirror] = useState(false);
  const [phase, setPhase] = useState<'stopped' | 'starting' | 'playing'>('stopped');
  const capability = useSyncExternalStore(subscribeToCapabilities, webcamCapability, serverCapability);
  const available = capability === 'supported';
  const [error, setError] = useState('');
  const [details, setDetails] = useState('');
  const mounted = useRef(false);

  const refresh = useCallback(async () => {
    try {
      const devices = await navigator.mediaDevices.enumerateDevices();
      if (mounted.current) setCameras(devices.filter(device => device.kind === 'videoinput'));
    } catch { /* Device labels may remain private until permission is granted. */ }
  }, []);

  const stop = useCallback(() => {
    session.stop();
    if (video.current) { video.current.pause(); video.current.srcObject = null; }
    setPhase('stopped'); setDetails('');
  }, [session]);

  useEffect(() => {
    mounted.current = true;
    const media = navigator.mediaDevices;
    media?.enumerateDevices().then(devices => {
      if (mounted.current) setCameras(devices.filter(device => device.kind === 'videoinput'));
    }).catch(() => { /* Permission may hide device labels. */ });
    const hidden = () => { if (document.hidden) stop(); };
    const leaving = () => stop();
    document.addEventListener('visibilitychange', hidden);
    window.addEventListener('pagehide', leaving);
    media?.addEventListener('devicechange', refresh);
    return () => {
      mounted.current = false;
      session.stop();
      document.removeEventListener('visibilitychange', hidden);
      window.removeEventListener('pagehide', leaving);
      media?.removeEventListener('devicechange', refresh);
    };
  }, [refresh, session, stop]);

  async function start() {
    setError(''); setPhase('starting'); setDetails('');
    let stream: MediaStream | null = null;
    try {
      stream = await session.start(() => navigator.mediaDevices.getUserMedia(webcamConstraints(deviceId, resolution)));
      if (!stream) return;
      if (!mounted.current || document.hidden || !video.current) { session.stop(); if (mounted.current) setPhase('stopped'); return; }
      const track = stream.getVideoTracks()[0];
      if (!track || track.readyState === 'ended') throw new Error('No active video track');
      const current = stream;
      track.addEventListener('ended', () => {
        if (mounted.current && session.isCurrent(current)) {
          stop(); setError('The webcam disconnected or camera access ended. Reconnect it and start again.');
        }
      }, { once: true });
      video.current.srcObject = stream;
      await video.current.play();
      if (!session.isCurrent(stream) || !mounted.current) return;
      const settings = track.getSettings();
      setDetails(`${track.label || 'Webcam'} · ${settings.width ?? '?'} × ${settings.height ?? '?'}${settings.frameRate ? ` · ${Math.round(settings.frameRate)} fps` : ''}`);
      setPhase('playing');
      void refresh();
    } catch (failure) {
      if (!mounted.current || (stream && !session.isCurrent(stream))) return;
      stop(); setError(webcamError(failure));
    }
  }

  const active = phase !== 'stopped';
  return <section className="camera-preview webcam-preview" aria-label="PC webcam preview">
    <h3>PC webcam</h3>
    <div className="camera-surface">
      <video ref={video} autoPlay playsInline muted hidden={phase !== 'playing'}
        className={mirror ? 'webcam-mirrored' : ''} aria-label="Live local webcam video" />
      {phase !== 'playing' && <p>{phase === 'starting' ? 'Waiting for camera access…' : 'Start your webcam to see its view here.'}</p>}
    </div>
    <p className="camera-state" aria-live="polite">{phase === 'playing' ? 'Live webcam preview' : phase === 'starting' ? 'Starting · check the browser permission prompt' : 'Webcam stopped'}
      {details && <small>{details}</small>}
    </p>
    <div className="camera-controls">
      <Button disabled={!available} onClick={active ? stop : () => void start()}>{phase === 'starting' ? 'Cancel' : active ? 'Stop webcam' : 'Start webcam'}</Button>
    </div>
    <details className="camera-options" open={!compact}>
    <summary>Webcam settings</summary>
    <div className="webcam-options">
      <label htmlFor="webcam-device">Camera</label>
      <select id="webcam-device" value={deviceId} disabled={active} onChange={event => setDeviceId(event.target.value)}>
        <option value="">Default camera</option>
        {cameras.filter(camera => camera.deviceId).map((camera, index) => <option key={camera.deviceId} value={camera.deviceId}>{camera.label || `Camera ${index + 1}`}</option>)}
      </select>
      <label htmlFor="webcam-resolution">Preferred resolution</label>
      <select id="webcam-resolution" value={resolution} disabled={active} onChange={event => setResolution(event.target.value)}>
        <option value="480">640 × 480</option><option value="720">1280 × 720</option><option value="1080">1920 × 1080</option>
      </select>
      <label className="webcam-mirror"><input type="checkbox" checked={mirror} onChange={event => setMirror(event.target.checked)} />Mirror preview</label>
    </div>
    <div className="camera-controls">
      <Button variant="outline" disabled={!available || active} onClick={() => void refresh()}>Refresh cameras</Button>
    </div>
    <p className="field-help">Stop the webcam before changing its camera or resolution. Hiding the tab stops the webcam.</p>
    </details>
    {(error || capability === 'unsupported') && <p className="form-error" role="alert">{error || unsupportedMessage}</p>}
    <p className="field-help">Browser-only video · no audio or recording.</p>
  </section>;
}
