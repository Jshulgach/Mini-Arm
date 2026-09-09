/** Owns only this preview's camera tracks, including late permission responses. */
export class WebcamSession {
  private generation = 0;
  private stream: MediaStream | null = null;

  async start(acquire: () => Promise<MediaStream>): Promise<MediaStream | null> {
    this.stop();
    const generation = this.generation;
    let stream: MediaStream;
    try { stream = await acquire(); }
    catch (error) { if (generation !== this.generation) return null; throw error; }
    if (generation !== this.generation) {
      stream.getTracks().forEach(track => track.stop());
      return null;
    }
    this.stream = stream;
    return stream;
  }

  isCurrent(stream: MediaStream) { return this.stream === stream; }

  stop() {
    this.generation += 1;
    const stream = this.stream;
    this.stream = null;
    stream?.getTracks().forEach(track => track.stop());
  }
}

export function webcamConstraints(deviceId: string, resolution: string): MediaStreamConstraints {
  const sizes: Record<string, [number, number]> = { '480': [640, 480], '720': [1280, 720], '1080': [1920, 1080] };
  const size = sizes[resolution] ?? sizes['720'];
  return {
    audio: false,
    video: {
      ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
      width: { ideal: size[0] }, height: { ideal: size[1] }, frameRate: { ideal: 30 },
    },
  };
}

export function webcamError(error: unknown): string {
  const name = error && typeof error === 'object' && 'name' in error ? error.name : '';
  if (name === 'NotAllowedError' || name === 'SecurityError')
    return 'Camera access was blocked. Allow camera access for this page in your browser or Windows privacy settings, then try again.';
  if (name === 'NotFoundError') return 'No webcam was found. Connect a camera and refresh the camera list.';
  if (name === 'NotReadableError') return 'The webcam could not be opened. Close other apps using it and try again.';
  if (name === 'OverconstrainedError') return 'That camera is unavailable or cannot use these settings. Choose the default camera or a lower resolution.';
  return 'The webcam preview could not start. Check its connection and browser permissions, then try again.';
}
