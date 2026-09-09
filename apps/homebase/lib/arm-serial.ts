// Narrow Web Serial surface. Motion commands are built only by typed wrappers.
export interface ArmSerialPort {
  getInfo(): { usbVendorId?: number; usbProductId?: number };
  open(options: { baudRate: number; bufferSize: number }): Promise<void>;
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
}
export interface ArmSerialAPI {
  requestPort(options: { filters: { usbVendorId: number; usbProductId: number }[] }): Promise<ArmSerialPort>;
  addEventListener?(type: 'disconnect', listener: (event: Event) => void): void;
  removeEventListener?(type: 'disconnect', listener: (event: Event) => void): void;
}
export const picoFilter = { usbVendorId: 0x239a, usbProductId: 0x8162 };
export function browserSerial(): ArmSerialAPI | undefined {
  return (navigator as Navigator & { serial?: ArmSerialAPI }).serial;
}
export type ArmCheck = { checkedAt: string; elapsedMs: number; jointCommands: number[]; servoCommands: (number | null)[]; transcript: string };

export function parseArmReport(transcript: string): Pick<ArmCheck, 'jointCommands' | 'servoCommands' | 'transcript'> {
  if (!transcript.includes('Testing 123') || !transcript.includes('Robot Info') || !/Simulated hardware:\s*False\b/.test(transcript)) {
    throw new Error('The selected port did not return the expected physical Mini-Arm controller report.');
  }
  const parse = (label: string, nullable: boolean) => {
    const match = transcript.match(new RegExp(`${label}:\\s*\\[([^\\]]*)\\]`));
    if (!match) throw new Error(`Missing ${label} in the controller response.`);
    const fields = match[1].split(',').map(v => v.trim());
    if (fields.length !== 6) throw new Error(`Expected six values for ${label}.`);
    return fields.map(v => {
      if (nullable && v === 'None') return null;
      if (!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/.test(v) || !Number.isFinite(Number(v))) throw new Error(`Invalid ${label} value.`);
      return Number(v);
    });
  };
  return { jointCommands: parse('Joint states', false) as number[], servoCommands: parse('Servo readings', true), transcript };
}

/** One explicitly requested check. Always releases the port afterward. */
export async function checkArmConnection(port: ArmSerialPort, signal: AbortSignal, timeoutMs = 3500): Promise<ArmCheck> {
  const result = await exchange(port, signal, 'test;info', timeoutMs);
  return { ...parseArmReport(result.transcript), checkedAt: result.checkedAt, elapsedMs: result.elapsedMs };
}

// Commands are constructed only by the typed wrappers in this module.
async function exchange(port: ArmSerialPort, signal: AbortSignal, command: string, timeoutMs: number) {
  signal.throwIfAborted();
  const info = port.getInfo();
  if (info.usbVendorId !== picoFilter.usbVendorId || info.usbProductId !== picoFilter.usbProductId) throw new Error('Select the CircuitPython Pico 2 W USB port (previously COM3).');
  let reader: ReadableStreamDefaultReader<Uint8Array> | undefined;
  let writer: WritableStreamDefaultWriter<Uint8Array> | undefined;
  let timer: ReturnType<typeof setTimeout> | undefined;
  let opened = false;
  let exchanged = false;
  let failure: Error | undefined;
  const stop = (reason: Error) => {
    failure ??= reason;
    void reader?.cancel().catch(() => {});
    void writer?.abort(reason).catch(() => {});
  };
  const cancel = () => stop(new DOMException('Connection check canceled.', 'AbortError'));
  const started = performance.now();
  try {
    await port.open({ baudRate: 115200, bufferSize: 4096 });
    opened = true;
    signal.throwIfAborted();
    if (!port.readable || !port.writable) throw new Error('This port has no readable/writable serial stream.');
    reader = port.readable.getReader(); writer = port.writable.getWriter();
    signal.addEventListener('abort', cancel, { once: true });
    timer = setTimeout(() => stop(new Error('No complete Mini-Arm reply before timeout. Close other serial monitors and check that firmware 1.4.10 or newer is running.')), timeoutMs);
    const token = crypto.randomUUID();
    const begin = `@sync:${token}-b`, end = `@sync:${token}-e`;
    const commands = `sync:${token}-b;${command};sync:${token}-e;`;
    const activeReader = reader;
    const collect = async () => {
      const decoder = new TextDecoder();
      let buffer = '', transcript = '', begun = false, received = 0;
      while (true) {
        const chunk = await activeReader.read();
        if (failure) throw failure;
        if (chunk.done) throw new Error('The USB port closed before the complete response arrived.');
        received += chunk.value.byteLength;
        if (received > 32768) throw new Error('Controller response exceeded the 32 KB limit.');
        buffer += decoder.decode(chunk.value, { stream: true });
        let newline: number;
        while ((newline = buffer.indexOf('\n')) >= 0) {
          const raw = buffer.slice(0, newline); buffer = buffer.slice(newline + 1);
          // CircuitPython may emit terminal status escapes; ignore formatting only.
          // eslint-disable-next-line no-control-regex -- ANSI/OSC sequences contain literal ESC and BEL bytes.
          const line = raw.replace(/\x1b\][^\x07]*(?:\x07|\x1b\\)/g, '').replace(/\x1b\[[0-?]*[ -/]*[@-~]/g, '').replace(/\r/g, '').trim();
          if (line === begin) { begun = true; transcript = ''; }
          else if (begun && line === end) return transcript;
          else if (begun) transcript += `${line}\n`;
        }
      }
    };
    const [, transcript] = await Promise.all([writer.write(new TextEncoder().encode(commands)), collect()]);
    exchanged = true;
    if (failure) throw failure;
    return { transcript, checkedAt: new Date().toISOString(), elapsedMs: Math.round(performance.now() - started) };
  } catch (error) {
    throw failure ?? error;
  } finally {
    clearTimeout(timer);
    signal.removeEventListener('abort', cancel);
    await reader?.cancel().catch(() => {});
    reader?.releaseLock();
    // Abort releases pending writes after a timeout or disconnect.
    if (!exchanged) await writer?.abort().catch(() => {});
    writer?.releaseLock();
    if (opened) await port.close();
  }
}

const testSpecs = {
  base: { channel: 15, index: 0, prefix: 'cal', protocol: 'miniarm-base-pulse-v1', hold: 'miniarm-base-hold-v1' },
  shoulder: { channel: 14, index: 1, prefix: 'shoulder', protocol: 'miniarm-shoulder-pulse-v1', hold: 'miniarm-shoulder-hold-v1' },
  elbow: { channel: 13, index: 2, prefix: 'elbow', protocol: 'miniarm-elbow-pulse-v1', hold: 'miniarm-elbow-hold-v1' },
  'wrist-rotation': { channel: 12, index: 3, prefix: 'wrist_rotation', protocol: 'miniarm-wrist-rotation-pulse-v1', hold: 'miniarm-wrist-rotation-hold-v1' },
  'wrist-bend': { channel: 2, index: 4, prefix: 'wrist_bend', protocol: 'miniarm-wrist-bend-pulse-v1', hold: 'miniarm-wrist-bend-hold-v1' },
  'gripper-link': { channel: 1, index: 5, prefix: 'gripper_link', protocol: 'miniarm-gripper-link-pulse-v1', hold: 'miniarm-gripper-link-hold-v1' },
  gripper: { channel: 0, index: 6, prefix: 'gripper', protocol: 'miniarm-gripper-pulse-v1', hold: 'miniarm-gripper-hold-v1' },
} as const;
export type TestJoint = keyof typeof testSpecs;
function jointFor(status: BaseTestStatus): TestJoint {
  for (const joint of Object.keys(testSpecs) as TestJoint[]) {
    if (status.protocol === testSpecs[joint].protocol && status.channel === testSpecs[joint].channel) return joint;
  }
  throw new Error('Unsupported joint or channel. Reconnect the joint test.');
}
export type BaseTestStatus = {
  protocol: `miniarm-${TestJoint}-pulse-v1`; channel: 15 | 14 | 13 | 12 | 2 | 1 | 0; duration_ms: 400;
  max_step: 1; window: 5; center: number | null; last: number | null;
  active: boolean; fault: string | null; locked: boolean; outputs: (number | null)[];
  hold_protocol?: string; hold_ms?: number; mode?: string | null; active_token?: string | null;
  sweep_protocol?: string; sweep_ms?: number; sweep_watchdog_ms?: number; sweep_rate?: number;
  sweep_target?: number | null; sweep_sequence?: number;
  jog_protocol?: string;
};
export function parseBaseTestStatus(transcript: string, joint: TestJoint = 'base'): BaseTestStatus {
  const spec = testSpecs[joint];
  const lines = transcript.split('\n').filter(line => line.startsWith('@cal:'));
  if (lines.length !== 1) throw new Error('The controller does not expose this joint test. Reconnect after updating its code with servo power disconnected.');
  const value = JSON.parse(lines[0].slice(5));
  if (!value || typeof value !== 'object') throw new Error('Invalid calibration response.');
  if (value.ok !== true) throw new Error(typeof value.error === 'string' ? value.error : 'Controller rejected the test.');
  const angle = (n: unknown) => n === null || (typeof n === 'number' && Number.isFinite(n) && n >= 0 && n <= 180);
  if (value.protocol !== spec.protocol || value.channel !== spec.channel || value.duration_ms !== 400 ||
      value.max_step !== 1 || value.window !== 5 || !angle(value.center) || !angle(value.last) ||
      (value.center === null) !== (value.last === null) ||
      (value.last !== null && Math.abs(value.last - value.center) > 5.000001) ||
      typeof value.active !== 'boolean' || typeof value.locked !== 'boolean' ||
      (value.last !== null && !value.locked) ||
      !(value.fault === null || typeof value.fault === 'string') ||
      !Array.isArray(value.outputs) || value.outputs.length !== 7 || !value.outputs.every(angle)) {
    throw new Error('Unexpected base-test capabilities. No further commands will be sent.');
  }
  return value;
}
export async function probeBaseTest(port: ArmSerialPort, signal: AbortSignal, joint: TestJoint = 'base'): Promise<BaseTestStatus> {
  const result = await exchange(port, signal, `${testSpecs[joint].prefix}_probe`, 3500);
  return parseBaseTestStatus(result.transcript, joint);
}
/** One selected USB device, sequential read-only probes; no initialization moves. */
export async function probeAllJoints(port: ArmSerialPort, signal: AbortSignal): Promise<Record<TestJoint, BaseTestStatus>> {
  const states = {} as Record<TestJoint, BaseTestStatus>;
  for (const joint of Object.keys(testSpecs) as TestJoint[]) {
    signal.throwIfAborted();
    const status = validateSweepStatus(await probeBaseTest(port, signal, joint));
    if (status.active || status.fault || status.outputs.some(v => v !== null)) throw new Error(`${joint}: output active or fault reported. Release the arm and reconnect with servo power off.`);
    states[joint] = status;
  }
  return states;
}
export async function pulseBaseTest(port: ArmSerialPort, signal: AbortSignal, angle: number, previous: BaseTestStatus): Promise<BaseTestStatus> {
  const joint = jointFor(previous), spec = testSpecs[joint];
  if (previous.active || previous.fault || previous.outputs.some(v => v !== null)) throw new Error('Reconnect and check that all seven outputs are off.');
  if (!Number.isFinite(angle) || angle < 0 || angle > 180 || (previous.last !== null &&
      (Math.abs(angle - previous.last) > 1.000001 || Math.abs(angle - previous.center!) > 5.000001))) throw new Error('Target is outside the current base-test range.');
  const result = await exchange(port, signal, `${spec.prefix}_pulse:${crypto.randomUUID()}:${angle}:${previous.last ?? 'none'}`, 3500);
  const status = parseBaseTestStatus(result.transcript, joint);
  if (status.active || status.fault || status.outputs.some(v => v !== null) || status.last !== angle || status.center !== (previous.center ?? angle)) throw new Error('PWM release or test state was not confirmed. Disconnect servo power and reconnect to inspect the controller.');
  return status;
}

export function validateHoldStatus(status: BaseTestStatus): BaseTestStatus {
  const spec = testSpecs[jointFor(status)];
  if (status.hold_protocol !== spec.hold || status.hold_ms !== 3000 ||
      ![null, 'pulse', 'hold'].includes(status.mode === undefined ? 'missing' : status.mode) ||
      !(status.active_token === null || (typeof status.active_token === 'string' && /^[a-f0-9-]{1,48}$/.test(status.active_token))) ||
      (status.active && (status.mode === null || status.active_token === null)) ||
      (!status.active && (status.mode !== null || status.active_token !== null))) {
    throw new Error('The controller does not expose the required hold capability. Reconnect after updating its code with servo power disconnected.');
  }
  return status;
}

function waitForRelease(signal: AbortSignal): Promise<void> {
  return new Promise(resolve => {
    if (signal.aborted) { resolve(); return; }
    const done = () => { clearTimeout(timer); signal.removeEventListener('abort', done); resolve(); };
    const timer = setTimeout(done, 3000);
    signal.addEventListener('abort', done, { once: true });
  });
}

/** A hard three-second hold of the LAST tested angle, with explicit early release.
 * Each exchange releases USB; the Pico deadline is never renewed by the browser.
 */
export async function holdBaseTest(port: ArmSerialPort, signal: AbortSignal, previous: BaseTestStatus,
  onHolding: (status: BaseTestStatus) => void): Promise<BaseTestStatus> {
  signal.throwIfAborted();
  validateHoldStatus(previous);
  const joint = jointFor(previous), spec = testSpecs[joint];
  if (previous.last === null || previous.active || previous.fault || previous.outputs.some(v => v !== null)) throw new Error('Run a pulse first and verify all outputs are off before holding that same target.');
  const token = crypto.randomUUID();
  let startFailure: unknown;
  try {
    const result = await exchange(port, signal, `${spec.prefix}_hold:${token}:${previous.last}`, 3500);
    const status = validateHoldStatus(parseBaseTestStatus(result.transcript, joint));
    if (!status.active || status.mode !== 'hold' || status.active_token !== token || status.last !== previous.last ||
        status.center !== previous.center || status.fault || status.outputs[spec.index] === null || status.outputs.some((v, i) => i !== spec.index && v !== null)) throw new Error('Hold activation was not confirmed.');
    onHolding(status);
    await waitForRelease(signal);
  } catch (error) { startFailure = error; }
  // A start may have executed despite lost acknowledgment. Always attempt an
  // owned release, using a fresh signal even when the user canceled the start.
  let released: BaseTestStatus;
  try {
    const reply = await exchange(port, new AbortController().signal, `${spec.prefix}_release:${token}`, 3500);
    released = validateHoldStatus(parseBaseTestStatus(reply.transcript, joint));
    if (released.active || released.fault || released.outputs.some(v => v !== null) ||
        released.last !== previous.last || released.center !== previous.center) throw new Error('Unexpected release state');
  } catch {
    throw new Error('Hold release was not confirmed. Disconnect servo power, then reconnect. The Pico has a three-second deadline; no hold was retried.');
  }
  if (startFailure && !signal.aborted) throw new Error(`${startFailure instanceof Error ? startFailure.message : 'Hold check failed.'} All outputs were subsequently confirmed off.`);
  return released;
}

export function validateSweepStatus(status: BaseTestStatus): BaseTestStatus {
  const joint = jointFor(status);
  if (status.sweep_protocol !== `miniarm-${joint}-sweep-v1` || status.sweep_ms !== 3000 ||
      status.sweep_watchdog_ms !== 600 || status.sweep_rate !== 10 ||
      !Number.isSafeInteger(status.sweep_sequence) || status.sweep_sequence! < 0 ||
      !(status.sweep_target === null || (typeof status.sweep_target === 'number' && Number.isFinite(status.sweep_target) &&
        status.sweep_target >= 0 && status.sweep_target <= 180 && status.center !== null && Math.abs(status.sweep_target - status.center) <= 5.000001)) ||
      (status.active ? status.mode !== 'sweep' || typeof status.active_token !== 'string' || !/^[a-f0-9-]{1,48}$/.test(status.active_token) : status.mode !== null || status.active_token !== null)) {
    throw new Error('The controller does not expose the required slider capability. Update its code with servo power disconnected, then reconnect.');
  }
  return status;
}

function sweepDelay(signal: AbortSignal): Promise<void> {
  return new Promise(resolve => {
    if (signal.aborted) { resolve(); return; }
    const done = () => { clearTimeout(timer); signal.removeEventListener('abort', done); resolve(); };
    const timer = setTimeout(done, 100);
    signal.addEventListener('abort', done, { once: true });
  });
}

/** Latest target only; no queued moves or retries. The Pico slews the output and
 * enforces both a 600 ms communication watchdog and an absolute 3 second limit.
 */
export async function sweepJoint(port: ArmSerialPort, signal: AbortSignal, previous: BaseTestStatus,
  getTarget: () => number, onStatus: (status: BaseTestStatus) => void, changedOnly = false): Promise<BaseTestStatus> {
  signal.throwIfAborted(); validateSweepStatus(previous);
  const joint = jointFor(previous), spec = testSpecs[joint];
  if ((!changedOnly && previous.last === null) || previous.active || previous.fault || previous.outputs.some(v => v !== null)) throw new Error('Set a starting angle first and verify all outputs are off.');
  if (changedOnly && previous.jog_protocol !== `miniarm-${joint}-jog-v1`) throw new Error('Install the immediate-jog controller update, then reconnect.');
  const initialTarget = changedOnly ? getTarget() : previous.last!;
  const center = previous.center ?? initialTarget;
  if (!Number.isFinite(initialTarget) || initialTarget < 0 || initialTarget > 180 || Math.abs(initialTarget - center) > 5.000001) throw new Error('Invalid starting jog target.');
  if (changedOnly && initialTarget === previous.last) return previous;
  const token = crypto.randomUUID();
  const begun = performance.now();
  let failure: unknown;
  const read = (transcript: string) => {
    const status = validateSweepStatus(parseBaseTestStatus(transcript, joint));
    if (changedOnly && status.jog_protocol !== `miniarm-${joint}-jog-v1`) throw new Error('Unexpected immediate-jog capability.');
    if (status.fault || status.center !== center || status.last === null) throw new Error('Unexpected slider state.');
    return status;
  };
  const checkActive = (status: BaseTestStatus) => {
    if (!status.active || status.active_token !== token || status.outputs[spec.index] === null ||
        status.outputs.some((v, i) => i !== spec.index && v !== null)) throw new Error('Slider activation or output was not confirmed.');
  };
  try {
    const command = changedOnly ? `${spec.prefix}_jog:${token}:${initialTarget}:${previous.last ?? 'none'}` : `${spec.prefix}_sweep:${token}:${previous.last}`;
    const reply = await exchange(port, signal, command, 1000);
    const start = read(reply.transcript); checkActive(start);
    if (start.sweep_sequence !== 0 || start.last !== (previous.last ?? initialTarget) || start.sweep_target !== initialTarget) throw new Error('Unexpected slider starting target.');
    onStatus(start);
    let sequence = 0;
    let lastSent = initialTarget;
    while (!signal.aborted && performance.now() - begun < 2800) {
      const target = getTarget();
      if (!Number.isFinite(target) || target < 0 || target > 180 || Math.abs(target - center) > 5.000001) throw new Error('Slider target outside the current test range.');
      sequence++;
      const command = changedOnly && target === lastSent ? `${spec.prefix}_keep:${token}:${sequence}` : `${spec.prefix}_target:${token}:${sequence}:${target}`;
      const reply = await exchange(port, signal, command, 1000);
      const status = read(reply.transcript); checkActive(status);
      if (status.sweep_sequence !== sequence || status.sweep_target !== target) throw new Error('Slider target acknowledgment mismatch.');
      lastSent = target;
      onStatus(status);
      await sweepDelay(signal);
    }
  } catch (error) { failure = error; }
  // A canceled or lost start acknowledgment may still have enabled the output.
  let released: BaseTestStatus;
  try {
    const reply = await exchange(port, new AbortController().signal, `${spec.prefix}_release:${token}`, 3500);
    released = read(reply.transcript);
    if (released.active || released.outputs.some(v => v !== null)) throw new Error('Output still active');
  } catch {
    throw new Error('Slider release was not confirmed. Disconnect servo power and reconnect. No motion was retried.');
  }
  if (failure && !signal.aborted) throw new Error(`${failure instanceof Error ? failure.message : 'Slider failed.'} All outputs were subsequently confirmed off.`);
  return released;
}

/** Explicit numeric Go: initialize once, otherwise use the same bounded jog
 * and release when the controller reports the requested command angle.
 */
export async function moveJointTarget(port: ArmSerialPort, signal: AbortSignal, previous: BaseTestStatus,
  target: number, onStatus: (status: BaseTestStatus) => void): Promise<BaseTestStatus> {
  signal.throwIfAborted();
  if (!Number.isFinite(target) || target < 0 || target > 180 ||
      (previous.center !== null && Math.abs(target - previous.center) > 5.000001)) throw new Error('Target is outside the current jog range.');
  if (previous.last === null) return pulseBaseTest(port, signal, target, previous);
  const untilTarget = new AbortController();
  const cancel = () => untilTarget.abort();
  signal.addEventListener('abort', cancel, { once: true });
  try {
    return await sweepJoint(port, untilTarget.signal, previous, () => target, status => {
      onStatus(status);
      if (status.last !== null && Math.abs(status.last - target) < .001) untilTarget.abort();
    }, previous.jog_protocol === `miniarm-${jointFor(previous)}-jog-v1`);
  } finally { signal.removeEventListener('abort', cancel); }
}
