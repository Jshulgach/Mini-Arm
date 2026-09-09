'use client';
import { useEffect, useRef, useState, useSyncExternalStore } from 'react';
import { Button } from '@/components/ui/button';
import { recoveredJoints } from '@/lib/arm-calibration';
import { formatJogAngle, jogBounds, parseJogTarget } from '@/lib/arm-jog';
import { homeArm } from '@/lib/arm-home';
import { browserSerial, picoFilter, probeAllJoints, pulseBaseTest, holdBaseTest, sweepJoint, moveJointTarget, type ArmSerialPort, type BaseTestStatus, type TestJoint } from '@/lib/arm-serial';

const subscribe = () => () => {};
const capability = () => window.isSecureContext && typeof browserSerial()?.requestPort === 'function';
const serverCapability = () => false;
type Motion = 'pulse' | 'hold' | 'sweep' | 'jog';
type Phase = { joint: TestJoint; kind: Motion | 'home' | 'releasing' | 'cooldown' } | null;
type States = Partial<Record<TestJoint, BaseTestStatus>>;

export function BaseTest({ channels }: { channels: Record<string, number> }) {
  const available = useSyncExternalStore(subscribe, capability, serverCapability);
  const [states, setStates] = useState<States>({});
  const statesRef = useRef<States>({});
  const [targets, setTargets] = useState<Partial<Record<TestJoint, string>>>({});
  const desired = useRef<Partial<Record<TestJoint, number>>>({});
  const [connecting, setConnecting] = useState(false);
  const [disconnecting, setDisconnecting] = useState(false);
  const disconnectRequested = useRef(false);
  const [phase, setPhase] = useState<Phase>(null);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [manualJoint, setManualJoint] = useState<TestJoint>('base');
  const [jogEnabled, setJogEnabled] = useState(false);
  const enabled = useRef(false);
  const liveJoint = useRef<TestJoint | null>(null);
  const pendingCenter = useRef<number | null>(null);
  const [jogCenter, setJogCenter] = useState<number | null>(null);
  const port = useRef<ArmSerialPort | null>(null);
  const active = useRef<AbortController | null>(null);
  const mounted = useRef(false);
  const cooldown = useRef<ReturnType<typeof setTimeout> | null>(null);

  function remember(joint: TestJoint, status: BaseTestStatus) {
    statesRef.current = { ...statesRef.current, [joint]: status };
    setStates(statesRef.current);
  }
  function invalidate() {
    statesRef.current = {}; setStates({}); port.current = null;
    desired.current = {}; setTargets({});
    enabled.current = false; setJogEnabled(false);
  }
  function finishDisconnect() {
    // Only called with no serial operation running: its owned release and port
    // cleanup must settle before a new connection can be selected.
    if (cooldown.current) clearTimeout(cooldown.current);
    cooldown.current = null;
    disconnectRequested.current = false;
    invalidate(); setPhase(null); setDisconnecting(false);
    setMessage('Disconnected. Jogging is off.');
  }
  function disconnect() {
    if (disconnectRequested.current) return;
    disconnectRequested.current = true;
    enabled.current = false; setJogEnabled(false); setDisconnecting(true);
    setMessage('Disconnecting · waiting for output release and USB cleanup…');
    if (active.current) {
      active.current.abort();
      setPhase(current => current ? { ...current, kind: 'releasing' } : current);
    } else finishDisconnect();
  }
  useEffect(() => {
    mounted.current = true;
    const leaving = () => {
      enabled.current = false; active.current?.abort();
      if (mounted.current) { setJogEnabled(false); setPhase(current => !active.current ? null : current ? { ...current, kind: 'releasing' } : current); }
    };
    const hidden = () => { if (document.hidden) leaving(); };
    const api = browserSerial();
    const unplugged = (event: Event) => {
      if (!port.current || (event.target !== (port.current as unknown) && (event as Event & {port?: ArmSerialPort}).port !== port.current)) return;
      enabled.current = false; setJogEnabled(false);
      if (active.current) {
        disconnectRequested.current = true; setDisconnecting(true);
        active.current.abort();
        setPhase(current => current ? { ...current, kind: 'releasing' } : current);
        setError('Pico USB disconnected during a command. Output release is unconfirmed; disconnect servo power.');
      } else {
        finishDisconnect();
        setError('Pico USB disconnected. Connect again after plugging it in.');
      }
    };
    api?.addEventListener?.('disconnect', unplugged);
    document.addEventListener('visibilitychange', hidden); window.addEventListener('pagehide', leaving);
    return () => {
      mounted.current = false; enabled.current = false; active.current?.abort();
      if (cooldown.current) clearTimeout(cooldown.current);
      api?.removeEventListener?.('disconnect', unplugged);
      document.removeEventListener('visibilitychange', hidden); window.removeEventListener('pagehide', leaving);
    };
  }, []);

  async function connect() {
    if (active.current || cooldown.current || disconnectRequested.current || port.current) return;
    const api = browserSerial(); if (!api) return;
    const controller = new AbortController(); active.current = controller;
    setConnecting(true); invalidate(); setError(''); setMessage('Choose the Pico once. Checking all seven joints…');
    try {
      const chosen = await api.requestPort({ filters: [picoFilter] });
      const results = await probeAllJoints(chosen, controller.signal);
      if (!mounted.current || controller.signal.aborted) return;
      port.current = chosen; statesRef.current = results; setStates(results);
      const known = recoveredJoints.filter(j => results[j.id].last !== null);
      desired.current = Object.fromEntries(known.map(j => [j.id, results[j.id].last!]));
      setTargets(Object.fromEntries(known.map(j => [j.id, formatJogAngle(results[j.id].last!)])));
      setMessage('All seven joints checked. Choose a slider below.');
    } catch (failure) {
      if (mounted.current) { invalidate(); setMessage(''); setError(failure instanceof Error ? failure.message : 'Connection failed.'); }
    } finally {
      active.current = null;
      if (mounted.current) {
        setConnecting(false);
        if (disconnectRequested.current) finishDisconnect();
      }
    }
  }

  async function move(joint: TestJoint, kind: Motion, stopAtTarget?: number) {
    const previous = statesRef.current[joint];
    const reference = recoveredJoints.find(j => j.id === joint)!;
    if (!enabled.current || active.current || cooldown.current || !port.current || !previous || previous.active || previous.fault ||
      previous.outputs.some(v => v !== null) || channels[joint] !== reference.channel) return;
    const controller = new AbortController(); active.current = controller;
    if (kind === 'jog') { liveJoint.current = joint; pendingCenter.current = previous.center ?? desired.current[joint]!; setJogCenter(pendingCenter.current); }
    setPhase({ joint, kind }); setError(''); setMessage(`${reference.name}: ${kind === 'pulse' ? 'sending brief move…' : kind === 'hold' ? 'holding…' : 'jogging…'}`);
    try {
      const update = (snapshot: BaseTestStatus) => {
        if (mounted.current) remember(joint, snapshot);
        if (stopAtTarget !== undefined && snapshot.last !== null && Math.abs(snapshot.last - stopAtTarget) < .001) {
          if (mounted.current) setPhase({ joint, kind: 'releasing' });
        }
      };
      const result = stopAtTarget !== undefined ? await moveJointTarget(port.current, controller.signal, previous, stopAtTarget, update) : kind === 'sweep' || kind === 'jog'
        ? await sweepJoint(port.current, controller.signal, previous, () => desired.current[joint]!, update, kind === 'jog')
        : kind === 'hold'
          ? await holdBaseTest(port.current, controller.signal, previous, update)
          : await pulseBaseTest(port.current, controller.signal, desired.current[joint]!, previous);
      if (mounted.current) {
        remember(joint, result);
        desired.current[joint] = result.last!;
        setTargets(current => ({ ...current, [joint]: formatJogAngle(result.last!) }));
        setMessage(`${reference.name} released · all outputs off.${stopAtTarget !== undefined && Math.abs(result.last! - stopAtTarget) >= .001 ? ` Target not reached; last command ${result.last!.toFixed(1)}°.` : ''}`);
      }
    } catch (failure) {
      if (mounted.current) {
        invalidate(); setMessage('');
        setError(`${failure instanceof Error ? failure.message : 'Joint command failed.'} If release is unconfirmed, disconnect servo power. Reconnect the arm before another move.`);
      }
    } finally {
      active.current = null; liveJoint.current = null; pendingCenter.current = null;
      if (mounted.current) {
        if (disconnectRequested.current) finishDisconnect();
        else {
          setPhase({ joint, kind: 'cooldown' });
          cooldown.current = setTimeout(() => { cooldown.current = null; setPhase(null); }, 1100);
        }
      }
    }
  }

  function changeSlider(joint: TestJoint, value: number) {
    const status = statesRef.current[joint];
    const bounds = {center: status?.center ?? (liveJoint.current === joint ? pendingCenter.current : null)};
    if (parseJogTarget(String(value), bounds) === null) return;
    setTargets(current => ({ ...current, [joint]: String(value) }));
    if (!enabled.current || !status) return;
    if (active.current) {
      // React's rendered draft can lag rapid input events. Always replace the
      // live desired target; the serial wrapper deduplicates transmitted values.
      if (liveJoint.current === joint && !active.current.signal.aborted) desired.current[joint] = value;
      return;
    }
    if (cooldown.current || status.last === value) return;
    desired.current[joint] = value;
    void move(joint, 'jog');
  }
  function goToTarget(joint: TestJoint) {
    if (!enabled.current || active.current || cooldown.current || !port.current) return;
    const status = statesRef.current[joint];
    const angle = parseJogTarget(targets[joint], status);
    if (!status || angle === null) return;
    desired.current[joint] = angle;
    void move(joint, status.last === null ? 'pulse' : 'sweep', angle);
  }
  function release() {
    if (phase && phase.kind !== 'cooldown') {
      active.current?.abort(); setPhase({ joint: phase.joint, kind: 'releasing' });
    }
  }
  function toggleJog(checked: boolean) {
    if (checked && (active.current || cooldown.current || !port.current)) return;
    enabled.current = checked; setJogEnabled(checked);
    if (!checked) release();
    else setMessage('Jogging enabled. Only new slider positions send motion commands.');
  }
  async function home() {
    if (!enabled.current || active.current || cooldown.current || !port.current) return;
    const controller = new AbortController(); active.current = controller;
    setPhase({joint:'base', kind:'home'}); setError(''); setMessage('Checking Home targets…');
    try {
      const count = await homeArm(port.current, controller.signal, statesRef.current, channels, (joint, status) => {
        if (mounted.current) {
          remember(joint, status);
          setPhase({joint, kind:controller.signal.aborted ? 'releasing' : 'home'});
          if (!status.active && status.last !== null) setTargets(current => ({ ...current, [joint]: formatJogAngle(status.last!) }));
        }
      });
      if (mounted.current) setMessage(count ? 'Home commands completed; outputs released. Gripper unchanged.' : 'Home angles already match the last commands. No movement sent.');
    } catch (failure) {
      if (mounted.current) {
        if (failure instanceof Error && failure.name === 'AbortError') setMessage('Home canceled. No remaining joints will be moved.');
        else { invalidate(); setMessage(''); setError(failure instanceof Error ? failure.message : 'Home failed; disconnect servo power.'); }
      }
    } finally {
      active.current = null;
      if (mounted.current) {
        if (disconnectRequested.current) finishDisconnect();
        else {
          setPhase({joint:'base', kind:'cooldown'});
          cooldown.current = setTimeout(() => { cooldown.current = null; setPhase(null); }, 1100);
        }
      }
    }
  }
  const connected = recoveredJoints.every(j => states[j.id]);
  const occupied = connecting || disconnecting || phase !== null;
  const immediateSupported = connected && recoveredJoints.every(j => states[j.id]?.jog_protocol === `miniarm-${j.id}-jog-v1`);
  const manualStatus = states[manualJoint];
  const manualReference = recoveredJoints.find(j => j.id === manualJoint)!;
  const manualDisabled = !jogEnabled || !connected || occupied || manualStatus?.last == null || channels[manualJoint] !== manualReference.channel;
  return <section className="arm-card base-test" aria-label="All joint controls">
    <div className="motor-toolbar"><div><h2>Motor controls</h2><span className="motor-connection">{disconnecting ? 'Releasing controller…' : connecting ? 'Checking controller…' : connected ? 'Pico selected · 7 joints checked' : 'Pico not connected'}</span></div>
      <div className="motor-toolbar-actions">
      <Button disabled={disconnecting || connecting || (!connected && (!available || occupied))} onClick={() => connected ? disconnect() : void connect()}>{disconnecting ? 'Disconnecting…' : connecting ? 'Connecting…' : connected ? 'Disconnect' : 'Connect'}</Button>
      <Button variant="outline" disabled={!connected || !jogEnabled || occupied} onClick={() => void home()}>Home</Button>
      </div>
    </div>
    <label className="motor-jog-toggle"><input type="checkbox" role="switch" aria-label="Jog enabled" aria-checked={jogEnabled} checked={jogEnabled} disabled={!immediateSupported || (!jogEnabled && occupied)} onChange={event => toggleJog(event.target.checked)} /><span className="jog-switch-track" aria-hidden="true" /><span className="jog-switch-copy"><strong>Jog {jogEnabled ? 'enabled' : 'disabled'}</strong><span>{jogEnabled ? 'Slider changes move the selected joint immediately.' : 'Sliders only preview targets.'}</span></span></label>
    {connected && !immediateSupported && <p className="field-help">Immediate jogging needs the Pico update. Reconnect after installation.</p>}
    <dl className="motor-operating-limits" aria-label="Jog operating limits">
      <div><dt>Control</dt><dd>{jogEnabled ? 'Immediate jog' : 'Target preview'}</dd></div>
      <div><dt>Speed limit</dt><dd>10<span>°/s</span></dd></div>
      <div><dt>Test window</dt><dd>±5<span>° from start</span></dd></div>
      <div><dt>Auto release</dt><dd>3<span>seconds</span></dd></div>
    </dl>
    <p className="motor-safety">Connect with servo power off. Support the arm before powering it for testing; other joints do not hold. Keep the power cutoff within reach.</p>
    <p id="arm-jog-help" className="field-help">Enable jogging, then adjust a slider. Unchanged positions send no new target. Disconnect or turn jogging off to end a move early. The first move from an unknown position can be large.</p>
    {!available && <p className="form-error">USB controls require Chrome or Edge on this PC. Open <a href="http://127.0.0.1:5173/arm" target="_blank" rel="noreferrer">the arm dashboard</a> there.</p>}
    {error && <p className="form-error" role="alert">{error}</p>}
    <div className="motor-feedback"><output className="arm-notice" aria-live="polite">{message || 'One joint at a time · no measured position feedback.'}</output></div>
    {/* eslint-disable-next-line jsx-a11y/no-noninteractive-tabindex -- Focus enables keyboard scrolling of the motor table on narrow screens. */}
    <section className="motor-table-scroll" aria-label="Joint targets and command status" tabIndex={0}>
    <table className="motor-table"><caption className="sr-only">Seven joint jog controls. Target and last command are servo degrees, not measured positions.</caption>
      <thead><tr><th scope="col">Joint</th><th scope="col">Jog</th><th scope="col">Target °</th><th scope="col">Last command °</th><th scope="col">Status</th></tr></thead><tbody>
      {recoveredJoints.map(reference => {
        const joint = reference.id, status = states[joint];
        const initial = !status || status.last === null;
        const current = phase?.joint === joint;
        const interactive = current && phase?.kind === 'jog';
        const mismatch = channels[joint] !== reference.channel;
        const disabled = !connected || mismatch || (!!occupied && !interactive);
        const effective = {center: status?.center ?? (current && phase.kind === 'jog' ? jogCenter : null)};
        const { min: minimum, max: maximum } = jogBounds(effective);
        const draft = targets[joint];
        const target = parseJogTarget(draft, effective);
        const displayed = target ?? status?.last ?? reference.neutral ?? 90;
        const rangeId = `jog-${joint}`;
        const rowStatus = mismatch ? 'Check wiring' : !status ? 'Offline' : current ? phase.kind === 'home' ? 'Home' : phase.kind === 'cooldown' ? 'Wait 1 s' : phase.kind === 'releasing' ? 'Releasing' : status.active ? phase.kind === 'jog' || phase.kind === 'sweep' ? 'Jogging' : 'Holding' : 'Starting' : !jogEnabled ? 'Preview' : initial ? 'Set start' : 'Ready';
        return <tr key={joint} className={current ? 'is-active' : ''}>
          <th scope="row"><label htmlFor={rangeId}>{reference.name}</label><small>CH {reference.channel}</small></th>
          <td className="motor-jog-cell">
          <input id={rangeId} type="range" aria-label={`${reference.name} jog`} min={minimum} max={maximum} step="0.1" value={displayed} disabled={disabled}
            aria-describedby={`arm-jog-help ${rangeId}-help`} aria-valuetext={target === null ? 'No valid target selected' : `${target} degrees servo target`}
            onPointerCancel={() => { if (liveJoint.current === joint) release(); }}
            onKeyDown={event => { if (event.key === 'Escape') release(); }}
            onChange={event => changeSlider(joint, Number(event.target.value))} />
          <div className="arm-slider-scale" id={`${rangeId}-help`}><span>{minimum}°</span><span>{maximum}°</span></div>
          </td>
          <td><div className="motor-target-input">
            <input type="number" aria-label={`${reference.name} target degrees`} min={minimum} max={maximum} step="0.1" placeholder="—" value={draft ?? ''}
              disabled={!connected || mismatch || occupied} aria-invalid={draft !== undefined && draft !== '' && target === null}
              onChange={event => setTargets(previous => ({ ...previous, [joint]: event.target.value }))}
              onKeyDown={event => { if (event.key === 'Enter') { event.preventDefault(); if (!event.repeat) goToTarget(joint); } }} />
            <Button variant="outline" aria-label={`Go to ${reference.name} target`} disabled={!jogEnabled || !connected || mismatch || occupied || target === null} onClick={() => goToTarget(joint)}>Go</Button>
          </div></td>
          <td className="motor-last-command">{status?.last == null ? '—' : status.last.toFixed(1)}</td>
          <td><span className={`motor-state${current ? ' is-active' : ''}${mismatch ? ' is-error' : ''}`}>{rowStatus}</span>
            {mismatch && <small className="form-error">Worksheet CH {channels[joint]}</small>}</td>
        </tr>;
      })}
      </tbody></table></section>
    <p className="motor-limit-note">Slider arrow keys: 0.1° steps · Numeric targets: Go or Enter to apply · Angles are commands, not measured positions.</p>
    <details className="arm-test-details"><summary>Joint references & manual tests</summary>
      <div className="motor-manual-controls"><label htmlFor="manual-joint">Joint</label><select id="manual-joint" value={manualJoint} disabled={occupied} onChange={event => setManualJoint(event.target.value as TestJoint)}>{recoveredJoints.map(j => <option value={j.id} key={j.id}>{j.name}</option>)}</select>
        <Button variant="outline" disabled={manualDisabled} onClick={() => { desired.current[manualJoint] = manualStatus!.last!; void move(manualJoint, 'pulse'); }}>Repeat pulse</Button>
        <Button variant="outline" disabled={manualDisabled} onClick={() => void move(manualJoint, 'hold')}>Hold · 3 s</Button></div>
      <p className="field-help">{manualReference.name}: channel {manualReference.channel}, {manualReference.neutral === null ? 'no calibrated neutral' : `unverified zero reference ${manualReference.neutral}°`}. Worksheet measurements remain separate from controller commands.</p>
      <p className="field-help">Home uses the saved model-zero references: base 90°, shoulder 90°, elbow 45°, wrist rotation 90°, wrist bend 110°, gripper link 90°. It moves one joint at a time and releases each output; the gripper is unchanged. All targets must fit the current test windows. These references are not measured calibration.</p>
      <p className="field-help">Wrist-bend model limits remain unresolved; raw servo ranges do not confirm mechanical clearance. The Pico releases a jog if updates stop for 600 ms or after 3 seconds. These software limits are not a hardware emergency stop. “Outputs off” describes PWM, not supply power.</p>
    </details>
  </section>;
}
