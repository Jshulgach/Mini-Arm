'use client';
import Link from 'next/link';
import { useEffect, useMemo, useRef, useState, useSyncExternalStore } from 'react';
import { ArrowLeft, Cable, Download, Save } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { CameraPreview } from '../camera-preview';
import { WebcamPreview } from '../webcam-preview';
import { BaseTest } from './base-test';
import { withConfigs } from '@/lib/home';
import { useDeviceSettings } from '@/lib/settings-store';
import { calibrationKey, exportCalibration, newCalibration, recoveredJoints, savedModelLimits, validateCalibration, type ArmCalibration, type JointCalibration } from '@/lib/arm-calibration';
import { browserSerial, checkArmConnection, picoFilter, type ArmCheck } from '@/lib/arm-serial';

const changedEvent = 'homebase-calibration-saved';
function subscribe(listener: () => void) {
  const storage = (event: StorageEvent) => { if (event.key === calibrationKey || event.key === null) listener(); };
  window.addEventListener('storage', storage);
  window.addEventListener(changedEvent, listener);
  return () => { window.removeEventListener('storage', storage); window.removeEventListener(changedEvent, listener); };
}
function snapshot() {
  try { return localStorage.getItem(calibrationKey) ?? ''; } catch { return 'storage-unavailable'; }
}
const serverSnapshot = () => null;
const subscribeCapabilities = () => () => {};
const serialCapability = () => window.isSecureContext && typeof browserSerial()?.requestPort === 'function';
const serverCapability = () => false;

export default function ArmWorkspace() {
  const raw = useSyncExternalStore(subscribe, snapshot, serverSnapshot);
  const { configs } = useDeviceSettings();
  const camera = withConfigs(configs).find(d => d.id === 'camera.gripper')!;
  const [notice, setNotice] = useState('');
  const stored = useMemo(() => {
    try { return { calibration: raw ? validateCalibration(JSON.parse(raw)) : newCalibration(), error: '' }; }
    catch { return { calibration: newCalibration(), error: 'Saved calibration could not be read. Nothing has been overwritten; a blank worksheet is shown.' }; }
  }, [raw]);
  function save(value: ArmCalibration) {
    const valid = validateCalibration(value);
    localStorage.setItem(calibrationKey, JSON.stringify(valid));
    window.dispatchEvent(new Event(changedEvent));
    setNotice('Worksheet saved in this browser. Firmware settings have not changed.');
  }
  return <main className="arm-workspace">
    <nav className="arm-navigation" aria-label="Workspace"><Link href="/" className="arm-back"><ArrowLeft size={17} /> Homebase</Link><span>Robotics / Manual control</span></nav>
    <header className="arm-heading">
      <div><p className="arm-eyebrow">ROBOT WORKSPACE</p><h1>Mini-Arm<span className="arm-title-dot">.</span></h1><p>Position. Observe. Refine.</p></div>
      <div className="arm-heading-meta"><span>DESK TEST</span><strong>07 <small>servo channels</small></strong><p>One joint at a time</p></div>
    </header>
    <div className="arm-layout">
      <div className="arm-main">
        <BaseTest channels={Object.fromEntries(stored.calibration.joints.map(item => [item.id, item.channel]))} />
        <details className="arm-card arm-disclosure">
          <summary>Calibration worksheet <span>Measurements & saved wiring</span></summary>
          <p className="field-help">Record measurements here. Saving changes does not move the arm or update its firmware.</p>
          {stored.error && <p role="alert" className="form-error">{stored.error}</p>}
          {notice && <output className="arm-notice">{notice}</output>}
          {raw === null ? <p>Loading local worksheet…</p> : <JointEditor key={raw} initial={stored.calibration} save={save} />}
        </details>
        <details className="arm-card arm-disclosure">
          <summary>USB diagnostics <span>Controller report & connection troubleshooting</span></summary>
          <ConnectionCheck />
        </details>
      </div>
      <aside className="arm-cameras" aria-label="Calibration camera views">
        <div className="camera-panel-heading"><p className="eyebrow">OBSERVATION</p><h2>Cameras</h2></div>
        <WebcamPreview compact />
        <CameraPreview address={camera.endpoint} compact />
      </aside>
    </div>
  </main>;
}

function ConnectionCheck() {
  const available = useSyncExternalStore(subscribeCapabilities, serialCapability, serverCapability);
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<ArmCheck | null>(null);
  const [error, setError] = useState('');
  const active = useRef<AbortController | null>(null);
  const mounted = useRef(false);
  useEffect(() => {
    mounted.current = true;
    const cancel = () => { if (document.hidden) active.current?.abort(); };
    const leave = () => active.current?.abort();
    document.addEventListener('visibilitychange', cancel); window.addEventListener('pagehide', leave);
    return () => { mounted.current = false; active.current?.abort(); document.removeEventListener('visibilitychange', cancel); window.removeEventListener('pagehide', leave); };
  }, []);
  async function check() {
    const api = browserSerial(); if (!api || active.current) return;
    const controller = new AbortController(); active.current = controller;
    setBusy(true); setError(''); setResult(null);
    try {
      const port = await api.requestPort({ filters: [picoFilter] });
      controller.signal.throwIfAborted();
      const report = await checkArmConnection(port, controller.signal);
      if (mounted.current && !controller.signal.aborted) setResult(report);
    } catch (failure) {
      if (mounted.current) {
        const name = failure instanceof Error ? failure.name : '';
        setError(name === 'NotFoundError' || name === 'AbortError' ? 'Check canceled. No motion commands were sent.' :
          `${failure instanceof Error ? failure.message : 'USB check failed.'} Close Thonny, Arduino Serial Monitor or another app using COM3 before trying again.`);
      }
    } finally { active.current = null; if (mounted.current) setBusy(false); }
  }
  return <section className="arm-card">
    <div className="arm-section-title"><Cable size={20} /><h2>USB connection check</h2></div>
    <p>Choose the Pico 2 W port, previously <strong>COM3</strong>. The check requests controller information and then releases the port.</p>
    <p className="field-help">Keep servo power disconnected for this first check. USB powers the controller. This page does not home, reset or flash it.</p>
    <div className="arm-actions"><Button onClick={() => void check()} disabled={!available || busy}>{busy ? 'Checking USB…' : 'Choose Pico & check'}</Button>
      {busy && <Button variant="outline" onClick={() => active.current?.abort()}>Cancel check</Button>}</div>
    {!available && <p className="field-help">USB access is unavailable in this browser. Open <a href="http://127.0.0.1:5173/arm">this workspace</a> in Chrome or Edge on the host PC.</p>}
    {error && <p className="form-error" role="alert">{error}</p>}
    {result && <div className="arm-check-result" aria-live="polite">
      <strong>Controller replied · port released</strong>
      <p className="field-help">Checked {new Date(result.checkedAt).toLocaleTimeString()} · {result.elapsedMs} ms including connection setup. This is a snapshot, not continuous monitoring.</p>
      <div className="arm-report-scroll"><table className="arm-report"><caption>Last controller commands — not measured joint positions</caption><thead><tr><th>Joint</th><th>Model °</th><th>Servo °</th></tr></thead>
        <tbody>{recoveredJoints.slice(0, 6).map((j, i) => <tr key={j.id}><th>{j.name}</th><td>{result.jointCommands[i]}</td><td>{result.servoCommands[i] === null ? 'Output off' : result.servoCommands[i]}</td></tr>)}</tbody></table></div>
      <p className="field-help">“Output off” is the controller’s PWM state at the check. It does not confirm servo power is disconnected. Gripper output state is not provided by this report.</p>
      <details><summary>Controller response</summary><pre className="arm-transcript">{result.transcript}</pre></details>
    </div>}
  </section>;
}

function JointEditor({ initial, save }: { initial: ArmCalibration; save: (value: ArmCalibration) => void }) {
  const [draft, setDraft] = useState(initial);
  const [selected, setSelected] = useState(0);
  const [error, setError] = useState('');
  const [dirty, setDirty] = useState(false);
  const [imported, setImported] = useState(false);
  const joint = draft.joints[selected], reference = recoveredJoints[selected];
  const fileInput = useRef<HTMLInputElement>(null);
  function update(patch: Partial<JointCalibration>) {
    setDraft(current => ({ ...current, joints: current.joints.map((j, i) => i === selected ? { ...j, ...patch } : j) }));
    setDirty(true); setError('');
  }
  function exportFile() {
    try {
      const contents = exportCalibration(draft);
      const url = URL.createObjectURL(new Blob([contents], { type: 'application/json' }));
      const link = document.createElement('a'); link.href = url; link.download = 'miniarm-calibration.json'; link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000); setError('');
    } catch (failure) { setError(failure instanceof Error ? failure.message : 'Could not export worksheet.'); }
  }
  async function importFile(file?: File) {
    if (!file) return;
    try {
      if (file.size > 32768) throw new Error('Choose a calibration JSON file smaller than 32 KB.');
      const value = validateCalibration(JSON.parse(await file.text()));
      setDraft(value); setDirty(true); setImported(true); setError('');
    } catch (failure) { setError(failure instanceof Error ? failure.message : 'Could not import worksheet.'); }
    finally { if (fileInput.current) fileInput.current.value = ''; }
  }
  return <>
    <fieldset className="arm-joint-picker"><legend className="sr-only">Choose a joint</legend>
      {recoveredJoints.map((j, i) => <button type="button" key={j.id} aria-pressed={selected === i} onClick={() => setSelected(i)}>
        <span>{j.name}</span><small>CH {draft.joints[i].channel} · {draft.joints[i].mappingChecked ? 'Mapping recorded' : 'Unverified'}</small>
      </button>)}
    </fieldset>
    <form onSubmit={event => { event.preventDefault(); try { save(draft); setDirty(false); setError(''); } catch (failure) { setError(failure instanceof Error ? failure.message : 'Could not save worksheet.'); } }}>
      <div className="arm-joint-title"><h3>{reference.name}</h3><span className="outline-tag">{selected === 6 ? 'Gripper' : `Model joint ${selected + 1}`}</span></div>
      <p className="arm-reference">Firmware reference, unverified: channel {reference.channel}{reference.neutral !== null ? ` · zero maps to ${reference.neutral}° servo · positive model angle ${reference.direction === 1 ? 'increases' : 'decreases'} servo angle` : ' · direct servo-angle commands; no calibrated neutral' }.</p>
      <p className="arm-reference">Saved URDF range: approximately {savedModelLimits[reference.id]} in model angles. This range appears in the January 2025 model; it is a reference, not a measured servo range.</p>
      {reference.id === 'wrist-bend' && <p className="form-error">Mapping conflict: the saved 110° offset maps the URDF range to about 20–200° servo. Resolve this before applying limits; 200° exceeds the servo input range.</p>}
      {reference.id === 'gripper' && <p className="field-help">The knuckle model angle has no verified conversion to the gripper servo command.</p>}
      <div className="arm-fields">
        <label htmlFor="arm-channel">PCA9685 channel<Input id="arm-channel" type="number" min="0" max="15" step="1" required value={Number.isNaN(joint.channel) ? '' : joint.channel} onChange={e => update({ channel: e.target.value === '' ? NaN : Number(e.target.value) })} /></label>
        <label htmlFor="arm-neutral">Measured neutral · servo °<Input id="arm-neutral" type="number" min="0" max="180" step="0.1" placeholder="Not measured" value={joint.neutral ?? ''} onChange={e => update({ neutral: e.target.value === '' ? null : Number(e.target.value) })} /></label>
        <label htmlFor="arm-minimum">Measured lower limit · servo °<Input id="arm-minimum" type="number" min="0" max="180" step="0.1" placeholder="Not measured" value={joint.minimum ?? ''} onChange={e => update({ minimum: e.target.value === '' ? null : Number(e.target.value) })} /></label>
        <label htmlFor="arm-maximum">Measured upper limit · servo °<Input id="arm-maximum" type="number" min="0" max="180" step="0.1" placeholder="Not measured" value={joint.maximum ?? ''} onChange={e => update({ maximum: e.target.value === '' ? null : Number(e.target.value) })} /></label>
        <label className="arm-field-wide">Positive model direction<select value={joint.direction ?? ''} onChange={e => update({ direction: e.target.value === '' ? null : Number(e.target.value) })}>
          <option value="">Not verified / not applicable</option><option value="1">Increases servo angle</option><option value="-1">Decreases servo angle</option>
        </select></label>
      </div>
      <p className="field-help">Record a conservative operating range after supervised testing. The 0–180° input bounds are the servo API range, not this arm’s mechanical limits.</p>
      <label className="arm-checkbox"><input type="checkbox" checked={joint.mappingChecked} onChange={e => update({ mappingChecked: e.target.checked })} /> I traced this channel’s wiring to the named joint.</label>
      <label className="arm-notes-label">Measurements and observations<textarea maxLength={1000} rows={3} value={joint.notes} onChange={e => update({ notes: e.target.value })} placeholder="Reference pose, observed direction, clearance, measurement method…" /></label>
      <p className="field-help">{draft.joints.filter(j => j.mappingChecked).length}/7 channel mappings recorded · {dirty ? 'Unsaved worksheet changes' : 'No unsaved changes'}. These confirmations do not enable motion.</p>
      {imported && <p className="field-help">Imported into this draft. Review the values, then save to replace this browser’s worksheet.</p>}
      {error && <p role="alert" className="form-error">{error}</p>}
      <div className="arm-actions"><Button type="submit"><Save size={16} /> Save worksheet</Button><Button type="button" variant="outline" onClick={exportFile}><Download size={16} /> Export JSON</Button>
        <Button type="button" variant="outline" disabled={dirty} onClick={() => fileInput.current?.click()}>Import JSON</Button>
        <input ref={fileInput} type="file" accept=".json,application/json" hidden onChange={e => void importFile(e.target.files?.[0])} /></div>
      <p className="field-help">Saved in this browser. Export to transfer to a dedicated PC. Save current edits before importing another worksheet.</p>
    </form>
  </>;
}
