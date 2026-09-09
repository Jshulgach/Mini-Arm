'use client';
import Link from 'next/link';
import { useState } from 'react';
import {
  ArrowLeft,
  ArrowRight,
  FlaskConical,
  Play,
  RotateCcw,
  Bell,
  Wifi,
  House,
  WashingMachine,
  Check,
  Minus,
} from 'lucide-react';
import {
  initialVirtualHome,
  simulateEvent,
  scenarios,
  scenarioIds,
  rules,
  type ScenarioId,
  type RuleId,
  type VirtualDevice,
} from '@/lib/virtual-home';
import './playground.css';

const deviceNames: Record<VirtualDevice, string> = {
  washer: 'Virtual washer',
  hub: 'Virtual network hub',
  entry: 'Virtual entry',
};
const deviceIcons = { washer: WashingMachine, hub: Wifi, entry: House };
const virtualTime = (sequence: number) =>
  `T+${String(sequence).padStart(2, '0')}:00`;
export default function Playground() {
  const [home, setHome] = useState(initialVirtualHome);
  const [scenario, setScenario] = useState<ScenarioId>('laundry');
  const [enabled, setEnabled] = useState<RuleId[]>(() =>
    rules.map((rule) => rule.id),
  );
  const [selectedRun, setSelectedRun] = useState<number | null>(null);
  const run =
    home.runs.find((item) => item.sequence === selectedRun) ?? home.runs[0];
  function reset() {
    setHome(initialVirtualHome());
    setSelectedRun(null);
  }
  return (
    <main className="virtual-home">
      <header className="virtual-top">
        <Link href="/canvas">
          <ArrowLeft size={16} /> Back to canvas
        </Link>
        <span>
          <FlaskConical size={16} /> SIMULATION ONLY
        </span>
      </header>
      <div className="virtual-heading">
        <div>
          <p>HOMEBASE LAB</p>
          <h1>What if your home could…</h1>
          <p>Try an event. See the response. Refine the rules.</p>
        </div>
        <button onClick={reset}>
          <RotateCcw size={16} /> Reset virtual home
        </button>
      </div>
      <div className="virtual-boundary">
        <FlaskConical size={20} />
        <p>
          Everything below is fictional and stays in this page. Events do not
          reach your devices, real notifications, or saved home data. Leaving
          resets the simulation.
        </p>
      </div>
      <div className="virtual-layout">
        <section
          className="virtual-panel virtual-controls"
          aria-labelledby="scenario-title"
        >
          <p className="virtual-eyebrow">01 / CHOOSE AN EVENT</p>
          <h2 id="scenario-title">Set things in motion</h2>
          <div className="virtual-scenarios">
            {scenarioIds.map((id) => (
              <button
                key={id}
                aria-pressed={scenario === id}
                onClick={() => setScenario(id)}
              >
                <strong>{scenarios[id].name}</strong>
                <span>{scenarios[id].description}</span>
              </button>
            ))}
          </div>
          <button
            className="virtual-run"
            onClick={() => {
              setHome((current) => simulateEvent(current, scenario, enabled));
              setSelectedRun(null);
            }}
          >
            <Play size={17} /> Simulate event
          </button>
          <p className="virtual-small">
            Each event advances the virtual clock one minute. Run the same event
            twice to test duplicate suppression.
          </p>
          <details className="virtual-rules">
            <summary>Preview rules · {enabled.length} enabled</summary>
            <p className="virtual-small">
              Changes apply to the next event. Previous previews keep the rules
              used at that time.
            </p>
            {rules.map((rule) => (
              <label key={rule.id}>
                <input
                  type="checkbox"
                  checked={enabled.includes(rule.id)}
                  onChange={(event) =>
                    setEnabled((ids) =>
                      event.target.checked
                        ? [...ids, rule.id]
                        : ids.filter((id) => id !== rule.id),
                    )
                  }
                />
                <span>{rule.name}</span>
              </label>
            ))}
          </details>
        </section>
        <div className="virtual-stage">
          <section aria-labelledby="virtual-state-title">
            <div className="virtual-section-title">
              <h2 id="virtual-state-title">Your virtual home</h2>
              <span>{virtualTime(home.sequence)} · simulated time</span>
            </div>
            <div className="virtual-devices">
              {(Object.keys(deviceNames) as VirtualDevice[]).map((id) => {
                const Icon = deviceIcons[id];
                return (
                  <article
                    key={id}
                    className={`virtual-device ${home.focus === id ? 'virtual-focused' : ''}`}
                    aria-label={deviceNames[id]}
                  >
                    <Icon size={24} />
                    <h3>{deviceNames[id]}</h3>
                    <strong>{home.devices[id]}</strong>
                    <small>
                      {home.focus === id
                        ? 'In focus · matched rule'
                        : 'Simulated state'}
                    </small>
                    {home.focus === id && (
                      <p>
                        {id === 'washer'
                          ? 'Laundry is ready. The card makes room for your attention.'
                          : id === 'hub'
                            ? 'Connection details are brought forward for review.'
                            : 'The proposed welcome-home response is ready to review.'}
                      </p>
                    )}
                  </article>
                );
              })}
            </div>
          </section>
          <section
            className="virtual-panel virtual-result"
            aria-labelledby="result-title"
          >
            <p className="virtual-eyebrow">02 / REVIEW THE RESPONSE</p>
            <h2 id="result-title">
              {run ? scenarios[run.scenario].name : 'A plan you can inspect'}
            </h2>
            {!run ? (
              <p className="virtual-empty">
                Simulate an event to see what changed, which rules matched, and
                what each rule would do.
              </p>
            ) : (
              <>
                <output className="virtual-transition" aria-live="polite">
                  {virtualTime(run.sequence)} ·{' '}
                  {deviceNames[scenarios[run.scenario].device]}: {run.before}{' '}
                  <ArrowRight size={15} /> {run.after}
                  {!run.changed && <strong>No new state change</strong>}
                </output>
                <ol className="virtual-decisions">
                  {run.decisions.map((decision) => (
                    <li
                      key={decision.id}
                      className={decision.matched ? 'matched' : 'skipped'}
                    >
                      <div>
                        {decision.matched ? (
                          <Check size={17} />
                        ) : (
                          <Minus size={17} />
                        )}
                        <strong>{decision.name}</strong>
                        <span>{decision.matched ? 'Matched' : 'Skipped'}</span>
                      </div>
                      <p>{decision.reason}</p>
                      {decision.effects.map((effect, index) => (
                        <div key={index} className="virtual-effect">
                          <b>
                            {effect.kind === 'proposal'
                              ? 'Proposal only'
                              : effect.kind === 'focus'
                                ? 'Widget focus'
                                : 'Simulated notification'}
                          </b>
                          <p>{effect.text}</p>
                        </div>
                      ))}
                    </li>
                  ))}
                </ol>
                <p className="virtual-small">
                  Preview complete. This page has no execution control for
                  physical devices.
                </p>
              </>
            )}
          </section>
          <div className="virtual-history-grid">
            <section
              className="virtual-panel"
              aria-labelledby="notifications-title"
            >
              <h2 id="notifications-title">
                <Bell size={17} /> Simulated notifications
              </h2>
              {home.notifications.length ? (
                <ul>
                  {home.notifications.map((notice, index) => (
                    <li key={`${notice.sequence}-${index}`}>
                      <small>{virtualTime(notice.sequence)}</small>
                      <p>{notice.text}</p>
                    </li>
                  ))}
                </ul>
              ) : (
                <p className="virtual-empty">
                  Notifications appear here when an enabled rule matches.
                </p>
              )}
            </section>
            <section className="virtual-panel" aria-labelledby="history-title">
              <h2 id="history-title">Event history</h2>
              <p className="virtual-small">
                Select an event to inspect its original rule preview. The
                virtual home above stays at its latest state.
              </p>
              {home.runs.length ? (
                <ol className="virtual-history">
                  {home.runs.map((item) => (
                    <li key={item.sequence}>
                      <button
                        aria-pressed={run?.sequence === item.sequence}
                        onClick={() => setSelectedRun(item.sequence)}
                      >
                        <small>{virtualTime(item.sequence)}</small>
                        {scenarios[item.scenario].name}
                        <span>
                          {item.changed ? 'State changed' : 'No change'}
                        </span>
                      </button>
                    </li>
                  ))}
                </ol>
              ) : (
                <p className="virtual-empty">No simulated events yet.</p>
              )}
            </section>
          </div>
        </div>
      </div>
    </main>
  );
}
