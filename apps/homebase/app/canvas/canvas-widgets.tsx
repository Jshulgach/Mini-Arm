'use client';
import Link from 'next/link';
import type { ReactNode } from 'react';
import type { CanvasController } from './use-canvas-controller';
import {
  ArrowRight,
  CloudSun,
  CornerDownLeft,
  RotateCcw,
  Send,
  Sparkles,
  Wind,
  Camera,
} from 'lucide-react';
import { WebcamPreview } from '../webcam-preview';
import { CameraPreview } from '../camera-preview';
import { addWidget, type WidgetId } from '@/lib/canvas-layout';
import { healthLabel } from '@/lib/gateway';
import { forecastLabel } from '@/lib/canvas-client';
import { ForecastIcon } from './canvas-icons';
export function CanvasWidgetContent({
  id,
  model,
}: {
  id: WidgetId;
  model: CanvasController;
}): ReactNode {
  const {
    setLayout,
    focusedDeviceIds,
    status,
    weather,
    setWeather,
    weatherError,
    weatherLoading,
    day,
    setDay,
    attention,
    setAttention,
    input,
    setInput,
    messages,
    busy,
    useAI,
    setUseAI,
    setRefresh,
    home,
    homeError,
    transcript,
    revealHome,
    ask,
  } = model;
  const selectedDay = weather?.days[day];
  if (id === 'webcam')
    return (
      <div className="canvas-camera-preview">
        <WebcamPreview compact />
      </div>
    );
  if (id === 'gripper') {
    const address = home?.devices.find(
      (device) => device.id === 'camera.gripper',
    )?.endpoint;
    return (
      <div className="canvas-camera-preview">
        {address ? (
          <CameraPreview address={address} compact />
        ) : (
          <p>
            Set the gripper camera’s address in{' '}
            <Link href="/systems">Systems</Link> to enable its preview.
          </p>
        )}
      </div>
    );
  }
  if (id === 'activity')
    return (
      <div className="canvas-activity">
        {!home ? (
          <p>{homeError || 'Loading recent activity…'}</p>
        ) : !home.events.length ? (
          <p>No events recorded yet.</p>
        ) : (
          <ol>
            {home.events.slice(0, 12).map((event) => (
              <li key={event.id}>
                <time dateTime={event.at}>
                  {new Date(event.at).toLocaleString([], {
                    month: 'short',
                    day: 'numeric',
                    hour: 'numeric',
                    minute: '2-digit',
                  })}
                </time>
                <p>{event.message}</p>
                <small>{event.source}</small>
              </li>
            ))}
          </ol>
        )}
        <Link href="/systems">
          All activity <ArrowRight size={15} />
        </Link>
      </div>
    );
  if (id === 'chat')
    return (
      <>
        <div
          className="canvas-chat-history"
          ref={transcript}
          role="log"
          aria-label="Homebase conversation"
          aria-live="polite"
        >
          {!messages.length && (
            <div className="canvas-chat-welcome">
              <Sparkles size={26} />
              <h2>Ask. Bring it into view.</h2>
              <p>
                Ask about devices, cameras, recent activity, or the forecast.
                The relevant widgets will come into focus.
              </p>
              <button
                onClick={() =>
                  void ask('What’s the weather going to be like tomorrow?')
                }
              >
                What’s tomorrow looking like? <ArrowRight size={15} />
              </button>
              <button onClick={() => void ask('What’s offline?')}>
                What’s offline?
              </button>
              <button onClick={() => void ask('Show my cameras')}>
                Show my cameras <Camera size={15} />
              </button>
            </div>
          )}
          {messages.map((message, index) => (
            <div key={index} className={`canvas-message ${message.role}`}>
              <span>
                {message.role === 'user' ? 'You' : message.mode || 'Homebase'}
              </span>
              <p>{message.text}</p>
              {message.homeResult && (
                <div className="canvas-home-answer">
                  <strong>{message.homeResult.title}</strong>
                  <small>
                    Registry snapshot ·{' '}
                    {new Date(message.homeResult.asOf).toLocaleString()}
                  </small>
                  <ul>
                    {message.homeResult.devices.map((device) => (
                      <li key={device.id}>
                        <b>{device.name}</b>
                        <span>
                          {device.room} · {device.label}
                        </span>
                        <small>
                          {device.observedAt
                            ? `${device.lastResult} · ${new Date(device.observedAt).toLocaleString()} · ${device.source}`
                            : 'No matching observation recorded'}
                          {device.monitored
                            ? ' · Monitoring on'
                            : ' · Monitoring off'}
                        </small>
                      </li>
                    ))}
                  </ul>
                  {message.homeResult.events.map((event) => (
                    <p key={event.id}>
                      {event.message}
                      <small>
                        {new Date(event.at).toLocaleString()} · {event.source}
                      </small>
                    </p>
                  ))}
                  <button onClick={() => revealHome(message.homeResult!)}>
                    Show related widgets <ArrowRight size={14} />
                  </button>
                </div>
              )}
              {message.weather && (
                <button
                  className="canvas-chat-result"
                  onClick={() => {
                    setLayout((items) => addWidget(items, 'weather'));
                    setWeather(message.weather!);
                    setDay(message.day ?? 1);
                    setAttention(true);
                  }}
                >
                  <CloudSun size={21} />
                  <span>
                    {message.weather.city} ·{' '}
                    {message.weather.days[message.day ?? 1].label}
                    <strong>
                      {message.weather.days[message.day ?? 1].high}
                      {message.weather.units} /{' '}
                      {message.weather.days[message.day ?? 1].low}
                      {message.weather.units}
                    </strong>
                    <small>
                      {message.weather.days[message.day ?? 1].rain}%
                      precipitation chance · Open-Meteo
                    </small>
                  </span>
                  <ArrowRight size={17} />
                </button>
              )}
            </div>
          ))}
          {busy && <p className="canvas-thinking">Preparing your answer…</p>}
        </div>
        <form
          className="canvas-chat-form"
          onSubmit={(e) => {
            e.preventDefault();
            void ask();
          }}
        >
          <label className="sr-only" htmlFor="canvas-question">
            Ask Homebase
          </label>
          <textarea
            id="canvas-question"
            maxLength={1200}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder="Ask about your home…"
            rows={2}
            onKeyDown={(e) => {
              if (
                e.key === 'Enter' &&
                !e.shiftKey &&
                !e.nativeEvent.isComposing
              ) {
                e.preventDefault();
                void ask();
              }
            }}
          />
          <button
            type="submit"
            aria-label="Send question"
            disabled={busy || !input.trim()}
          >
            <Send size={19} />
          </button>
        </form>
        <div className="canvas-chat-footer">
          <label>
            <input
              type="checkbox"
              checked={useAI}
              disabled={!status?.aiReady || busy}
              onChange={(e) => setUseAI(e.target.checked)}
            />{' '}
            Use AI {status?.aiReady ? '' : '· not configured'}
          </label>
          <span>
            {useAI
              ? 'General chat uses OpenAI · device answers stay local'
              : 'History in this workspace session'}
          </span>
        </div>
      </>
    );
  if (id === 'weather')
    return (
      <div className="canvas-weather">
        <div className="canvas-weather-location">
          <span>
            {weather?.city || status?.city || 'Pittsburgh'}{' '}
            {weather && (
              <a
                className="canvas-compact-source"
                href="https://open-meteo.com/"
                target="_blank"
                rel="noreferrer"
              >
                Open-Meteo ↗
              </a>
            )}
          </span>
          <button
            disabled={weatherLoading || busy}
            onClick={() => setRefresh((n) => n + 1)}
            aria-label="Refresh forecast"
          >
            <RotateCcw size={15} />
          </button>
        </div>
        {weatherLoading && !weather && (
          <p className="canvas-empty">Loading the home forecast…</p>
        )}
        {weatherError && (
          <div role="alert" className="canvas-empty">
            <p>{weatherError}</p>
            <button onClick={() => setRefresh((n) => n + 1)}>Try again</button>
          </div>
        )}
        {selectedDay && weather && (
          <>
            <div className="canvas-weather-main">
              <ForecastIcon code={selectedDay.code} />
              <div>
                <span>
                  {forecastLabel(
                    selectedDay,
                    weather.timezone,
                    home?.serverTime || weather.fetchedAt,
                  )}{' '}
                  · {selectedDay.date}
                </span>
                <strong>
                  {selectedDay.high}
                  <small>{weather.units}</small>
                </strong>
                <p>{selectedDay.condition}</p>
              </div>
            </div>
            <div className="canvas-weather-stats">
              <span>
                Low{' '}
                <b>
                  {selectedDay.low}
                  {weather.units}
                </b>
              </span>
              <span>
                Precipitation <b>{selectedDay.rain}%</b>
              </span>
              <span>
                <Wind size={14} /> Wind <b>{selectedDay.wind} mph</b>
              </span>
            </div>
            <button
              className="canvas-weather-expand"
              onClick={() => setAttention(true)}
            >
              Show full forecast <ArrowRight size={14} />
            </button>
            <div className="canvas-forecast-days" aria-label="Forecast day">
              {weather.days.map((item, i) => (
                <button
                  key={item.date}
                  aria-pressed={day === i}
                  onClick={() => setDay(i)}
                >
                  <span>
                    {forecastLabel(
                      item,
                      weather.timezone,
                      home?.serverTime || weather.fetchedAt,
                    )}
                  </span>
                  <strong>
                    {item.high}° <small>{item.low}°</small>
                  </strong>
                  <span>{item.rain}% precip.</span>
                </button>
              ))}
            </div>
            <p className="canvas-source">
              <a
                href="https://open-meteo.com/"
                target="_blank"
                rel="noreferrer"
              >
                Forecast: Open-Meteo ↗
              </a>
              <span>
                {Date.parse(home?.serverTime || weather.fetchedAt) -
                  Date.parse(weather.fetchedAt) >
                900000
                  ? 'Cached forecast · retrieved '
                  : 'Retrieved '}
                {new Date(weather.fetchedAt).toLocaleString([], {
                  timeZone: weather.timezone,
                  month: 'short',
                  day: 'numeric',
                  hour: 'numeric',
                  minute: '2-digit',
                })}{' '}
                · {weather.timezone}
              </span>
            </p>
          </>
        )}
        {attention && (
          <button className="canvas-return" onClick={() => setAttention(false)}>
            <CornerDownLeft size={15} /> Return to my layout
          </button>
        )}
      </div>
    );
  return (
    <div className="canvas-home-status">
      {!home ? (
        <p>{homeError || 'Loading your home registry…'}</p>
      ) : (
        <>
          <div className="canvas-home-count">
            <strong>{home.devices.length}</strong>
            <span>registered devices</span>
          </div>
          <p>
            {home.rooms.filter((room) => room !== 'Unassigned').length} rooms ·{' '}
            {home.devices.filter((d) => d.monitor).length} monitored endpoints
          </p>
          {homeError && (
            <p role="alert">
              Gateway unavailable. Showing the last registry snapshot.
            </p>
          )}
          {focusedDeviceIds?.length === 0 && (
            <p>No devices matched this answer.</p>
          )}
          <div className="canvas-device-status">
            {home.devices
              .filter(
                (d) =>
                  focusedDeviceIds === null || focusedDeviceIds.includes(d.id),
              )
              .map((d) => (
                <div key={d.id}>
                  <span>{d.name}</span>
                  <small>
                    {healthLabel(d, home.health[d.id], home.serverTime)}
                    {home.health[d.id]?.endpoint === d.endpoint && (
                      <span className="canvas-observed">
                        {new Date(
                          home.health[d.id].observedAt,
                        ).toLocaleString()}{' '}
                        · {home.health[d.id].source}
                      </span>
                    )}
                  </small>
                </div>
              ))}
          </div>
          <Link href="/systems">
            Open systems &amp; activity <ArrowRight size={15} />
          </Link>
        </>
      )}
    </div>
  );
}
