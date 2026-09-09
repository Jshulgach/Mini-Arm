// Only registry snapshots supplied by the local gateway are accepted here.
export function deviceObservation(device, health, now) {
  const matches = health && health.endpoint === device.endpoint;
  const timestamp = matches ? Date.parse(health.observedAt) : NaN;
  const valid =
    Number.isFinite(timestamp) &&
    timestamp <= now &&
    ['reachable', 'unreachable'].includes(health?.state);
  const stale = valid && now - timestamp > 90000;
  const state = !valid
    ? device.monitor
      ? 'awaiting'
      : 'unmonitored'
    : stale
      ? 'stale'
      : health.state;
  const labels = {
    awaiting: 'Awaiting check',
    unmonitored: 'Not monitored',
    stale: 'Stale observation',
    reachable: 'Port reachable',
    unreachable: 'Port unreachable',
  };
  return {
    id: device.id,
    name: device.name,
    room: device.room,
    kind: device.kind,
    monitored: Boolean(device.monitor),
    state,
    label: labels[state],
    observedAt: valid ? health.observedAt : null,
    source: valid ? health.source : 'Home registry',
    lastResult: valid
      ? health.state === 'reachable'
        ? 'Port reachable'
        : 'Port unreachable'
      : null,
  };
}
export function homeAnswer(message, home, now) {
  const text = message.toLowerCase();
  const devices = home?.devices ?? [];
  const named = devices.filter(
    (d) =>
      text.includes(d.name.toLowerCase()) || text.includes(d.id.toLowerCase()),
  );
  const room = home?.rooms?.find(
    (r) => r !== 'Unassigned' && text.includes(r.toLowerCase()),
  );
  const cameras = /\b(cameras?|webcam|gripper view)\b/i.test(text);
  const activity = /\b(activity|events|happened|changed)\b/i.test(text);
  const offline = /\b(offline|unreachable|disconnected)\b/i.test(text);
  const relevant =
    named.length ||
    room ||
    cameras ||
    activity ||
    offline ||
    /\b(devices?|monitoring|monitored|home status|house status|online)\b/i.test(
      text,
    );
  if (!relevant) return null;
  if (
    /\b(turn|move|jog|grab|start|stop|enable|disable|connect|disconnect|reboot|set|close|open|raise|lower)\b/i.test(
      text,
    ) &&
    !/\b(show|display|bring|view)\b/i.test(text)
  )
    return {
      text: 'I can show device observations and camera widgets, but I cannot operate your devices from chat.',
      mode: 'Home assistant',
    };
  if (!home)
    return {
      text: 'The home registry is unavailable. Please try again when the local gateway is running.',
      mode: 'Home assistant',
    };
  const rows = devices.map((d) =>
    deviceObservation(d, home.health?.[d.id], now),
  );
  let selected = rows;
  if (named.length)
    selected = selected.filter((d) => named.some((n) => n.id === d.id));
  if (room) selected = selected.filter((d) => d.room === room);
  if (cameras) selected = selected.filter((d) => d.kind === 'camera');
  const scope = selected;
  if (offline) selected = selected.filter((d) => d.state === 'unreachable');
  const focusWidgets = activity
    ? ['activity']
    : cameras
      ? [
          ...new Set([
            'home',
            ...selected.flatMap((d) =>
              d.id === 'camera.webcam'
                ? ['webcam']
                : d.id === 'camera.gripper'
                  ? ['gripper']
                  : [],
            ),
          ]),
        ]
      : ['home'];
  const title = activity
    ? 'Recent home activity'
    : cameras
      ? 'Your cameras'
      : offline
        ? 'Recent failed port checks'
        : room
          ? `${room} devices`
          : named.length
            ? 'Device observations'
            : 'Home status';
  const counts = Object.fromEntries(
    ['reachable', 'unreachable', 'stale', 'unmonitored', 'awaiting'].map(
      (s) => [s, scope.filter((d) => d.state === s).length],
    ),
  );
  let answer = `${scope.length} registered device${scope.length === 1 ? '' : 's'}${room ? ` in ${room}` : ''}: ${counts.reachable} port reachable, ${counts.unreachable} port unreachable, ${counts.stale} stale, ${counts.unmonitored} not monitored, ${counts.awaiting} awaiting a check.`;
  if (offline)
    answer = selected.length
      ? `${selected.length} device${selected.length === 1 ? ' has' : 's have'} a recent failed port check: ${selected.map((d) => d.name).join(', ')}.`
      : 'No recent failed port checks are recorded for these devices. That does not mean they are all online.';
  if (offline)
    answer += ` ${counts.stale} stale observation(s); ${counts.unmonitored} not monitored; ${counts.awaiting} awaiting a check.`;
  if (cameras)
    answer +=
      ' Available camera widgets are shown paused. Ring previews are not integrated yet. I cannot see the images or locate people.';
  const events = activity
    ? (home.events ?? [])
        .slice(0, 6)
        .map(({ id, at, source, message }) => ({ id, at, source, message }))
    : [];
  if (activity)
    answer = events.length
      ? `Here are the ${events.length} most recent recorded gateway events. These are history, not new device checks.`
      : 'No gateway events have been recorded yet.';
  if (!activity)
    answer +=
      ' A port check does not confirm device operation. Observations older than 90 seconds are marked stale.';
  return {
    text: answer,
    mode: 'Home assistant',
    homeResult: {
      title,
      asOf: new Date(now).toISOString(),
      devices: activity ? [] : selected,
      events,
      focusWidgets,
    },
  };
}
