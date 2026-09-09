// Deterministic, in-memory simulation. This module has no device or network adapters.
export const scenarioIds = [
  'laundry',
  'disconnect',
  'arrival',
  'recover',
] as const;
export type ScenarioId = (typeof scenarioIds)[number];
export type VirtualDevice = 'washer' | 'hub' | 'entry';
export type EffectKind = 'notification' | 'focus' | 'proposal';
export const scenarios: Record<
  ScenarioId,
  { name: string; description: string; device: VirtualDevice; target: string }
> = {
  laundry: {
    name: 'Laundry finishes',
    description: 'The virtual washer changes from washing to complete.',
    device: 'washer',
    target: 'Complete',
  },
  disconnect: {
    name: 'A device disconnects',
    description: 'The virtual network hub becomes unreachable.',
    device: 'hub',
    target: 'Unreachable',
  },
  arrival: {
    name: 'Someone arrives',
    description: 'A fictional arrival sensor changes from away to home.',
    device: 'entry',
    target: 'Home',
  },
  recover: {
    name: 'Connection returns',
    description: 'The virtual network hub becomes reachable again.',
    device: 'hub',
    target: 'Reachable',
  },
};
export const rules = [
  {
    id: 'laundry-notice',
    name: 'Tell me when laundry is done',
    trigger: 'laundry',
    effects: [
      { kind: 'notification', text: 'Laundry is ready to unload.' },
      { kind: 'focus', text: 'Bring the laundry card into focus.' },
    ],
  },
  {
    id: 'connection-notice',
    name: 'Surface a lost connection',
    trigger: 'disconnect',
    effects: [
      { kind: 'notification', text: 'Virtual network hub is unreachable.' },
      { kind: 'focus', text: 'Bring the connection card into focus.' },
    ],
  },
  {
    id: 'arrival-scene',
    name: 'Preview a welcome-home scene',
    trigger: 'arrival',
    effects: [
      { kind: 'focus', text: 'Bring the arrival card into focus.' },
      {
        kind: 'proposal',
        text: 'Propose opening the living room shade. Requires a supported device, operating limits, and a reviewed action before real execution.',
      },
    ],
  },
  {
    id: 'recovery-notice',
    name: 'Tell me when connection returns',
    trigger: 'recover',
    effects: [
      { kind: 'notification', text: 'Virtual network hub is reachable again.' },
      { kind: 'focus', text: 'Bring the connection card into focus.' },
    ],
  },
] as const;
export type RuleId = (typeof rules)[number]['id'];
export type Effect = { kind: EffectKind; text: string };
export type Run = {
  sequence: number;
  scenario: ScenarioId;
  before: string;
  after: string;
  changed: boolean;
  decisions: {
    id: RuleId;
    name: string;
    matched: boolean;
    reason: string;
    effects: Effect[];
  }[];
};
export type VirtualHome = {
  sequence: number;
  devices: Record<VirtualDevice, string>;
  focus: VirtualDevice | null;
  notifications: { sequence: number; text: string }[];
  runs: Run[];
};
export function initialVirtualHome(): VirtualHome {
  return {
    sequence: 0,
    devices: { washer: 'Washing', hub: 'Reachable', entry: 'Away' },
    focus: null,
    notifications: [],
    runs: [],
  };
}
export function simulateEvent(
  state: VirtualHome,
  id: ScenarioId,
  enabled: readonly RuleId[],
): VirtualHome {
  if (!scenarioIds.includes(id)) throw new Error('Unknown virtual scenario.');
  const scenario = scenarios[id],
    before = state.devices[scenario.device],
    changed = before !== scenario.target,
    sequence = state.sequence + 1;
  const decisions = rules.map((rule) => {
    const matched = changed && enabled.includes(rule.id) && rule.trigger === id;
    const reason = !changed
      ? 'No state change; duplicate effects suppressed.'
      : !enabled.includes(rule.id)
        ? 'Disabled for this event.'
        : rule.trigger !== id
          ? 'A different event triggers this rule.'
          : 'Trigger matched this state change.';
    return {
      id: rule.id,
      name: rule.name,
      matched,
      reason,
      effects: matched ? rule.effects.map((effect) => ({ ...effect })) : [],
    };
  });
  const effects = decisions.flatMap((d) => d.effects);
  return {
    sequence,
    devices: { ...state.devices, [scenario.device]: scenario.target },
    focus: effects.some((e) => e.kind === 'focus') ? scenario.device : null,
    notifications: [
      ...effects
        .filter((e) => e.kind === 'notification')
        .map((e) => ({ sequence, text: e.text })),
      ...state.notifications,
    ].slice(0, 12),
    runs: [
      {
        sequence,
        scenario: id,
        before,
        after: scenario.target,
        changed,
        decisions,
      },
      ...state.runs,
    ].slice(0, 20),
  };
}
