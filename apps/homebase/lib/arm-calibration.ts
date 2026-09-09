export const calibrationKey = 'homebase.miniarm-calibration.v1';
// Approximate model-space degrees from miniarm.urdf, retained from Jan 2025.
// Display references only: never copied into servo-space measurements or commands.
export const savedModelLimits: Record<string, string> = {
  base: '−90° to +90°', shoulder: '−90° to +90°', elbow: '−45° to +135°',
  'wrist-rotation': '−90° to +90°', 'wrist-bend': '−90° to +90°',
  'gripper-link': '−90° to +90°', gripper: '−135° to 0° (knuckle joint)',
};
export const recoveredJoints = [
  { id: 'base', name: 'Base', channel: 15, neutral: 90, direction: -1 },
  { id: 'shoulder', name: 'Shoulder', channel: 14, neutral: 90, direction: -1 },
  { id: 'elbow', name: 'Elbow', channel: 13, neutral: 45, direction: 1 },
  { id: 'wrist-rotation', name: 'Wrist rotation', channel: 12, neutral: 90, direction: 1 },
  { id: 'wrist-bend', name: 'Wrist bend', channel: 2, neutral: 110, direction: -1 },
  { id: 'gripper-link', name: 'Gripper link', channel: 1, neutral: 90, direction: 1 },
  { id: 'gripper', name: 'Gripper', channel: 0, neutral: null, direction: null },
] as const;

export type JointCalibration = {
  id: string;
  channel: number;
  neutral: number | null;
  direction: number | null;
  minimum: number | null;
  maximum: number | null;
  mappingChecked: boolean;
  notes: string;
};
export type ArmCalibration = { version: 1; joints: JointCalibration[] };

export function newCalibration(): ArmCalibration {
  return { version: 1, joints: recoveredJoints.map(j => ({
    id: j.id, channel: j.channel, neutral: null, direction: null,
    minimum: null, maximum: null, mappingChecked: false, notes: '',
  })) };
}

// Measurements intentionally start empty. Recovered firmware defaults are a
// separate reference, never automatically promoted to physical measurements.
export function validateCalibration(value: unknown): ArmCalibration {
  if (!value || typeof value !== 'object' || !('version' in value) || value.version !== 1 ||
      !('joints' in value) || !Array.isArray(value.joints) || value.joints.length !== 7) {
    throw new Error('Expected a version 1 calibration with all seven joints.');
  }
  const entries = value.joints;
  const channels = new Set<number>();
  const joints = recoveredJoints.map(reference => {
    const matches = entries.filter((j: unknown) => j && typeof j === 'object' && 'id' in j && j.id === reference.id);
    if (matches.length !== 1) throw new Error(`Missing or repeated joint: ${reference.name}.`);
    const j = matches[0];
    if (!Number.isInteger(j.channel) || j.channel < 0 || j.channel > 15 || channels.has(j.channel)) {
      throw new Error(`${reference.name}: channels must be unique integers from 0 to 15.`);
    }
    channels.add(j.channel);
    for (const field of ['neutral', 'minimum', 'maximum']) {
      if (j[field] !== null && (typeof j[field] !== 'number' || !Number.isFinite(j[field]) || j[field] < 0 || j[field] > 180)) {
        throw new Error(`${reference.name}: angles must be blank or between 0 and 180 servo degrees.`);
      }
    }
    if (j.minimum !== null && j.maximum !== null && j.minimum >= j.maximum) throw new Error(`${reference.name}: minimum must be below maximum.`);
    if (j.neutral !== null && ((j.minimum !== null && j.neutral < j.minimum) || (j.maximum !== null && j.neutral > j.maximum))) {
      throw new Error(`${reference.name}: neutral must be within the recorded travel limits.`);
    }
    if (![null, -1, 1].includes(j.direction)) throw new Error(`${reference.name}: invalid direction.`);
    if (typeof j.mappingChecked !== 'boolean' || typeof j.notes !== 'string' || j.notes.length > 1000) throw new Error(`${reference.name}: invalid notes or mapping confirmation.`);
    return { id: reference.id, channel: j.channel, neutral: j.neutral, direction: j.direction,
      minimum: j.minimum, maximum: j.maximum, mappingChecked: j.mappingChecked, notes: j.notes };
  });
  return { version: 1, joints };
}

export function exportCalibration(value: unknown) {
  return JSON.stringify({ ...validateCalibration(value),
    purpose: 'Manual calibration worksheet. Not applied to firmware. Not authorization for motion.',
    units: 'Servo command degrees, not encoder measurements',
  }, null, 2);
}
