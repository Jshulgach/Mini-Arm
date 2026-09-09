import { recoveredJoints } from './arm-calibration.ts';
import { moveJointTarget, type ArmSerialPort, type BaseTestStatus, type TestJoint } from './arm-serial.ts';
import { parseJogTarget } from './arm-jog.ts';

// Existing firmware __HOME__ model zero, converted by the recovered offsets.
// The gripper has no defined home and is deliberately excluded.
export function planArmHome(states: Partial<Record<TestJoint, BaseTestStatus>>, channels: Record<string, number>) {
  const plan: { joint: TestJoint; target: number; previous: BaseTestStatus }[] = [];
  for (const reference of recoveredJoints) {
    const status = states[reference.id];
    if (!status || status.active || status.fault || status.outputs.some(v => v !== null)) throw new Error('Reconnect with all outputs off before Home.');
    if (channels[reference.id] !== reference.channel) throw new Error(`${reference.name}: resolve the worksheet channel before Home.`);
    if (reference.neutral === null) continue;
    if (parseJogTarget(String(reference.neutral), status) === null) throw new Error(`${reference.name}: Home ${reference.neutral}° is outside the current test window. No Home moves were sent.`);
    if (status.last !== reference.neutral) plan.push({joint:reference.id, target:reference.neutral, previous:status});
  }
  return plan;
}

export async function homeArm(port: ArmSerialPort, signal: AbortSignal, states: Partial<Record<TestJoint, BaseTestStatus>>,
  channels: Record<string, number>, onStatus: (joint: TestJoint, status: BaseTestStatus) => void) {
  signal.throwIfAborted();
  const plan = planArmHome(states, channels); // Validate every target before any write.
  for (const [index, step] of plan.entries()) {
    signal.throwIfAborted();
    const result = await moveJointTarget(port, signal, step.previous, step.target, status => onStatus(step.joint, status));
    onStatus(step.joint, result);
    signal.throwIfAborted();
    if (Math.abs(result.last! - step.target) >= .001) throw new Error(`${step.joint}: Home target was not reached. Remaining moves canceled.`);
    if (index < plan.length - 1) await new Promise<void>((resolve, reject) => {
      const cancel = () => { clearTimeout(timer); reject(new DOMException('Home canceled.', 'AbortError')); };
      const timer = setTimeout(() => { signal.removeEventListener('abort', cancel); resolve(); }, 1100);
      signal.addEventListener('abort', cancel, {once:true});
    });
  }
  return plan.length;
}
