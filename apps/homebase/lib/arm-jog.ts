import type { BaseTestStatus } from './arm-serial';

export function jogBounds(status?: Pick<BaseTestStatus, 'center'>) {
  return status?.center == null ? { min: 0, max: 180 } : {
    min: Math.max(0, status.center - 5), max: Math.min(180, status.center + 5),
  };
}

/** Draft editing never sends a command. Reject blank and out-of-range drafts
 * instead of converting empty input to zero or silently clamping it.
 */
export function parseJogTarget(draft: string | undefined, status?: Pick<BaseTestStatus, 'center'>): number | null {
  if (draft === undefined || draft.trim() === '') return null;
  const value = Number(draft);
  const { min, max } = jogBounds(status);
  return Number.isFinite(value) && value >= min && value <= max ? value : null;
}

export function formatJogAngle(value: number): string {
  return String(Number(value.toFixed(3)));
}
