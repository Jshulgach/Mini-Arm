'use client';
import { useMemo, useSyncExternalStore } from 'react';
import { parseConfigs, storageKey, withConfigs, rooms as defaultRooms, relations as defaultRelations, type Configs } from './home';
import { useGateway, saveGateway } from './gateway';
const listeners = new Set<() => void>();
function subscribe(listener: () => void) {
  listeners.add(listener);
  const changed = (e: StorageEvent) => {
    if (e.key === storageKey || e.key === null) listener();
  };
  window.addEventListener('storage', changed);
  return () => {
    listeners.delete(listener);
    window.removeEventListener('storage', changed);
  };
}
function snapshot() {
  try {
    return localStorage.getItem(storageKey) ?? '{}';
  } catch {
    return 'storage-unavailable';
  }
}
function serverSnapshot() {
  return null;
}
export function useDeviceSettings() {
  const gateway = useGateway();
  const raw = useSyncExternalStore(subscribe, snapshot, serverSnapshot);
  const parsed = useMemo(() => {
    try {
      return { configs: parseConfigs(raw), error: '' };
    } catch {
      return {
        configs: {} as Configs,
        error:
          'Saved settings could not be loaded. Original inventory is shown.',
      };
    }
  }, [raw]);
  async function saveConfigs(next: Configs) {
    if (gateway.data) {
      if (gateway.error) throw new Error('Gateway unavailable. Settings were not saved.');
      await saveGateway({...gateway.data, devices:gateway.data.devices.map(d=>({...d,...next[d.id]}))});
      return;
    }
    localStorage.setItem(storageKey, JSON.stringify(next));
    for (const listener of listeners) listener();
  }
  return {
    configs: gateway.data ? Object.fromEntries(gateway.data.devices.map(d=>[d.id,{name:d.name,room:d.room,endpoint:d.endpoint}])) : parsed.configs,
    inventory: gateway.data?.devices ?? withConfigs(parsed.configs),
    rooms: gateway.data?.rooms ?? defaultRooms,
    relations: gateway.data?.relations ?? defaultRelations,
    ready: raw !== null,
    storageError: gateway.error || parsed.error,
    saveConfigs,
  };
}
