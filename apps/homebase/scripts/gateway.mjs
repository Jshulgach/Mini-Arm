import { DatabaseSync } from 'node:sqlite';
import { mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import { connect } from 'node:net';
import { randomUUID } from 'node:crypto';
import { devices as inventory, rooms as initialRooms, relations as initialRelations } from '../lib/home.ts';

const kinds = ['robot', 'camera', 'shade', 'appliance', 'printer', 'network'];
const relationKinds = ['network', 'physical', 'logical'];
const object = value => value && typeof value === 'object' && !Array.isArray(value);
function fail(message, status = 400) { throw Object.assign(new Error(message), { status }); }
function string(value, max = 100, blank = false) {
  if (typeof value !== 'string' || value.length > max || (!blank && !value.trim())) fail('Invalid text field.');
  return value.trim();
}
function id(value) { if (typeof value !== 'string' || !/^[a-z0-9][a-z0-9._-]{0,79}$/.test(value)) fail('Invalid identifier.'); return value; }
function flag(value) { if (typeof value !== 'boolean') fail('Expected a boolean.'); return value; }
function unique(values) { if (new Set(values).size !== values.length) fail('Duplicate names or identifiers.'); }
export function networkTarget(endpoint) {
  let url; try { url = new URL(endpoint); } catch { fail('Configure a private LAN HTTP endpoint first.'); }
  if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash) fail('Invalid monitoring endpoint.');
  const octets = url.hostname.split('.');
  if (octets.length !== 4 || octets.some(p => !/^\d{1,3}$/.test(p) || Number(p) > 255)) fail('Monitoring requires a private LAN IPv4 address.');
  const [a, b] = octets.map(Number);
  if (!(a === 10 || (a === 192 && b === 168) || (a === 172 && b >= 16 && b <= 31))) fail('Monitoring is limited to private LAN addresses.');
  return { host: url.hostname, port: Number(url.port || (url.protocol === 'https:' ? 443 : 80)) };
}
function endpoint(value) {
  const text = string(value, 300, true);
  if (text) {
    let url; try { url = new URL(text); } catch { fail('Use a full HTTP or HTTPS endpoint.'); }
    if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash) fail('Endpoints cannot contain credentials, queries or fragments.');
  }
  return text;
}
export function validateConfiguration(input) {
  if (!object(input) || !Array.isArray(input.rooms) || !Array.isArray(input.devices) || !Array.isArray(input.relations) || !Array.isArray(input.rules)) fail('Invalid configuration.');
  if (input.rooms.length > 40 || input.devices.length > 100 || input.relations.length > 200 || input.rules.length > 100) fail('Configuration is too large.');
  const rooms = input.rooms.map(r => string(r, 60)); unique(rooms);
  if (!rooms.includes('Unassigned')) fail('Keep the Unassigned room.');
  const devices = input.devices.map(d => {
    if (!object(d) || !rooms.includes(d.room) || !kinds.includes(d.kind)) fail('Invalid device room or kind.');
    const result = { id: id(d.id), name: string(d.name, 80), model: string(d.model, 160, true), kind: d.kind, room: d.room,
      adapter: string(d.adapter, 160), endpoint: endpoint(d.endpoint), notes: string(d.notes, 2000, true), next: string(d.next, 500, true), monitor: flag(d.monitor) };
    if (result.monitor) networkTarget(result.endpoint);
    return result;
  }); unique(devices.map(d => d.id));
  const ids = new Set(devices.map(d => d.id));
  if (inventory.some(d=>!ids.has(d.id))) fail('Keep the original household devices in the registry.');
  const relations = input.relations.map(r => {
    if (!object(r) || !ids.has(r.from) || !ids.has(r.to) || r.from === r.to || !relationKinds.includes(r.type)) fail('Invalid device relationship.');
    return {from:r.from, to:r.to, type:r.type, label:string(r.label, 60)};
  }); unique(relations.map(r => `${r.from}|${r.to}|${r.type}|${r.label}`));
  const rules = input.rules.map(r => {
    if (!object(r) || !ids.has(r.deviceId) || !['reachable', 'unreachable'].includes(r.when)) fail('Invalid notification rule.');
    return {id:id(r.id), name:string(r.name, 100), deviceId:r.deviceId, when:r.when, enabled:flag(r.enabled)};
  }); unique(rules.map(r => r.id));
  return {rooms, devices, relations, rules};
}
export function initialConfiguration() {
  return validateConfiguration({rooms:initialRooms, devices:inventory.map(d => ({...d, monitor:false,
    ...(d.id === 'host.octopi' ? {notes:'Owner reports the Raspberry Pi left the network on September 8. Cause and current address are unverified.', next:'Reconnect the Pi and verify its address before enabling monitoring.'} : {}),
    ...(d.id === 'robot.miniarm' ? {notes:'Owner disconnected the arm and paused robot work. Joint calibration remains unresolved.', next:'Resume calibration separately; no robot commands are available from Systems.'} : {})})),
    relations:initialRelations.map(r => ({...r, type:r.label === 'mounted on' ? 'physical' : 'logical'})), rules:[]});
}
export function tcpReachable(target, timeoutMs = 1500) {
  return new Promise(resolve => {
    const socket = connect(target);
    let finished = false;
    const done = result => { if (finished) return; finished = true; clearTimeout(timer); socket.destroy(); resolve(result); };
    const timer = setTimeout(() => done(false), timeoutMs);
    socket.once('connect', () => done(true)); socket.once('error', () => done(false));
  });
}
export function createGateway(filename, {dial = tcpReachable, now = () => Date.now()} = {}) {
  if (filename !== ':memory:') mkdirSync(dirname(filename), {recursive:true});
  const db = new DatabaseSync(filename);
  db.exec('PRAGMA journal_mode=WAL; PRAGMA busy_timeout=3000; CREATE TABLE IF NOT EXISTS config (id INTEGER PRIMARY KEY CHECK(id=1), revision INTEGER NOT NULL, json TEXT NOT NULL); CREATE TABLE IF NOT EXISTS health (id TEXT PRIMARY KEY, json TEXT NOT NULL); CREATE TABLE IF NOT EXISTS events (seq INTEGER PRIMARY KEY AUTOINCREMENT, json TEXT NOT NULL); CREATE TABLE IF NOT EXISTS notifications (id TEXT PRIMARY KEY, json TEXT NOT NULL);');
  const startedAt = new Date(now()).toISOString();
  const timestamp = () => new Date(now()).toISOString();
  const event = value => {
    db.prepare('INSERT INTO events(json) VALUES (?)').run(JSON.stringify({id:randomUUID(), at:timestamp(), ...value}));
    db.exec('DELETE FROM events WHERE seq NOT IN (SELECT seq FROM events ORDER BY seq DESC LIMIT 2000)');
  };
  if (!db.prepare('SELECT id FROM config').get()) {
    db.prepare('INSERT INTO config VALUES (1, 1, ?)').run(JSON.stringify(initialConfiguration()));
    event({type:'gateway.created', source:'gateway', message:'Persistent home registry created. Monitoring is off for every device.'});
  }
  event({type:'gateway.started', source:'gateway', message:'Gateway started. Device observations retain their original timestamps.'});
  let closed = false;
  const pending = new Map();
  function configuration() { const r=db.prepare('SELECT revision,json FROM config WHERE id=1').get(); return {revision:r.revision,...JSON.parse(r.json)}; }
  function snapshot() {
    const config = configuration();
    const health = Object.fromEntries(db.prepare('SELECT id,json FROM health').all().map(r => [r.id, JSON.parse(r.json)]));
    return {...config, health, events:db.prepare('SELECT json FROM events ORDER BY seq DESC LIMIT 100').all().map(r => JSON.parse(r.json)),
      notifications:db.prepare('SELECT json FROM notifications ORDER BY rowid DESC LIMIT 100').all().map(r => JSON.parse(r.json)), startedAt, serverTime:timestamp()};
  }
  function save(input) {
    const config = validateConfiguration(input);
    if (!Number.isSafeInteger(input.revision)) fail('Missing configuration revision.');
    db.exec('BEGIN IMMEDIATE');
    try {
      const previous=configuration();
      if (input.revision !== previous.revision) fail('Settings changed in another window. Reload before saving.',409);
      db.prepare('UPDATE config SET revision=revision+1,json=? WHERE id=1').run(JSON.stringify(config));
      for (const old of previous.devices) {
        const next=config.devices.find(d=>d.id===old.id);
        if (!next || next.endpoint!==old.endpoint || next.monitor!==old.monitor) db.prepare('DELETE FROM health WHERE id=?').run(old.id);
      }
      event({type:'configuration.saved',source:'user',message:'Device, room, relationship or automation settings saved.'});
      db.exec('COMMIT');
    } catch(error) { db.exec('ROLLBACK'); throw error; }
    return snapshot();
  }
  async function check(deviceId) {
    if (pending.has(deviceId)) return pending.get(deviceId);
    const device=configuration().devices.find(d=>d.id===deviceId);
    if (!device) fail('Device not found.',404);
    const target=networkTarget(endpoint(device.endpoint));
    const operation=(async()=>{
      const begin=now();const reachable=await dial(target);
      if (closed) return;
      const current=configuration().devices.find(d=>d.id===deviceId);
      // A delayed observation cannot overwrite an edited or disabled endpoint.
      if (!current || current.endpoint!==device.endpoint || current.monitor!==device.monitor) return snapshot();
      const prior=db.prepare('SELECT json FROM health WHERE id=?').get(deviceId);
      const old=prior?JSON.parse(prior.json):null;
      const result={state:reachable?'reachable':'unreachable',observedAt:timestamp(),source:'TCP port check',endpoint:device.endpoint,latencyMs:Math.max(0,now()-begin)};
      db.exec('BEGIN IMMEDIATE');
      try {
        db.prepare('INSERT OR REPLACE INTO health VALUES (?,?)').run(deviceId,JSON.stringify(result));
        if (!old || old.state!==result.state) {
          event({type:'reachability.changed',deviceId,source:result.source,message:`${device.name}: port ${target.port} ${result.state}.`});
          for (const rule of configuration().rules.filter(r=>r.enabled&&r.deviceId===deviceId&&r.when===result.state)) {
            const notification={id:randomUUID(),at:timestamp(),ruleId:rule.id,deviceId,read:false,message:`${rule.name}: ${device.name} port ${target.port} is ${result.state}.`};
            db.prepare('INSERT INTO notifications VALUES (?,?)').run(notification.id,JSON.stringify(notification));
            event({type:'notification.created',source:'automation',deviceId,message:notification.message});
          }
          db.exec('DELETE FROM notifications WHERE rowid NOT IN (SELECT rowid FROM notifications ORDER BY rowid DESC LIMIT 500)');
        }
        db.exec('COMMIT');
      } catch(error){db.exec('ROLLBACK');throw error;}
      return snapshot();
    })();
    pending.set(deviceId,operation);
    try{return await operation;}finally{pending.delete(deviceId);}
  }
  function testRule(ruleId, state) {
    const rule=configuration().rules.find(r=>r.id===ruleId);
    if (!rule || !['reachable','unreachable'].includes(state)) fail('Choose a saved rule and simulated state.');
    const matches=rule.when===state;
    return {simulation:true,matches,enabled:rule.enabled,message:matches?'This condition matches. An enabled rule would create a local notification.':'This condition does not match. No notification would be created.'};
  }
  function acknowledge(notificationId) {
    string(notificationId,80);
    const row=db.prepare('SELECT json FROM notifications WHERE id=?').get(notificationId);
    if (!row) fail('Notification not found.',404);
    db.prepare('UPDATE notifications SET json=? WHERE id=?').run(JSON.stringify({...JSON.parse(row.json),read:true}),notificationId);
    return snapshot();
  }
  return {snapshot,save,check,testRule,acknowledge,export:()=>({version:1,...configuration()}),
    async poll(){for(const d of configuration().devices.filter(d=>d.monitor)) {if(closed)break;await check(d.id);}},
    async close(){if(closed)return;closed=true;await Promise.allSettled(pending.values());db.close();}};
}

export async function handleGateway(req,res,gateway,assistant=null) {
  if (!req.url?.startsWith('/api/')) return false;
  res.setHeader('Content-Type','application/json; charset=utf-8');res.setHeader('Cache-Control','no-store');
  try {
    const host=req.headers.host;
    if (!host || !/^(127\.0\.0\.1|localhost)(:\d{1,5})?$/.test(host)) fail('Local access only.',403);
    if (req.headers.origin && req.headers.origin!==`http://${host}`) fail('Cross-origin access is not allowed.',403);
    if (req.headers['sec-fetch-site'] && !['same-origin','none'].includes(req.headers['sec-fetch-site'])) fail('Cross-site access is not allowed.',403);
    const route=new URL(req.url,`http://${host}`).pathname;
    let body;
    if (req.method==='POST'||req.method==='PUT') {
      if (!req.headers['content-type']?.startsWith('application/json')) fail('Use application/json.',415);
      let raw='';for await(const chunk of req){raw+=chunk.toString();if(Buffer.byteLength(raw)>200000)fail('Request too large.',413);}
      try{body=JSON.parse(raw);}catch{fail('Invalid JSON.');}
      if(!object(body))fail('Expected a JSON object.');
    }
    let result;
    if(req.method==='GET'&&route==='/api/assistant'&&assistant)result=assistant.status();
    else if(req.method==='POST'&&route==='/api/assistant'&&assistant)result=await assistant.ask(body,gateway.snapshot());
    else if(req.method==='POST'&&route==='/api/weather'&&assistant)result=await assistant.forecast(body.units);
    else if(req.method==='GET'&&route==='/api/home')result=gateway.snapshot();
    else if(req.method==='PUT'&&route==='/api/home')result=gateway.save(body);
    else if(req.method==='GET'&&route==='/api/export')result=gateway.export();
    else if(req.method==='POST'&&route==='/api/check')result=await gateway.check(id(body.id));
    else if(req.method==='POST'&&route==='/api/rule-test')result=gateway.testRule(id(body.id),body.state);
    else if(req.method==='POST'&&route==='/api/ack')result=gateway.acknowledge(body.id);
    else fail('API route not found.',404);
    res.writeHead(200);res.end(JSON.stringify(result));
  }catch(error){res.writeHead(error.status??500);res.end(JSON.stringify({error:error.status?error.message:'Gateway request failed. Check the local service log.'}));if(!error.status)console.error(error);}
  return true;
}
