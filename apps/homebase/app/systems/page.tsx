'use client';
import Link from 'next/link';
import { useRef, useState } from 'react';
import { ArrowLeft, ArrowRight, Activity, Network, Settings2, Bell, Plus, Download } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Tabs, TabsList, TabsTrigger, TabsContent } from '@/components/ui/tabs';
import { useGateway, saveGateway, refreshGateway, gatewayAction, gatewayRequest, healthLabel, type HomeSnapshot, type RegisteredDevice, type Configuration, type Rule, type Relation } from '@/lib/gateway';
import { devices as originalDevices, parseConfigs, storageKey } from '@/lib/home';
import './systems.css';

const newId = (prefix:string) => `${prefix}.${crypto.randomUUID().slice(0,8)}`;
const time = (value:string) => new Date(value).toLocaleString();
function config(data:HomeSnapshot):Configuration {return {revision:data.revision,rooms:data.rooms,devices:data.devices,relations:data.relations,rules:data.rules};}
export default function Systems() {
  const {data,error}=useGateway();
  const [selected,setSelected]=useState('host.octopi');
  const [notice,setNotice]=useState('');
  const [failure,setFailure]=useState('');
  const [busy,setBusy]=useState(false);
  const [imported,setImported]=useState<Configuration|null>(null);
  const file=useRef<HTMLInputElement>(null);
  async function work(action:()=>Promise<unknown>,message:string){if(busy)return;setBusy(true);setFailure('');try{await action();setNotice(message);}catch(e){setFailure(e instanceof Error?e.message:'Request failed.');}finally{setBusy(false);}}
  async function importFile(upload?:File){
    if(!upload||!data)return;
    try{if(upload.size>200000)throw Error('Choose a settings backup smaller than 200 KB.');const value=JSON.parse(await upload.text());
      if(value.version!==1||!Array.isArray(value.devices)||!Array.isArray(value.rooms)||!Array.isArray(value.rules)||!Array.isArray(value.relations))throw Error('Invalid version 1 settings backup.');
      setImported({...value,revision:data.revision,devices:value.devices.map((d:RegisteredDevice)=>({...d,monitor:false})),rules:value.rules.map((r:Rule)=>({...r,enabled:false}))});
    }catch(e){setFailure(e instanceof Error?e.message:'Import failed.');}finally{if(file.current)file.current.value='';}
  }
  return <main className="systems-workspace">
    <nav className="arm-navigation"><Link href="/" className="arm-back"><ArrowLeft size={16}/> Homebase</Link><span>Systems / Local gateway</span></nav>
    <header className="arm-heading"><div><p className="arm-eyebrow">HOME OPERATIONS</p><h1>Systems<span className="arm-title-dot">.</span></h1><p>Devices, connections, and the events between them.</p></div><div className="system-service"><span className={data&&!error?'service-ready':'service-unavailable'}/>{error?'Gateway unavailable':data?'Local gateway ready':'Connecting to gateway…'}<small>Settings and history saved on this PC</small></div></header>
    {(error||failure)&&<p className="system-alert" role="alert">{failure||error}</p>}
    {notice&&<output className="system-notice" aria-live="polite">{notice}</output>}
    {!data?<section className="arm-card"><h2>Connecting to the home registry</h2><p className="field-help">This workspace needs the local Homebase service. Device credentials are not requested here.</p><Button onClick={()=>void refreshGateway()}>Retry</Button></section>:<>
    <div className="system-summary"><div><strong>{data.devices.length}</strong><span>Devices in registry</span></div><div><strong>{data.devices.filter(d=>d.monitor).length}</strong><span>Network monitors enabled</span></div><div><strong>{data.rules.filter(r=>r.enabled).length}</strong><span>Local notification rules</span></div><div><strong>{data.notifications.filter(n=>!n.read).length}</strong><span>Unread notifications</span></div></div>
    <Tabs defaultValue="devices">
      <div className="system-navigation"><TabsList className="view-tabs"><TabsTrigger value="devices"><Settings2/> Devices</TabsTrigger><TabsTrigger value="rooms"><Network/> Rooms & links</TabsTrigger><TabsTrigger value="rules"><Bell/> Automations</TabsTrigger><TabsTrigger value="activity"><Activity/> Activity</TabsTrigger></TabsList>
      <div className="system-actions"><a className="system-download" href="/api/export" download="homebase-settings.json"><Download size={15}/> Backup settings</a><Button variant="outline" disabled={busy||!!error} onClick={()=>file.current?.click()}>Import</Button><input ref={file} type="file" accept="application/json,.json" hidden onChange={e=>void importFile(e.target.files?.[0])}/></div></div>
      {imported&&<section className="system-import"><strong>Review settings restore</strong><p>{imported.devices.length} devices · {imported.rooms.length} rooms · {imported.rules.length} rules. This replaces current settings. Monitoring and rules will be off; history is retained.</p><Button disabled={busy||!!error} onClick={()=>void work(async()=>{await saveGateway(imported);setImported(null);},'Settings restored. Monitoring and rules are off.')}>Restore these settings</Button><Button variant="outline" onClick={()=>setImported(null)}>Cancel</Button></section>}
      <TabsContent value="devices"><div className="system-device-layout">
        <section className="system-card"><div className="system-section-heading"><h2>Device registry</h2><Button variant="outline" disabled={busy||!!error} onClick={()=>{const id=newId('device');void work(async()=>{await saveGateway({...config(data),devices:[...data.devices,{id,name:'New device',model:'',kind:'network',room:'Unassigned',adapter:'Not configured',endpoint:'',notes:'',next:'Configure its name, room and endpoint.',monitor:false}]});setSelected(id);},'Device added.');}}><Plus size={15}/> Add</Button></div>
          <div className="system-device-list">{data.devices.map(d=><button className={`system-device-row ${selected===d.id?'selected':''}`} key={d.id} onClick={()=>setSelected(d.id)} aria-pressed={selected===d.id}><span><strong>{d.name}</strong><small>{d.room} · {d.model||d.kind}</small></span><span className={`system-health ${data.health[d.id]?.state??'unknown'}`}>{healthLabel(d,data.health[d.id],data.serverTime)}</span></button>)}</div>
          <details className="system-details"><summary>Bring in older browser settings</summary><p className="field-help">Review the existing browser names, rooms and endpoints before using them in this registry. Nothing contacts a device.</p><Button disabled={busy||!!error} variant="outline" onClick={()=>{try{const old=parseConfigs(localStorage.getItem(storageKey));if(!Object.keys(old).length)throw Error('No older browser settings found.');const devices=data.devices.map(d=>({...d,...old[d.id],monitor:false}));setImported({...config(data),rooms:[...new Set([...data.rooms,...devices.map(d=>d.room)])],devices,rules:data.rules.map(r=>({...r,enabled:false}))});}catch(e){setFailure(e instanceof Error?e.message:'Could not read browser settings.');}}}>Review browser import</Button></details>
        </section>
        {data.devices.find(d=>d.id===selected)&&<DeviceEditor key={`${selected}-${data.devices.find(d=>d.id===selected)?.id}`} data={data} device={data.devices.find(d=>d.id===selected)!} disabled={busy||!!error} work={work}/>}</div></TabsContent>
      <TabsContent value="rooms"><RoomsAndLinks data={data} disabled={busy||!!error} work={work}/></TabsContent>
      <TabsContent value="rules"><Rules data={data} disabled={busy||!!error} work={work}/></TabsContent>
      <TabsContent value="activity"><div className="system-two-panels"><section className="system-card"><h2>Notifications</h2><p className="field-help">Local dashboard notifications. No email, text messages, or device actions.</p>{!data.notifications.length&&<p className="system-empty">No notifications yet. Build a connection rule or test its conditions in Automations.</p>}{data.notifications.map(n=><article className="system-event" key={n.id}><time>{time(n.at)}</time><p>{n.message}</p>{!n.read?<Button variant="outline" disabled={busy||!!error} onClick={()=>void work(()=>gatewayAction('ack',n.id),'Notification marked read.')}>Mark read</Button>:<small>Read</small>}</article>)}</section>
        <section className="system-card"><h2>Activity history</h2><p className="field-help">Latest 100 events. The service retains up to 2,000 events.</p>{data.events.map(e=><article className="system-event" key={e.id}><time>{time(e.at)}</time><p>{e.message}</p><small>{e.source} · {e.type}</small></article>)}</section></div></TabsContent>
    </Tabs>
    <footer><span>Registry revision {data.revision} · Gateway started {time(data.startedAt)}</span><span>Port reachability is not appliance state or device identity.</span></footer>
    </>}
  </main>;
}
type Work=(action:()=>Promise<unknown>,message:string)=>Promise<void>;
function DeviceEditor({data,device,disabled,work}:{data:HomeSnapshot;device:RegisteredDevice;disabled:boolean;work:Work}){
  const [draft,setDraft]=useState(device);const [revision,setRevision]=useState(data.revision);
  const health=data.health[device.id];
  const patch=(value:Partial<RegisteredDevice>)=>setDraft(d=>({...d,...value}));
  return <section className="system-card"><p className="eyebrow">DEVICE SETTINGS</p><h2>{device.name}</h2><p className="field-help">{device.id}</p>
    <form className="system-form" onSubmit={e=>{e.preventDefault();void work(async()=>{const saved=await saveGateway({...config(data),revision,devices:data.devices.map(d=>d.id===draft.id?draft:d)});setRevision(saved.revision);},'Device settings saved.');}}>
      <label>Name<input value={draft.name} maxLength={80} required onChange={e=>patch({name:e.target.value})}/></label>
      <label>Model<input value={draft.model} maxLength={160} onChange={e=>patch({model:e.target.value})}/></label>
      <div className="system-form-pair"><label>Room<select value={draft.room} onChange={e=>patch({room:e.target.value})}>{data.rooms.map(r=><option key={r}>{r}</option>)}</select></label><label>Device type<select value={draft.kind} onChange={e=>patch({kind:e.target.value as RegisteredDevice['kind']})}>{['network','camera','robot','shade','appliance','printer'].map(k=><option key={k}>{k}</option>)}</select></label></div>
      <label>Endpoint<input type="url" value={draft.endpoint} maxLength={300} placeholder="http://192.168.1.100" onChange={e=>patch({endpoint:e.target.value})}/></label>
      <p className="field-help">Use an address without passwords or tokens. Vendor account connections will be separate adapters.</p>
      <label className="system-checkbox"><input type="checkbox" checked={draft.monitor} onChange={e=>patch({monitor:e.target.checked})}/> Check this network port every 30 seconds</label>
      <p className="field-help">Requires a private LAN IPv4 address. TCP checks send no HTTP requests or device commands.</p>
      <label>Notes<textarea value={draft.notes} rows={4} maxLength={2000} onChange={e=>patch({notes:e.target.value})}/></label>
      <div className="system-actions"><Button type="submit" disabled={disabled}>Save device</Button><Button type="button" variant="outline" disabled={disabled} onClick={()=>{setDraft(device);setRevision(data.revision);}}>Reload saved</Button></div>
    </form>
    <div className="system-observation"><h3>Latest network observation</h3><strong>{healthLabel(device,health,data.serverTime)}</strong>{health&&<><p>{time(health.observedAt)} · {health.latencyMs} ms</p><small>{health.source} · {health.endpoint}</small></>}
      <Button variant="outline" disabled={disabled||!device.endpoint} onClick={()=>void work(()=>gatewayAction('check',device.id),'Network check completed. This checks the saved endpoint only.')}>Check saved endpoint now</Button></div>
    {!originalDevices.some(d=>d.id===device.id)&&<p className="field-help">This custom device is available in the house inventory and automation selectors.</p>}
  </section>;
}
function RoomsAndLinks({data,disabled,work}:{data:HomeSnapshot;disabled:boolean;work:Work}){
  const [room,setRoom]=useState('');const [from,setFrom]=useState(data.devices[0].id);const [to,setTo]=useState(data.devices[1]?.id??'');const [label,setLabel]=useState('connected to');const [type,setType]=useState<Relation['type']>('network');
  const [selected,setSelected]=useState(data.devices[0].id);
  const positions=Object.fromEntries(data.devices.map((d,i)=>{const angle=(i/data.devices.length)*Math.PI*2-Math.PI/2;return[d.id,{x:550+410*Math.cos(angle),y:330+235*Math.sin(angle)}];}));
  return <div className="system-two-panels system-map-layout"><section className="system-card"><h2>Rooms</h2><p className="field-help">Room membership is configured here; it is not live location tracking.</p><div className="system-room-list">{data.rooms.map(r=><div key={r}><strong>{r}</strong><span>{data.devices.filter(d=>d.room===r).length} devices</span></div>)}</div>
    <form className="system-form" onSubmit={e=>{e.preventDefault();void work(async()=>{await saveGateway({...config(data),rooms:[...data.rooms,room.trim()]});setRoom('');},'Room added to the house and device selectors.');}}><label>New room<input required maxLength={60} value={room} onChange={e=>setRoom(e.target.value)}/></label><Button disabled={disabled||!room.trim()}>Add room</Button></form>
    <h2 className="system-subheading">Add a relationship</h2><form className="system-form" onSubmit={e=>{e.preventDefault();void work(()=>saveGateway({...config(data),relations:[...data.relations,{from,to,label,type}]}),'Relationship saved.');}}><label>From<select value={from} onChange={e=>setFrom(e.target.value)}>{data.devices.map(d=><option value={d.id} key={d.id}>{d.name}</option>)}</select></label><label>Relationship<input required maxLength={60} value={label} onChange={e=>setLabel(e.target.value)}/></label><label>To<select value={to} onChange={e=>setTo(e.target.value)}>{data.devices.map(d=><option value={d.id} key={d.id}>{d.name}</option>)}</select></label><label>Type<select value={type} onChange={e=>setType(e.target.value as Relation['type'])}><option value="network">Network connection</option><option value="physical">Physical attachment</option><option value="logical">Logical dependency</option></select></label><Button disabled={disabled||from===to}>Save relationship</Button></form>
    </section><section className="system-card"><h2>Connection graph</h2><p className="field-help">Select a device to highlight its recorded relationships. These links are owner configured, not automatically discovered.</p>
    <div className="system-graph-scroll"><svg viewBox="0 0 1100 660" className="system-graph" aria-label="Recorded device relationships">
      {data.relations.map((r,i)=><line key={i} x1={positions[r.from].x} y1={positions[r.from].y} x2={positions[r.to].x} y2={positions[r.to].y} className={`${r.type} ${r.from===selected||r.to===selected?'highlight':''}`}/>)}
      {/* eslint-disable-next-line jsx-a11y/prefer-tag-over-role -- SVG groups use keyboard handlers and button semantics for graph selection. */}
      {data.devices.map(d=><g key={d.id} role="button" tabIndex={0} aria-label={`Select ${d.name}`} aria-pressed={selected===d.id} onClick={()=>setSelected(d.id)} onKeyDown={e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();setSelected(d.id);}}} className={selected===d.id?'graph-selected':''} transform={`translate(${positions[d.id].x},${positions[d.id].y})`}><rect x="-78" y="-25" width="156" height="50" rx="10"/><text textAnchor="middle" y="-1">{d.name.length>21?d.name.slice(0,20)+'…':d.name}</text><text className="graph-room" textAnchor="middle" y="17">{d.room.length>24?d.room.slice(0,23)+'…':d.room}</text><title>{d.name} · {d.room}</title></g>)}
    </svg></div><div className="system-graph-legend"><span>— Physical</span><span>┄ Network</span><span>··· Logical</span></div>
    {data.relations.filter(r=>r.from===selected||r.to===selected).map(r=><div className="system-relation" key={`${r.from}-${r.to}-${r.type}-${r.label}`}><p>{data.devices.find(d=>d.id===r.from)?.name} <ArrowRight size={13}/> {r.label} <ArrowRight size={13}/> {data.devices.find(d=>d.id===r.to)?.name}<small>{r.type} · owner configured</small></p><Button variant="outline" disabled={disabled} onClick={()=>void work(()=>saveGateway({...config(data),relations:data.relations.filter(item=>item!==r)}),'Relationship removed.')}>Remove</Button></div>)}
    {!data.relations.some(r=>r.from===selected||r.to===selected)&&<p className="system-empty">No relationships recorded for this device.</p>}
    </section></div>;
}
function Rules({data,disabled,work}:{data:HomeSnapshot;disabled:boolean;work:Work}){
  const [name,setName]=useState('');const [deviceId,setDeviceId]=useState('host.octopi');const [when,setWhen]=useState<Rule['when']>('unreachable');const [result,setResult]=useState('');
  return <div className="system-two-panels"><section className="system-card"><p className="eyebrow">WHEN → THEN</p><h2>Connection notifications</h2><p className="field-help">When a port changes state, add a notification to Homebase. Rules run in the local service even when the dashboard is closed.</p>
    <form className="system-form" onSubmit={e=>{e.preventDefault();void work(async()=>{await saveGateway({...config(data),rules:[...data.rules,{id:newId('rule'),name,deviceId,when,enabled:false}]});setName('');},'Rule saved disabled. Test it, then enable it when ready.');}}><label>Rule name<input required maxLength={100} value={name} onChange={e=>setName(e.target.value)} placeholder="Tell me when OctoPi becomes unreachable"/></label><label>When this device<select value={deviceId} onChange={e=>setDeviceId(e.target.value)}>{data.devices.map(d=><option value={d.id} key={d.id}>{d.name}</option>)}</select></label><label>Reports this port state<select value={when} onChange={e=>setWhen(e.target.value as Rule['when'])}><option value="unreachable">Unreachable</option><option value="reachable">Reachable</option></select></label><div className="system-rule-action">Then <strong>Create a local notification</strong></div><Button disabled={disabled||!name.trim()}>Save rule</Button></form>
    <p className="field-help">Checks must be enabled on the device for background monitoring. Repeated unchanged results do not repeat notifications. A first matching observation also triggers an enabled rule.</p></section>
    <section className="system-card"><h2>Saved automations</h2>{!data.rules.length&&<p className="system-empty">Create a rule and try it with simulated states. Tests never change device observations or create real notifications.</p>}{data.rules.map(rule=><article className="system-rule" key={rule.id}><h3>{rule.name}</h3><p>{data.devices.find(d=>d.id===rule.deviceId)?.name} → {rule.when} → local notification</p><label className="system-checkbox"><input type="checkbox" checked={rule.enabled} disabled={disabled} onChange={e=>void work(()=>saveGateway({...config(data),rules:data.rules.map(r=>r.id===rule.id?{...r,enabled:e.target.checked}:r)}),'Rule setting saved.')}/> Enabled</label><div className="system-actions">{(['reachable','unreachable'] as const).map(state=><Button key={state} variant="outline" disabled={disabled} onClick={()=>void work(async()=>{const test=await gatewayRequest<{message:string;enabled:boolean}>('rule-test','POST',{id:rule.id,state});setResult(`${rule.name} · simulated ${state}: ${test.message}${test.enabled?'':' Rule is currently disabled.'}`);},'Simulation completed; no real observation or notification was created.')}>Test {state}</Button>)}<Button variant="outline" disabled={disabled} onClick={()=>void work(()=>saveGateway({...config(data),rules:data.rules.filter(r=>r.id!==rule.id)}),'Rule removed.')}>Remove</Button></div></article>)}{result&&<output className="system-simulation" aria-live="polite">{result}</output>}</section></div>;
}
