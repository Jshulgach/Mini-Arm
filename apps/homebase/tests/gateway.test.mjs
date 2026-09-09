import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname, resolve } from 'node:path';
import { Readable } from 'node:stream';
import { createGateway, initialConfiguration, validateConfiguration, networkTarget, handleGateway } from '../scripts/gateway.mjs';

test('gateway starts without monitoring, validates endpoints, identifiers and relationships',()=>{
  const seed=initialConfiguration();
  assert.ok(seed.devices.every(d=>!d.monitor));assert.equal(seed.rules.length,0);
  for(const address of ['http://127.0.0.1','http://169.254.169.254','http://8.8.8.8','http://localhost','http://[::1]','http://user:password@192.168.1.1','file:///tmp/file',''])assert.throws(()=>networkTarget(address));
  assert.deepEqual(networkTarget('http://192.168.1.159:5000/api/printer'),{host:'192.168.1.159',port:5000});
  assert.throws(()=>validateConfiguration({...seed,rooms:['Kitchen']}));
  assert.throws(()=>validateConfiguration({...seed,devices:[seed.devices[0],seed.devices[0]]}));
  assert.throws(()=>validateConfiguration({...seed,relations:[{from:'missing',to:seed.devices[0].id,label:'link',type:'network'}]}));
  assert.throws(()=>validateConfiguration({...seed,devices:seed.devices.map((d,i)=>i?d:{...d,endpoint:'http://192.168.1.1?token=secret'})}));
});

test('registry survives restart, rejects stale writes, and exports only configuration',async()=>{
  const directory=mkdtempSync(join(tmpdir(),'homebase-registry-'));const file=join(directory,'test.sqlite');
  let gateway=createGateway(file,{dial:async()=>assert.fail('No device should be contacted')});
  try{
    const old=gateway.snapshot();const next=gateway.save({...old,rooms:[...old.rooms,'Office']});
    assert.equal(next.revision,old.revision+1);assert.throws(()=>gateway.save(old),/another window/);
    await gateway.poll();const exported=gateway.export();assert.ok(!('health'in exported)&&!('events'in exported));
    await gateway.close();gateway=createGateway(file);
    assert.ok(gateway.snapshot().rooms.includes('Office'));assert.equal(gateway.snapshot().revision,next.revision);
  }finally{await gateway.close();assert.equal(dirname(resolve(directory)),resolve(tmpdir()));rmSync(directory,{recursive:true,force:true});}
});

test('observations, transition notifications, acknowledgments and simulations stay distinct',async()=>{
  let reachable=false,calls=0,clock=100000;
  const gateway=createGateway(':memory:',{dial:async()=>{calls++;return reachable;},now:()=>clock});
  try{
    const first=gateway.snapshot();gateway.save({...first,rules:[{id:'rule.offline',name:'OctoPi offline',deviceId:'host.octopi',when:'unreachable',enabled:true}]});
    const before=gateway.snapshot();const simulation=gateway.testRule('rule.offline','unreachable');
    assert.equal(simulation.matches,true);assert.equal(calls,0);assert.deepEqual(gateway.snapshot().health,{});assert.equal(gateway.snapshot().events.length,before.events.length);
    await gateway.check('host.octopi');let state=gateway.snapshot();assert.equal(state.health['host.octopi'].state,'unreachable');assert.equal(state.notifications.length,1);
    clock+=30000;await gateway.check('host.octopi');assert.equal(gateway.snapshot().notifications.length,1,'unchanged failures do not spam');
    reachable=true;clock+=30000;await gateway.check('host.octopi');assert.equal(gateway.snapshot().notifications.length,1);
    reachable=false;clock+=30000;await gateway.check('host.octopi');assert.equal(gateway.snapshot().notifications.length,2);
    state=gateway.snapshot();gateway.acknowledge(state.notifications[0].id);assert.equal(gateway.snapshot().notifications[0].read,true);
    gateway.save({...gateway.snapshot(),devices:state.devices.map(d=>d.id==='host.octopi'?{...d,endpoint:'http://192.168.1.160'}:d)});
    assert.equal(gateway.snapshot().health['host.octopi'],undefined,'endpoint changes invalidate old observations');
  }finally{await gateway.close();}
});

test('pending checks coalesce and cannot overwrite a changed endpoint',async()=>{
  let finish,calls=0;const gateway=createGateway(':memory:',{dial:()=>{calls++;return new Promise(resolve=>{finish=resolve;});}});
  try{
    const a=gateway.check('host.octopi'),b=gateway.check('host.octopi');assert.equal(calls,1);
    const state=gateway.snapshot();gateway.save({...state,devices:state.devices.map(d=>d.id==='host.octopi'?{...d,endpoint:'http://192.168.1.160'}:d)});
    finish(true);await Promise.all([a,b]);assert.equal(gateway.snapshot().health['host.octopi'],undefined);
  }finally{await gateway.close();}
});

async function request(gateway,path,method='GET',body,headers={}){
  const req=Readable.from(body===undefined?[]:[Buffer.from(JSON.stringify(body))]);req.method=method;req.url=path;req.headers={host:'127.0.0.1:5173',...(body===undefined?{}:{'content-type':'application/json'}),...headers};
  const res={status:0,text:'',setHeader(){},writeHead(code){this.status=code;},end(value){this.text=value;}};
  await handleGateway(req,res,gateway);return {status:res.status,value:JSON.parse(res.text)};
}
test('local API rejects cross-site mutation, DNS rebinding, bad bodies and unsupported actions',async()=>{
  let calls=0;const gateway=createGateway(':memory:',{dial:async()=>{calls++;return true;}});
  try{
    assert.equal((await request(gateway,'/api/home')).status,200);
    assert.equal((await request(gateway,'/api/home','GET',undefined,{host:'attacker.example:5173'})).status,403);
    assert.equal((await request(gateway,'/api/home','PUT',gateway.snapshot(),{origin:'https://attacker.example'})).status,403);
    assert.equal((await request(gateway,'/api/check','POST',{id:'host.octopi'},{'sec-fetch-site':'cross-site'})).status,403);
    assert.equal((await request(gateway,'/api/home','PUT',[],{})).status,400);
    assert.equal((await request(gateway,'/api/check','POST',{id:'host.octopi'},{'content-type':'text/plain'})).status,415);
    assert.equal((await request(gateway,'/api/move','POST',{})).status,404);assert.equal(calls,0);
    assert.equal((await request(gateway,'/api/check','POST',{id:'host.octopi'})).status,200);assert.equal(calls,1);
  }finally{await gateway.close();}
});
