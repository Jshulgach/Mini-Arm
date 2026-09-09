import test from 'node:test';
import assert from 'node:assert/strict';
import { planArmHome, homeArm } from '../lib/arm-home.ts';
import { recoveredJoints } from '../lib/arm-calibration.ts';
import { picoFilter } from '../lib/arm-serial.ts';

const channels = Object.fromEntries(recoveredJoints.map(j => [j.id,j.channel]));
function states() {
  return Object.fromEntries(recoveredJoints.map(j => [j.id, {ok:true, protocol:`miniarm-${j.id}-pulse-v1`, channel:j.channel,
    center:null,last:null,active:false,fault:null,outputs:Array(7).fill(null),duration_ms:400,max_step:1,window:5,locked:false}]));
}
test('Home uses the six existing model-zero servo references and excludes the gripper', () => {
  const plan = planArmHome(states(),channels);
  assert.deepEqual(plan.map(p=>p.target),[90,90,45,90,110,90]);
  assert.equal(plan.length,6); assert.ok(plan.every(p=>p.joint!=='gripper'));
});
test('Home skips unchanged commands and validates all bounds and mappings before motion', async () => {
  const known = states();
  for (const j of recoveredJoints) if (j.neutral!==null) Object.assign(known[j.id],{last:j.neutral,center:j.neutral,locked:true});
  assert.deepEqual(planArmHome(known,channels),[]);
  known['gripper-link'].center=30; known['gripper-link'].last=30;
  const untouched = {open:()=>assert.fail('Preflight must finish before USB opens')};
  await assert.rejects(homeArm(untouched,new AbortController().signal,known,channels,()=>{}),/outside/);
  assert.throws(()=>planArmHome(states(),{...channels,elbow:3}),/channel/);
  const faulted=states(); faulted.gripper.fault='bus error';
  assert.throws(()=>planArmHome(faulted,channels),/outputs off/);
});
test('cancel after the first Home result never commands the next joint', async () => {
  const stop = new AbortController(); const initial=states();
  let input; const commands=[];
  const port={getInfo:()=>picoFilter,async open(){
    this.readable=new ReadableStream({start(c){input=c;}});
    this.writable=new WritableStream({write(bytes){
      const command=new TextDecoder().decode(bytes); commands.push(command);
      assert.match(command,/;cal_pulse:[a-f0-9-]+:90:none;/);
      const fence=command.match(/^sync:(.*)-b;/)[1];
      const result={...initial.base,last:90,center:90,locked:true};
      input.enqueue(new TextEncoder().encode(`@sync:${fence}-b\n@cal:${JSON.stringify(result)}\n@sync:${fence}-e\n`));
    }});
  },async close(){assert.equal(this.readable.locked,false);assert.equal(this.writable.locked,false);}};
  await assert.rejects(homeArm(port,stop.signal,initial,channels,(_joint,status)=>{assert.equal(status.active,false);stop.abort();}),{name:'AbortError'});
  assert.equal(commands.length,1);
});
