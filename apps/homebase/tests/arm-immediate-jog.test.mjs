import test from 'node:test';
import assert from 'node:assert/strict';
import { picoFilter, sweepJoint } from '../lib/arm-serial.ts';

function ready(initial = false) {
  return {ok:true,protocol:'miniarm-base-pulse-v1',channel:15,duration_ms:400,max_step:1,window:5,
    center:initial?null:90,last:initial?null:90,locked:!initial,active:false,fault:null,outputs:Array(7).fill(null),
    mode:null,active_token:null,sweep_protocol:'miniarm-base-sweep-v1',sweep_ms:3000,sweep_watchdog_ms:600,sweep_rate:10,
    sweep_target:null,sweep_sequence:0,jog_protocol:'miniarm-base-jog-v1'};
}
function fakeJog(initial=false) {
  let state=ready(initial);
  const port={writes:[],getInfo:()=>picoFilter,async open(){
    let input;
    this.readable=new ReadableStream({start(c){input=c;}});
    this.writable=new WritableStream({write(bytes){
      const command=new TextDecoder().decode(bytes);port.writes.push(command);
      const fence=command.match(/^sync:(.*)-b;/)[1];
      const [name,token,a,b]=command.split(';')[1].split(':');
      if(name==='cal_jog'){
        assert.equal(b,String(state.last??'none'));
        state={...state,center:state.center??Number(a),last:state.last??Number(a),locked:true,active:true,
          mode:'sweep',active_token:token,sweep_target:Number(a),sweep_sequence:0,outputs:[state.last??Number(a),...Array(6).fill(null)]};
      }else if(name==='cal_keep'||name==='cal_target'){
        assert.equal(token,state.active_token);assert.equal(Number(a),state.sweep_sequence+1);
        state={...state,sweep_sequence:Number(a),sweep_target:name==='cal_target'?Number(b):state.sweep_target};
        if(name==='cal_keep') assert.equal(b,undefined,'Keep-alives cannot carry positions');
      }else if(name==='cal_release') state={...state,active:false,mode:null,active_token:null,outputs:Array(7).fill(null)};
      else assert.fail('Unexpected command '+name);
      input.enqueue(new TextEncoder().encode(`@sync:${fence}-b\n@cal:${JSON.stringify(state)}\n@sync:${fence}-e\n`));
    }});
  },async close(){assert.equal(this.readable.locked,false);assert.equal(this.writable.locked,false);}};
  return port;
}
test('immediate jog sends positions only on change and uses position-free keep-alives otherwise',async()=>{
  const port=fakeJog();const stop=new AbortController();let target=92;
  await sweepJoint(port,stop.signal,ready(),()=>target,status=>{
    if(status.sweep_sequence===1) target=93;
    if(status.sweep_sequence===3) stop.abort();
  },true);
  assert.equal(port.writes.length,5);
  assert.match(port.writes[0],/;cal_jog:[a-f0-9-]+:92:90;/);
  assert.match(port.writes[1],/;cal_keep:[a-f0-9-]+:1;/);
  assert.match(port.writes[2],/;cal_target:[a-f0-9-]+:2:93;/);
  assert.match(port.writes[3],/;cal_keep:[a-f0-9-]+:3;/);
  assert.match(port.writes[4],/;cal_release:/);
});
test('unchanged idle position sends nothing and new initial target requires explicit value',async()=>{
  const idle=fakeJog();
  await sweepJoint(idle,new AbortController().signal,ready(),()=>90,()=>assert.fail('No motion callback expected'),true);
  assert.equal(idle.writes.length,0);
  const initial=fakeJog(true);const stop=new AbortController();
  const result=await sweepJoint(initial,stop.signal,ready(true),()=>80,s=>{if(s.sweep_sequence===1)stop.abort();},true);
  assert.match(initial.writes[0],/cal_jog:[a-f0-9-]+:80:none;/);
  assert.equal(result.center,80);assert.equal(result.last,80);assert.equal(result.active,false);
  const unsupported=fakeJog();
  await assert.rejects(sweepJoint(unsupported,new AbortController().signal,{...ready(),jog_protocol:undefined},()=>92,()=>{},true),/update/);
  assert.equal(unsupported.writes.length,0);
});
