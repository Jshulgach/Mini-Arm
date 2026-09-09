import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { createRequire } from 'node:module';
import { readFile, mkdir, mkdtemp, writeFile, unlink, rmdir } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// Exercise the actual component handlers with deterministic hooks and deferred
// serial operations. This is not browser rendering or a hardware connection.
test('Connect/Disconnect waits for jog and Home cleanup, clears state, and preserves release errors', async () => {
  const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
  const source = (await readFile(path.join(root, 'app/arm/base-test.tsx'), 'utf8'))
    .replace("from 'react'", "from 'test-hooks'") + '\nexport { harness } from "test-hooks";';
  const hooks = `
    export const harness = { slots:[], cursor:0, effects:[], operation:null, requests:0, port:{}, listeners:{} };
    export function useState(initial) { const i=harness.cursor++; if(!(i in harness.slots)) harness.slots[i]=initial;
      return [harness.slots[i], value=>{harness.slots[i]=typeof value==='function'?value(harness.slots[i]):value;}]; }
    export function useRef(initial) { return useState({current:initial})[0]; }
    export function useEffect(effect) { const ref=useRef(false); if(!ref.current){ref.current=true;harness.effects.push(effect);} }
    export function useSyncExternalStore() { return true; }
    export function begin(signal, previous) { return new Promise((resolve,reject)=>{harness.operation={signal,previous,resolve,reject};}); }
  `;
  const serial = `
    import {harness,begin} from 'test-hooks';
    import {recoveredJoints} from '@/lib/arm-calibration';
    export const picoFilter={};
    export const browserSerial=()=>({requestPort:async()=>{harness.requests++;return harness.port;},
      addEventListener:(name,callback)=>{harness.listeners[name]=callback;},removeEventListener:(name)=>{delete harness.listeners[name];}});
    export const probeAllJoints=async()=>Object.fromEntries(recoveredJoints.map(j=>[j.id,{
      protocol:'miniarm-'+j.id+'-pulse-v1',jog_protocol:'miniarm-'+j.id+'-jog-v1',channel:j.channel,
      last:j.neutral??90,center:j.neutral??90,active:false,fault:null,outputs:Array(7).fill(null)}]));
    export const sweepJoint=(port,signal,previous,target,update)=>{
      update({...previous,active:true});const pending=begin(signal,previous);harness.operation.target=target;return pending;};
    export const pulseBaseTest=()=>{throw Error('Unexpected pulse');};
    export const holdBaseTest=pulseBaseTest,moveJointTarget=pulseBaseTest;
  `;
  const modules = {
    'test-hooks': hooks,
    '@/lib/arm-serial': serial,
    '@/lib/arm-home': `import {begin} from 'test-hooks';export const homeArm=(port,signal)=>begin(signal,null);`,
    '@/components/ui/button': `export const Button='button';`,
  };
  const bundle = await build({absWorkingDir:root, stdin:{contents:source,loader:'tsx',resolveDir:root}, bundle:true,
    write:false,platform:'node',format:'cjs',packages:'external',alias:{'@':root},jsx:'automatic',logLevel:'silent',
    plugins:[{name:'control-fixtures',setup(b){
      b.onResolve({filter:/^(test-hooks|@\/lib\/arm-serial|@\/lib\/arm-home|@\/components\/ui\/button)$/},args=>({path:args.path,namespace:'fixture'}));
      b.onLoad({filter:/.*/,namespace:'fixture'},args=>({contents:modules[args.path],loader:'js',resolveDir:root}));
    }}]});
  const cache=path.join(root,'node_modules','.cache');await mkdir(cache,{recursive:true});
  const directory=await mkdtemp(path.join(cache,'arm-disconnect-'));const filename=path.join(directory,'test.cjs');
  const require=createRequire(import.meta.url);
  const oldWindow=Object.getOwnPropertyDescriptor(globalThis,'window'),oldDocument=Object.getOwnPropertyDescriptor(globalThis,'document');
  const events={addEventListener(){},removeEventListener(){}};
  const channels={base:15,shoulder:14,elbow:13,'wrist-rotation':12,'wrist-bend':2,'gripper-link':1,gripper:0};
  try {
    await writeFile(filename,bundle.outputFiles[0].text);
    Object.defineProperty(globalThis,'window',{configurable:true,value:events});
    Object.defineProperty(globalThis,'document',{configurable:true,value:{...events,hidden:false}});
    const {BaseTest,harness}=require(filename);
    let nodes=[];
    function walk(node){if(Array.isArray(node))node.forEach(walk);else if(node?.props){nodes.push(node);walk(node.props.children);}}
    function render(){harness.cursor=0;nodes=[];walk(BaseTest({channels}));harness.effects.splice(0).forEach(effect=>effect());}
    const button=label=>nodes.find(n=>n.type==='button'&&n.props.children===label);
    const input=label=>nodes.find(n=>n.props['aria-label']===label);
    const tick=()=>new Promise(resolve=>setImmediate(resolve));
    async function connect(){render();assert.ok(button('Connect'));button('Connect').props.onClick();await tick();render();assert.ok(button('Disconnect'));}
    function enable(){input('Jog enabled').props.onChange({target:{checked:true}});render();}
    await connect();
    button('Disconnect').props.onClick();render();
    assert.ok(button('Connect'));assert.equal(input('Jog enabled').props.checked,false);
    assert.equal(input('Base target degrees').props.value,'');assert.equal(harness.operation,null);
    for(const mode of ['jog','home','release-error']){
      await connect();enable();
      if(mode==='home')button('Home').props.onClick();
      else {
        const change=input('Base jog').props.onChange;
        change({target:{value:'91'}});
        change({target:{value:'90'}});
        assert.equal(harness.operation.target(),90,'A reversal before React redraws must replace the pending target');
      }
      render();const operation=harness.operation;
      assert.ok(operation);assert.equal(button('Disconnect').props.disabled,false,'Disconnect remains available while moving');
      button('Disconnect').props.onClick();render();
      assert.equal(operation.signal.aborted,true);
      assert.equal(input('Jog enabled').props.checked,false);
      assert.equal(button('Disconnecting…').props.disabled,true);
      assert.equal(button('Connect'),undefined,'Cannot connect while release/port cleanup is pending');
      if(mode==='home')operation.reject(new DOMException('Canceled','AbortError'));
      else if(mode==='release-error')operation.reject(new Error('Release unconfirmed'));
      else operation.resolve({...operation.previous,active:false,outputs:Array(7).fill(null)});
      await tick();render();
      assert.ok(button('Connect'));assert.equal(input('Base jog').props.disabled,true);
      assert.equal(input('Base target degrees').props.value,'');
      assert.equal(input('Jog enabled').props.checked,false);
      if(mode==='release-error')assert.ok(nodes.some(n=>n.props.role==='alert'&&n.props.children.includes('Release unconfirmed')));
    }
    assert.equal(harness.requests,4);
    await connect();enable();
    harness.listeners.disconnect({target:{}});render();
    assert.ok(button('Disconnect'),'Unplugging another serial device must not disconnect this arm');
    harness.listeners.disconnect({target:harness.port});render();
    assert.ok(button('Connect'));assert.equal(input('Jog enabled').props.checked,false);
    await connect();enable();
    input('Base jog').props.onChange({target:{value:'91'}});render();
    harness.listeners.disconnect({target:harness.port});render();
    assert.equal(harness.operation.signal.aborted,true);
    assert.ok(button('Disconnecting…'));
    harness.operation.reject(new Error('USB lost; release unconfirmed'));
    await tick();render();
    assert.ok(button('Connect'));assert.equal(input('Jog enabled').props.checked,false);
    assert.ok(nodes.some(n=>n.props.role==='alert'&&n.props.children.includes('release unconfirmed')));
  } finally {
    if(oldWindow)Object.defineProperty(globalThis,'window',oldWindow);else delete globalThis.window;
    if(oldDocument)Object.defineProperty(globalThis,'document',oldDocument);else delete globalThis.document;
    delete require.cache[filename];await unlink(filename);await rmdir(directory);
  }
});
