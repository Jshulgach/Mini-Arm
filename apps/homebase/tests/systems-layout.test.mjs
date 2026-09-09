import test from 'node:test';
import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { createRequire } from 'node:module';
import { mkdir, mkdtemp, writeFile, unlink, rmdir } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { createGateway } from '../scripts/gateway.mjs';

test('Systems renders the seeded registry and settings with monitoring disabled',async()=>{
  const gateway=createGateway(':memory:');const snapshot=gateway.snapshot();await gateway.close();
  const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
  const bundle=await build({absWorkingDir:root,entryPoints:['app/systems/page.tsx'],bundle:true,write:false,platform:'node',format:'cjs',packages:'external',alias:{'@':root},jsx:'automatic',logLevel:'silent',
    plugins:[{name:'gateway-render-fixture',setup(b){
      b.onResolve({filter:/^@\/lib\/gateway$/},()=>({path:'gateway',namespace:'fixture'}));
      b.onResolve({filter:/^next\/link$/},()=>({path:'link',namespace:'link-fixture'}));
      b.onLoad({filter:/.*/,namespace:'link-fixture'},()=>({contents:"export default 'a';",loader:'js'}));
      b.onLoad({filter:/.*/,namespace:'fixture'},()=>({contents:`export const useGateway=()=>({data:${JSON.stringify(snapshot)},error:''});export const healthLabel=()=> 'Not monitored';export const saveGateway=()=>{throw Error('No mutation during render');};export const refreshGateway=saveGateway,gatewayAction=saveGateway,gatewayRequest=saveGateway;`,loader:'js'}));
      b.onLoad({filter:/\.css$/},()=>({contents:'',loader:'js'}));
    }}]});
  const cache=path.join(root,'node_modules','.cache');await mkdir(cache,{recursive:true});const directory=await mkdtemp(path.join(cache,'systems-layout-'));const filename=path.join(directory,'render.cjs');const require=createRequire(import.meta.url);
  try{
    await writeFile(filename,bundle.outputFiles[0].text);const Page=require(filename).default;
    const html=renderToStaticMarkup(React.createElement(Page));
    for(const text of ['Device registry','OctoPi','Raspberry Pi','Rooms &amp; links','Automations','Activity','Save device','Check saved endpoint now','Owner reports'])assert.ok(html.includes(text),text);
    assert.equal((html.match(/Not monitored/g)??[]).length,13);
    assert.ok(!html.includes('checked=""'),'Initial monitoring checkbox must be off');
    assert.ok(!html.includes('NaN')&&!html.includes('undefined'));
  }finally{delete require.cache[filename];await unlink(filename);await rmdir(directory);}
});
