#pragma once
static const char settingsPage[] = R"HTML(<!doctype html><html lang="en"><meta name="viewport" content="width=device-width"><title>Gripper camera image settings</title>
<style>
*{box-sizing:border-box}body{margin:0;padding:16px;font:14px system-ui;background:#101c23;color:#e5f0de}h2{font-size:19px;margin:0 0 8px}p{line-height:1.5;color:#adc1ca}.control{border-top:1px solid #334952;padding:12px 0}.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}label{flex:1;min-width:120px}input[type=range]{width:100%;accent-color:#b4d895}output{min-width:34px;text-align:right;font-variant-numeric:tabular-nums}button{color:inherit;background:#285145;border:1px solid #416152;border-radius:6px;padding:7px 12px;cursor:pointer;font:inherit}button:disabled,input:disabled{opacity:.45;cursor:default}small{display:block;color:#9aafb9;margin-top:6px}.toolbar{display:flex;flex-wrap:wrap;gap:8px;margin:14px 0}#message{min-height:38px}#message[data-error=true]{color:#ffc1a5}fieldset{border:0;margin:0;padding:0}input[type=checkbox]{width:18px;height:18px;accent-color:#b4d895}
</style>
<h2>Image settings</h2><p>Adjust the gripper camera while watching its preview. Changes last until restart.</p>
<p id="message" role="status" aria-live="polite">Loading camera settings…</p>
<div class="toolbar"><button id="reload" type="button">Reload values</button><button id="reset" type="button" disabled>Reset startup settings</button></div>
<fieldset id="controls" disabled></fieldset>
<p><small>Manual exposure uses sensor units, not milliseconds. More exposure can brighten the image but blur movement; more gain can add noise. Lower JPEG quality numbers mean sharper, larger images.</small></p>
<script>
const specs=[
 ['brightness','Brightness',-2,2],['contrast','Contrast',-2,2],['saturation','Saturation',-2,2],
 ['aec','Automatic exposure',0,1,'toggle'],['ae_level','Auto exposure compensation',-2,2],['aec_value','Manual exposure',0,1200],
 ['agc','Automatic gain',0,1,'toggle'],['agc_gain','Manual gain',0,30],['awb','Automatic white balance',0,1,'toggle'],
 ['hmirror','Mirror horizontally',0,1,'toggle'],['vflip','Flip vertically',0,1,'toggle'],['quality','JPEG quality',4,63]
];
const controls=document.getElementById('controls'),message=document.getElementById('message'),reload=document.getElementById('reload'),reset=document.getElementById('reset');
let values=null,busy=false;
function notify(text,error=false){message.textContent=text;message.dataset.error=String(error);}
function lock(value){busy=value;controls.disabled=value||!values;reload.disabled=value;reset.disabled=value||!values;}
function validState(data){return data&&data.values&&specs.every(([name])=>Number.isInteger(data.values[name]));}
function render(){
 controls.replaceChildren();
 for(const [name,label,min,max,kind] of specs){
  const group=document.createElement('div');group.className='control';
  const row=document.createElement('div');row.className='row';
  const title=document.createElement('label');title.htmlFor='setting-'+name;title.textContent=label;
  const input=document.createElement('input');input.id='setting-'+name;
  const autoDisabled=(name==='aec_value'&&values.aec===1)||(name==='agc_gain'&&values.agc===1)||(name==='ae_level'&&values.aec===0);
  input.disabled=autoDisabled;
  row.append(title);
  if(kind==='toggle'){
   input.type='checkbox';input.checked=values[name]===1;row.append(input);
   input.onchange=()=>change({name,value:input.checked?1:0});
  }else{
   input.type='range';input.min=min;input.max=max;input.step=1;input.value=values[name];
   const output=document.createElement('output');output.htmlFor=input.id;output.textContent=String(values[name]);
   const apply=document.createElement('button');apply.type='button';apply.textContent='Apply';apply.disabled=autoDisabled;apply.setAttribute('aria-label','Apply '+label.toLowerCase());
   input.oninput=()=>{output.textContent=input.value;};
   apply.onclick=()=>change({name,value:Number(input.value)});
   row.append(output,apply);group.append(row,input);
  }
  if(kind==='toggle')group.append(row);
  if(autoDisabled){const hint=document.createElement('small');hint.textContent=name==='ae_level'?'Available with automatic exposure enabled.':'Disable the automatic control above to adjust this value.';group.append(hint);}
  controls.append(group);
 }
}
async function request(body){
 const response=await fetch('/settings/state',{method:body?'POST':'GET',cache:'no-store',headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined,signal:AbortSignal.timeout(5000)});
 if(!response.ok)throw new Error((await response.text()).slice(0,180)||'Camera request failed');
 const state=await response.json();if(!validState(state))throw new Error('Camera returned invalid settings');return state.values;
}
async function load(){lock(true);try{values=await request();render();notify('Values read from the camera.');}catch(error){values=null;notify('Cannot read settings: '+error.message,true);}finally{lock(false);}}
async function change(body){
 if(busy)return;lock(true);notify('Applying…');
 try{values=await request(body);render();notify(body.reset?'Startup settings restored.':'Applied; values read back from the camera.');}
 catch(error){try{values=await request();render();}catch{values=null;}notify('Could not confirm change: '+error.message+' Reload values before retrying.',true);}
 finally{lock(false);}
}
reload.onclick=load;reset.onclick=()=>change({reset:true});load();
</script></html>)HTML";
static esp_err_t settingsPageHandler(httpd_req_t *req) {
  noCache(req);httpd_resp_set_type(req,"text/html");
  return httpd_resp_send(req,settingsPage,HTTPD_RESP_USE_STRLEN);
}
