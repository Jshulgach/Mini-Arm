'use client';
import { useSyncExternalStore } from 'react';
import type { Device } from './home';
export type RegisteredDevice = Device & {monitor:boolean};
export type Relation = {from:string;to:string;type:'network'|'physical'|'logical';label:string};
export type Rule = {id:string;name:string;deviceId:string;when:'reachable'|'unreachable';enabled:boolean};
export type Configuration = {revision:number;rooms:string[];devices:RegisteredDevice[];relations:Relation[];rules:Rule[]};
export type Health = {state:'reachable'|'unreachable';observedAt:string;source:string;endpoint:string;latencyMs:number};
export type HomeSnapshot = Configuration & {health:Record<string,Health>;events:{id:string;at:string;type:string;source:string;message:string;deviceId?:string}[];
  notifications:{id:string;at:string;read:boolean;message:string}[];startedAt:string;serverTime:string};
let cache:{data:HomeSnapshot|null;error:string}={data:null,error:''};
const initial=cache;
const listeners=new Set<()=>void>();
let timer:ReturnType<typeof setInterval>|undefined;
let reading:Promise<void>|null=null;
function publish(data:HomeSnapshot) {
  if(cache.data && (data.revision<cache.data.revision || (data.revision===cache.data.revision && data.serverTime<cache.data.serverTime)))return;
  cache={data,error:''};listeners.forEach(f=>f());
}
export async function gatewayRequest<T>(path:string,method='GET',body?:unknown):Promise<T>{
  const options:RequestInit={method,signal:AbortSignal.timeout(10000)};
  if(body!==undefined){if(method==='GET'||method==='HEAD')throw new Error('Read requests cannot have a body.');options.headers={'Content-Type':'application/json'};options.body=JSON.stringify(body);}
  const response=await fetch(`/api/${path}`,options);
  const value=await response.json().catch(()=>({error:'The local gateway is unavailable. Start the Homebase service.'}));
  if(!response.ok)throw new Error(value && typeof value==='object' && 'error' in value && typeof value.error==='string'?value.error:'Gateway request failed.');
  return value as T;
}
export async function refreshGateway(){
  if(reading)return reading;
  reading=(async()=>{try{publish(await gatewayRequest<HomeSnapshot>('home'));}catch(e){cache={...cache,error:e instanceof Error?e.message:'Gateway unavailable'};listeners.forEach(f=>f());}finally{reading=null;}})();
  return reading;
}
export async function saveGateway(config:Configuration){const data=await gatewayRequest<HomeSnapshot>('home','PUT',config);publish(data);return data;}
export async function gatewayAction(path:'check'|'ack',id:string){publish(await gatewayRequest<HomeSnapshot>(path,'POST',{id}));}
function subscribe(listener:()=>void){
  listeners.add(listener);
  if(listeners.size===1){void refreshGateway();timer=setInterval(()=>void refreshGateway(),5000);}
  return()=>{listeners.delete(listener);if(!listeners.size)clearInterval(timer);};
}
export function useGateway(){return useSyncExternalStore(subscribe,()=>cache,()=>initial);}
export function healthLabel(device:RegisteredDevice,health:Health|undefined,serverTime:string){
  if(!health || health.endpoint!==device.endpoint)return device.monitor?'Awaiting check':'Not monitored';
  if(Math.max(Date.now(),Date.parse(serverTime))-Date.parse(health.observedAt)>90000)return 'Stale observation';
  return health.state==='reachable'?'Port reachable':'Port unreachable';
}
