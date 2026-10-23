#pragma once
#include <cmath>
#include "cJSON.h"

// One bounded setting per request; serialized by the port-80 HTTP server.
struct CameraControl { const char *name; int minimum; int maximum; };
static const CameraControl cameraControls[] = {
  {"brightness",-2,2}, {"contrast",-2,2}, {"saturation",-2,2},
  {"aec",0,1}, {"ae_level",-2,2}, {"aec_value",0,1200},
  {"agc",0,1}, {"agc_gain",0,30}, {"awb",0,1},
  {"hmirror",0,1}, {"vflip",0,1}, {"quality",4,63}
};
static constexpr size_t controlCount = sizeof(cameraControls)/sizeof(cameraControls[0]);
static camera_status_t startupSettings;

static int controlValue(const camera_status_t &s, size_t i) {
  switch(i) {
    case 0:return s.brightness; case 1:return s.contrast; case 2:return s.saturation;
    case 3:return s.aec; case 4:return s.ae_level; case 5:return s.aec_value;
    case 6:return s.agc; case 7:return s.agc_gain; case 8:return s.awb;
    case 9:return s.hmirror; case 10:return s.vflip; case 11:return s.quality;
    default:return 0;
  }
}
static int writeControl(sensor_t *s, size_t i, int value) {
  switch(i) {
    case 0:return s->set_brightness(s,value); case 1:return s->set_contrast(s,value);
    case 2:return s->set_saturation(s,value); case 3:return s->set_exposure_ctrl(s,value);
    case 4:return s->set_ae_level(s,value); case 5:return s->set_aec_value(s,value);
    case 6:return s->set_gain_ctrl(s,value); case 7:return s->set_agc_gain(s,value);
    case 8:return s->set_whitebal(s,value); case 9:return s->set_hmirror(s,value);
    case 10:return s->set_vflip(s,value); case 11:return s->set_quality(s,value);
    default:return -1;
  }
}
static esp_err_t settingsStateHandler(httpd_req_t *req) {
  sensor_t *sensor=esp_camera_sensor_get();
  if(!sensor) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Camera not ready");
  cJSON *root=cJSON_CreateObject();
  if(!root) return ESP_ERR_NO_MEM;
  cJSON_AddStringToObject(root,"firmware","1.1.0");
  cJSON_AddNumberToObject(root,"sensor_pid",sensor->id.PID);
  cJSON *values=cJSON_AddObjectToObject(root,"values");
  cJSON *defaults=cJSON_AddObjectToObject(root,"defaults");
  if(!values || !defaults){cJSON_Delete(root);return ESP_ERR_NO_MEM;}
  for(size_t i=0;i<controlCount;i++) {
    cJSON_AddNumberToObject(values,cameraControls[i].name,controlValue(sensor->status,i));
    cJSON_AddNumberToObject(defaults,cameraControls[i].name,controlValue(startupSettings,i));
  }
  char *body=cJSON_PrintUnformatted(root);cJSON_Delete(root);
  if(!body) return ESP_ERR_NO_MEM;
  noCache(req);httpd_resp_set_type(req,"application/json");
  esp_err_t result=httpd_resp_send(req,body,HTTPD_RESP_USE_STRLEN);
  cJSON_free(body);return result;
}
static esp_err_t settingsChangeHandler(httpd_req_t *req) {
  // Native settings page performs same-origin JSON POSTs. No cross-origin API.
  char type[64]={},origin[160]={},host[128]={};
  if(httpd_req_get_hdr_value_str(req,"Content-Type",type,sizeof(type))!=ESP_OK ||
     (strcmp(type,"application/json") && strcmp(type,"application/json; charset=utf-8")))
    return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Expected application/json");
  if(httpd_req_get_hdr_value_len(req,"Origin")) {
    if(httpd_req_get_hdr_value_str(req,"Origin",origin,sizeof(origin))!=ESP_OK ||
       httpd_req_get_hdr_value_str(req,"Host",host,sizeof(host))!=ESP_OK)
      return httpd_resp_send_err(req,HTTPD_403_FORBIDDEN,"Origin rejected");
    char expected[160];snprintf(expected,sizeof(expected),"http://%s",host);
    if(strcmp(origin,expected)) return httpd_resp_send_err(req,HTTPD_403_FORBIDDEN,"Origin rejected");
  }
  if(req->content_len<2 || req->content_len>192)
    return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid body length");
  char body[193]={};int offset=0;
  while(offset<req->content_len) {
    int received=httpd_req_recv(req,body+offset,req->content_len-offset);
    if(received<=0) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Incomplete body");
    offset+=received;
  }
  const char *end=nullptr;
  cJSON *json=cJSON_ParseWithLengthOpts(body,offset+1,&end,true);
  if(!json || !cJSON_IsObject(json)){cJSON_Delete(json);return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid JSON");}
  sensor_t *sensor=esp_camera_sensor_get();
  if(!sensor){cJSON_Delete(json);return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Camera not ready");}
  int result=0;
  const cJSON *reset=cJSON_GetObjectItemCaseSensitive(json,"reset");
  if(cJSON_GetArraySize(json)==1 && cJSON_IsTrue(reset)) {
    // Restore manual registers first, then auto algorithms, avoiding guesses at defaults.
    const size_t order[]={0,1,2,4,5,7,8,9,10,11,3,6};
    for(size_t i:order) {
      int value=controlValue(startupSettings,i);
      if(value<cameraControls[i].minimum || value>cameraControls[i].maximum) {result=-1;break;}
      if(writeControl(sensor,i,value)!=0){result=-1;break;}
    }
  } else {
    const cJSON *name=cJSON_GetObjectItemCaseSensitive(json,"name");
    const cJSON *value=cJSON_GetObjectItemCaseSensitive(json,"value");
    size_t selected=controlCount;
    if(cJSON_GetArraySize(json)==2 && cJSON_IsString(name) && cJSON_IsNumber(value))
      for(size_t i=0;i<controlCount;i++) if(!strcmp(name->valuestring,cameraControls[i].name)) selected=i;
    if(selected==controlCount || !std::isfinite(value->valuedouble) ||
       std::floor(value->valuedouble)!=value->valuedouble ||
       value->valuedouble<cameraControls[selected].minimum || value->valuedouble>cameraControls[selected].maximum) {
      cJSON_Delete(json);return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Unknown setting or out-of-range integer");
    }
    if((selected==5 && sensor->status.aec) || (selected==7 && sensor->status.agc)) {
      cJSON_Delete(json);return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Turn off the corresponding automatic control first");
    }
    result=writeControl(sensor,selected,value->valueint);
  }
  cJSON_Delete(json);
  if(result!=0) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Sensor rejected a setting; reload values (changes may be partial)");
  return settingsStateHandler(req);
}
