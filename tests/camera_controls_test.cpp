// Exercise the actual firmware request handlers with fake HTTP and sensor I/O.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <map>
#include <algorithm>
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_FAIL=-1, ESP_ERR_NO_MEM=-2;
constexpr int HTTPD_400_BAD_REQUEST=400, HTTPD_403_FORBIDDEN=403, HTTPD_500_INTERNAL_SERVER_ERROR=500;
constexpr int HTTPD_RESP_USE_STRLEN=-1;
struct httpd_req_t {
  std::string body, reply; int content_len=0, offset=0, status=200;
  std::map<std::string,std::string> headers={{"Content-Type","application/json"},{"Host","192.168.1.189"},{"Origin","http://192.168.1.189"}};
};
int httpd_req_get_hdr_value_len(httpd_req_t*r,const char*k){return r->headers.count(k)?r->headers[k].size():0;}
int httpd_req_get_hdr_value_str(httpd_req_t*r,const char*k,char*b,size_t n){
 if(!r->headers.count(k)||r->headers[k].size()>=n)return ESP_FAIL;
 std::strcpy(b,r->headers[k].c_str());return ESP_OK;
}
int httpd_req_recv(httpd_req_t*r,char*b,int n){
 n=std::min({n,3,int(r->body.size())-r->offset}); // Deliberately fragmented body.
 if(n<=0)return 0;std::memcpy(b,r->body.data()+r->offset,n);r->offset+=n;return n;
}
int httpd_resp_send_err(httpd_req_t*r,int status,const char*message){r->status=status;r->reply=message;return ESP_FAIL;}
void httpd_resp_set_type(httpd_req_t*,const char*){}
void noCache(httpd_req_t*){}
int httpd_resp_send(httpd_req_t*r,const char*body,int n){r->reply=n<0?std::string(body):std::string(body,n);return ESP_OK;}
struct camera_status_t {int brightness=0,contrast=0,saturation=0,aec=1,ae_level=0,aec_value=300,agc=1,agc_gain=0,awb=1,hmirror=0,vflip=0,quality=12;};
struct sensor_t {
 camera_status_t status;struct {int PID=0x26;} id;
 int (*set_brightness)(sensor_t*,int);int (*set_contrast)(sensor_t*,int);int (*set_saturation)(sensor_t*,int);
 int (*set_exposure_ctrl)(sensor_t*,int);int (*set_ae_level)(sensor_t*,int);int (*set_aec_value)(sensor_t*,int);
 int (*set_gain_ctrl)(sensor_t*,int);int (*set_agc_gain)(sensor_t*,int);int (*set_whitebal)(sensor_t*,int);
 int (*set_hmirror)(sensor_t*,int);int (*set_vflip)(sensor_t*,int);int (*set_quality)(sensor_t*,int);
};
static sensor_t sensor;static int writes=0;static bool reject=false;
sensor_t* esp_camera_sensor_get(){return &sensor;}
#define SETTER(name,field) int name(sensor_t*s,int v){if(reject)return -1;++writes;s->status.field=v;return 0;}
SETTER(brightness,brightness) SETTER(contrast,contrast) SETTER(saturation,saturation)
SETTER(exposure,aec) SETTER(compensation,ae_level) SETTER(manualExposure,aec_value)
SETTER(gain,agc) SETTER(manualGain,agc_gain) SETTER(whiteBalance,awb)
SETTER(mirror,hmirror) SETTER(flip,vflip) SETTER(quality,quality)
#include "../firmware/esp32-camera/GripperCamera/camera_controls.h"
httpd_req_t request(std::string body){httpd_req_t req;req.body=body;req.content_len=body.size();return req;}
int main(){
 sensor.set_brightness=brightness;sensor.set_contrast=contrast;sensor.set_saturation=saturation;
 sensor.set_exposure_ctrl=exposure;sensor.set_ae_level=compensation;sensor.set_aec_value=manualExposure;
 sensor.set_gain_ctrl=gain;sensor.set_agc_gain=manualGain;sensor.set_whitebal=whiteBalance;
 sensor.set_hmirror=mirror;sensor.set_vflip=flip;sensor.set_quality=quality;
 startupSettings=sensor.status;
 // Reject malformed, unknown, duplicate, noninteger, out-of-range and truncated bodies before I/O.
 const char* bad[]={"{}","[]","{bad}","{\"name\":\"brightness\",\"value\":3}",
 "{\"name\":\"brightness\",\"value\":0.5}","{\"name\":\"brightness\",\"value\":\"1\"}",
 "{\"name\":\"brightness\",\"value\":1e999}","{\"name\":\"unknown\",\"value\":1}",
 "{\"reset\":false}","{\"reset\":true,\"name\":\"brightness\"}",
 "{\"name\":\"brightness\",\"value\":1,\"value\":2}","{\"name\":\"brightness\",\"value\":1}tail"};
 for(auto body:bad){auto req=request(body);settingsChangeHandler(&req);assert(req.status==400);assert(writes==0);}
 auto shortBody=request("{}");shortBody.content_len=10;settingsChangeHandler(&shortBody);assert(shortBody.status==400&&writes==0);
 auto huge=request(std::string(193,' '));settingsChangeHandler(&huge);assert(huge.status==400&&writes==0);
 auto origin=request("{\"name\":\"brightness\",\"value\":1}");origin.headers["Origin"]="http://unrelated.example";
 settingsChangeHandler(&origin);assert(origin.status==403&&writes==0);
 auto form=request("{\"name\":\"brightness\",\"value\":1}");form.headers["Content-Type"]="text/plain";
 settingsChangeHandler(&form);assert(form.status==400&&writes==0);
 auto manual=request("{\"name\":\"aec_value\",\"value\":800}");settingsChangeHandler(&manual);assert(manual.status==400&&writes==0);
 auto autoOff=request("{\"name\":\"aec\",\"value\":0}");assert(settingsChangeHandler(&autoOff)==ESP_OK);
 manual=request("{\"name\":\"aec_value\",\"value\":800}");assert(settingsChangeHandler(&manual)==ESP_OK);
 assert(sensor.status.aec_value==800&&sensor.status.aec==0&&writes==2);
 auto state=request("");settingsStateHandler(&state);cJSON* json=cJSON_Parse(state.reply.c_str());
 assert(cJSON_GetObjectItem(cJSON_GetObjectItem(json,"values"),"aec_value")->valueint==800);cJSON_Delete(json);
 auto reset=request("{\"reset\":true}");assert(settingsChangeHandler(&reset)==ESP_OK);
 assert(sensor.status.aec==1&&sensor.status.aec_value==300&&sensor.status.quality==12);
 reject=true;auto failed=request("{\"name\":\"brightness\",\"value\":1}");settingsChangeHandler(&failed);
 assert(failed.status==500&&sensor.status.brightness==0);
 std::puts("Camera handler tests passed: fragmented JSON, validation, origin, auto/manual modes, readback, reset and sensor failure.");
}
