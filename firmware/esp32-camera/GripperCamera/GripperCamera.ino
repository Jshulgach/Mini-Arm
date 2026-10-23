// Mini-Arm gripper camera firmware. Author: Jonathan Shulgach and contributors.
// Uses Espressif Arduino-ESP32 3.3.8 / esp32-camera APIs.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_config.h"

#if !CAMERA_BOARD_CONFIRMED || !defined(CAMERA_MODEL_AI_THINKER)
#error "Confirm your AI Thinker ESP32-CAM board in camera_config.h before building. Other boards need their own pin profile."
#endif
#if CAMERA_STREAM_FPS < 1 || CAMERA_STREAM_FPS > 15
#error "CAMERA_STREAM_FPS must be 1-15"
#endif
#if CAMERA_JPEG_QUALITY < 4 || CAMERA_JPEG_QUALITY > 63
#error "CAMERA_JPEG_QUALITY must be 4-63"
#endif

static httpd_handle_t webServer = nullptr;
static httpd_handle_t streamServer = nullptr;
static bool cameraReady = false;
static unsigned long lastReconnect = 0;

static void noCache(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
}

#include "camera_controls.h"
#include "settings_page.h"

static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *frame = esp_camera_fb_get();
  if (!frame) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera capture failed");
  if (frame->format != PIXFORMAT_JPEG) {
    esp_camera_fb_return(frame);
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Expected JPEG");
  }
  noCache(req);
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t result = httpd_resp_send(req, reinterpret_cast<const char *>(frame->buf), frame->len);
  esp_camera_fb_return(frame);
  return result;
}

static esp_err_t streamHandler(httpd_req_t *req) {
  noCache(req);
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=gripperframe");
  while (WiFi.status() == WL_CONNECTED) {
    unsigned long started = millis();
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) return ESP_FAIL;
    char header[128];
    int length = snprintf(header, sizeof(header),
      "--gripperframe\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)frame->len);
    esp_err_t result = httpd_resp_send_chunk(req, header, length);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(frame->buf), frame->len);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, "\r\n", 2);
    esp_camera_fb_return(frame);
    if (result != ESP_OK) return result;
    unsigned long elapsed = millis() - started;
    unsigned long interval = 1000 / CAMERA_STREAM_FPS;
    delay(elapsed < interval ? interval - elapsed : 1);
  }
  return ESP_FAIL;
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char body[256];
  snprintf(body, sizeof(body),
    "{\"device\":\"miniarm-camera\",\"firmware\":\"1.1.0\",\"camera_ready\":%s,\"uptime_ms\":%lu,\"rssi_dbm\":%d,\"psram\":%s,\"stream_port\":81}",
    cameraReady ? "true" : "false", millis(), WiFi.RSSI(), psramFound() ? "true" : "false");
  noCache(req);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t rootHandler(httpd_req_t *req) {
  static const char page[] = R"HTML(<!doctype html><meta name="viewport" content="width=device-width"><title>Mini-Arm gripper camera</title>
<style>body{font:18px system-ui;background:#101b22;color:#eef4ed;max-width:800px;margin:30px auto;padding:20px}img{width:100%;background:#000}button,a{font:inherit;margin:12px 12px 12px 0;color:inherit}button{background:#285145;padding:10px;border:0;border-radius:8px}</style>
<h1>Mini-Arm gripper camera</h1><p>Local camera preview. No recording.</p><button id="start">Start stream</button><button id="stop">Stop</button><a href="/capture" target="_blank">Snapshot</a><img id="feed" alt="Camera preview starts when you press Start stream"><p id="status">Stopped</p>
<details><summary>Image settings</summary><iframe title="Gripper camera image settings" src="/settings" loading="lazy" style="width:100%;height:1080px;border:1px solid #334952;border-radius:8px;margin-top:14px"></iframe></details>
<script>const feed=document.getElementById('feed'),status=document.getElementById('status');document.getElementById('start').onclick=()=>{feed.src='http://'+location.hostname+':81/stream';status.textContent='Stream requested';};document.getElementById('stop').onclick=()=>{feed.removeAttribute('src');status.textContent='Stopped';};feed.onerror=()=>{status.textContent='Stream unavailable. Check camera power and Wi-Fi.';};document.addEventListener('visibilitychange',()=>{if(document.hidden)document.getElementById('stop').click();});</script>)HTML";
  noCache(req);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static bool startServers() {
  httpd_config_t web = HTTPD_DEFAULT_CONFIG();
  web.server_port = 80;
  web.recv_wait_timeout = 2;
  web.send_wait_timeout = 2;
  if (httpd_start(&webServer, &web) != ESP_OK) return false;
  httpd_uri_t root = {}; root.uri="/"; root.method=HTTP_GET; root.handler=rootHandler;
  httpd_uri_t capture = {}; capture.uri="/capture"; capture.method=HTTP_GET; capture.handler=captureHandler;
  httpd_uri_t status = {}; status.uri="/status"; status.method=HTTP_GET; status.handler=statusHandler;
  if (httpd_register_uri_handler(webServer,&root)!=ESP_OK || httpd_register_uri_handler(webServer,&capture)!=ESP_OK || httpd_register_uri_handler(webServer,&status)!=ESP_OK) return false;
  httpd_uri_t settings = {}; settings.uri="/settings"; settings.method=HTTP_GET; settings.handler=settingsPageHandler;
  httpd_uri_t state = {}; state.uri="/settings/state"; state.method=HTTP_GET; state.handler=settingsStateHandler;
  httpd_uri_t change = {}; change.uri="/settings/state"; change.method=HTTP_POST; change.handler=settingsChangeHandler;
  if(httpd_register_uri_handler(webServer,&settings)!=ESP_OK || httpd_register_uri_handler(webServer,&state)!=ESP_OK || httpd_register_uri_handler(webServer,&change)!=ESP_OK) return false;
  httpd_config_t stream = web;
  stream.server_port = 81;
  stream.ctrl_port = web.ctrl_port + 1;
  if (httpd_start(&streamServer, &stream) != ESP_OK) return false;
  httpd_uri_t endpoint = {}; endpoint.uri="/stream"; endpoint.method=HTTP_GET; endpoint.handler=streamHandler;
  return httpd_register_uri_handler(streamServer,&endpoint)==ESP_OK;
}

void setup() {
  Serial.begin(115200);
  pinMode(4,OUTPUT); digitalWrite(4,LOW); // AI Thinker flash LED stays off.
  camera_config_t config = {};
  config.ledc_channel=LEDC_CHANNEL_0; config.ledc_timer=LEDC_TIMER_0;
  // AI Thinker profile from Espressif CameraWebServer camera_pins.h (3.3.8).
  config.pin_d0=5; config.pin_d1=18; config.pin_d2=19; config.pin_d3=21;
  config.pin_d4=36; config.pin_d5=39; config.pin_d6=34; config.pin_d7=35;
  config.pin_xclk=0; config.pin_pclk=22; config.pin_vsync=25; config.pin_href=23;
  config.pin_sccb_sda=26; config.pin_sccb_scl=27; config.pin_pwdn=32; config.pin_reset=-1;
  config.xclk_freq_hz=20000000; config.pixel_format=PIXFORMAT_JPEG;
  config.frame_size=psramFound()?FRAMESIZE_VGA:FRAMESIZE_QVGA;
  config.jpeg_quality=CAMERA_JPEG_QUALITY;
  config.fb_count=psramFound()?2:1;
  config.fb_location=psramFound()?CAMERA_FB_IN_PSRAM:CAMERA_FB_IN_DRAM;
  config.grab_mode=CAMERA_GRAB_LATEST;
  esp_err_t error=esp_camera_init(&config);
  if(error!=ESP_OK){Serial.printf("Camera init failed: 0x%x\n",error);return;}
  sensor_t *sensor=esp_camera_sensor_get();
  sensor->set_hmirror(sensor,CAMERA_MIRROR); sensor->set_vflip(sensor,CAMERA_FLIP);
  startupSettings=sensor->status;
  cameraReady=true;
  if(strlen(CAMERA_WIFI_SSID)==0){Serial.println("Set Wi-Fi credentials in camera_config.h and rebuild.");return;}
  WiFi.mode(WIFI_STA); WiFi.setHostname(CAMERA_HOSTNAME); WiFi.setAutoReconnect(true);
  WiFi.begin(CAMERA_WIFI_SSID,CAMERA_WIFI_PASSWORD);
  Serial.println("Connecting to configured Wi-Fi...");
}

void loop() {
  if (!cameraReady || strlen(CAMERA_WIFI_SSID)==0) {delay(1000);return;}
  if (WiFi.status()==WL_CONNECTED) {
    if (!webServer) {
      if (!startServers()) {
        if(streamServer){httpd_stop(streamServer);streamServer=nullptr;}
        if(webServer){httpd_stop(webServer);webServer=nullptr;}
        Serial.println("HTTP startup failed; retrying");delay(1000);return;
      }
      MDNS.begin(CAMERA_HOSTNAME); MDNS.addService("http","tcp",80);
      Serial.printf("Camera: http://%s/ | capture: /capture | stream: port 81/stream\n",WiFi.localIP().toString().c_str());
    }
  } else if (millis()-lastReconnect>15000) {
    lastReconnect=millis();Serial.println("Wi-Fi disconnected; retrying...");WiFi.reconnect();
  }
  delay(100);
}
