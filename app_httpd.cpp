/*
  ESP32_CAM_Robot_Car
  app_httpd.cpp
  Uses L298N Motor Driver

*/

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp_system.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "Arduino.h"
#include "driver/ledc.h"
#include <Update.h>
#include <esp32-hal-ledc.h>

#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15
#define LED_PIN 4 // Define LED pin

// Define Speed variables
int speed = 255;
int noStop = 0;

// Setting Motor PWM properties
const int freq = 2000;
const int motorPWMChannnel = 8;
const int lresolution = 8;

volatile unsigned int motor_speed = 200;
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 250;

// Placeholder for functions
void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
void setupLED();
void test_pwm_output();
uint8_t robo = 0;

typedef struct
{
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index)
  {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
  {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG)
  {
    fb_len = fb->len;
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  }
  else
  {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
    fb_len = jchunk.len;
  }
  esp_camera_fb_return(fb);

  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    static int64_t last_frame = 0;
    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        Serial.println("Failed to set response type");
        return res;
    }

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            if (fb->format != PIXFORMAT_JPEG) {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted) {
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            if (res != ESP_OK) {
                Serial.println("Failed to send JPEG header");
            }
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            if (res != ESP_OK) {
                Serial.println("Failed to send JPEG data");
            }
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            if (res != ESP_OK) {
                Serial.println("Failed to send stream boundary");
            }
        }

        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK) {
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        Serial.printf("MJPG: %uB %ums (%.1ffps)\n", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

enum state
{
  fwd,
  rev,
  stp
};
state actstate = stp;

static esp_err_t cmd_handler(httpd_req_t *req)
{
  char *buf;
  size_t buf_len;
  char variable[32] = {
      0,
  };
  char value[32] = {
      0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    buf = (char *)malloc(buf_len);
    if (!buf)
    {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK)
      {
      }
      else
      {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    }
    else
    {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  }
  else
  {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  Serial.printf("Received request: var=%s, val=%s\n", variable, value);
  Serial.printf("Parsed value: %d\n", val);

  // Updated command handling
  if (!strcmp(variable, "framesize"))
  {
    if (s->pixformat == PIXFORMAT_JPEG)
      res = s->set_framesize(s, (framesize_t)val);
  }
  else if (!strcmp(variable, "quality"))
    res = s->set_quality(s, val);
  else if (!strcmp(variable, "flash"))
  {
    ledc_channel_config_t flash_channel;
    flash_channel.gpio_num = 4;  // Flash LED pin
    flash_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    flash_channel.channel = LEDC_CHANNEL_7;
    flash_channel.intr_type = LEDC_INTR_DISABLE;
    flash_channel.timer_sel = LEDC_TIMER_1;
    flash_channel.duty = val;
    flash_channel.hpoint = 0;
    flash_channel.flags.output_invert = 0;
    ledc_channel_config(&flash_channel);
    Serial.printf("LED Control: Duty cycle set to %d\n", val);
  }
  else if (!strcmp(variable, "flashoff"))
  {
    ledc_channel_config_t flash_channel;
    flash_channel.gpio_num = 4;  // Flash LED pin
    flash_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    flash_channel.channel = LEDC_CHANNEL_7;
    flash_channel.intr_type = LEDC_INTR_DISABLE;
    flash_channel.timer_sel = LEDC_TIMER_1;
    flash_channel.duty = val;
    flash_channel.hpoint = 0;
    flash_channel.flags.output_invert = 0;
    ledc_channel_config(&flash_channel);
    Serial.printf("LED Control: Duty cycle set to %d\n", val);
  }
  else if (!strcmp(variable, "speed"))
  {
    if (val > 255)
      val = 255;
    else if (val < 0)
      val = 0;
    speed = val;
    Serial.printf("Speed updated: %d\n", speed);
  }
  else if (!strcmp(variable, "nostop"))
  {
    noStop = val;
  }
  else if (!strcmp(variable, "car"))
  {
    Serial.printf("Received command: var=%s, val=%s\n", variable, value);
    if (val == 1)
    {
      Serial.println("Forward");
      robot_fwd();
      robo = 1;
    }
    else if (val == 2)
    {
      Serial.println("Left");
      robot_left();
      robo = 1;
    }
    else if (val == 3)
    {
      Serial.println("Stop");
      robot_stop();
    }
    else if (val == 4)
    {
      Serial.println("Right");
      robot_right();
      robo = 1;
    }
    else if (val == 5)
    {
      Serial.println("Backward");
      robot_back();
      robo = 1;
    }
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req)
{
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1">
        <title>ESP32 CAM Robot</title>
        <style>
    body {
        font-family: Arial, Helvetica, sans-serif;
        background: #181818;
        color: #efefef;
        font-size: 16px;
    }
    h2 {
        font-size: 18px;
    }
    section.main {
        display: flex;
    }
    #menu,
    section.main {
        flex-direction: column;
    }
    #menu {
        display: none;
        flex-wrap: nowrap;
        min-width: 340px;
        background: #363636;
        padding: 8px;
        border-radius: 4px;
        margin-top: 1px;
        margin-right: 10px;
    }
    #content {
        display: flex;
        flex-wrap: wrap;
        align-items: stretch;
    }
    figure {
        padding: 0;
        margin: 0;
        -webkit-margin-before: 0;
        margin-block-start: 0;
        -webkit-margin-after: 0;
        margin-block-end: 0;
        -webkit-margin-start: 0;
        margin-inline-start: 0;
        -webkit-margin-end: 0;
        margin-inline-end: 0;
    }
    figure img {
        display: block;
        width: 100%;
        height: auto;
        border-radius: 4px;
        margin-top: 15px;
    }
    @media (min-width: 800px) and (orientation: landscape) {
        #content {
            display: flex;
            flex-wrap: nowrap;
            align-items: stretch;
        }
        figure img {
            display: block;
            max-width: 100%;
            max-height: calc(100vh - 40px);
            width: auto;
            height: auto;
        }
        figure {
            padding: 0;
            margin: 0;
            -webkit-margin-before: 0;
            margin-block-start: 0;
            -webkit-margin-after: 0;
            margin-block-end: 0;
            -webkit-margin-start: 0;
            margin-inline-start: 0;
            -webkit-margin-end: 0;
            margin-inline-end: 0;
        }
    }
    section #buttons {
        display: flex;
        flex-wrap: nowrap;
        justify-content: space-between;
    }
    #nav-toggle {
        cursor: pointer;
        display: block;
    }
    #nav-toggle-cb {
        outline: 0;
        opacity: 0;
        width: 0;
        height: 0;
    }
    #nav-toggle-cb:checked + #menu {
        display: flex;
    }
    .input-group {
        display: flex;
        flex-wrap: nowrap;
        line-height: 22px;
        margin: 5px 0;
    }
    .input-group > label {
        display: inline-block;
        padding-right: 10px;
        min-width: 47%;
    }
    .input-group input,
    .input-group select {
        flex-grow: 1;
    }
    .range-max,
    .range-min {
        display: inline-block;
        padding: 0 5px;
    }
    button {
        display: block;
        margin: 5px;
        padding: 5px 12px;
        border: 0;
        line-height: 28px;
        cursor: pointer;
        color: #fff;
        background: #30D5C8; /* Turquoise */
        border-radius: 5px;
        font-size: 16px;
        outline: 0;
        width: 100px;
    }
    .button2 {background-color: #98FF98; width: 100px;} /* Mint green */
    .button3 {background-color: #f44336; width: 100px;} /* Red */ 
    .button4 {background-color: #E6E6FA; color: black; width: 120px;} /* Lavender */ 
    .button5 {background-color: #555555; width: 100px;} /* Black */
    .button6 {visibility: hidden; width: 100px;} /* Hidden */

    button:hover {
        background: #98FF98; /* Mint green */
    }
    button:active {
        background: #30D5C8; /* Turquoise */
    }
    button.disabled {
        cursor: default;
        background: #a0a0a0; /* Gray */
    }
    input[type="range"] {
        -webkit-appearance: none;
        width: 80%;
        height: 22px;
        background: #363636;
        cursor: pointer;
        margin: 0;
    }
    input[type="range"]:focus {
        outline: 0;
    }
    input[type="range"]::-webkit-slider-runnable-track {
        width: 80%;
        height: 2px;
        cursor: pointer;
        background: #efefef;
        border-radius: 0;
        border: 0 solid #efefef;
    }
    input[type="range"]::-webkit-slider-thumb {
        border: 1px solid rgba(0, 0, 30, 0);
        height: 22px;
        width: 22px;
        border-radius: 50px;
        background: #ff3034;
        cursor: pointer;
        -webkit-appearance: none;
        margin-top: -11.5px;
    }
    input[type="range"]:focus::-webkit-slider-runnable-track {
        background: #efefef;
    }
    input[type="range"]::-moz-range-track {
        width: 80%;
        height: 2px;
        cursor: pointer;
        background: #efefef;
        border-radius: 0;
        border: 0 solid #efefef;
    }
    input[type="range"]::-moz-range-thumb {
        border: 1px solid rgba(0, 0, 30, 0);
        height: 22px;
        width: 22px;
        border-radius: 50px;
        background: #ff3034;
        cursor: pointer;
    }
    input[type="range"]::-ms-track {
        width: 80%;
        height: 2px;
        cursor: pointer;
        background: 0 0;
        border-color: transparent;
        color: transparent;
    }
    input[type="range"]::-ms-fill-lower {
        background: #efefef;
        border: 0 solid #efefef;
        border-radius: 0;
    }
    input[type="range"]::-ms-fill-upper {
        background: #efefef;
        border: 0 solid #efefef;
        border-radius: 0;
    }
    input[type="range"]::-ms-thumb {
        border: 1px solid rgba(0, 0, 30, 0);
        height: 22px;
        width: 22px;
        border-radius: 50px;
        background: #ff3034;
        cursor: pointer;
        height: 2px;
    }
    input[type="range"]:focus::-ms-fill-lower {
        background: #efefef;
    }
    input[type="range"]:focus::-ms-fill-upper {
        background: #363636;
    }
    .switch {
        display: block;
        position: relative;
        line-height: 22px;
        font-size: 16px;
        height: 22px;
    }
    .switch input {
        outline: 0;
        opacity: 0;
        width: 0;
        height: 0;
    }
    .slider {
        width: 50px;
        height: 22px;
        border-radius: 22px;
        cursor: pointer;
        background-color: grey;
    }
    .slider,
    .slider:before {
        display: inline-block;
        transition: 0.4s;
    }
    .slider:before {
        position: relative;
        content: "";
        border-radius: 50%;
        height: 16px;
        width: 16px;
        left: 4px;
        top: 3px;
        background-color: #fff;
    }
    input:checked + .slider {
        background-color: #ff3034;
    }
    input:checked + .slider:before {
        -webkit-transform: translateX(26px);
        transform: translateX(26px);
    }
    select {
        border: 1px solid #363636;
        font-size: 14px;
        height: 22px;
        outline: 0;
        border-radius: 5px;
    }
    .image-container {
        position: absolute;
        top: 150px;
        left: 50%;
        margin-right: -50%;
        transform: translate(-50%, -50%);
        min-width: 160px;
        
    }

    .control-container {
        position: relative;
        top: 450px;
        left: 50%;
        margin-right: -50%;
        transform: translate(-50%, -50%);
        
   
    }

    .slider-container {
        position: relative;
        top: 550px;
        left: auto;
        margin-right: auto;
        
       
        
   
    }
    .close {
        position: absolute;
        right: 5px;
        top: 5px;
        background: #ff3034;
        width: 16px;
        height: 16px;
        border-radius: 100px;
        color: #fff;
        text-align: center;
        line-height: 18px;
        cursor: pointer;
    }
    .hidden {
        display: none;
    }
    .rotate90 {
        -webkit-transform: rotate(0deg);
        -moz-transform: rotate(0deg);
        -o-transform: rotate(0deg);
        -ms-transform: rotate(0deg);
        transform: rotate(0deg);
    }
</style>

    </head>
    <body>
    <br/>
    
        <section class="main">
        <figure>
      <div id="stream-container" class="image-container">
        <div class="close" id="close-stream">×</div>
        <img id="stream" src="" class="rotate90">
      </div>
    </figure>
    <br/>

          <section id="buttons">

                <div id="controls" class="control-container">
                  <table>
                  <tr><td align="center"><button class="button button6" id="get-still">Image</button></td><td align="center"><button id="toggle-stream">Start</button></td><td></td></tr>
                  <tr><td></td><td align="center"><button class="button button2" id="forward" onclick="fetch(document.location.origin+'/control?var=car&val=1');">FORWARD</button></td><td></td></tr>
                  <tr><td align="center"><button class="button button2" id="turnleft" onclick="fetch(document.location.origin+'/control?var=car&val=2');">LEFT</button></td><td align="center"></td><td align="center"><button class="button button2" id="turnright" onclick="fetch(document.location.origin+'/control?var=car&val=4');">RIGHT</button></td></tr>
                  <tr><td></td><td align="center"><button class="button button2" id="backward" onclick="fetch(document.location.origin+'/control?var=car&val=5');">REVERSE</button></td><td></td></tr>
                  <tr><td align="center"><button class="button button4" id="flash" onclick="fetch(document.location.origin+'/control?var=flash&val=256');">LIGHT ON</button></td><td align="center"></td><td align="center"><button class="button button4" id="flashoff" onclick="fetch(document.location.origin+'/control?var=flashoff&val=0');">LIGHT OFF</button></td></tr>
                  
                  <tr><td align="right">Speed:</td><td align="center" colspan="2"><input type="range" id="speed" min="0" max="255" value="200" onchange="try{fetch(document.location.origin+'/control?var=speed&val='+this.value);}catch(e){}"></td><td>  </td></tr>
                  <!--<tr><td align="right">Quality:</td><td align="center" colspan="2"><input type="range" id="quality" min="10" max="63" value="10" onchange="try{fetch(document.location.origin+'/control?var=quality&val='+this.value);}catch(e){}"></td><td>  </td></tr>
                  <tr><td align="right">Size:</td><td align="center" colspan="2"><input type="range" id="framesize" min="0" max="6" value="5" onchange="try{fetch(document.location.origin+'/control?var=framesize&val='+this.value);}catch(e){}"></td><td>  </td></tr>
                  -->
                  </table>
                </div>
               <br/>
               
            </section>         
        </section>   
        <script>
          document.addEventListener('DOMContentLoaded',function(){function b(B){let C;switch(B.type){case'checkbox':C=B.checked?1:0;break;case'range':case'select-one':C=B.value;break;case'button':case'submit':C='1';break;default:return;}if(B.id&&C!==undefined){const D=`${c}/control?var=${B.id}&val=${C}`;fetch(D).then(E=>{console.log(`request to ${D} finished, status: ${E.status}`)})}else{console.error("Invalid control parameters:",B.id,C)}}var c=document.location.origin;const e=B=>{B.classList.add('hidden')},f=B=>{B.classList.remove('hidden')},g=B=>{B.classList.add('disabled'),B.disabled=!0},h=B=>{B.classList.remove('disabled'),B.disabled=!1},i=(B,C,D)=>{D=!(null!=D)||D;let E;'checkbox'===B.type?(E=B.checked,C=!!C,B.checked=C):(E=B.value,B.value=C),D&&E!==C?b(B):!D&&('aec'===B.id?C?e(v):f(v):'agc'===B.id?C?(f(t),e(s)):(e(t),f(s)):'awb_gain'===B.id?C?f(x):e(x):'face_recognize'===B.id&&(C?h(n):g(n)))};document.querySelectorAll('.close').forEach(B=>{B.onclick=()=>{e(B.parentNode)}}),fetch(`${c}/status`).then(function(B){return B.json()}).then(function(B){document.querySelectorAll('.default-action').forEach(C=>{i(C,B[C.id],!1)})});const j=document.getElementById('stream'),k=document.getElementById('stream-container'),l=document.getElementById('get-still'),m=document.getElementById('toggle-stream'),n=document.getElementById('face_enroll'),o=document.getElementById('close-stream'),p=()=>{window.stop(),m.innerHTML='Start',console.log("Stream stopped")},q=()=>{j.src=`${c+':81'}/stream`,f(k),m.innerHTML='Stop',console.log("Stream started, src set to:", j.src)};l.onclick=()=>{p(),j.src=`${c}/capture?_cb=${Date.now()}`,f(k),console.log("Capture image, src set to:", j.src)},o.onclick=()=>{p(),e(k),console.log("Stream container closed")},m.onclick=()=>{const isStreaming = 'Stop' === m.innerHTML; alert(`Toggle stream button clicked, current state: ${isStreaming ? 'Stop' : 'Start'}`); isStreaming ? p() : q();},n.onclick=()=>{b(n)},document.querySelectorAll('.default-action').forEach(B=>{B.onchange=()=>b(B)});const r=document.getElementById('agc'),s=document.getElementById('agc_gain-group'),t=document.getElementById('gainceiling-group');r.onchange=()=>{b(r),r.checked?(f(t),e(s))};const u=document.getElementById('aec'),v=document.getElementById('aec_value-group');u.onchange=()=>{b(u),u.checked?e(v):f(v)};const w=document.getElementById('awb_gain'),x=document.getElementById('wb_mode-group');w.onchange=()=>{b(w),w.checked?f(x):e(x)};const y=document.getElementById('face_detect'),z=document.getElementById('face_recognize'),A=document.getElementById('framesize');A.onchange=()=>{b(A),5<A.value&&(i(y,!1),i(z,!1))},y.onchange=()=>{return 5<A.value?(alert('Please select CIF or lower resolution before enabling this feature!'),void i(y,!1)):void(b(y),!y.checked&&(g(n),i(z,!1)))},z.onchange=()=>{return 5<A.value?(alert('Please select CIF or lower resolution before enabling this feature!'),void i(z,!1)):void(b(z),z.checked?(h(n),i(y,!0)):g(n))}});
        </script>
        <script>
document.addEventListener('DOMContentLoaded', function() {
    console.log("JavaScript loaded and DOMContentLoaded triggered.");

    const m = document.getElementById('toggle-stream');
    if (m) {
        console.log("Start button found.");
        m.onclick = () => {
            console.log("Start button clicked.");
            const isStreaming = 'Stop' === m.innerHTML;
            console.log("Current state:", isStreaming ? "Stop" : "Start");
            isStreaming ? p() : q();
        };
    } else {
        console.error("Start button not found.");
    }

    const j = document.getElementById('stream');
    if (j) {
        console.log("Stream image element found.");
    } else {
        console.error("Stream image element not found.");
    }

    const p = () => {
        console.log("Stopping stream.");
        window.stop();
        m.innerHTML = 'Start';
    };

    const q = () => {
        console.log("Starting stream.");
        j.src = `${document.location.origin}:81/stream`;
        console.log("Stream source set to:", j.src);
        m.innerHTML = 'Stop';
    };
});
</script>
    </body>
</html>
)rawliteral";

// HTML for update page with improved styling and error handling
static const char* PROGMEM updateIndex = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 CAM Robot Car Update</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background: #f0f0f0;
        }
        .container {
            background: white;
            border-radius: 5px;
            padding: 20px;
            max-width: 400px;
            margin: 0 auto;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        h2 {
            color: #333;
            margin-top: 0;
        }
        .upload-form {
            margin-top: 20px;
        }
        input[type="file"] {
            display: block;
            margin: 10px 0;
            width: 100%;
        }
        input[type="submit"] {
            background: #4CAF50;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        input[type="submit"]:hover {
            background: #45a049;
        }
        #progress {
            margin-top: 20px;
            display: none;
        }
        .progress-bar {
            background: #f1f1f1;
            height: 20px;
            border-radius: 10px;
            overflow: hidden;
        }
        .progress-fill {
            background: #4CAF50;
            height: 100%;
            width: 0%;
            transition: width 0.3s;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>ESP32 CAM Robot Car Firmware Update</h2>
        <form method='POST' action='/update' enctype='multipart/form-data' class="upload-form">
            <input type='file' name='update' accept='.bin'>
            <input type='submit' value='Update Firmware'>
        </form>
        <div id="progress">
            <p>Update Progress:</p>
            <div class="progress-bar">
                <div class="progress-fill"></div>
            </div>
        </div>
    </div>
</body>
</html>
)";

static esp_err_t handle_update_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, updateIndex, strlen(updateIndex));
    return ESP_OK;
}

static esp_err_t handle_update_post(httpd_req_t *req) {
    char buf[1024];
    int ret;
    size_t remaining = req->content_len;

    // Check if there's enough space for the update
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("[OTA] Not enough space for update");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Not enough space for update");
        return ESP_FAIL;
    }

    Serial.printf("[OTA] Update started, total size: %d bytes\n", req->content_len);

    // Process the firmware file in chunks
    while (remaining > 0) {
        size_t chunk_size = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
        ret = httpd_req_recv(req, buf, chunk_size);
        
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            Serial.println("[OTA] Failed to receive data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }

        if (Update.write((uint8_t*)buf, ret) != ret) {
            Serial.println("[OTA] Failed to write data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to write data");
            return ESP_FAIL;
        }

        Serial.printf("[OTA] Progress: %d%%\n", (int)((req->content_len - remaining) * 100 / req->content_len));
        remaining -= ret;
    }

    if (!Update.end(true)) {
        Serial.println("[OTA] Update failed to complete");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Update failed to complete");
        return ESP_FAIL;
    }

    Serial.println("[OTA] Update successful");
    httpd_resp_sendstr(req, "Update successful! Rebooting...");
    
    delay(1000);
    ESP.restart();
    
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL};

  httpd_uri_t status_uri = {
      .uri = "/status",
      .method = HTTP_GET,
      .handler = status_handler,
      .user_ctx = NULL};

  httpd_uri_t cmd_uri = {
      .uri = "/control",
      .method = HTTP_GET,
      .handler = cmd_handler,
      .user_ctx = NULL};

  httpd_uri_t capture_uri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = capture_handler,
      .user_ctx = NULL};

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  httpd_uri_t update_uri = {
      .uri = "/update",
      .method = HTTP_GET,
      .handler = handle_update_get,
      .user_ctx = NULL
  };

  httpd_uri_t update_post_uri = {
      .uri = "/update",
      .method = HTTP_POST,
      .handler = handle_update_post,
      .user_ctx = NULL
  };

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &update_uri);
    httpd_register_uri_handler(camera_httpd, &update_post_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

unsigned int get_speed(unsigned int sp)
{
  return map(sp, 0, 100, 0, 255);
}

// Wrapper function to initialize PWM channels
void initPWMChannel(int channel, int freq, int resolution, int pin) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)resolution,
        .timer_num = (ledc_timer_t)(channel / 4),
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)(channel / 4),
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void robot_setup() {
    Serial.println("Initializing PWM channels for motors...");
    // Initialize PWM channels for motor control
    initPWMChannel(3, 2000, 8, RIGHT_M0);
    initPWMChannel(4, 2000, 8, RIGHT_M1);
    initPWMChannel(5, 2000, 8, LEFT_M0);
    initPWMChannel(6, 2000, 8, LEFT_M1);

    // Debug logs for initialization
    Serial.println("PWM channels initialized and attached to GPIO pins.");

    // Ensure motors are stopped
    robot_stop();

    // Debug log to confirm motors are stopped
    Serial.println("Motors stopped during setup.");

    // Test PWM output after all initializers
    test_pwm_output();
}

void setupLED() {
    // Configure LED pin as output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
    Serial.println("LED setup complete.");
}

// Motor Control Functions

void update_speed() {
    // Update all active channels with new speed value
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, get_speed(motor_speed));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

void robot_stop()
{
  Serial.println("Stopping motors...");
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void robot_fwd() {
    Serial.println("Executing robot_fwd()");
    // Front motors forward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
    move_interval = 250;
    previous_time = millis();
    Serial.printf("robot_fwd: PWM values - RIGHT_M0: %d, RIGHT_M1: %d, LEFT_M0: %d, LEFT_M1: %d\n", 0, speed, 0, speed);
}

void robot_back() {
    Serial.println("Executing robot_back()");
    // Front motors backward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
    move_interval = 250;
    previous_time = millis();
    Serial.printf("robot_back: PWM values - RIGHT_M0: %d, RIGHT_M1: %d, LEFT_M0: %d, LEFT_M1: %d\n", speed, 0, speed, 0);
}

void robot_right() {
    Serial.println("Executing robot_right()");
    // Turn right: Left motors forward, right motors backward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
    move_interval = 100;
    previous_time = millis();
    Serial.printf("robot_right: PWM values - RIGHT_M0: %d, RIGHT_M1: %d, LEFT_M0: %d, LEFT_M1: %d\n", 0, speed, speed, 0);
}

void robot_left() {
    Serial.println("Executing robot_left()");
    // Turn left: Right motors forward, left motors backward
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed);
    
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
    move_interval = 100;
    previous_time = millis();
    Serial.printf("robot_left: PWM values - RIGHT_M0: %d, RIGHT_M1: %d, LEFT_M0: %d, LEFT_M1: %d\n", speed, 0, 0, speed);
}

void test_pwm_output() {
    Serial.println("Testing PWM output with constant duty cycle...");

    // Set a constant duty cycle for all motor control pins
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 128); // 50% duty cycle for RIGHT_M0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 128); // 50% duty cycle for RIGHT_M1
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 128); // 50% duty cycle for LEFT_M0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, 128); // 50% duty cycle for LEFT_M1

    // Update the duty cycle to apply changes
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);

    Serial.println("PWM output test complete. Observe motor behavior.");
}
