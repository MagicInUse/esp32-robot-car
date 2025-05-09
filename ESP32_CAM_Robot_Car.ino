/*
  ESP32_CAM_Robot_Car
  app_httpd.cpp
  Uses L298N Motor Driver
  
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/ledc.h"
#include <Update.h>

// Firmware version to be updated on major milestones
#define FIRMWARE_VERSION "1.0.0"

// Setup Access Point Credentials
const char* ssid1 = "MadKatz Hot Rod";
const char* password1 = "1234567890";

extern volatile unsigned int motor_speed;
extern void robot_stop();
extern void robot_setup();
extern uint8_t robo;
extern volatile unsigned long previous_time;        
extern volatile unsigned long move_interval; 

// Camera pinout configuration for AI-THINKER ESP32-CAM
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// LED Control
#define LED_PIN           4
#define LED_TIMER        LEDC_TIMER_1
#define LED_MODE         LEDC_LOW_SPEED_MODE
#define LED_OUTPUT       LEDC_CHANNEL_7  // Use proper enum value instead of raw number
#define LED_FREQ         5000
#define LED_RESOLUTION   LEDC_TIMER_8_BIT

void startCameraServer();

void initLED() {
    ledc_timer_config_t led_timer;
    led_timer.speed_mode = LED_MODE;
    led_timer.duty_resolution = LED_RESOLUTION;
    led_timer.timer_num = LED_TIMER;
    led_timer.freq_hz = LED_FREQ;
    led_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&led_timer);

    ledc_channel_config_t led_channel;
    led_channel.gpio_num = LED_PIN;
    led_channel.speed_mode = LED_MODE;
    led_channel.channel = LED_OUTPUT;
    led_channel.intr_type = LEDC_INTR_DISABLE;
    led_channel.timer_sel = LED_TIMER;
    led_channel.duty = 0;
    led_channel.hpoint = 0;
    led_channel.flags.output_invert = 0;
    ledc_channel_config(&led_channel);
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Check if PSRAM is available and set higher resolution if it is
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 2);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("ESP32 CAM Robot Car");
  Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);

  // Initialize camera
  initCamera();

  // Initialize LED
  initLED();

  // Configure WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid1, password1);
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("OTA Update available at http://" + myIP.toString() + "/update");
  
  startCameraServer();

  // Setup robot
  robot_setup();
  
  // Flash LED to indicate setup complete
  for (int i=0; i<5; i++) {
    ledc_set_duty(LED_MODE, LED_OUTPUT, 10);
    ledc_update_duty(LED_MODE, LED_OUTPUT);
    delay(50);
    ledc_set_duty(LED_MODE, LED_OUTPUT, 0);
    ledc_update_duty(LED_MODE, LED_OUTPUT);
    delay(50);    
  }
  digitalWrite(33, LOW);
      
  previous_time = millis();
}

void loop() {
  if(robo) {
    unsigned long currentMillis = millis();
    if (currentMillis - previous_time >= move_interval) {
      previous_time = currentMillis;
      delay(2000); // Add a 2-second delay before stopping the motors
      robot_stop();
      char rsp[32];
      sprintf(rsp, "SPEED: %d", motor_speed);
      Serial.println("Stop");
      robo = 0;
    }
  }
  delay(1);
  yield();
}
