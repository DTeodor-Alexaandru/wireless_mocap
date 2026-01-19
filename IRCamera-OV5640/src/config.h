#pragma once
#include <Arduino.h>
#include "esp_camera.h"

// ===================== MODE SELECT (GPIO1 toggle) =====================
// INPUT_PULLUP: HIGH => stream+wifi ; LOW => detection only
static constexpr int MODE_PIN = 1;

static constexpr bool ENABLE_STREAM_WHEN_HIGH = true;

// ===================== WiFi =====================
static const char* WIFI_SSID = "UPB-Guest";
static const char* WIFI_PASS = "";

// ===================== Camera pins (Adafruit OV5640 breakout -> ESP32-S3) =====================
// Assumes OV5640 breakout XCLK jumper is set to INT, so pin_xclk = -1.
#define CAM_PIN_PWDN   15
#define CAM_PIN_RESET  -1

#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5

#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK   13
#define CAM_PIN_XCLK   -1

#define CAM_PIN_D0     11
#define CAM_PIN_D1     9
#define CAM_PIN_D2     8
#define CAM_PIN_D3     10
#define CAM_PIN_D4     12
#define CAM_PIN_D5     18
#define CAM_PIN_D6     17
#define CAM_PIN_D7     16

// ===================== Camera settings =====================
static constexpr framesize_t CAM_FRAME_SIZE = FRAMESIZE_VGA; // 640x480
static constexpr int CAM_JPEG_QUALITY = 15;                 // 0 best quality, 63 worst
static constexpr int CAM_FB_COUNT = 2;

// Manual exposure/gain for IR-pass stability (tune)
static constexpr int CAM_AEC_VALUE = 2500; // try 200, 400, 800, 1200
static constexpr int CAM_AGC_GAIN  = 30;   // try 0..30

static const uint8_t RECV_MAC[6] = { 0x7C,0xDF,0xA1,0xFB,0x71,0x30 };