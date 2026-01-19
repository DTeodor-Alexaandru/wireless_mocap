#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "camera_app.h"
#include "web_stream.h"
#include "ir_detect.h"
#include "espnow_sender.h"

static void wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(MODE_PIN, INPUT_PULLUP);
  bool stream_mode = (digitalRead(MODE_PIN) == HIGH) && ENABLE_STREAM_WHEN_HIGH;

  Serial.println();
  Serial.println("=== IR detect + ESP-NOW sender ===");
  Serial.printf("psramFound(): %s\n", psramFound() ? "YES" : "NO");
  Serial.printf("Mode: %s (GPIO%d=%d)\n", stream_mode ? "STREAM+DETECT" : "DETECT ONLY",
                MODE_PIN, digitalRead(MODE_PIN));

  // If streaming is enabled, connect WiFi first so ESP-NOW is on the same channel
  if (stream_mode) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting to WiFi");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("WiFi connect failed (stream may not work). ESP-NOW channel must still match receiver.");
    }
  } else {
    // Detection-only mode: still set STA (no need to connect)
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
  }

  // ESP-NOW init (channel = current; if you want forced channel, pass 1..14)
  if (!espnow_sender_init(RECV_MAC, 0)) {
    Serial.println("ESP-NOW init failed");
  } else {
    Serial.print("ESP-NOW ready. Sending to: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  RECV_MAC[0], RECV_MAC[1], RECV_MAC[2], RECV_MAC[3], RECV_MAC[4], RECV_MAC[5]);
  }

  // Camera init + capture task
  Serial.println("Initializing camera...");
  if (!camera_app_init()) {
    Serial.println("camera_app_init failed");
    while (true) delay(1000);
  }

  // Capture task on core 1, higher priority than detect
  camera_app_start_capture_task(/*core=*/1, /*prio=*/3, /*stack=*/8192);

  // Detection task on core 0 (or core 1 if you prefer), moderate priority
  ir_detect_start_task(/*core=*/0, /*prio=*/2, /*stack=*/16384);

  // Start web server (optional)
  if (stream_mode) {
    web_stream_start();
    Serial.println("Web stream started.");
  }

  Serial.println("Setup done.");
}

void loop() {
  static uint32_t last_count = 0;
  static uint32_t last_ms = 0;

  uint32_t now = millis();
  if (last_ms == 0) last_ms = now;

  if (now - last_ms >= 1000) {
    uint32_t cnt = g_capture_count.load(std::memory_order_relaxed);
    Serial.printf("Capture FPS (ESP32): %u\n", cnt - last_count);
    last_count = cnt;
    last_ms = now;
  }
  delay(10);
}
