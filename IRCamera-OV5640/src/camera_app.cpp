#include "camera_app.h"
#include "config.h"
#include <atomic>

#include "esp_camera.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

std::atomic<uint32_t> g_capture_count{0};

static SemaphoreHandle_t s_frame_mutex = nullptr;
static uint8_t* s_latest_jpeg = nullptr;
static size_t s_latest_len = 0;
static size_t s_latest_cap = 0;

static void camera_power_cycle() {
  pinMode(CAM_PIN_PWDN, OUTPUT);
  digitalWrite(CAM_PIN_PWDN, 1);
  delay(50);
  digitalWrite(CAM_PIN_PWDN, 0);
  delay(50);
}

static bool ensure_frame_capacity(size_t needed) {
  if (needed <= s_latest_cap) return true;

  size_t new_cap = needed + (needed / 2) + 2048;

  uint8_t* new_buf = (uint8_t*)heap_caps_malloc(new_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!new_buf) new_buf = (uint8_t*)heap_caps_malloc(new_cap, MALLOC_CAP_8BIT);
  if (!new_buf) return false;

  if (s_latest_jpeg) free(s_latest_jpeg);
  s_latest_jpeg = new_buf;
  s_latest_cap  = new_cap;
  s_latest_len  = 0;
  return true;
}

SemaphoreHandle_t camera_app_mutex() { return s_frame_mutex; }

const uint8_t* camera_app_latest_jpeg(size_t& len) {
  len = s_latest_len;
  return s_latest_jpeg;
}

bool camera_app_copy_latest(uint8_t* dst, size_t cap, size_t& out_len) {
  out_len = 0;
  if (!dst || cap == 0) return false;

  if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return false;

  if (!s_latest_jpeg || s_latest_len == 0 || s_latest_len > cap) {
    xSemaphoreGive(s_frame_mutex);
    return false;
  }
  memcpy(dst, s_latest_jpeg, s_latest_len);
  out_len = s_latest_len;
  xSemaphoreGive(s_frame_mutex);
  return true;
}

bool camera_app_init() {
  if (!s_frame_mutex) s_frame_mutex = xSemaphoreCreateMutex();
  if (!s_frame_mutex) return false;

  camera_power_cycle();

  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;

  cfg.pin_d0 = CAM_PIN_D0;
  cfg.pin_d1 = CAM_PIN_D1;
  cfg.pin_d2 = CAM_PIN_D2;
  cfg.pin_d3 = CAM_PIN_D3;
  cfg.pin_d4 = CAM_PIN_D4;
  cfg.pin_d5 = CAM_PIN_D5;
  cfg.pin_d6 = CAM_PIN_D6;
  cfg.pin_d7 = CAM_PIN_D7;

  cfg.pin_xclk  = CAM_PIN_XCLK;
  cfg.pin_pclk  = CAM_PIN_PCLK;
  cfg.pin_vsync = CAM_PIN_VSYNC;
  cfg.pin_href  = CAM_PIN_HREF;

  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;

  cfg.pin_pwdn  = CAM_PIN_PWDN;
  cfg.pin_reset = CAM_PIN_RESET;

  cfg.xclk_freq_hz = 20000000; // unused when pin_xclk=-1

  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = CAM_FRAME_SIZE;
  cfg.jpeg_quality = CAM_JPEG_QUALITY;
  cfg.fb_count     = CAM_FB_COUNT;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;
  cfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAM_FRAME_SIZE);
    s->set_quality(s, CAM_JPEG_QUALITY);

    // IR-pass stability: manual exposure/gain
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_aec_value(s, CAM_AEC_VALUE);
    s->set_agc_gain(s, CAM_AGC_GAIN);

    s->set_awb_gain(s, 0);
    s->set_wb_mode(s, 0);
    s->set_special_effect(s, 0);
  }

  return true;
}

static void capture_task(void* pv) {
  (void)pv;

  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (fb->format == PIXFORMAT_JPEG && fb->len > 0) {
      if (xSemaphoreTake(s_frame_mutex, portMAX_DELAY) == pdTRUE) {
        if (ensure_frame_capacity(fb->len)) {
          memcpy(s_latest_jpeg, fb->buf, fb->len);
          s_latest_len = fb->len;
        } else {
          s_latest_len = 0;
        }
        xSemaphoreGive(s_frame_mutex);
      }
      g_capture_count.fetch_add(1, std::memory_order_relaxed);
    }

    esp_camera_fb_return(fb);
    taskYIELD();
  }
}

void camera_app_start_capture_task(int core, int prio, int stack) {
  xTaskCreatePinnedToCore(capture_task, "capture_task", stack, nullptr, prio, nullptr, core);
}
