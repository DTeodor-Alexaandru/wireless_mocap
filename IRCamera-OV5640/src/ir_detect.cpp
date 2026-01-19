// ir_detect.cpp
// QVGA JPEG (320x240) -> RGB565 (2X downscale to 160x120) -> threshold -> RLE CCL -> centroids (back in QVGA coords)

#include "ir_detect.h"
#include "camera_app.h"
#include "detect_state.h"
#include "espnow_sender.h"
#include "config.h"

#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <Arduino.h>
#include <cstring>

// ===================== Source (camera) resolution (VGA) =====================
static constexpr int SRC_W = 640;
static constexpr int SRC_H = 480;

// ===================== Decode scale for detection =====================
// VGA -> 2X downscale => 420x240
static constexpr jpg_scale_t DECODE_SCALE = JPG_SCALE_2X;
static constexpr int SCALE_DIV = 2;

// Derived detection resolution
static constexpr int DET_W = SRC_W / SCALE_DIV;  // 420
static constexpr int DET_H = SRC_H / SCALE_DIV;  // 240

// ===================== Threshold / blob filtering =====================
static constexpr uint8_t  LUMA_THR = 75;
static constexpr uint32_t MIN_AREA = 1;       // tiny markers
static constexpr uint32_t MAX_AREA = 1500;    // reject glare

static constexpr int MAX_RUNS_PER_ROW = 200;
static constexpr int MAX_COMPONENTS   = 200;
static constexpr int MAX_PRINT        = 10;

// Print rate from detect task (ms). Set 0 to disable.
static constexpr uint32_t DEBUG_PRINT_PERIOD_MS = 500;

// ===================== Structures =====================
struct Run { int x0, x1; int label; }; // [x0, x1)
struct Comp { int parent; uint32_t area, sumx, sumy; bool used; };

// ===================== Buffers =====================
static uint8_t*  s_jpeg_copy = nullptr;
static size_t    s_jpeg_copy_cap = 0;

static uint16_t* s_rgb565 = nullptr;          // DET_W*DET_H pixels
static size_t    s_rgb565_cap_bytes = 0;

static uint32_t s_det_frames = 0;
static uint32_t s_det_last_ms = 0;

// Prefer PSRAM if available
static void* caps_malloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_8BIT);
  return p;
}

static bool ensure_jpeg_copy(size_t needed) {
  if (needed <= s_jpeg_copy_cap) return true;

  size_t new_cap = needed + (needed / 2) + 1024;
  uint8_t* nb = (uint8_t*)caps_malloc(new_cap);
  if (!nb) return false;

  if (s_jpeg_copy) free(s_jpeg_copy);
  s_jpeg_copy = nb;
  s_jpeg_copy_cap = new_cap;
  return true;
}

static bool ensure_rgb565() {
  const size_t need = (size_t)DET_W * (size_t)DET_H * 2;
  if (s_rgb565 && need <= s_rgb565_cap_bytes) return true;

  uint16_t* nb = (uint16_t*)caps_malloc(need);
  if (!nb) return false;

  if (s_rgb565) free(s_rgb565);
  s_rgb565 = nb;
  s_rgb565_cap_bytes = need;
  return true;
}

// RGB565 -> luma (cheap integer approx)
static inline uint8_t rgb565_luma(uint16_t p) {
  uint8_t r = ((p >> 11) & 0x1F) << 3;
  uint8_t g = ((p >> 5)  & 0x3F) << 2;
  uint8_t b = ( p        & 0x1F) << 3;
  return (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
}

static int find_root(Comp* comps, int a) {
  while (comps[a].parent != a) {
    comps[a].parent = comps[comps[a].parent].parent;
    a = comps[a].parent;
  }
  return a;
}

static void unite(Comp* comps, int a, int b) {
  int ra = find_root(comps, a);
  int rb = find_root(comps, b);
  if (ra == rb) return;

  if (comps[ra].area < comps[rb].area) { int t = ra; ra = rb; rb = t; }
  comps[rb].parent = ra;
  comps[ra].area += comps[rb].area;
  comps[ra].sumx += comps[rb].sumx;
  comps[ra].sumy += comps[rb].sumy;
  comps[rb].used = false;
}

static inline uint32_t sum_range(int x0, int x1) {
  uint32_t len = (uint32_t)(x1 - x0);
  return (uint32_t)(x0 + (x1 - 1)) * len / 2;
}

static void detect_and_publish(const uint16_t* img) {
  Run prev_runs[MAX_RUNS_PER_ROW];
  Run curr_runs[MAX_RUNS_PER_ROW];
  int prev_n = 0;

  Comp comps[MAX_COMPONENTS];
  int comp_n = 0;

  auto new_comp = [&]() -> int {
    if (comp_n >= MAX_COMPONENTS) return -1;
    comps[comp_n] = { comp_n, 0, 0, 0, true };
    return comp_n++;
  };

  // ===== RLE connected components on DET_W x DET_H =====
  for (int y = 0; y < DET_H; y++) {
    int curr_n = 0;
    int x = 0;
    const uint16_t* row = img + (y * DET_W);

    while (x < DET_W) {
      while (x < DET_W && rgb565_luma(row[x]) < LUMA_THR) x++;
      if (x >= DET_W) break;

      int x0 = x;
      while (x < DET_W && rgb565_luma(row[x]) >= LUMA_THR) x++;
      int x1 = x;

      if (curr_n >= MAX_RUNS_PER_ROW) {
        detect_state_set("IR blobs: too many runs (noise)");
        // Send heartbeat with 0 points if you want:
        // espnow_send_markers(nullptr, 0);
        return;
      }
      curr_runs[curr_n++] = { x0, x1, -1 };
    }

    int pi = 0;
    for (int ci = 0; ci < curr_n; ci++) {
      Run& cr = curr_runs[ci];

      while (pi < prev_n && prev_runs[pi].x1 <= cr.x0) pi++;

      int label = -1;
      int pj = pi;
      while (pj < prev_n && prev_runs[pj].x0 < cr.x1) {
        int pl = prev_runs[pj].label;
        if (pl >= 0) {
          if (label < 0) label = pl;
          else unite(comps, label, pl);
        }
        pj++;
      }

      if (label < 0) {
        label = new_comp();
        if (label < 0) {
          detect_state_set("IR blobs: too many components");
          // espnow_send_markers(nullptr, 0);
          return;
        }
      }

      label = find_root(comps, label);
      cr.label = label;

      uint32_t len = (uint32_t)(cr.x1 - cr.x0);
      comps[label].area += len;
      comps[label].sumx += sum_range(cr.x0, cr.x1);
      comps[label].sumy += (uint32_t)y * len;
    }

    for (int ci = 0; ci < curr_n; ci++) {
      curr_runs[ci].label = find_root(comps, curr_runs[ci].label);
    }

    prev_n = curr_n;
    for (int i = 0; i < curr_n; i++) prev_runs[i] = curr_runs[i];
  }

  // ===== Build output + build ESP-NOW payload =====
  char line[256];
  int off = 0;

  // Interleaved x,y points to send via ESP-NOW: [x0,y0,x1,y1,...]
  int16_t xy[MAX_PRINT * 2];
  uint8_t npts = 0;

  // Count valid detections (after filtering)
  int valid = 0;
  for (int i = 0; i < comp_n; i++) {
    if (!comps[i].used) continue;
    if (comps[i].parent != i) continue; // roots only
    if (comps[i].area < MIN_AREA || comps[i].area > MAX_AREA) continue;
    valid++;
  }

  off += snprintf(line + off, sizeof(line) - off, "IR blobs: %d ", valid);

  int printed = 0;
  for (int i = 0; i < comp_n && printed < MAX_PRINT; i++) {
    if (!comps[i].used) continue;
    if (comps[i].parent != i) continue;
    if (comps[i].area < MIN_AREA || comps[i].area > MAX_AREA) continue;

    int cx = (int)(comps[i].sumx / comps[i].area);
    int cy = (int)(comps[i].sumy / comps[i].area);

    // Scale back up to QVGA coordinates (or whatever SRC_* you used)
    int x_qvga = cx * SCALE_DIV;
    int y_qvga = cy * SCALE_DIV;

    // Store for ESP-NOW as int16
    if (npts < MAX_PRINT) {
      xy[npts * 2 + 0] = (int16_t)x_qvga;
      xy[npts * 2 + 1] = (int16_t)y_qvga;
      npts++;
    }

    off += snprintf(line + off, sizeof(line) - off,
                    "(%d,%d)[%u] ",
                    x_qvga, y_qvga, (unsigned)comps[i].area);
    printed++;
    if (off >= (int)sizeof(line)) break;
  }

  if (valid == 0) {
    snprintf(line, sizeof(line), "IR blobs: 0");
  }

  // Publish to web/serial state
  detect_state_set(line);

  // Send via ESP-NOW (coordinates). If you prefer only on detections, wrap with (npts > 0).
  espnow_send_markers((npts > 0) ? xy : nullptr, npts);

  // Throttled serial print
  if (DEBUG_PRINT_PERIOD_MS > 0) {
    static uint32_t last_print_ms = 0;
    uint32_t now = millis();
    if (now - last_print_ms >= DEBUG_PRINT_PERIOD_MS) {
      Serial.println(line);
      last_print_ms = now;
    }
  }
}

static void detect_task(void* pv) {
  (void)pv;

  if (!ensure_rgb565()) {
    Serial.println("detect_task: RGB565 alloc failed");
    vTaskDelete(nullptr);
    return;
  }

  uint32_t iter = 0;

  for (;;) {
    size_t len = 0;
    SemaphoreHandle_t m = camera_app_mutex();

    if (xSemaphoreTake(m, pdMS_TO_TICKS(50)) == pdTRUE) {
      size_t src_len = 0;
      const uint8_t* src = camera_app_latest_jpeg(src_len);
      if (src && src_len > 0 && ensure_jpeg_copy(src_len)) {
        memcpy(s_jpeg_copy, src, src_len);
        len = src_len;
      }
      xSemaphoreGive(m);
    }

    if (len == 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (jpg2rgb565(s_jpeg_copy, len, (uint8_t*)s_rgb565, DECODE_SCALE)) {
      detect_and_publish(s_rgb565);

      s_det_frames++;
      uint32_t now_ms = millis();
      if (s_det_last_ms == 0) s_det_last_ms = now_ms;

      if (now_ms - s_det_last_ms >= 1000) {
        Serial.printf("Detect FPS: %u\n", (unsigned)s_det_frames);
        s_det_frames = 0;
        s_det_last_ms = now_ms;
      }
    } else {
      detect_state_set("IR blobs: decode failed");
    }

    iter++;
    if ((iter & 0x0F) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    else taskYIELD();
  }
}

void ir_detect_start_task(int core, int prio, int stack) {
  xTaskCreatePinnedToCore(detect_task, "detect_task", stack, nullptr, prio, nullptr, core);
}
