#include "detect_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>

static SemaphoreHandle_t s_mutex = nullptr;
static char s_last[256] = "IR blobs: (none)";

static void ensure_mutex() {
  if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
}

void detect_state_set(const char* s) {
  ensure_mutex();
  if (!s_mutex) return;

  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    strncpy(s_last, s ? s : "", sizeof(s_last) - 1);
    s_last[sizeof(s_last) - 1] = '\0';
    xSemaphoreGive(s_mutex);
  }
}

void detect_state_get(char* out, size_t out_cap) {
  if (!out || out_cap == 0) return;
  ensure_mutex();
  if (!s_mutex) { out[0] = '\0'; return; }

  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    strncpy(out, s_last, out_cap - 1);
    out[out_cap - 1] = '\0';
    xSemaphoreGive(s_mutex);
  } else {
    // If busy, return a safe fallback
    strncpy(out, "IR blobs: (busy)", out_cap - 1);
    out[out_cap - 1] = '\0';
  }
}
