#pragma once
#include <Arduino.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

bool camera_app_init();
void camera_app_start_capture_task(int core = 1, int prio = 3, int stack = 8192);

SemaphoreHandle_t camera_app_mutex();
const uint8_t* camera_app_latest_jpeg(size_t& len);   // valid only while mutex held
bool camera_app_copy_latest(uint8_t* dst, size_t cap, size_t& out_len); // copies under lock
bool camera_app_ensure_local_copy(size_t needed);

extern std::atomic<uint32_t> g_capture_count;