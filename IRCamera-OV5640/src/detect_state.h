#pragma once
#include <Arduino.h>

// Store latest detection string (thread-safe for tasks/HTTP)
void detect_state_set(const char* s);
void detect_state_get(char* out, size_t out_cap);
