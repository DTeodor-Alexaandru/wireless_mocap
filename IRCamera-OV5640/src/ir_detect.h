#pragma once
#include <Arduino.h>
#include "detect_state.h"

void ir_detect_start_task(int core = 0, int prio = 2, int stack = 16384);
