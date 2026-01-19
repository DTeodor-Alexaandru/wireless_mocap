#pragma once
#include <Arduino.h>
#include <cstdint>

bool espnow_sender_init(const uint8_t peer_mac[6], int forced_channel = 0);

bool espnow_send_markers(const int16_t* points_xy, uint8_t count);
