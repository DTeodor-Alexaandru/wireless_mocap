#pragma once
#include <Arduino.h>
#include <cstdint>

// Initialize ESP-NOW and register a single peer (the receiver).
// If you already connect to WiFi for streaming, call this AFTER WiFi is connected
// so ESP-NOW uses the same channel.
//
// If you are NOT connected to WiFi, you can optionally force a channel (1..14)
// (must match the receiver's channel).
bool espnow_sender_init(const uint8_t peer_mac[6], int forced_channel = 0);

// Send up to max_count points (x,y) as int16_t.
// points_xy must be interleaved: [x0,y0,x1,y1,...]
bool espnow_send_markers(const int16_t* points_xy, uint8_t count);
