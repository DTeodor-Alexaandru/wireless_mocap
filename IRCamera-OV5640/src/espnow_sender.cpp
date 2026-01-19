#include "espnow_sender.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static uint8_t s_peer_mac[6] = {0};
static bool s_inited = false;
static uint32_t s_seq = 0;

#pragma pack(push, 1)
struct MarkerHeader {
  uint32_t magic;  // 'MKRP' = 0x4D4B5250
  uint32_t seq;
  uint32_t ms;
  uint8_t  count;
};
#pragma pack(pop)

static void on_sent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  (void)status;
}

bool espnow_sender_init(const uint8_t peer_mac[6], int forced_channel) {
  if (!peer_mac) return false;

  memcpy(s_peer_mac, peer_mac, 6);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (forced_channel >= 1 && forced_channel <= 14) {
    esp_wifi_set_channel((uint8_t)forced_channel, WIFI_SECOND_CHAN_NONE);
  }

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_send_cb(on_sent);

  // Add peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, s_peer_mac, 6);
  peer.ifidx = WIFI_IF_STA;
  peer.channel = 0;     // 0 = use current channel
  peer.encrypt = false;

  esp_err_t e = esp_now_add_peer(&peer);
  if (e != ESP_OK && e != ESP_ERR_ESPNOW_EXIST) {
    return false;
  }

  s_inited = true;
  return true;
}

bool espnow_send_markers(const int16_t* points_xy, uint8_t count) {
  if (!s_inited) return false;
  if (!points_xy && count > 0) return false;

  const uint8_t MAX_POINTS = 20;
  if (count > MAX_POINTS) count = MAX_POINTS;

  // Build packet in a small stack buffer
  uint8_t buf[13 + MAX_POINTS * 4] = {0};

  MarkerHeader h;
  h.magic = 0x4D4B5250UL; // 'MKRP'
  h.seq   = s_seq++;
  h.ms    = millis();
  h.count = count;

  memcpy(buf, &h, sizeof(h));

  // Append int16 x,y pairs (little-endian on ESP32)
  if (count > 0) {
    memcpy(buf + sizeof(h), points_xy, (size_t)count * 2 * sizeof(int16_t));
  }

  const int len = (int)(sizeof(h) + (size_t)count * 2 * sizeof(int16_t));
  esp_err_t res = esp_now_send(s_peer_mac, buf, len);
  return (res == ESP_OK);
}
