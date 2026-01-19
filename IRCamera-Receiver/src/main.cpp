#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <math.h>

// ===================== Camera model (OpenCV rad-tan) =====================
struct CameraModel {
  float fx, fy, cx, cy;
  float k1, k2, p1, p2, k3;
};

static constexpr CameraModel CAM = {
  .fx = 289.6279631884157f,
  .fy = 287.9244412454326f,
  .cx = 301.73071808653646f,
  .cy = 244.6318943203005f,
  .k1 = -0.08492335903539473f,
  .k2 =  0.07544587446815652f,
  .p1 = -0.0004627603830659724f,
  .p2 = -0.0019418083577975405f,
  .k3 = -0.031046398950348036f
};

// Baseline (meters) - set to your measured value
static constexpr float BASELINE_M = 0.426f;

// Triangulation guard
static constexpr float MIN_DENOM = 1e-6f;

// Undistort iterations (6..10 is typical)
static constexpr int UNDISTORT_ITERS = 8;

static inline void pixel_to_normalized(float u, float v, const CameraModel& cam, float& x, float& y) {
  x = (u - cam.cx) / cam.fx;
  y = (v - cam.cy) / cam.fy;
}

// Iterative inversion of OpenCV distortion model (radial-tangential)
static inline void undistort_normalized_iter(float xd, float yd, const CameraModel& cam, int iters, float& xu, float& yu) {
  float x = xd;
  float y = yd;

  for (int i = 0; i < iters; i++) {
    float r2 = x*x + y*y;
    float r4 = r2*r2;
    float r6 = r4*r2;

    float radial = 1.0f + cam.k1*r2 + cam.k2*r4 + cam.k3*r6;

    float dx = 2.0f*cam.p1*x*y + cam.p2*(r2 + 2.0f*x*x);
    float dy = cam.p1*(r2 + 2.0f*y*y) + 2.0f*cam.p2*x*y;

    x = (xd - dx) / radial;
    y = (yd - dy) / radial;
  }

  xu = x;
  yu = y;
}

// Parallel/rectified triangulation using undistorted normalized rays
// Returns true if OK
static inline bool triangulate_parallel(float xL, float yL, float xR, float yR, float B, float& X, float& Y, float& Z) {
  float denom = (xL - xR);
  if (fabsf(denom) < MIN_DENOM) return false;

  float z = B / denom;

  // enforce positive Z
  if (z < 0) z = -z;

  X = xL * z;
  Y = yL * z;
  Z = z;
  return true;
}


// ===================== WiFi (lock channel) =====================
static const char* WIFI_SSID = "UPB-Guest";
static const char* WIFI_PASS = "";

// ===================== Packet format (must match sender) =====================
#pragma pack(push, 1)
struct MarkerHeader {
  uint32_t magic;   // 'MKRP' = 0x4D4B5250
  uint32_t seq;
  uint32_t ms;      // sender millis()
  uint8_t  count;   // number of points
};
#pragma pack(pop)

static constexpr uint32_t MKRP_MAGIC = 0x4D4B5250UL;

// ===================== Stereo parameters =====================
// Intrinsics (from your JSON)
static constexpr float fx = 289.6279631884157f;
static constexpr float fy = 287.9244412454326f;
static constexpr float cx = 301.73071808653646f;
static constexpr float cy = 244.6318943203005f;

// Baseline in meters
static constexpr float B  = 0.463f;

// If your senders transmit QVGA coords (0..319,0..239) but intrinsics are VGA,
// set to 2 to scale up: x_vga = x_in * 2.
static constexpr int INPUT_COORD_SCALE = 2;   // 1 for VGA coords, 2 for QVGA->VGA

// Matching constraint: abs(yL - yR) <= 20 pixels (after scaling)
static constexpr int Y_MATCH_TOL = 20;

// Reject near-zero disparity
static constexpr float MIN_DISPARITY_PX = 1.0f;

// How long (ms) we consider a sender's points "fresh"
static constexpr uint32_t FRESH_MS = 200;

// ===================== Sender table =====================
static constexpr int MAX_SENDERS = 10;
static constexpr int MAX_PTS     = 10;  // must be >= sender's MAX_PRINT

struct SenderInfo {
  uint8_t  mac[6];
  bool     used = false;
  int8_t   last_rssi = 0;
};

static SenderInfo g_senders[MAX_SENDERS];

// Latest data from sender 0 and 1
struct PointsFrame {
  bool     valid = false;
  uint32_t seq = 0;
  uint32_t recv_ms = 0;
  uint8_t  count = 0;
  int16_t  xy[MAX_PTS * 2]; // interleaved x,y
};

static PointsFrame g_frame0;
static PointsFrame g_frame1;

// Avoid printing the same triangulation repeatedly
static uint32_t g_last_pair_seq0 = 0xFFFFFFFF;
static uint32_t g_last_pair_seq1 = 0xFFFFFFFF;

// ===================== Helpers =====================
static void print_mac(const uint8_t* mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int get_sender_id(const uint8_t* mac) {
  for (int i = 0; i < MAX_SENDERS; i++) {
    if (g_senders[i].used && memcmp(g_senders[i].mac, mac, 6) == 0) return i;
  }
  for (int i = 0; i < MAX_SENDERS; i++) {
    if (!g_senders[i].used) {
      g_senders[i].used = true;
      memcpy(g_senders[i].mac, mac, 6);
      return i;
    }
  }
  return -1;
}

static void store_points_frame(PointsFrame& dst, const MarkerHeader* h, const uint8_t* payload_xy, int payload_len) {
  dst.valid = true;
  dst.seq = h->seq;
  dst.recv_ms = millis();

  uint8_t n = h->count;
  if (n > MAX_PTS) n = MAX_PTS;

  const int need = (int)n * 2 * (int)sizeof(int16_t);
  if (payload_len < need) {
    dst.valid = false;
    dst.count = 0;
    return;
  }

  dst.count = n;
  memcpy(dst.xy, payload_xy, need);
}

static inline void scale_point(int16_t xin, int16_t yin, float& x, float& y) {
  x = (float)xin * (float)INPUT_COORD_SCALE;
  y = (float)yin * (float)INPUT_COORD_SCALE;
}

static void triangulate_if_possible() {
  const uint32_t now = millis();

  if (!g_frame0.valid || !g_frame1.valid) return;
  if ((now - g_frame0.recv_ms) > FRESH_MS) return;
  if ((now - g_frame1.recv_ms) > FRESH_MS) return;

  // Avoid recomputing identical frame pairs
  if (g_frame0.seq == g_last_pair_seq0 && g_frame1.seq == g_last_pair_seq1) return;
  g_last_pair_seq0 = g_frame0.seq;
  g_last_pair_seq1 = g_frame1.seq;

  bool usedR[MAX_PTS] = {false};

  for (uint8_t i = 0; i < g_frame0.count; i++) {
    // Left point (incoming)
    int16_t xL_in = g_frame0.xy[i * 2 + 0];
    int16_t yL_in = g_frame0.xy[i * 2 + 1];

    // Scale to VGA pixel coordinates if needed
    float uL = (float)xL_in * (float)INPUT_COORD_SCALE;
    float vL = (float)yL_in * (float)INPUT_COORD_SCALE;

    // Find best match in right by y proximity in PIXEL space
    int best_j = -1;
    float best_dy = 1e9f;

    for (uint8_t j = 0; j < g_frame1.count; j++) {
      if (usedR[j]) continue;

      int16_t xR_in = g_frame1.xy[j * 2 + 0];
      int16_t yR_in = g_frame1.xy[j * 2 + 1];

      float uR = (float)xR_in * (float)INPUT_COORD_SCALE;
      float vR = (float)yR_in * (float)INPUT_COORD_SCALE;

      float dy = fabsf(vL - vR);
      if (dy <= (float)Y_MATCH_TOL && dy < best_dy) {
        best_dy = dy;
        best_j = (int)j;
      }
    }

    if (best_j < 0) continue;
    usedR[best_j] = true;

    // Right matched point (incoming)
    int16_t xR_in = g_frame1.xy[best_j * 2 + 0];
    int16_t yR_in = g_frame1.xy[best_j * 2 + 1];

    float uR = (float)xR_in * (float)INPUT_COORD_SCALE;
    float vR = (float)yR_in * (float)INPUT_COORD_SCALE;

    // ----- Distorted normalized -----
    float xLd, yLd, xRd, yRd;
    pixel_to_normalized(uL, vL, CAM, xLd, yLd);
    pixel_to_normalized(uR, vR, CAM, xRd, yRd);

    // ----- Undistort normalized -----
    float xL, yL, xR, yR;
    undistort_normalized_iter(xLd, yLd, CAM, UNDISTORT_ITERS, xL, yL);
    undistort_normalized_iter(xRd, yRd, CAM, UNDISTORT_ITERS, xR, yR);

    // ----- Triangulate -----
    float X, Y, Z;
    if (!triangulate_parallel(xL, yL, xR, yR, BASELINE_M, X, Y, Z)) {
      continue;
    }

    // Print ONLY if determined
    Serial.printf("X,Y,Z = %.3f %.3f %.3f (m)\n", X, Y, Z);
  }
}


// ===================== ESP-NOW recv callback =====================
static void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len <= 0) return;

  const uint8_t* mac = info->src_addr;
  int sender_id = get_sender_id(mac);

  int8_t rssi = 0;
  if (info->rx_ctrl) rssi = info->rx_ctrl->rssi;

  if (sender_id >= 0) g_senders[sender_id].last_rssi = rssi;

  if (len < (int)sizeof(MarkerHeader)) return;

  const MarkerHeader* h = (const MarkerHeader*)data;
  if (h->magic != MKRP_MAGIC) return;

  const int expected = (int)sizeof(MarkerHeader) + (int)h->count * 2 * (int)sizeof(int16_t);
  if (len < expected) return;

  const uint8_t* payload = data + sizeof(MarkerHeader);
  const int payload_len = len - (int)sizeof(MarkerHeader);

  // Store only first two senders (SenderID 0 and 1)
  if (sender_id == 0) {
    store_points_frame(g_frame0, h, payload, payload_len);
  } else if (sender_id == 1) {
    store_points_frame(g_frame1, h, payload, payload_len);
  }

  // Try triangulation whenever either sender updates
  if (sender_id == 0 || sender_id == 1) {
    triangulate_if_possible();
  }
}

// ===================== WiFi channel lock =====================
static void wifi_lock_channel() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 8000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected (ESP-NOW still works if channels match).");
  }

  Serial.print("Receiver STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n=== ESP-NOW Receiver + Stereo Triangulation ===");
  Serial.printf("Using INPUT_COORD_SCALE=%d (incoming coords scaled by this before triangulation)\n", INPUT_COORD_SCALE);
  Serial.printf("Y match tolerance: %d px\n", Y_MATCH_TOL);
  wifi_lock_channel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(on_recv);
  Serial.println("Ready.");
}

void loop() {
  delay(1000);
}
