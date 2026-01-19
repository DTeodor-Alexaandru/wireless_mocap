#include "web_stream.h"
#include "camera_app.h"
#include "detect_state.h"

#include "esp_http_server.h"
#include <cstring>

static httpd_handle_t s_server = nullptr;        // port 80
static httpd_handle_t s_stream_server = nullptr; // port 81

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char* STREAM_PART_HEADER  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-S3 OV5640</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 1.5rem; }
    img  { max-width: 100%; height: auto; border: 1px solid #ccc; }
    code { background: #f5f5f5; padding: 0.2rem 0.35rem; }
    pre  { background: #f5f5f5; padding: 0.75rem; border: 1px solid #ddd; }
  </style>
</head>
<body>
  <h1>ESP32-S3 OV5640</h1>
  <p>
    MJPEG: <code>:81/stream</code> |
    Single JPEG: <code>/jpg</code> |
    Detection: <code>/detect</code>
  </p>

  <h3>Last detection</h3>
  <pre id="det">loading...</pre>

  <h3>Stream</h3>
  <img id="img" />

  <script>
    // Stream on port 81 so it doesn't block other endpoints on port 80
    document.getElementById('img').src = `http://${location.hostname}:81/stream`;

    async function refreshDet(){
      try{
        const r = await fetch('/detect', {cache:'no-store'});
        document.getElementById('det').textContent = await r.text();
      }catch(e){
        document.getElementById('det').textContent = 'error';
      }
    }
    refreshDet();
    setInterval(refreshDet, 250);
  </script>
</body>
</html>
)HTML";

static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t detect_handler(httpd_req_t* req) {
  char buf[256];
  detect_state_get(buf, sizeof(buf));
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t jpg_handler(httpd_req_t* req) {
  SemaphoreHandle_t m = camera_app_mutex();
  if (xSemaphoreTake(m, pdMS_TO_TICKS(200)) != pdTRUE) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mutex timeout");
    return ESP_FAIL;
  }

  size_t len = 0;
  const uint8_t* jpg = camera_app_latest_jpeg(len);
  if (!jpg || len == 0) {
    xSemaphoreGive(m);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "No frame yet", HTTPD_RESP_USE_STRLEN);
  }

  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char*)jpg, len);
  xSemaphoreGive(m);
  return res;
}

// Stream handler (runs forever) — must be on a separate server instance
static esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  uint8_t* local = nullptr;
  size_t local_cap = 0;

  while (true) {
    SemaphoreHandle_t m = camera_app_mutex();
    if (xSemaphoreTake(m, pdMS_TO_TICKS(200)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    size_t len = 0;
    const uint8_t* jpg = camera_app_latest_jpeg(len);
    if (!jpg || len == 0) {
      xSemaphoreGive(m);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (len > local_cap) {
      uint8_t* nl = (uint8_t*)realloc(local, len);
      if (!nl) {
        xSemaphoreGive(m);
        free(local);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream realloc failed");
        return ESP_FAIL;
      }
      local = nl;
      local_cap = len;
    }

    memcpy(local, jpg, len);
    xSemaphoreGive(m);

    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) break;

    char header[128];
    int hlen = snprintf(header, sizeof(header), STREAM_PART_HEADER, (unsigned)len);
    res = httpd_resp_send_chunk(req, header, hlen);
    if (res != ESP_OK) break;

    res = httpd_resp_send_chunk(req, (const char*)local, len);
    if (res != ESP_OK) break;

    taskYIELD();
  }

  free(local);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

bool web_stream_start() {
  // -------- Server on port 80 (index, jpg, detect) --------
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.server_port = 80;
  c.ctrl_port   = 32768;
  c.lru_purge_enable = true;

  if (httpd_start(&s_server, &c) != ESP_OK) return false;

  httpd_uri_t uri_index  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = nullptr };
  httpd_uri_t uri_jpg    = { .uri = "/jpg",    .method = HTTP_GET, .handler = jpg_handler,    .user_ctx = nullptr };
  httpd_uri_t uri_detect = { .uri = "/detect", .method = HTTP_GET, .handler = detect_handler, .user_ctx = nullptr };

  httpd_register_uri_handler(s_server, &uri_index);
  httpd_register_uri_handler(s_server, &uri_jpg);
  httpd_register_uri_handler(s_server, &uri_detect);

  // -------- Stream server on port 81 (stream only) --------
  httpd_config_t s = HTTPD_DEFAULT_CONFIG();
  s.server_port = 81;
  s.ctrl_port   = 32769; // must be different from port 80 server
  s.lru_purge_enable = true;

  if (httpd_start(&s_stream_server, &s) != ESP_OK) return false;

  httpd_uri_t uri_stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = nullptr };
  httpd_register_uri_handler(s_stream_server, &uri_stream);

  return true;
}

void web_stream_stop() {
  if (s_stream_server) { httpd_stop(s_stream_server); s_stream_server = nullptr; }
  if (s_server)        { httpd_stop(s_server);        s_server = nullptr; }
}
