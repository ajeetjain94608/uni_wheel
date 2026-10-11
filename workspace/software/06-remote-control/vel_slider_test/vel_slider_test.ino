/**
 * AP WebSocket Velocity Slider Test
 *
 * Adapted from the reference repo's thumbstick_test.ino - this layer (WiFi AP,
 * AsyncWebServer, WebSocket, dual-core command sharing) is fully hardware-agnostic,
 * so it's reused essentially as-is. The only real change is one axis instead of
 * two: {vel} instead of {vel, yaw} - see index_html.h's docstring for why.
 *
 * Core 0: WiFi AP + AsyncWebServer + WebSocket handler
 * Core 1: Receives the velocity command (printed to Serial for now)
 *
 * Dependencies (install via Arduino Library Manager):
 *   - ESP Async WebServer  (me-no-dev/ESPAsyncWebServer)
 *   - AsyncTCP             (me-no-dev/AsyncTCP)
 *   - ArduinoJson
 *
 * Once flashed:
 *   1. Connect to WiFi network "UniWheel"  (password: "uniwheel1")
 *   2. Open browser to http://192.168.4.1
 *   3. Drag the slider up/down - watch Serial Monitor for values
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "index_html.h"

// ---------------------------------------------------------------------------
// AP credentials and static IP
// ---------------------------------------------------------------------------
static const char* AP_SSID     = "UniWheel";
static const char* AP_PASSWORD = "uniwheel1";  // WPA2 requires >= 8 chars

static const IPAddress AP_IP      (192, 168,  4,  1);
static const IPAddress AP_GATEWAY (192, 168,  4,  1);
static const IPAddress AP_SUBNET  (255, 255, 255,  0);

// ---------------------------------------------------------------------------
// Shared command state (written by Core 0 WS callback, read by Core 1)
// ---------------------------------------------------------------------------
static volatile float g_cmd_vel = 0.0f;  // forward/back  [-1, 1]
static portMUX_TYPE   g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

inline void setCommand(float vel) {
  portENTER_CRITICAL(&g_cmd_mux);
  g_cmd_vel = vel;
  portEXIT_CRITICAL(&g_cmd_mux);
}

inline float getCommand() {
  portENTER_CRITICAL(&g_cmd_mux);
  float vel = g_cmd_vel;
  portEXIT_CRITICAL(&g_cmd_mux);
  return vel;
}

// ---------------------------------------------------------------------------
// Server and WebSocket
// ---------------------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onWsEvent(AsyncWebSocket* server,
               AsyncWebSocketClient* client,
               AwsEventType type,
               void* arg,
               uint8_t* data,
               size_t len)
{
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected\n", client->id());

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected - zeroing command\n", client->id());
    setCommand(0.0f);

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      char buf[64];
      size_t copy_len = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, data, copy_len);
      buf[copy_len] = '\0';

      StaticJsonDocument<64> doc;
      if (deserializeJson(doc, buf) == DeserializationError::Ok) {
        float vel = doc["vel"] | 0.0f;
        vel = constrain(vel, -1.0f, 1.0f);
        setCommand(vel);
      }
    }
  }
}

void serverTask(void* pvParameters) {
  Serial.println("[Core 0] Starting WiFi AP...");

  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.printf("[Core 0] AP started: SSID=%s  IP=%s\n",
                AP_SSID,
                WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("[Core 0] HTTP + WebSocket server running");

  for (;;) {
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== AP WebSocket Velocity Slider Test ===");

  xTaskCreatePinnedToCore(
    serverTask,
    "serverTask",
    8192,
    nullptr,
    1,
    nullptr,
    0    // Core 0
  );
}

void loop() {
  // Read the shared command and print it - replace this with actor inference +
  // motor control once combined into uniwheel_rc/uniwheel_bot_rc.ino
  float vel = getCommand();
  Serial.printf("[Core 1] cmd_vel=%.3f\n", vel);
  vTaskDelay(pdMS_TO_TICKS(200));
}
