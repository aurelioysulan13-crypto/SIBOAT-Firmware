/* ===========================================================================
   SIBOAT - MAIN ESP32 SUPER FINAL FIRMWARE
   (NodeMCU ESP-32S 38-pin  /  board: "ESP32 Dev Module")
   ---------------------------------------------------------------------------
   Works on Arduino-ESP32 core 2.x AND 3.x (ArduinoDroid uses a newer core that
   no longer has ledcSetup/ledcAttachPin, so PWM here talks to the ESP-IDF LEDC
   driver directly, which is identical on every core version).
   NO external libraries are needed: only core libraries: WiFi, WebServer, ESPmDNS, ArduinoOTA, Preferences, LittleFS, BLE
   (all built into the ESP32 core). The WebSocket server, JSON parser and servo
   PWM are implemented inside this file.

   WHAT THIS SKETCH DOES
   - Wi-Fi station on the phone hotspot, static IP 10.90.102.50, mDNS siboat.local
   - REST API on port 80  (same endpoints the SIBOAT/TriSeaBot dashboard calls)
   - WebSocket telemetry + joystick on  ws://<ip>:81/ws
   - UART2 to ESP32-CAM, 115200 baud:  TX = GPIO17, RX = GPIO35
        CAM -> MAIN  <CAT,1|2|3,conf,SEQ,n[,label]>   <REJ,conf,SEQ,n,label>
                     <HB,ok,ip,fps,inferMs>
        MAIN -> CAM  ACK1 / ACK2 / ACK3 / REJECT / ERROR      <ST,cls,safe>
   - 3 x BTS7960 (left prop, right prop, conveyor), 2 x MG90S, HC-SR04
   - Safe mode, emergency stop, stationary mode, persistent counters
   =========================================================================== */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <FS.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <time.h>
#include <stddef.h>
#if defined(__has_include)
#if __has_include(<esp_random.h>)
#include <esp_random.h>
#endif
#endif
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "driver/ledc.h"      // ESP-IDF LEDC driver (same API on core 2.x and 3.x)

#define FW_VERSION "1.6.0-final-ota-ble-outbox"
#define SIBOAT_BLE_SERVICE_UUID        "e9b10000-9b5d-4c79-a012-5349424f4154"
#define SIBOAT_BLE_WRITE_UUID          "e9b10001-9b5d-4c79-a012-5349424f4154"
#define SIBOAT_BLE_NOTIFY_UUID         "e9b10002-9b5d-4c79-a012-5349424f4154"

// Cloud RPC traffic is deliberately isolated from the safety/control loop.
// Jobs are heap-backed pointers so the main Arduino loop never places a
// multi-kilobyte payload on its limited task stack.
#define CLOUD_RPC_QUEUE_DEPTH 4
#define CLOUD_RPC_PATH_SIZE 112
#define CLOUD_RPC_PAYLOAD_SIZE 8192
#define CLOUD_RPC_TASK_STACK 12288
struct CloudRpcJob {
  uint8_t kind;
  uint32_t sequence;
  char path[CLOUD_RPC_PATH_SIZE];
  char payload[CLOUD_RPC_PAYLOAD_SIZE];
};
enum : uint8_t { CLOUD_JOB_DISCOVERY=1, CLOUD_JOB_RUNTIME=2, CLOUD_JOB_EVENT=3, CLOUD_JOB_PERFORMANCE=4, CLOUD_JOB_SESSION=5 };
static QueueHandle_t cloudRpcQueue = nullptr;
static TaskHandle_t cloudRpcTaskHandle = nullptr;
static volatile bool discoveryEverPublished = false;
static uint32_t lastControllerEventCloudSeq = 0;
static uint32_t lastControllerEventCloudAttemptMs = 0;
// Performance item outcomes are sent one at a time from the persistent LittleFS
// outbox. A failed upload leaves the event file in place for retry after reconnection.
static volatile uint32_t lastPerformanceEventCloudSeq = 0;
static volatile uint32_t lastPerformanceEventQueuedSeq = 0;
static uint32_t lastPerformanceEventAttemptMs = 0;
static volatile bool performanceSessionCloudKnown = false;
static volatile bool performanceSessionCloudActive = false;
static volatile bool performanceSessionJobQueued = false;
static bool outboxMounted = false;
static bool outboxStorageHealthy = false;
static bool outboxStorageFault = false;
static bool outboxHasCorruptRecord = false;
static uint32_t outboxWriteFailures = 0;
static uint32_t lastOutboxRetryMs = 0;
static uint32_t outboxRetryDelayMs = 2000UL;
static const char* OUTBOX_DIR = "/outbox";
static uint32_t lastPerformanceSessionAttemptMs = 0;
// ---- v1.6 additions: persistent outbox cache, backend state, boot/deployment clock, OTA ----
#define OUTBOX_SCHEMA 2
#define OUTBOX_RESERVE_BYTES 16384UL     // free flash kept for LittleFS metadata; below this the hold policy applies
#define STATUS_BUF_SIZE 9000
static bool outboxReformatted = false;
static bool outboxFull = false;
static uint32_t outboxQuarantined = 0;
static uint32_t outboxCount = 0;          // cached pending (committed) records
static uint32_t outboxNextSeq = 0;        // cached oldest pending sequence (0 = unknown -> rescan)
static uint32_t outboxUsedCache = 0;
static uint32_t outboxUsedAt = 0;
static volatile uint32_t perfAckSeq = 0;  // cloud worker -> main loop acknowledgement hand-off
static volatile bool perfAckReady = false;
static volatile uint8_t perfFailStreak = 0;
static volatile bool backendReachable = false;
static volatile uint32_t lastBackendOkMs = 0;
static volatile uint8_t backendFailStreak = 0;
static uint32_t bootNumber = 0;           // persistent boot counter (NVS)
static uint32_t deployBaseSec = 0;        // persistent cumulative powered-on seconds before this boot (NVS)
static uint32_t lastDeploySaveMs = 0;
static bool otaStarted = false;
static bool otaBusy = false;
static char g_statusBuf[STATUS_BUF_SIZE];

// Optional direct Bluetooth Low Energy control channel. Only one remote
// command channel is active at a time; the on-device stationary loop is
// independent of both BLE and Wi-Fi.
struct BleCommand { char json[384]; };
static QueueHandle_t bleCommandQueue = nullptr;
static BLECharacteristic* bleWriteCharacteristic = nullptr;
static BLECharacteristic* bleNotifyCharacteristic = nullptr;
static volatile bool bleCentralConnected = false;
static uint32_t bleLastCommandMs = 0;
#define BLE_TX_SLOTS 4
static volatile bool bleAuthorized = false;
static uint8_t  bleAuthFailures = 0;
static uint32_t bleLockUntilMs = 0;
static uint16_t bleMtu = 23;
static String   bleTx[BLE_TX_SLOTS];
static uint8_t  bleTxHead = 0, bleTxCount = 0;
static size_t   bleTxOff = 0;
static uint32_t bleTxLastMs = 0;

/* ============================== USER CONFIG =============================== */

#include "siboat_secrets.h"

struct WifiCred {
  const char* ssid;
  const char* pass;
};
// Networks are tried in this order (an SSID that is not visible is skipped
// after ~2.5 s). Delete the lines you do not need.
static const WifiCred WIFI_CREDS[] = {
  { SIBOAT_SSID1, SIBOAT_PASS1 },
  { SIBOAT_SSID2, SIBOAT_PASS2 },
  { SIBOAT_SSID3, SIBOAT_PASS3 },
  { "", "" }
};
static const int WIFI_CRED_COUNT = sizeof(WIFI_CREDS) / sizeof(WIFI_CREDS[0]);

static bool wifiCredConfigured(int index) {
  return index >= 0 && index < WIFI_CRED_COUNT && WIFI_CREDS[index].ssid && WIFI_CREDS[index].ssid[0] != '\0';
}

static int nextWifiCred(int start) {
  for (int step = 0; step < WIFI_CRED_COUNT; step++) {
    int idx = (start + step) % WIFI_CRED_COUNT;
    if (wifiCredConfigured(idx)) return idx;
  }
  return -1;
}

static const IPAddress MAIN_STATIC_IP(10, 90, 102, 50);
#define MAIN_HOSTNAME        "siboat"          // -> siboat.local
#define CAMERA_DEFAULT_IP    "10.90.102.51"

// ---- GPIO map (final wiring document) ----
#define PIN_L_RPWM   16
#define PIN_L_LPWM   32
#define PIN_L_EN     33
#define PIN_R_RPWM   13
#define PIN_R_LPWM   18
#define PIN_R_EN     19
#define PIN_C_RPWM   25
#define PIN_C_LPWM   26
#define PIN_C_EN     27
#define PIN_SERVO1   21      // CAT2 Soft Plastics flap
#define PIN_SERVO2   22      // CAT3 Biodegradables flap
#define PIN_TRIG     23
#define PIN_ECHO     34      // through the 1k / 2k divider
#define PIN_UART_TX  17
#define PIN_UART_RX  35
#define UART_BAUD    115200

// ---- motor direction: set to true if a motor turns the wrong way ----
#define INVERT_LEFT      false
#define INVERT_RIGHT     false
#define INVERT_CONVEYOR  false

// ---- LEDC: motors use channels 0-5 on timer 0, servos use channels 6-7 on timer 1 ----
#define CH_L_R   0
#define CH_L_L   1
#define CH_R_R   2
#define CH_R_L   3
#define CH_C_R   4
#define CH_C_L   5
#define CH_SV1   6
#define CH_SV2   7
#define MOTOR_PWM_HZ    20000
#define MOTOR_PWM_BITS  8
#define MOTOR_RAMP_STEP 8            // duty counts per 10 ms tick (0->255 in ~320 ms)

// ---- servos ----
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2400
#define SERVO_SLEW_DPS   450.0f      // degrees per second
#define SERVO_HOLD_MS    3000UL      // extended time at 180 degrees
#define SERVO_MOVE_TIMEOUT_MS 3000UL // watchdog for a move that never finishes

// ---- HC-SR04 ----
#define DEFAULT_BASELINE_CM    20.0f   // empty conveyor
#define DEFAULT_OBJECT_DELTA_CM 1.5f   // trigger = baseline - delta  (= 18.5 cm)
#define POSITION_WAIT_MS       3000UL
#define SR04_SAMPLE_MS         60UL
#define SR04_HITS_REQUIRED     2
#define SR04_INVALID_LIMIT     8       // consecutive invalid echoes -> sensor fault

// ---- classification / camera ----
#define MIN_CONF               0.50f
#define CAM_HB_TIMEOUT_MS      5000UL
#define BOOT_CAM_GRACE_MS      12000UL
// 1 = camera loss puts the Main in safe mode (final behaviour).
// Set to 0 ONLY for bench-testing motors/servos with no camera connected.
#define CAMERA_REQUIRED_FOR_OPERATION 1

// ---- dashboard supervision ----
#define DASH_TIMEOUT_MS        10000UL   // no REST call and no WebSocket client
#define SAFE_CLEAR_HOLD_MS     1500UL

// Explicit operational-readiness gate. READY is not the same thing as simply
// being connected or outside Safe Mode. The Main controller is authoritative.
#define READINESS_HOLD_MS             1500UL
#define READINESS_SR04_STALE_MS       4000UL
#define READINESS_CHECK_MS             750UL
// Passive BTS7960/MG90S hardware has no return/ACK wire in the current wiring.
// Keep this 0 so the controller can verify the command path without inventing
// physical feedback. Set to 1 only after real feedback lines are implemented.
#define REQUIRE_PHYSICAL_HARDWARE_ACK 0
#define RUNTIME_CLOUD_INTERVAL_MS      5000UL
#define CONTROLLER_EVENT_CLOUD_MS      5000UL

// ---- cloud-assisted local IP discovery ----
// The ESP32 remains the local source of truth. This registry only tells the
// hosted Web/Android dashboard which local address to try. No service_role
// or secret Supabase key is used here; only the browser-safe publishable key.
#define SIBOAT_SUPABASE_URL            "https://fgvpopxbbgtilbzwsopv.supabase.co"
#define SIBOAT_SUPABASE_PUBLISHABLE_KEY "sb_publishable_Lk_J53yt3jxAsilwyli-tg_cOxdKN8_"
#define SIBOAT_DISCOVERY_DEVICE_ID     "SIBOAT-MAIN"
#define SIBOAT_DISCOVERY_RPC_PATH      "/rest/v1/rpc/seabot_register_device"
#define SIBOAT_DISCOVERY_INTERVAL_MS   10000UL
#define SIBOAT_DISCOVERY_TIMEOUT_MS    2500UL

// ---- defaults ----
#define DEFAULT_STATIONARY     true
#define DEFAULT_CONV_SPEED     65
#define DEFAULT_JOY_SPEED      75
#define DEFAULT_JOY_SENS       80

/* ================================ TYPES =================================== */
struct Motor {
  uint8_t pinR, pinL, pinEn, chR, chL;
  int16_t cur, tgt;
  bool    invert;
};

struct EventRec {
  uint16_t schemaVersion;
  uint32_t seq, camSeq, upMs, epoch;
  char     bootId[40];
  char     cat[8];
  char     label[32];
  float    conf;
  char     servo[18];
  float    dist;
  char     ack[8];
  uint8_t  retries;
  bool     rejected;
  char     result[20];
  uint8_t  timeSrc;        // 0 = no wall clock when recorded, 1 = NTP-validated epoch
  uint32_t bootNum;        // persistent boot counter
  uint32_t deploySec;      // cumulative powered-on seconds at the event (deployment-relative time)
  uint32_t crc;            // CRC-32 of every byte above (must stay last)
};

#define WS_MAX_CLIENTS 3
#define WS_BUF         640
struct WsClient {
  WiFiClient c;
  uint8_t    st;        // 0 free, 1 handshake, 2 open
  uint16_t   len;
  uint32_t   t0;
  uint8_t    buf[WS_BUF];
};

struct Jw {
  char*  b;
  size_t n;
  size_t l;
  bool   first;
};

/* ============================== CONSTANTS ================================= */
#define R_DASH       0x01
#define R_CAM_LOST   0x02
#define R_CAM_FAULT  0x04
#define R_SR04       0x10   // latched
#define R_SERVO      0x20   // latched

#define SEG_IDLE     0
#define SEG_WAIT_POS 1
#define SEG_EXTEND   2
#define SEG_HOLD     3
#define SEG_RETRACT  4

#define EVT_MAX      96

#define W_BACKOFF    0
#define W_CONNECTING 1
#define W_UP         2

/* ================================ GLOBALS ================================= */
static WebServer   web(80);
static WiFiServer  wsServer(81);
static Preferences prefs;
static WsClient    wsc[WS_MAX_CLIENTS];

static Motor mL = { PIN_L_RPWM, PIN_L_LPWM, PIN_L_EN, CH_L_R, CH_L_L, 0, 0, INVERT_LEFT };
static Motor mR = { PIN_R_RPWM, PIN_R_LPWM, PIN_R_EN, CH_R_R, CH_R_L, 0, 0, INVERT_RIGHT };
static Motor mC = { PIN_C_RPWM, PIN_C_LPWM, PIN_C_EN, CH_C_R, CH_C_L, 0, 0, INVERT_CONVEYOR };

// operating state
static bool     powerOn = true;
static bool     estop = false;
static bool     safeMode = false;
static bool     operationalReady = false;
static bool     controllerInitialized = false;
static bool     stallState = false;
static bool     criticalFault = false;
static char     exceptionState[24] = "NORMAL";
static bool     stationaryMode = DEFAULT_STATIONARY;
static bool     stationaryAutoStartPending = DEFAULT_STATIONARY;
static bool     safetyArmed = false;
static bool     conveyorOn = false;
static bool     classificationEnabled = false;
static bool     outsideLights = false;     // virtual flags (no lights are wired)
static bool     insideLights = false;
static uint8_t  latchedFaults = 0;
static uint8_t  recoverFlags = 0;
static uint32_t safeClearSince = 0;
static uint32_t readinessHealthySince = 0;
static uint32_t lastReadinessCheckMs = 0;
static uint32_t lastSr04ValidMs = 0;
static bool     hcSr04Ready = false;
static bool     servoPathReady[2] = { false, false };
static bool     motorDriverPathReady[3] = { false, false, false };
static uint32_t controllerEventSeq = 0;
static uint32_t lastRuntimeCloudMs = 0;
static char     lastControllerEventJson[640] = "";
static char     lastControllerEventSummary[180] = "";
static char     lastError[40] = "";
static int      conveyorSpeed = DEFAULT_CONV_SPEED;
static int      joySpeed = DEFAULT_JOY_SPEED;
static int      joySens = DEFAULT_JOY_SENS;
static int16_t  joyL = 0, joyR = 0;
static char     navStr[10] = "NEUTRAL";

// dashboard supervision
static uint32_t lastApiMs = 0;
static uint32_t pendingRestartAt = 0;

// cloud discovery heartbeat
static uint32_t lastDiscoveryPublishMs = 0;
static String discoveryBootId;

// counters (persistent)
static uint32_t cat1Count = 0, cat2Count = 0, cat3Count = 0, rejectedCount = 0;
static uint32_t eventSeq = 0, timeoutCount = 0;
static bool evtDurable[EVT_MAX] = {};  // RAM-ring slots whose event also exists in LittleFS
// counters (RAM)
static uint32_t uartRetryCount = 0, uartErrorCount = 0, wifiLostCount = 0;

// last classification
static char     lastCategory[8] = "";
static char     lastLabel[32] = "";
static float    lastConfidence = 0.0f;
static uint32_t lastClassUpMs = 0;
static float    lastDistCm = -1.0f;

// HC-SR04 calibration (persistent)
static float baselineCm = DEFAULT_BASELINE_CM;
static float objectDeltaCm = DEFAULT_OBJECT_DELTA_CM;

// camera link
static bool     camSeen = false;
static bool     camHbOk = false;
static uint32_t lastHbMs = 0;
static char     camIp[20] = CAMERA_DEFAULT_IP;
static uint32_t camFps = 0, camInferMs = 0;

// UART parser + dedupe
static char     uBuf[160];
static uint8_t  uLen = 0;
static bool     uIn = false;
static uint32_t lastSeenSeq = 0;
static bool     lastSeenValid = false;
static uint8_t  seenAttempts = 0;
static uint32_t lastAccSeq = 0;
static bool     lastAccValid = false;
static char     lastAccAck[8] = "";
static uint32_t lastAccMs = 0;

// segregation state machine
static uint8_t  segState = SEG_IDLE;
static uint8_t  segCls = 0;
static uint32_t segCamSeq = 0;
static char     segLabel[32] = "";
static float    segConf = 0.0f;
static uint8_t  segRetries = 0;
static char     segAck[8] = "";
static uint32_t segDeadline = 0, segNextSample = 0, segSince = 0, segHoldUntil = 0;
static uint8_t  segHits = 0, segInvalid = 0;

// servos
static float    servoPos[2] = { 0.0f, 0.0f };
static float    servoTgt[2] = { 0.0f, 0.0f };
static uint32_t servoLastMs = 0;

// events
static EventRec evts[EVT_MAX];
static uint8_t  evtHead = 0, evtCount = 0;

// Wi-Fi state machine
static uint8_t   wState = W_BACKOFF;
static int       wCred = 0;
static bool      wUseStatic = false;
static bool      staticFailed = false;
static bool      haveLearned = false;
static uint32_t  wStart = 0, wNext = 0;
static IPAddress learnedGw, learnedMask;
static bool      serversStarted = false;

// periodic jobs
static uint32_t lastTelemetryMs = 0, lastStMs = 0, lastMotorMs = 0;
static uint32_t prefsDirtyAt = 0;
static bool     telemetryDirty = false;
static bool     stDirty = true;
static uint8_t  lastStSentCls = 255, lastStSentSafe = 255, lastStSentReady = 255, lastStSentStationary = 255;
static bool     wsHadClients = false;

/* ========================== FORWARD DECLARATIONS ========================== */
static void  updateTargets();
static void joystickApply(float fx, float fy, float fsens);
static void  stopAllOutputs();
static void  hardStopPropulsion();
static void  segAbort(const char* why);
static void  enterSafe();
static void  exitSafe();
static void  markDirty();
static void  savePrefsSoon();
static void  recordEvent(const char* cat, const char* label, float conf, uint32_t camSeq,
                         const char* servo, float dist, const char* ack, uint8_t retries,
                         bool rejected, const char* result);
static void queuePerformanceEventService(uint32_t now);
static bool persistOutboxEvent(const EventRec& e);
static bool readOldestOutboxEvent(EventRec& e);
static bool removeOutboxEvent(uint32_t sequence);
static void initializeOutbox();
static void retryOutboxWriteService(uint32_t now);
static uint32_t countOutboxRecords();
static void setupBleControl();
static void bleCommandService(uint32_t now);
static bool bleLinkAlive(uint32_t now);
static bool remoteLinkAlive(uint32_t now);
static void bleTxService(uint32_t now);
static uint32_t deploySecNow();
static void deployService(uint32_t now);
static void otaSetup();
static uint32_t outboxUsedBytesCached(bool force);
static bool blePinConfigured();
static const char* syncStateStr(uint32_t now);
static const char* controlChannelStr(uint32_t now);
static const char* safeReasonStr();
static const char* servoStateStr(int i);
static void sendJson(int code, const char* body);
static bool httpControlBlockedByBle();
static void stationaryAutoStartService(uint32_t now);
static void collectionSessionCloudService(uint32_t now);
static void  wsBroadcast(const char* msg);
static int   wsClientCount();
static void  buildStatus(char* out, size_t n);
static void  doEstop(bool active);
static void  setStationary(bool on);
static bool  propAllowed();
static bool  conveyorAllowed();
static bool  operationAllowed();
static bool  dashActive(uint32_t now);
static void  readinessService(uint32_t now);
static void  buildReadinessChecks(char* out, size_t n, uint32_t now, bool* allReadyOut);
static void  buildHardwareStatus(char* out, size_t n, uint32_t now);
static void  recordControllerEvent(const char* direction, const char* eventType, const char* component, const char* command, const char* result, const char* detail);
static void  runtimeCloudService(uint32_t now);
static bool enqueueCloudRpc(uint8_t kind, const char* rpcPath, const String& payload, uint32_t sequence = 0);
static void cloudRpcWorker(void* parameter);
static const char* stateStr();
static String discoveryJsonEscape(const String& value);
static void ensureDiscoveryBootId();
static void postRuntimeStateNow();

/* ============================ SMALL UTILITIES ============================= */
static uint32_t epochNow() {
  time_t t = time(nullptr);
  return (t > 1700000000) ? (uint32_t)t : 0;
}

static void sanitize(char* dst, const char* src, size_t n) {
  size_t i = 0;
  if (n == 0) return;
  while (src && src[i] && i < n - 1) {
    char ch = src[i];
    bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
              (ch >= '0' && ch <= '9') || ch == ' ' || ch == '-' || ch == '_' || ch == '.';
    dst[i] = ok ? ch : '_';
    i++;
  }
  dst[i] = '\0';
}

// Plain decimal parser (does not depend on strtoul, whose signature differs between toolchains)
static uint32_t parseU32(const char* s) {
  uint32_t v = 0;
  if (!s) return 0;
  while (*s == ' ') s++;
  while (*s >= '0' && *s <= '9') {
    v = v * 10u + (uint32_t)(*s - '0');
    s++;
  }
  return v;
}

static void setErr(const char* s) {
  strncpy(lastError, s, sizeof(lastError) - 1);
  lastError[sizeof(lastError) - 1] = '\0';
}

static int wifiPercent() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  int r = WiFi.RSSI();
  int p = 2 * (r + 100);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

/* -------- tiny JSON writer -------- */
static void jwRaw(Jw& j, const char* s) {
  size_t k = strlen(s);
  if (j.l + k < j.n - 1) {
    memcpy(j.b + j.l, s, k);
    j.l += k;
    j.b[j.l] = '\0';
  }
}
static void jwInit(Jw& j, char* b, size_t n) {
  j.b = b; j.n = n; j.l = 0; j.first = true; j.b[0] = '\0';
  jwRaw(j, "{");
}
static void jwKey(Jw& j, const char* k) {
  if (!j.first) jwRaw(j, ",");
  j.first = false;
  jwRaw(j, "\""); jwRaw(j, k); jwRaw(j, "\":");
}
static void jwStr(Jw& j, const char* k, const char* v) {
  jwKey(j, k); jwRaw(j, "\""); jwRaw(j, v); jwRaw(j, "\"");
}
static void jwInt(Jw& j, const char* k, long v) {
  char t[24]; snprintf(t, sizeof(t), "%ld", v);
  jwKey(j, k); jwRaw(j, t);
}
static void jwUInt(Jw& j, const char* k, unsigned long v) {
  char t[24]; snprintf(t, sizeof(t), "%lu", v);
  jwKey(j, k); jwRaw(j, t);
}
static void jwFlt(Jw& j, const char* k, float v, int dec) {
  char t[24]; snprintf(t, sizeof(t), "%.*f", dec, (double)v);
  jwKey(j, k); jwRaw(j, t);
}
static void jwBool(Jw& j, const char* k, bool v) {
  jwKey(j, k); jwRaw(j, v ? "true" : "false");
}
static void jwNull(Jw& j, const char* k) {
  jwKey(j, k); jwRaw(j, "null");
}
static void jwEnd(Jw& j) { jwRaw(j, "}"); }

/* -------- tiny JSON reader (flat objects) -------- */
static const char* jsonFind(const char* body, const char* key) {
  if (!body) return nullptr;
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(body, pat);
  if (!p) return nullptr;
  p += strlen(pat);
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p != ':') return nullptr;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  return p;
}
static bool jsonNum(const char* body, const char* key, float* out) {
  const char* p = jsonFind(body, key);
  if (!p) return false;
  if (*p == '"') p++;
  if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' || *p == '.')) return false;
  *out = (float)atof(p);
  return true;
}
static bool jsonStr(const char* body, const char* key, char* out, size_t n) {
  const char* p = jsonFind(body, key);
  if (!p || *p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
  out[i] = '\0';
  return true;
}
// returns -1 when not present, else 0/1. Accepts true/false, "ON"/"OFF", 1/0
static int jsonBoolish(const char* body, const char* key) {
  const char* p = jsonFind(body, key);
  if (!p) return -1;
  if (*p == '"') p++;
  if (strncasecmp(p, "true", 4) == 0 || strncasecmp(p, "on", 2) == 0 || *p == '1') return 1;
  if (strncasecmp(p, "false", 5) == 0 || strncasecmp(p, "off", 3) == 0 || *p == '0') return 0;
  return -1;
}


/* ============================ BLUETOOTH LE CONTROL ========================= */
// Optional direct control channel. Protocol (documented in docs/BLUETOOTH.md):
//   client -> device : write characteristic, UTF-8 JSON  {"id":N,"path":"/api/...","body":{...}}
//   device -> client : notify characteristic, UTF-8 JSON  {"id":N,"ok":true,"body":{...}}\n
//   Replies are split into MTU-sized notification chunks and END WITH A NEWLINE; the client
//   joins chunks until it sees '\n'.
// Security: the link carries no commands until the client sends {"path":"/api/auth","pin":"..."}
// with the PIN compiled in from siboat_secrets.h (SIBOAT_BLE_PIN). Without a configured PIN the
// BLE service is not started at all. Only status and "emergency stop ON" work before auth.
#ifndef SIBOAT_BLE_PIN
#define SIBOAT_BLE_PIN ""
#endif
static const char* const BLE_PIN = SIBOAT_BLE_PIN;
static bool blePinConfigured() {
  const size_t n = strlen(BLE_PIN);
  if (n < 4 || n > 8) return false;
  for (size_t i = 0; i < n; i++) if (BLE_PIN[i] < '0' || BLE_PIN[i] > '9') return false;
  return true;
}
static bool blePinMatches(const char* given) {
  const size_t n = strlen(BLE_PIN), g = strlen(given);
  uint8_t diff = (uint8_t)(n ^ g);
  for (size_t i = 0; i < n; i++) diff |= (uint8_t)(BLE_PIN[i] ^ (i < g ? given[i] : 0));
  return diff == 0;
}

static bool bleLinkAlive(uint32_t now) {
  return bleCentralConnected && bleAuthorized && bleLastCommandMs != 0 &&
         (uint32_t)(now - bleLastCommandMs) < DASH_TIMEOUT_MS;
}
// A remote operator link exists when an authorised BLE client is active OR the Wi-Fi dashboard is live.
static bool remoteLinkAlive(uint32_t now) {
  return bleLinkAlive(now) || (WiFi.status() == WL_CONNECTED && dashActive(now));
}

class SiboatBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;
    bleCentralConnected = true;
    bleAuthorized = false;          // every new connection must authenticate again
    bleLastCommandMs = 0;
    Serial.println("[BLE] client connected; waiting for PIN authentication");
  }
  void onDisconnect(BLEServer* server) override {
    (void)server;
    bleCentralConnected = false;
    bleAuthorized = false;
    bleMtu = 23;
    Serial.println("[BLE] client disconnected; advertising restarted");
    BLEDevice::startAdvertising();
  }
  // Not marked 'override' on purpose: if a core changes this signature the service still works
  // with the safe default chunk size (20 bytes) instead of failing to compile.
  void onMtuChanged(BLEServer* server, esp_ble_gatts_cb_param_t* param) {
    (void)server;
    if (param && param->mtu.mtu >= 23 && param->mtu.mtu <= 517) bleMtu = param->mtu.mtu;
  }
};

class SiboatBleCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    if (!bleCommandQueue) return;
    auto value = characteristic->getValue();      // String on Arduino-ESP32 3.x, std::string on 2.x
    const size_t n = value.length();
    if (n == 0 || n >= sizeof(BleCommand::json)) return;
    BleCommand command;
    memset(&command, 0, sizeof(command));
    memcpy(command.json, value.c_str(), n);
    xQueueSend(bleCommandQueue, &command, 0);
  }
};

static void setupBleControl() {
  if (!blePinConfigured()) {
    Serial.println("[BLE] disabled: SIBOAT_BLE_PIN (4-8 digits) is not set in siboat_secrets.h");
    return;
  }
  bleCommandQueue = xQueueCreate(4, sizeof(BleCommand));
  if (!bleCommandQueue) {
    Serial.println("[BLE] command queue allocation failed; Bluetooth control is unavailable");
    return;
  }
  BLEDevice::init("SIBOAT MAIN ESP32");
  BLEDevice::setMTU(185);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new SiboatBleServerCallbacks());
  BLEService* service = server->createService(SIBOAT_BLE_SERVICE_UUID);
  bleWriteCharacteristic = service->createCharacteristic(
    SIBOAT_BLE_WRITE_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  bleWriteCharacteristic->setCallbacks(new SiboatBleCommandCallbacks());
  bleNotifyCharacteristic = service->createCharacteristic(
    SIBOAT_BLE_NOTIFY_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  bleNotifyCharacteristic->addDescriptor(new BLE2902());
  bleNotifyCharacteristic->setValue("{\"ok\":true,\"state\":\"BLE_READY\"}");
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SIBOAT_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.println("[BLE] SIBOAT control service advertising (PIN required); camera video remains Wi-Fi only");
}

// Reply queue: whole messages, sent as newline-terminated chunks by bleTxService() without blocking.
static void bleEnqueueResponse(const String& message) {
  if (!bleNotifyCharacteristic || !bleCentralConnected) return;
  if (bleTxCount >= BLE_TX_SLOTS) return;          // link saturated: client times out and retries
  const uint8_t idx = (uint8_t)((bleTxHead + bleTxCount) % BLE_TX_SLOTS);
  bleTx[idx] = message + "\n";
  bleTxCount++;
}

static void bleTxService(uint32_t now) {
  if (!bleNotifyCharacteristic || bleTxCount == 0) return;
  if (!bleCentralConnected) { bleTxCount = 0; bleTxOff = 0; return; }
  if ((uint32_t)(now - bleTxLastMs) < 15UL) return;  // pace notifications so the stack never overruns
  String& cur = bleTx[bleTxHead];
  size_t chunk = (bleMtu > 23) ? (size_t)bleMtu - 3U : 20U;
  if (chunk > 180U) chunk = 180U;
  const size_t remain = cur.length() - bleTxOff;
  if (chunk > remain) chunk = remain;
  bleNotifyCharacteristic->setValue((uint8_t*)(cur.c_str() + bleTxOff), chunk);
  bleNotifyCharacteristic->notify();
  bleTxOff += chunk;
  bleTxLastMs = now;
  if (bleTxOff >= cur.length()) {
    cur = String();
    bleTxOff = 0;
    bleTxHead = (uint8_t)((bleTxHead + 1) % BLE_TX_SLOTS);
    bleTxCount--;
  }
}

static void sendBleResponse(uint32_t id, bool ok, const String& bodyJson, const char* errorText = nullptr) {
  String response = "{\"id\":" + String(id) + ",\"ok\":" + (ok ? "true" : "false");
  if (ok) response += ",\"body\":" + bodyJson;
  else response += ",\"error\":\"" + String(errorText ? errorText : "COMMAND_REJECTED") + "\"";
  response += "}";
  bleEnqueueResponse(response);
}

static const char* controlChannelStr(uint32_t now) {
  if (stationaryMode && !remoteLinkAlive(now)) return "stationary-local";
  if (bleLinkAlive(now)) return "bluetooth";
  if (WiFi.status() == WL_CONNECTED && dashActive(now)) return "wifi";
  return "none";
}

static const char* syncStateStr(uint32_t now) {
  (void)now;
  if (!outboxMounted || outboxStorageFault) return "STORAGE_FAULT";
  if (outboxCount == 0) return lastPerformanceEventCloudSeq ? "COMPLETE" : "IDLE";
  if (WiFi.status() != WL_CONNECTED) return "PENDING_OFFLINE";
  if (!backendReachable) return "BACKEND_UNREACHABLE";
  if (lastPerformanceEventQueuedSeq != lastPerformanceEventCloudSeq || perfAckReady) return "SYNCING";
  return "PENDING";
}

static void buildBleStatus(char* out, size_t n, bool authorized) {
  const uint32_t now = millis();
  Jw j;
  jwInit(j, out, n);
  jwStr(j, "state", stateStr());
  jwStr(j, "firmware", FW_VERSION);
  jwBool(j, "bleAuthorized", authorized);
  if (!authorized) { jwEnd(j); return; }
  const bool camOnline = camSeen && camHbOk && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  jwStr(j, "nav", navStr);
  jwBool(j, "conveyor", conveyorOn);
  jwBool(j, "classification", classificationEnabled);
  jwBool(j, "classificationEnabled", classificationEnabled);
  jwBool(j, "outsideLights", outsideLights);
  jwBool(j, "insideLights", insideLights);
  jwBool(j, "stationaryMode", stationaryMode);
  jwBool(j, "safeMode", safeMode);
  jwStr(j, "safeReason", safeMode ? safeReasonStr() : "");
  jwBool(j, "emergency", estop);
  jwBool(j, "power", powerOn);
  jwBool(j, "operationalReady", operationalReady);
  jwStr(j, "operationStatus", operationalReady ? "READY" : "NOT READY");
  jwBool(j, "runtimeSafetyArmed", safetyArmed);
  jwBool(j, "stall", stallState);
  jwBool(j, "criticalFault", criticalFault);
  jwStr(j, "exceptionState", exceptionState);
  jwBool(j, "cameraConnected", camOnline);
  jwStr(j, "lastCategory", lastCategory);
  jwFlt(j, "lastConfidence", lastConfidence, 2);
  jwStr(j, "lastLabel", lastLabel);
  jwStr(j, "servo1State", servoStateStr(0));
  jwStr(j, "servo2State", servoStateStr(1));
  if (lastDistCm >= 0.0f) jwFlt(j, "hcSr04Distance", lastDistCm, 1); else jwNull(j, "hcSr04Distance");
  jwUInt(j, "cat1Count", cat1Count);
  jwUInt(j, "cat2Count", cat2Count);
  jwUInt(j, "cat3Count", cat3Count);
  jwUInt(j, "rejectedCount", rejectedCount);
  jwUInt(j, "collectedCount", (unsigned long)cat1Count + cat2Count + cat3Count + rejectedCount);
  jwUInt(j, "segregatedCount", (unsigned long)cat1Count + cat2Count + cat3Count);
  jwStr(j, "lastError", lastError);
  jwUInt(j, "uptime", now / 1000UL);
  jwStr(j, "wifiStatus", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  jwBool(j, "backendReachable", backendReachable);
  jwStr(j, "syncState", syncStateStr(now));
  jwUInt(j, "outboxPendingCount", countOutboxRecords());
  jwBool(j, "outboxStorageFault", outboxStorageFault);
  jwStr(j, "controlChannel", controlChannelStr(now));
  jwEnd(j);
}

static void bleCommandService(uint32_t now) {
  if (!bleCommandQueue || !bleNotifyCharacteristic) return;
  BleCommand command;
  if (xQueueReceive(bleCommandQueue, &command, 0) != pdTRUE) return;
  uint32_t id = 0; float tmp = 0.0f;
  if (jsonNum(command.json, "id", &tmp) && tmp >= 0.0f) id = (uint32_t)tmp;
  char path[56] = {0};
  if (!jsonStr(command.json, "path", path, sizeof(path))) { sendBleResponse(id, false, "{}", "PATH_REQUIRED"); return; }

  // ---- authentication ----
  if (strcmp(path, "/api/auth") == 0) {
    if ((int32_t)(bleLockUntilMs - now) > 0) { sendBleResponse(id, false, "{}", "BLE_LOCKED"); return; }
    char pin[16] = {0};
    if (!jsonStr(command.json, "pin", pin, sizeof(pin))) { sendBleResponse(id, false, "{}", "PIN_REQUIRED"); return; }
    if (blePinMatches(pin)) {
      bleAuthorized = true; bleAuthFailures = 0; bleLastCommandMs = now;
      recordControllerEvent("BLUETOOTH", "AUTH", "BLE Control", "PIN", "ACCEPTED", "Bluetooth control client authorised");
      sendBleResponse(id, true, "{\"authorized\":true}");
    } else {
      if (bleAuthFailures < 250) bleAuthFailures++;
      if (bleAuthFailures >= 5) {
        const uint8_t shift = (uint8_t)((bleAuthFailures - 5) > 4 ? 4 : (bleAuthFailures - 5));
        bleLockUntilMs = now + (60000UL << shift);
      }
      recordControllerEvent("BLUETOOTH", "AUTH", "BLE Control", "PIN", "REJECTED", "Wrong Bluetooth PIN");
      sendBleResponse(id, false, "{}", "BAD_PIN");
    }
    return;
  }

  // ---- before authentication only status and emergency-stop-ON are allowed ----
  if (!bleAuthorized) {
    if (strcmp(path, "/api/status") == 0) {
      static char pub[160];
      buildBleStatus(pub, sizeof(pub), false);
      sendBleResponse(id, true, String(pub));
    } else if (strcmp(path, "/api/estop") == 0 && jsonBoolish(command.json, "active") != 0) {
      doEstop(true);
      sendBleResponse(id, true, "{\"ok\":true}");
    } else {
      sendBleResponse(id, false, "{}", "BLE_AUTH_REQUIRED");
    }
    return;
  }
  bleLastCommandMs = now;

  bool ok = true; const char* err = nullptr;
  String body = "{\"ok\":true}";
  float x = 0.0f, y = 0.0f, value = 0.0f, sensitivity = (float)joySens;
  int stateValue = -1;
  if (strcmp(path, "/api/status") == 0 || strcmp(path, "/api/readiness") == 0) {
    static char st[1400];
    buildBleStatus(st, sizeof(st), true);
    body = String(st);
  } else if (strcmp(path, "/api/joystick") == 0) {
    jsonNum(command.json, "x", &x); jsonNum(command.json, "y", &y); jsonNum(command.json, "sensitivity", &sensitivity);
    joystickApply(x, y, sensitivity);
    ok = propAllowed(); if (!ok) err = stationaryMode ? "STATIONARY_MODE" : "PROPULSION_BLOCKED";
    else if (x != 0.0f || y != 0.0f) safetyArmed = true;
  } else if (strcmp(path, "/api/joystick/speed") == 0) {
    if (!jsonNum(command.json, "value", &value)) { ok = false; err = "VALUE_REQUIRED"; }
    else { joySpeed = (int)constrain((int)value, 0, 100); savePrefsSoon(); }
  } else if (strcmp(path, "/api/joystick/sensitivity") == 0) {
    if (!jsonNum(command.json, "value", &value)) { ok = false; err = "VALUE_REQUIRED"; }
    else { joySens = (int)constrain((int)value, 0, 100); savePrefsSoon(); }
  } else if (strcmp(path, "/api/conveyor/speed") == 0) {
    if (!jsonNum(command.json, "value", &value)) { ok = false; err = "VALUE_REQUIRED"; }
    else { conveyorSpeed = (int)constrain((int)value, 0, 100); updateTargets(); savePrefsSoon(); }
  } else if (strcmp(path, "/api/conveyor") == 0) {
    stateValue = jsonBoolish(command.json, "state");
    if (stateValue < 0) { ok = false; err = "STATE_REQUIRED"; }
    else if (stateValue && !conveyorAllowed()) { ok = false; err = "NOT_OPERATIONALLY_READY"; }
    else { conveyorOn = stateValue == 1; if (!conveyorOn) stationaryAutoStartPending = false; else safetyArmed = true; updateTargets(); markDirty(); }
  } else if (strcmp(path, "/api/classification") == 0) {
    stateValue = jsonBoolish(command.json, "state");
    if (stateValue < 0) { ok = false; err = "STATE_REQUIRED"; }
    else if (stateValue && !operationAllowed()) { ok = false; err = "NOT_OPERATIONALLY_READY"; }
    else { classificationEnabled = stateValue == 1; if (!classificationEnabled) stationaryAutoStartPending = false; else safetyArmed = true; markDirty(); }
  } else if (strcmp(path, "/api/lights/outside") == 0 || strcmp(path, "/api/lights/inside") == 0) {
    stateValue = jsonBoolish(command.json, "state");
    if (stateValue >= 0) { if (path[12] == 'o') outsideLights = stateValue == 1; else insideLights = stateValue == 1; markDirty(); }
  } else if (strcmp(path, "/api/estop") == 0) {
    stateValue = jsonBoolish(command.json, "active"); doEstop(stateValue != 0);
  } else if (strcmp(path, "/api/stationary") == 0) {
    stateValue = jsonBoolish(command.json, "state"); if (stateValue < 0) stateValue = jsonBoolish(command.json, "enabled");
    if (stateValue < 0) { ok = false; err = "STATE_REQUIRED"; } else setStationary(stateValue == 1);
  } else if (strcmp(path, "/api/power") == 0) {
    stateValue = jsonBoolish(command.json, "state");
    if (stateValue == 0) { stopAllOutputs(); powerOn = false; operationalReady = false; readinessHealthySince = 0; safetyArmed = false; }
    else { powerOn = true; operationalReady = false; readinessHealthySince = 0; }
    markDirty();
  } else if (strcmp(path, "/api/shutdown") == 0) {
    stopAllOutputs(); powerOn = false; operationalReady = false; readinessHealthySince = 0; safetyArmed = false; markDirty();
  } else if (strcmp(path, "/api/restart") == 0) {
    stopAllOutputs(); pendingRestartAt = now + 800UL; if (pendingRestartAt == 0) pendingRestartAt = 1;
  } else if (strcmp(path, "/api/devices") == 0) {
    const bool camOnline = camSeen && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS);
    body = String("{\"ok\":true,\"count\":") + String(1 + (camOnline ? 1 : 0)) + String(",\"camera\":") + (camOnline ? "true}" : "false}");
  } else if (strcmp(path, "/api/priority/request") == 0 || strcmp(path, "/api/priority/release") == 0) {
    body = "{\"ok\":true}";
  } else if (strcmp(path, "/api/developer/verify") == 0) {
    body = "{\"ok\":true,\"verified\":false}";
  } else {
    ok = false; err = "UNSUPPORTED_BLE_COMMAND";
  }
  sendBleResponse(id, ok, body, err);
}

static bool httpControlBlockedByBle() {
  if (!bleLinkAlive(millis())) return false;        // only an authorised, active BLE client owns the control channel
  sendJson(409, "{\"ok\":false,\"error\":\"BLE_CONTROL_CHANNEL_ACTIVE\"}");
  return true;
}

/* ============================ PWM (ESP-IDF LEDC) ========================== */
static void pwmTimerInit(ledc_timer_t timer, uint32_t freqHz, uint8_t bits) {
  ledc_timer_config_t t;
  memset(&t, 0, sizeof(t));
  t.speed_mode = LEDC_HIGH_SPEED_MODE;
  t.duty_resolution = (ledc_timer_bit_t)bits;
  t.timer_num = timer;
  t.freq_hz = freqHz;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
}

static void pwmChannelInit(uint8_t channel, uint8_t pin, ledc_timer_t timer) {
  ledc_channel_config_t c;
  memset(&c, 0, sizeof(c));
  c.gpio_num = pin;
  c.speed_mode = LEDC_HIGH_SPEED_MODE;
  c.channel = (ledc_channel_t)channel;
  c.intr_type = LEDC_INTR_DISABLE;
  c.timer_sel = timer;
  c.duty = 0;
  c.hpoint = 0;
  ledc_channel_config(&c);
}

static void pwmWrite(uint8_t channel, uint32_t duty) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel);
}

/* ============================== MOTOR DRIVERS ============================= */
static void motorWrite(Motor& m, int v) {
  if (m.invert) v = -v;
  if (v > 255) v = 255;
  if (v < -255) v = -255;
  if (v > 0) {
    pwmWrite(m.chR, (uint32_t)v);
    pwmWrite(m.chL, 0);
    digitalWrite(m.pinEn, HIGH);
  } else if (v < 0) {
    pwmWrite(m.chR, 0);
    pwmWrite(m.chL, (uint32_t)(-v));
    digitalWrite(m.pinEn, HIGH);
  } else {
    pwmWrite(m.chR, 0);
    pwmWrite(m.chL, 0);
    digitalWrite(m.pinEn, LOW);
  }
}

static void motorHardStop(Motor& m) {
  m.cur = 0;
  m.tgt = 0;
  motorWrite(m, 0);
}

static void motorRamp(Motor& m) {
  if (m.cur < m.tgt) {
    m.cur += MOTOR_RAMP_STEP;
    if (m.cur > m.tgt) m.cur = m.tgt;
  } else if (m.cur > m.tgt) {
    m.cur -= MOTOR_RAMP_STEP;
    if (m.cur < m.tgt) m.cur = m.tgt;
  }
  motorWrite(m, m.cur);
}

static bool operationAllowed() { return powerOn && !estop && !safeMode && operationalReady && !outboxStorageFault; }
static bool propAllowed()      { return operationAllowed() && !stationaryMode; }
static bool conveyorAllowed()  { return operationAllowed(); }

static void hardStopPropulsion() {
  joyL = 0; joyR = 0;
  strcpy(navStr, "NEUTRAL");
  motorHardStop(mL);
  motorHardStop(mR);
}

static void updateTargets() {
  if (!propAllowed()) {
    joyL = 0; joyR = 0;
    strcpy(navStr, "NEUTRAL");
    if (mL.cur != 0 || mR.cur != 0 || mL.tgt != 0 || mR.tgt != 0) hardStopPropulsion();
  } else {
    mL.tgt = joyL;
    mR.tgt = joyR;
  }
  if (conveyorOn && conveyorAllowed()) {
    mC.tgt = (int16_t)((conveyorSpeed * 255) / 100);
  } else {
    mC.tgt = 0;
    if (!conveyorAllowed() && (mC.cur != 0)) motorHardStop(mC);
  }
}

static void motorService(uint32_t now) {
  if ((uint32_t)(now - lastMotorMs) < 10) return;
  lastMotorMs = now;
  updateTargets();
  if (!propAllowed()) {           // never drive propulsion when not allowed
    if (mL.cur != 0 || mR.cur != 0) hardStopPropulsion();
  } else {
    motorRamp(mL);
    motorRamp(mR);
  }
  motorRamp(mC);
}

static void joystickApply(float fx, float fy, float fsens) {
  int x = (int)fx, y = (int)fy, sens = (int)fsens;
  if (x > 100) x = 100;
  if (x < -100) x = -100;
  if (y > 100) y = 100;
  if (y < -100) y = -100;
  if (sens < 10) sens = 10;
  if (sens > 100) sens = 100;
  if (abs(x) < 6) x = 0;
  if (abs(y) < 6) y = 0;

  if (!propAllowed()) {
    joyL = 0; joyR = 0;
    strcpy(navStr, "NEUTRAL");
    return;
  }
  float fwd = y / 100.0f;
  float turn = (x / 100.0f) * (sens / 100.0f);
  float l = fwd + turn;
  float r = fwd - turn;
  float mx = fabsf(l) > fabsf(r) ? fabsf(l) : fabsf(r);
  if (mx > 1.0f) { l /= mx; r /= mx; }
  float sp = joySpeed / 100.0f;
  joyL = (int16_t)(l * sp * 255.0f);
  joyR = (int16_t)(r * sp * 255.0f);

  if (x == 0 && y == 0) strcpy(navStr, "NEUTRAL");
  else if (abs(y) >= abs(x)) strcpy(navStr, y > 0 ? "FORWARD" : "BACKWARD");
  else strcpy(navStr, x > 0 ? "RIGHT" : "LEFT");
}

/* ================================ SERVOS ================================== */
static void servoOut(int i, float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
  uint32_t duty = (uint32_t)(us * 16383.0f / 20000.0f + 0.5f);
  pwmWrite(i == 0 ? CH_SV1 : CH_SV2, duty);
}

static void servoHardZero() {
  for (int i = 0; i < 2; i++) {
    servoPos[i] = 0.0f;
    servoTgt[i] = 0.0f;
    servoOut(i, 0.0f);
  }
}

static void servoService(uint32_t now) {
  if ((uint32_t)(now - servoLastMs) < 10) return;
  float dt = (now - servoLastMs) / 1000.0f;
  servoLastMs = now;
  float step = SERVO_SLEW_DPS * dt;
  for (int i = 0; i < 2; i++) {
    if (servoPos[i] < servoTgt[i]) {
      servoPos[i] += step;
      if (servoPos[i] > servoTgt[i]) servoPos[i] = servoTgt[i];
    } else if (servoPos[i] > servoTgt[i]) {
      servoPos[i] -= step;
      if (servoPos[i] < servoTgt[i]) servoPos[i] = servoTgt[i];
    }
    servoOut(i, servoPos[i]);
  }
}

static const char* servoStateStr(int i) {
  if (servoTgt[i] >= 179.5f && servoPos[i] >= 179.5f) return "EXTENDED";
  if (servoTgt[i] <= 0.5f && servoPos[i] <= 0.5f) return "RETRACTED";
  return "MOVING";
}

/* ================================ HC-SR04 ================================= */
static float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (d == 0) return -1.0f;
  float cm = (float)d * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f;
  return cm;
}

static float triggerCm() { return baselineCm - objectDeltaCm; }

/* ============================ PERSISTENT STORAGE ========================== */
static void loadPrefs() {
  prefs.begin("siboat", false);
  cat1Count = prefs.getUInt("c1", 0);
  cat2Count = prefs.getUInt("c2", 0);
  cat3Count = prefs.getUInt("c3", 0);
  rejectedCount = prefs.getUInt("rj", 0);
  eventSeq = prefs.getUInt("sq", 0);
  timeoutCount = prefs.getUInt("tm", 0);
  stationaryMode = prefs.getBool("stat", DEFAULT_STATIONARY);
  baselineCm = prefs.getFloat("base", DEFAULT_BASELINE_CM);
  objectDeltaCm = prefs.getFloat("delta", DEFAULT_OBJECT_DELTA_CM);
  conveyorSpeed = prefs.getInt("cspd", DEFAULT_CONV_SPEED);
  joySpeed = prefs.getInt("jspd", DEFAULT_JOY_SPEED);
  joySens = prefs.getInt("jsen", DEFAULT_JOY_SENS);
  if (baselineCm < 5.0f || baselineCm > 300.0f) baselineCm = DEFAULT_BASELINE_CM;
  if (objectDeltaCm < 0.3f || objectDeltaCm > 15.0f) objectDeltaCm = DEFAULT_OBJECT_DELTA_CM;
  if (conveyorSpeed < 0 || conveyorSpeed > 100) conveyorSpeed = DEFAULT_CONV_SPEED;
  if (joySpeed < 0 || joySpeed > 100) joySpeed = DEFAULT_JOY_SPEED;
  if (joySens < 0 || joySens > 100) joySens = DEFAULT_JOY_SENS;
}

static void saveCounters() {
  prefs.putUInt("c1", cat1Count);
  prefs.putUInt("c2", cat2Count);
  prefs.putUInt("c3", cat3Count);
  prefs.putUInt("rj", rejectedCount);
  prefs.putUInt("sq", eventSeq);
  prefs.putUInt("tm", timeoutCount);
}

static void saveSettings() {
  prefs.putBool("stat", stationaryMode);
  prefs.putFloat("base", baselineCm);
  prefs.putFloat("delta", objectDeltaCm);
  prefs.putInt("cspd", conveyorSpeed);
  prefs.putInt("jspd", joySpeed);
  prefs.putInt("jsen", joySens);
}

static void savePrefsSoon() { prefsDirtyAt = millis() + 1500UL; if (prefsDirtyAt == 0) prefsDirtyAt = 1; }

static void prefsService(uint32_t now) {
  if (prefsDirtyAt != 0 && (int32_t)(now - prefsDirtyAt) >= 0) {
    prefsDirtyAt = 0;
    saveSettings();
  }
}

static void markDirty() { telemetryDirty = true; stDirty = true; }

/* =============================== EVENT LOG ================================ */
static void bumpEventRetries(uint32_t camSeq, uint8_t retries) {
  for (uint8_t k = 0; k < evtCount; k++) {
    int idx = (int)evtHead - 1 - (int)k;
    while (idx < 0) idx += EVT_MAX;
    if (evts[idx].camSeq == camSeq) { evts[idx].retries = retries; return; }
  }
}

static void eventToJson(const EventRec& e, char* out, size_t n, bool withType) {
  Jw j;
  jwInit(j, out, n);
  if (withType) jwStr(j, "type", "event");
  jwStr(j, "deviceId", SIBOAT_DISCOVERY_DEVICE_ID);
  jwUInt(j, "eventSequence", e.seq);
  jwStr(j, "bootId", e.bootId);
  jwUInt(j, "camSeq", e.camSeq);
  jwStr(j, "category", e.cat);
  jwStr(j, "label", e.label);
  jwFlt(j, "confidence", e.conf, 2);
  jwUInt(j, "uptimeMs", e.upMs);
  jwUInt(j, "timestamp", e.epoch);
  jwStr(j, "timeSource", e.timeSrc ? "ntp" : "none");
  jwUInt(j, "bootNumber", e.bootNum);
  jwUInt(j, "deploySeconds", e.deploySec);
  jwStr(j, "servoAction", e.servo);
  if (e.dist >= 0.0f) jwFlt(j, "hcSr04Distance", e.dist, 1); else jwNull(j, "hcSr04Distance");
  jwStr(j, "ackStatus", e.ack);
  jwUInt(j, "uartRetries", e.retries);
  jwBool(j, "rejected", e.rejected);
  jwStr(j, "result", e.result);
  jwEnd(j);
}

/* ===================== PERSISTENT EVENT OUTBOX (LittleFS) ================== */
// One small CRC-protected file per outcome event: /outbox/e<seq>.evt, written as .tmp then renamed.
//  * first boot after flashing: the empty partition is formatted once, then mounted
//  * unreadable / torn / wrong-version records are renamed to *.bad (kept for inspection) and
//    NEVER stop the machine; only a failed write of a NEW event triggers the documented hold
//  * the pending count and oldest sequence are cached in RAM (no directory scan per poll)
//  * only the main loop task touches the filesystem; the cloud worker just posts an acknowledgement
static uint32_t crc32Bytes(const uint8_t* d, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}
static uint32_t eventCrc(const EventRec& e) {
  return crc32Bytes(reinterpret_cast<const uint8_t*>(&e), offsetof(EventRec, crc));
}

static String outboxPath(uint32_t sequence, const char* suffix) {
  char name[48];
  snprintf(name, sizeof(name), "/outbox/e%010lu.%s", (unsigned long)sequence, suffix ? suffix : "evt");
  return String(name);
}

static bool outboxSequenceFromName(const String& fullName, uint32_t* sequenceOut, bool allowTemp) {
  int slash = fullName.lastIndexOf('/');
  String base = (slash >= 0) ? fullName.substring(slash + 1) : fullName;
  if (!base.startsWith("e")) return false;
  int dot = base.indexOf('.');
  if (dot < 2) return false;
  String ext = base.substring(dot + 1);
  if (ext != "evt" && !(allowTemp && ext == "tmp")) return false;   // "evt.bad"/"tmp.bad" are ignored
  String digits = base.substring(1, dot);
  for (size_t i = 0; i < digits.length(); ++i) if (!isDigit(digits[i])) return false;
  if (sequenceOut) *sequenceOut = (uint32_t)digits.toInt();
  return true;
}

static uint32_t outboxUsedBytesCached(bool force) {
  if (!outboxMounted) return 0;
  const uint32_t now = millis();
  if (force || outboxUsedAt == 0 || (uint32_t)(now - outboxUsedAt) > 15000UL) {
    outboxUsedCache = (uint32_t)LittleFS.usedBytes();
    outboxUsedAt = now ? now : 1U;
  }
  return outboxUsedCache;
}
static bool outboxHasRoom() {
  const uint32_t total = (uint32_t)LittleFS.totalBytes();
  const uint32_t used = outboxUsedBytesCached(true);
  return total > used && (total - used) >= OUTBOX_RESERVE_BYTES;
}

static bool quarantineOutboxFile(const String& path) {
  const String bad = path + ".bad";
  if (LittleFS.exists(bad)) LittleFS.remove(bad);
  bool ok = LittleFS.rename(path, bad);
  if (!ok) ok = LittleFS.remove(path);        // last resort: one bad file must never wedge the queue
  outboxQuarantined++;
  outboxHasCorruptRecord = true;
  return ok;
}

static bool mountOutboxFilesystem() {
  if (LittleFS.begin(false)) return true;
  delay(60);
  if (LittleFS.begin(false)) return true;
  Serial.println("[OUTBOX] no mountable LittleFS (first boot after flashing, or damaged); formatting the storage partition once");
  if (!LittleFS.format()) { Serial.println("[OUTBOX] format failed"); return false; }
  outboxReformatted = true;
  return LittleFS.begin(false);
}

// Rebuild the cached pending count / oldest sequence from the directory listing.
static void rescanOutboxIndex() {
  uint32_t count = 0, smallest = UINT32_MAX, largest = 0;
  File dir = LittleFS.open(OUTBOX_DIR);
  if (dir && dir.isDirectory()) {
    File file = dir.openNextFile();
    while (file) {
      const String leaf = String(file.name());
      file.close();
      uint32_t sequence = 0;
      if (outboxSequenceFromName(leaf, &sequence, false)) {
        count++;
        if (sequence < smallest) smallest = sequence;
        if (sequence > largest) largest = sequence;
      }
      file = dir.openNextFile();
    }
    dir.close();
  }
  outboxCount = count;
  outboxNextSeq = count ? smallest : 0;
  if (largest > eventSeq) eventSeq = largest;   // never reuse a sequence that is still on flash
}

// 0 = nothing to fix, 1 = complete .tmp whose rename was interrupted, 2 = unreadable/invalid record.
static int findOutboxProblem(String& pathOut, uint32_t& sequenceOut) {
  File dir = LittleFS.open(OUTBOX_DIR);
  if (!dir || !dir.isDirectory()) return 0;
  File file = dir.openNextFile();
  while (file) {
    const String leaf = String(file.name());     // basename on every core
    const size_t size = file.size();
    uint32_t sequence = 0;
    if (outboxSequenceFromName(leaf, &sequence, true)) {
      const bool isTmp = leaf.endsWith(".tmp");
      bool good = false;
      if (size == sizeof(EventRec)) {
        EventRec rec;
        const size_t got = file.read(reinterpret_cast<uint8_t*>(&rec), sizeof(EventRec));
        good = got == sizeof(EventRec) && rec.schemaVersion == OUTBOX_SCHEMA && rec.seq == sequence &&
               rec.bootId[0] != '\0' && rec.crc == eventCrc(rec);
      }
      if (isTmp || !good) {
        pathOut = String(OUTBOX_DIR) + "/" + leaf;
        sequenceOut = sequence;
        file.close();
        dir.close();
        return (isTmp && good) ? 1 : 2;
      }
    }
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
  return 0;
}

static void initializeOutbox() {
  outboxHasCorruptRecord = false;
  outboxStorageHealthy = false;
  outboxMounted = mountOutboxFilesystem();
  if (!outboxMounted) {
    outboxStorageFault = true;
    Serial.println("[OUTBOX] persistent storage unavailable; collection stays on hold (no event could be made durable)");
    return;
  }
  if (!LittleFS.exists(OUTBOX_DIR) && !LittleFS.mkdir(OUTBOX_DIR)) {
    outboxStorageFault = true;
    Serial.println("[OUTBOX] cannot create the outbox directory; collection stays on hold");
    return;
  }
  for (int pass = 0; pass < 64; ++pass) {
    String path; uint32_t sequence = 0;
    const int problem = findOutboxProblem(path, sequence);
    if (problem == 0) break;
    if (sequence > eventSeq) eventSeq = sequence;   // never reuse a sequence number that exists on flash
    if (problem == 1) {
      const String destination = outboxPath(sequence, "evt");
      if (LittleFS.exists(destination)) {
        LittleFS.remove(path);                    // the committed record wins; drop the duplicate temp copy
      } else if (LittleFS.rename(path, destination)) {
        Serial.printf("[OUTBOX] recovered pending event #%lu after an interrupted write\n", (unsigned long)sequence);
      } else {
        quarantineOutboxFile(path);
      }
    } else {
      Serial.printf("[OUTBOX] unreadable record %s set aside as .bad; operation continues\n", path.c_str());
      quarantineOutboxFile(path);
    }
  }
  rescanOutboxIndex();
  outboxStorageHealthy = true;
  outboxStorageFault = false;
  outboxUsedBytesCached(true);
  Serial.printf("[OUTBOX] persistent storage ready: %lu pending event(s)%s%s\n", (unsigned long)outboxCount,
                outboxReformatted ? ", storage was formatted on this boot" : "",
                outboxQuarantined ? ", some unreadable records were set aside" : "");
}

static bool persistOutboxEvent(const EventRec& e) {
  if (!outboxMounted) { outboxStorageHealthy = false; return false; }
  const String finalPath = outboxPath(e.seq, "evt");
  const String tempPath = outboxPath(e.seq, "tmp");
  if (LittleFS.exists(finalPath)) { outboxStorageHealthy = true; return true; }
  if (!outboxHasRoom()) {
    outboxStorageHealthy = false;
    outboxWriteFailures++;
    outboxFull = true;
    Serial.printf("[OUTBOX] STORAGE FULL: event #%lu not stored (pending=%lu); collection is held until space is freed\n",
                  (unsigned long)e.seq, (unsigned long)outboxCount);
    return false;
  }
  outboxFull = false;
  EventRec rec = e;
  rec.crc = eventCrc(rec);
  if (LittleFS.exists(tempPath) && !LittleFS.remove(tempPath)) { outboxStorageHealthy = false; outboxWriteFailures++; return false; }
  File f = LittleFS.open(tempPath, FILE_WRITE);
  if (!f) { outboxStorageHealthy = false; outboxWriteFailures++; return false; }
  const size_t written = f.write(reinterpret_cast<const uint8_t*>(&rec), sizeof(EventRec));
  f.flush();
  const bool sizeOk = written == sizeof(EventRec) && f.size() == sizeof(EventRec);
  f.close();
  if (!sizeOk) {
    LittleFS.remove(tempPath);
    outboxStorageHealthy = false;
    outboxWriteFailures++;
    Serial.printf("[OUTBOX] incomplete persistent write for event #%lu (%u/%u bytes)\n",
                  (unsigned long)e.seq, (unsigned)written, (unsigned)sizeof(EventRec));
    return false;
  }
  if (!LittleFS.rename(tempPath, finalPath)) {
    outboxStorageHealthy = false;
    outboxWriteFailures++;
    Serial.printf("[OUTBOX] failed to commit event #%lu\n", (unsigned long)e.seq);
    return false;
  }
  outboxCount++;
  if (outboxNextSeq == 0 || e.seq < outboxNextSeq) outboxNextSeq = e.seq;
  outboxStorageHealthy = true;
  return true;
}

// Read and validate one committed record; an unreadable record is quarantined, not fatal.
static bool readOutboxRecord(uint32_t sequence, EventRec& e) {
  const String path = outboxPath(sequence, "evt");
  File record = LittleFS.open(path, FILE_READ);
  if (!record) return false;
  bool good = false;
  if (record.size() == sizeof(EventRec)) {
    const size_t n = record.read(reinterpret_cast<uint8_t*>(&e), sizeof(EventRec));
    good = n == sizeof(EventRec) && e.schemaVersion == OUTBOX_SCHEMA && e.seq == sequence &&
           e.bootId[0] != '\0' && e.crc == eventCrc(e);
  }
  record.close();
  if (!good) {
    Serial.printf("[OUTBOX] record %s failed integrity checks; set aside as .bad\n", path.c_str());
    quarantineOutboxFile(path);
    if (outboxCount) outboxCount--;
    outboxNextSeq = 0;
    return false;
  }
  return true;
}

static bool readOldestOutboxEvent(EventRec& e) {
  if (!outboxMounted) return false;
  for (int attempt = 0; attempt < 4; ++attempt) {
    if (outboxNextSeq == 0 || !LittleFS.exists(outboxPath(outboxNextSeq, "evt"))) rescanOutboxIndex();
    if (outboxCount == 0 || outboxNextSeq == 0) return false;
    if (readOutboxRecord(outboxNextSeq, e)) return true;
  }
  return false;
}

static bool removeOutboxEvent(uint32_t sequence) {
  if (!outboxMounted) return false;
  const String path = outboxPath(sequence, "evt");
  if (LittleFS.exists(path)) {
    if (!LittleFS.remove(path)) {
      Serial.printf("[OUTBOX] server acknowledged event #%lu but local removal failed; idempotent retry may occur\n", (unsigned long)sequence);
      return false;
    }
    if (outboxCount) outboxCount--;
  }
  if (sequence == outboxNextSeq) outboxNextSeq = outboxCount ? sequence + 1U : 0U;   // fast path; rescan if a gap exists
  outboxUsedBytesCached(true);
  return true;
}

static void retryOutboxWriteService(uint32_t now) {
  if (!outboxMounted || (uint32_t)(now - lastOutboxRetryMs) < outboxRetryDelayMs) return;
  lastOutboxRetryMs = now;
  if (evtCount == 0) return;
  int oldest = (int)evtHead - (int)evtCount;
  while (oldest < 0) oldest += EVT_MAX;
  for (uint8_t k = 0; k < evtCount; ++k) {
    const uint8_t slot = (uint8_t)((oldest + k) % EVT_MAX);
    if (evtDurable[slot]) continue;
    if (!persistOutboxEvent(evts[slot])) {
      outboxStorageHealthy = false;
      outboxStorageFault = true;
      outboxRetryDelayMs = (outboxRetryDelayMs < 30000UL) ? outboxRetryDelayMs * 2UL : 60000UL;
      if (outboxRetryDelayMs > 60000UL) outboxRetryDelayMs = 60000UL;
      return;
    }
    evtDurable[slot] = true;
    outboxStorageFault = false;
    outboxStorageHealthy = true;
    outboxRetryDelayMs = 2000UL;
    operationalReady = false;
    readinessHealthySince = 0;
    setErr("");
    if (stationaryMode) stationaryAutoStartPending = true;   // stationary mode resumes on its own after readiness is re-acquired
    Serial.printf("[OUTBOX] event #%lu is now durable; %s\n", (unsigned long)evts[slot].seq,
                  stationaryMode ? "stationary collection will restart after a fresh readiness check"
                                 : "outputs stay stopped until the operator restarts after a fresh readiness check");
    recordControllerEvent("MAIN", "STORAGE", "Persistent Event Outbox", "RETRY_WRITE", "RECOVERED",
                          stationaryMode ? "Previously failed event is now stored. Stationary collection restarts after readiness."
                                         : "Previously failed event is now stored. Collection stays stopped until the operator restarts it.");
    markDirty();
    return;
  }
}

static uint32_t countOutboxRecords() { return outboxMounted ? outboxCount : 0U; }

static uint32_t deploySecNow() { return deployBaseSec + millis() / 1000UL; }
static void deployService(uint32_t now) {
  if ((uint32_t)(now - lastDeploySaveMs) < 60000UL) return;
  lastDeploySaveMs = now;
  if (powerOn) prefs.putUInt("dep", deploySecNow());     // at most ~60 s of operating time can be lost on a hard power cut
}

static void recordEvent(const char* cat, const char* label, float conf, uint32_t camSeq,
                        const char* servo, float dist, const char* ack, uint8_t retries,
                        bool rejected, const char* result) {
  EventRec& e = evts[evtHead];
  memset(&e, 0, sizeof(e));
  ensureDiscoveryBootId();
  e.schemaVersion = OUTBOX_SCHEMA;
  eventSeq++;
  e.seq = eventSeq;
  strncpy(e.bootId, discoveryBootId.c_str(), sizeof(e.bootId) - 1);
  e.camSeq = camSeq;
  e.upMs = millis();
  e.epoch = epochNow();
  e.timeSrc = e.epoch ? 1 : 0;
  e.bootNum = bootNumber;
  e.deploySec = deploySecNow();
  strncpy(e.cat, cat, sizeof(e.cat) - 1); e.cat[sizeof(e.cat) - 1] = '\0';
  sanitize(e.label, label ? label : "", sizeof(e.label));
  e.conf = conf;
  strncpy(e.servo, servo, sizeof(e.servo) - 1); e.servo[sizeof(e.servo) - 1] = '\0';
  e.dist = dist;
  strncpy(e.ack, ack, sizeof(e.ack) - 1); e.ack[sizeof(e.ack) - 1] = '\0';
  e.retries = retries;
  e.rejected = rejected;
  strncpy(e.result, result, sizeof(e.result) - 1); e.result[sizeof(e.result) - 1] = '\0';
  const uint8_t eventSlot = evtHead;
  const bool persisted = persistOutboxEvent(e);
  evtDurable[eventSlot] = persisted;
  if (!persisted) {
    outboxStorageFault = true;
    outboxStorageHealthy = false;
    stationaryAutoStartPending = false;
    operationalReady = false;
    readinessHealthySince = 0;
    safetyArmed = false;
    conveyorOn = false;
    classificationEnabled = false;
    joyL = joyR = 0;
    strcpy(navStr, "NEUTRAL");
    hardStopPropulsion();
    motorHardStop(mC);
    segState = SEG_IDLE;
    servoHardZero();
    setErr("OUTBOX STORAGE FAULT");
    Serial.printf("[OUTBOX] SAFETY HOLD: event #%lu is not yet durable; collection stopped until the event can be persisted\n", (unsigned long)e.seq);
  }
  evtHead = (evtHead + 1) % EVT_MAX;
  if (evtCount < EVT_MAX) evtCount++;
  if (!persisted) {
    recordControllerEvent("MAIN", "STORAGE", "Persistent Event Outbox", "WRITE", "BLOCKED", "Event write failed. Collection stopped; pending event remains in RAM for retry, but a power loss before recovery can lose this not-yet-durable event.");
  }

  saveCounters();
  Serial.printf("[EVT #%lu] %s %s conf=%.2f servo=%s dist=%.1f ack=%s retries=%u result=%s\n",
                (unsigned long)e.seq, e.cat, e.label, (double)e.conf, e.servo, (double)e.dist,
                e.ack, (unsigned)e.retries, e.result);

  char buf[640];
  eventToJson(e, buf, sizeof(buf), true);
  wsBroadcast(buf);
  telemetryDirty = true;
}

/* ============================== UART -> CAMERA ============================ */
static void uartReply(const char* s) {
  Serial2.print(s);
  Serial2.print('\n');
}

static void uartSendState() {
  uint8_t cls = (classificationEnabled && operationAllowed()) ? 1 : 0;
  uint8_t safe = (safeMode || estop || !powerOn) ? 1 : 0;
  uint8_t ready = operationalReady ? 1 : 0;
  uint8_t stationary = stationaryMode ? 1 : 0;
  char b[40];
  snprintf(b, sizeof(b), "<ST,%u,%u,%u,%u>", (unsigned)cls, (unsigned)safe, (unsigned)ready, (unsigned)stationary);
  Serial2.print(b);
  Serial2.print('\n');
  lastStSentCls = cls;
  lastStSentSafe = safe;
  lastStSentReady = ready;
  lastStSentStationary = stationary;
}

static void stService(uint32_t now) {
  uint8_t cls = (classificationEnabled && operationAllowed()) ? 1 : 0;
  uint8_t safe = (safeMode || estop || !powerOn) ? 1 : 0;
  uint8_t ready = operationalReady ? 1 : 0;
  uint8_t stationary = stationaryMode ? 1 : 0;
  if (stDirty || cls != lastStSentCls || safe != lastStSentSafe || ready != lastStSentReady || stationary != lastStSentStationary || (uint32_t)(now - lastStMs) >= 1000UL) {
    stDirty = false;
    lastStMs = now;
    uartSendState();
  }
}

static int splitTokens(char* s, char** tok, int maxTok) {
  int n = 0;
  char* p = s;
  while (n < maxTok) {
    tok[n++] = p;
    char* c = strchr(p, ',');
    if (!c) break;
    *c = '\0';
    p = c + 1;
  }
  return n;
}

static void replyError(const char* why) {
  uartErrorCount++;
  setErr(why);
  uartReply("ERROR");
  telemetryDirty = true;
}

static bool parseSeqAndLabel(char** tok, int n, int seqMarkerIdx, uint32_t* seq, const char** label) {
  // <..., SEQ, n [, label]>   (the literal SEQ marker is optional for robustness)
  *label = "";
  if (n > seqMarkerIdx && strcmp(tok[seqMarkerIdx], "SEQ") == 0) {
    if (n <= seqMarkerIdx + 1) return false;
    *seq = parseU32(tok[seqMarkerIdx + 1]);
    if (n > seqMarkerIdx + 2) *label = tok[seqMarkerIdx + 2];
    return true;
  }
  if (n > seqMarkerIdx) {
    *seq = parseU32(tok[seqMarkerIdx]);
    if (n > seqMarkerIdx + 1) *label = tok[seqMarkerIdx + 1];
    return true;
  }
  return false;
}

static void setLast(const char* cat, const char* label, float conf) {
  strncpy(lastCategory, cat, sizeof(lastCategory) - 1);
  lastCategory[sizeof(lastCategory) - 1] = '\0';
  sanitize(lastLabel, label, sizeof(lastLabel));
  lastConfidence = conf;
  lastClassUpMs = millis();
}

// returns true if this packet is a retransmission that was already accepted (ACK re-sent)
static bool handleRepeat(uint32_t seq, uint8_t* attemptsOut) {
  bool repeat = (lastSeenValid && seq == lastSeenSeq);
  if (repeat) {
    if (seenAttempts < 250) seenAttempts++;
    uartRetryCount++;
  } else {
    lastSeenSeq = seq;
    lastSeenValid = true;
    seenAttempts = 1;
  }
  *attemptsOut = seenAttempts;
  if (repeat && lastAccValid && seq == lastAccSeq && (uint32_t)(millis() - lastAccMs) < 60000UL) {
    uartReply(lastAccAck);
    bumpEventRetries(seq, (uint8_t)(seenAttempts - 1));
    if (segState != SEG_IDLE && segCamSeq == seq) segRetries = (uint8_t)(seenAttempts - 1);
    telemetryDirty = true;
    return true;
  }
  return false;
}

static void acceptedAck(const char* ack, uint32_t seq) {
  strncpy(lastAccAck, ack, sizeof(lastAccAck) - 1);
  lastAccAck[sizeof(lastAccAck) - 1] = '\0';
  lastAccSeq = seq;
  lastAccValid = true;
  lastAccMs = millis();
  uartReply(ack);
}

static void handleCat(char** tok, int n) {
  if (n < 4) { replyError("MALFORMED CAT"); return; }
  int cls = atoi(tok[1]);
  float conf = (float)atof(tok[2]);
  uint32_t seq = 0;
  const char* label = "";
  if (!parseSeqAndLabel(tok, n, 3, &seq, &label)) { replyError("MALFORMED CAT"); return; }
  if (cls < 1 || cls > 3 || conf < MIN_CONF || conf > 1.001f) { replyError("BAD CAT VALUES"); return; }

  uint8_t attempts = 1;
  if (handleRepeat(seq, &attempts)) return;
  uint8_t retries = (uint8_t)(attempts - 1);

  if (!(operationAllowed() && classificationEnabled)) {
    replyError(safeMode ? "REJECTED: SAFE MODE" : (estop ? "REJECTED: E-STOP" : "REJECTED: CLASSIFICATION OFF"));
    return;
  }
  char ack[8];
  snprintf(ack, sizeof(ack), "ACK%d", cls);
  char lab[32];
  sanitize(lab, label, sizeof(lab));
  if (lab[0] == '\0') strcpy(lab, "-");

  if (cls == 1) {
    acceptedAck(ack, seq);
    setLast("CAT1", lab, conf);
    cat1Count++;
    recordEvent("CAT1", lab, conf, seq, "NONE", -1.0f, ack, retries, false, "CAT1_DIRECT");
    return;
  }

  // CAT2 / CAT3: needs a free segregation slot
  if (segState != SEG_IDLE) { replyError("BUSY: SEGREGATION ACTIVE"); return; }
  acceptedAck(ack, seq);
  setLast(cls == 2 ? "CAT2" : "CAT3", lab, conf);
  segCls = (uint8_t)cls;
  segCamSeq = seq;
  strncpy(segLabel, lab, sizeof(segLabel) - 1); segLabel[sizeof(segLabel) - 1] = '\0';
  segConf = conf;
  segRetries = retries;
  strncpy(segAck, ack, sizeof(segAck) - 1); segAck[sizeof(segAck) - 1] = '\0';
  segHits = 0;
  segInvalid = 0;
  segNextSample = millis();
  segDeadline = millis() + POSITION_WAIT_MS;
  segSince = millis();
  segState = SEG_WAIT_POS;
  telemetryDirty = true;
}

static void handleRej(char** tok, int n) {
  if (n < 3) { replyError("MALFORMED REJ"); return; }
  float conf = (float)atof(tok[1]);
  uint32_t seq = 0;
  const char* label = "";
  if (!parseSeqAndLabel(tok, n, 2, &seq, &label)) { replyError("MALFORMED REJ"); return; }
  if (conf < MIN_CONF || conf > 1.001f) { replyError("BAD REJ VALUES"); return; }

  uint8_t attempts = 1;
  if (handleRepeat(seq, &attempts)) return;
  if (!(operationAllowed() && classificationEnabled)) {
    replyError(safeMode ? "REJECTED: SAFE MODE" : (estop ? "REJECTED: E-STOP" : "REJECTED: CLASSIFICATION OFF"));
    return;
  }
  char lab[32];
  sanitize(lab, label, sizeof(lab));
  if (lab[0] == '\0') strcpy(lab, "-");
  acceptedAck("REJECT", seq);
  setLast("REJECT", lab, conf);
  rejectedCount++;
  recordEvent("REJECT", lab, conf, seq, "NONE", -1.0f, "REJECT", (uint8_t)(attempts - 1), true, "REJECT_LOGGED");
}

static void handleHb(char** tok, int n) {
  if (n < 2) return;
  const bool wasSeen = camSeen;
  const bool wasOk = camHbOk;
  camSeen = true;
  lastHbMs = millis();
  camHbOk = (atoi(tok[1]) == 1);
  if (n >= 3 && tok[2][0] >= '0' && tok[2][0] <= '9') {
    strncpy(camIp, tok[2], sizeof(camIp) - 1);
    camIp[sizeof(camIp) - 1] = '\0';
  }
  if (n >= 4) camFps = parseU32(tok[3]);
  if (n >= 5) camInferMs = parseU32(tok[4]);
  if (!wasSeen) recordControllerEvent("CAM", "CONNECT", "ESP32-CAM", "HEARTBEAT", camHbOk ? "OK" : "FAULT", camHbOk ? "CAM heartbeat established" : "CAM reported unhealthy model state");
  else if (wasOk != camHbOk) recordControllerEvent("CAM", "HEALTH", "ESP32-CAM", "HEARTBEAT", camHbOk ? "OK" : "FAULT", camHbOk ? "CAM model recovered" : "CAM model fault reported");
}

static void uartHandlePacket(char* s) {
  char* tok[10];
  int n = splitTokens(s, tok, 10);
  if (n <= 0) return;
  if (strcmp(tok[0], "CAT") == 0) handleCat(tok, n);
  else if (strcmp(tok[0], "REJ") == 0) handleRej(tok, n);
  else if (strcmp(tok[0], "HB") == 0) handleHb(tok, n);
  else if (strcmp(tok[0], "ACK") == 0) {
    // CAM -> MAIN component feedback. Example: <ACK,ST,OK,1,0>
    const char* component = n >= 2 ? tok[1] : "UNKNOWN";
    const char* result = n >= 3 ? tok[2] : "OK";
    char detail[96]; detail[0] = '\0';
    if (n >= 4) {
      for (int i = 3; i < n; i++) {
        if (detail[0]) strncat(detail, ",", sizeof(detail)-strlen(detail)-1);
        strncat(detail, tok[i], sizeof(detail)-strlen(detail)-1);
      }
    }
    recordControllerEvent("CAM", "FEEDBACK", component, "ACK", result, detail[0] ? detail : "CAM acknowledged command/state");
  }
  else replyError("UNKNOWN PACKET");
}

static void uartPoll() {
  while (Serial2.available() > 0) {
    int ch = Serial2.read();
    if (ch < 0) break;
    if (ch == '<') { uIn = true; uLen = 0; continue; }
    if (!uIn) continue;
    if (ch == '>') {
      uBuf[uLen] = '\0';
      uIn = false;
      uLen = 0;
      uartHandlePacket(uBuf);
    } else if (ch == '\n' || uLen >= sizeof(uBuf) - 1) {
      uIn = false;
      uLen = 0;
      uartErrorCount++;
      setErr("GARBLED UART FRAME");
    } else {
      uBuf[uLen++] = (char)ch;
    }
  }
}

static const char* uartStatusStr(uint32_t now) {
  if (!camSeen) return "NO CAMERA";
  if ((uint32_t)(now - lastHbMs) > CAM_HB_TIMEOUT_MS) return "TIMEOUT";
  if (!camHbOk) return "CAMERA FAULT";
  return "OK";
}

/* ============================= SEGREGATION ================================ */
static void segFinishIdle() {
  segState = SEG_IDLE;
  servoTgt[0] = 0.0f;
  servoTgt[1] = 0.0f;
  markDirty();
}

static void segAbort(const char* why) {
  if (segState != SEG_IDLE) {
    recordEvent(segCls == 2 ? "CAT2" : "CAT3", segLabel, segConf, segCamSeq, "ABORTED", lastDistCm,
                segAck, segRetries, false, why);
  }
  segState = SEG_IDLE;
  servoHardZero();
}

static void segregationService(uint32_t now) {
  if (segState == SEG_IDLE) {
    // keep the flaps retracted whenever nothing is being routed
    if (servoTgt[0] != 0.0f) servoTgt[0] = 0.0f;
    if (servoTgt[1] != 0.0f) servoTgt[1] = 0.0f;
    return;
  }
  if (!operationAllowed()) { segAbort("ABORT_SAFE"); return; }

  int si = (segCls == 2) ? 0 : 1;

  switch (segState) {
    case SEG_WAIT_POS: {
      if ((int32_t)(now - segNextSample) >= 0) {
        segNextSample = now + SR04_SAMPLE_MS;
        float d = readDistanceCm();
        if (d < 0.0f) {
          segInvalid++;
          if (segInvalid >= SR04_INVALID_LIMIT) {
            setErr("HC-SR04 FAILURE");
            latchedFaults |= R_SR04;
            segAbort("HC-SR04_FAULT");   // records the event, zeroes the servos
            return;
          }
        } else {
          segInvalid = 0;
          lastDistCm = d;
          if (d <= triggerCm()) segHits++; else segHits = 0;
          if (segHits >= SR04_HITS_REQUIRED) {
            // position confirmed -> divert
            char servoTxt[18];
            snprintf(servoTxt, sizeof(servoTxt), "SERVO%d_180_3S", si + 1);
            if (segCls == 2) cat2Count++; else cat3Count++;
            recordEvent(segCls == 2 ? "CAT2" : "CAT3", segLabel, segConf, segCamSeq, servoTxt, d,
                        segAck, segRetries, false, "DIVERTED");
            if (outboxStorageFault) { segState = SEG_IDLE; servoHardZero(); return; }
            servoTgt[si] = 180.0f;
            segSince = now;
            segState = SEG_EXTEND;
            markDirty();
            return;
          }
        }
      }
      if ((int32_t)(now - segDeadline) >= 0) {
        timeoutCount++;
        recordEvent(segCls == 2 ? "CAT2" : "CAT3", segLabel, segConf, segCamSeq, "NONE", lastDistCm,
                    segAck, segRetries, false, "POSITION_TIMEOUT");
        segFinishIdle();
      }
      break;
    }
    case SEG_EXTEND:
      if (servoPos[si] >= 179.5f) {
        segHoldUntil = now + SERVO_HOLD_MS;
        segSince = now;
        segState = SEG_HOLD;
      } else if ((uint32_t)(now - segSince) > SERVO_MOVE_TIMEOUT_MS) {
        setErr("SERVO FAULT");
        latchedFaults |= R_SERVO;
        segAbort("SERVO_FAULT");
      }
      break;
    case SEG_HOLD:
      if ((int32_t)(now - segHoldUntil) >= 0) {
        servoTgt[si] = 0.0f;
        segSince = now;
        segState = SEG_RETRACT;
        markDirty();
      }
      break;
    case SEG_RETRACT:
      if (servoPos[si] <= 0.5f) {
        segFinishIdle();            // next classification is allowed immediately
      } else if ((uint32_t)(now - segSince) > SERVO_MOVE_TIMEOUT_MS) {
        setErr("SERVO FAULT");
        latchedFaults |= R_SERVO;
        segAbort("SERVO_FAULT");
      }
      break;
    default:
      segState = SEG_IDLE;
      break;
  }
}

static const char* safeReasonStr();

/* ============================== SAFETY / MODES ============================ */
static bool dashActive(uint32_t now) {
  if (wsClientCount() > 0) return true;
  return (lastApiMs != 0) && ((uint32_t)(now - lastApiMs) < DASH_TIMEOUT_MS);
}

static void stopAllOutputs() {
  hardStopPropulsion();
  motorHardStop(mC);
  conveyorOn = false;
  classificationEnabled = false;
  segAbort("ABORT_STOP");
  servoHardZero();
  markDirty();
}

static void enterSafe() {
  operationalReady = false;
  readinessHealthySince = 0;
  safeMode = true;
  stopAllOutputs();
  Serial.printf("[SAFE] SAFE MODE ENTERED: %s\n", safeReasonStr());
  recordControllerEvent("MAIN", "SAFETY", "SYSTEM", "ENTER_SAFE", "BLOCKED", safeReasonStr());
  markDirty();
}

static void exitSafe() {
  safeMode = false;
  operationalReady = false;
  readinessHealthySince = 0;
  setErr("");
  Serial.println("[SAFE] safe mode cleared; readiness must be reacquired before outputs");
  recordControllerEvent("MAIN", "SAFETY", "SYSTEM", "EXIT_SAFE", "RECOVERED", "Safety inputs recovered; operation remains locked until readiness is reacquired");
  markDirty();
}

static const char* safeReasonStr() {
  if (latchedFaults & R_SR04) return "HC-SR04 FAULT";
  if (latchedFaults & R_SERVO) return "SERVO FAULT";
  if (recoverFlags & R_CAM_LOST) return "CAMERA LOST";
  if (recoverFlags & R_CAM_FAULT) return "CAMERA MODEL FAULT";
  if (recoverFlags & R_DASH) return WiFi.status() == WL_CONNECTED ? "DASHBOARD LOST" : "NETWORK / COMMUNICATION LOST";
  return "";
}

static void buildReadinessChecks(char* out, size_t n, uint32_t now, bool* allReadyOut) {
  const bool camReady = camSeen && camHbOk && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  const bool dashReady = dashActive(now);
  const bool wifiReady = (WiFi.status() == WL_CONNECTED);
  const bool sr04Fresh = hcSr04Ready && lastSr04ValidMs != 0 && ((uint32_t)(now - lastSr04ValidMs) <= READINESS_SR04_STALE_MS);
  const bool servoPath = servoPathReady[0] && servoPathReady[1];
  const bool driverPath = motorDriverPathReady[0] && motorDriverPathReady[1] && motorDriverPathReady[2];
  const bool faultsClear = (latchedFaults == 0) && !estop && powerOn;
  const bool operationIdle = !conveyorOn && !classificationEnabled && segState == SEG_IDLE && joyL == 0 && joyR == 0;
  const bool classificationReady = camReady;
  const bool bleLink = bleLinkAlive(now);
  const bool wifiRequired = !stationaryMode && !bleLink;
  const bool dashboardRequired = !stationaryMode && !bleLink;
  const bool localOrRemoteLinkReady = stationaryMode || remoteLinkAlive(now);

  // Physical ACK is intentionally NOT a required readiness gate until real
  // electrical feedback exists. This prevents the firmware from pretending it
  // can verify passive motor/servo connection that current wiring cannot prove.
  const bool physicalFeedback = false;

  const bool persistentLogReady = outboxMounted && outboxStorageHealthy && !outboxStorageFault;
  const bool verifiableReady = controllerInitialized && localOrRemoteLinkReady && persistentLogReady && powerOn && !estop && !safeMode &&
                                (!CAMERA_REQUIRED_FOR_OPERATION || camReady) && sr04Fresh &&
                                classificationReady && faultsClear && (operationalReady || operationIdle);   // idle is required only while ACQUIRING readiness
  if (allReadyOut) *allReadyOut = verifiableReady;

  snprintf(out, n,
    "[{\"id\":\"main_controller\",\"name\":\"ESP32 Main Controller\",\"required\":true,\"met\":%s,\"status\":\"%s\",\"detail\":\"Controller initialized and responding\"},"
    "{\"id\":\"wifi\",\"name\":\"Wi-Fi\",\"required\":%s,\"met\":%s,\"status\":\"%s\",\"detail\":\"Local network link; optional during autonomous stationary operation\"},"
    "{\"id\":\"dashboard\",\"name\":\"Dashboard Communication\",\"required\":%s,\"met\":%s,\"status\":\"%s\",\"detail\":\"Recent REST/WebSocket communication; optional during autonomous stationary operation\"},"
    "{\"id\":\"camera\",\"name\":\"ESP32-CAM\",\"required\":%s,\"met\":%s,\"status\":\"%s\",\"detail\":\"Heartbeat/model health\"},"
    "{\"id\":\"hcsr04\",\"name\":\"HC-SR04\",\"required\":true,\"met\":%s,\"status\":\"%s\",\"detail\":\"Recent valid distance reading\"},"
    "{\"id\":\"servo1\",\"name\":\"Flap Servo 1\",\"required\":false,\"met\":%s,\"status\":\"%s\",\"detail\":\"PWM path initialized; physical position feedback is unavailable\"},"
    "{\"id\":\"servo2\",\"name\":\"Flap Servo 2\",\"required\":false,\"met\":%s,\"status\":\"%s\",\"detail\":\"PWM path initialized; physical position feedback is unavailable\"},"
    "{\"id\":\"motor_left\",\"name\":\"Left Propulsion Motor Driver\",\"required\":false,\"met\":%s,\"status\":\"%s\",\"detail\":\"Command path initialized; physical ACK unavailable\"},"
    "{\"id\":\"motor_right\",\"name\":\"Right Propulsion Motor Driver\",\"required\":false,\"met\":%s,\"status\":\"%s\",\"detail\":\"Command path initialized; physical ACK unavailable\"},"
    "{\"id\":\"motor_conveyor\",\"name\":\"Conveyor Motor Driver\",\"required\":false,\"met\":%s,\"status\":\"%s\",\"detail\":\"Command path initialized; physical ACK unavailable\"},"
    "{\"id\":\"classification\",\"name\":\"Classification System\",\"required\":true,\"met\":%s,\"status\":\"%s\",\"detail\":\"Camera/model subsystem is required for operational use\"},"
    "{\"id\":\"faults\",\"name\":\"Safety / Latched Faults\",\"required\":true,\"met\":%s,\"status\":\"%s\",\"detail\":\"No latched sensor/servo fault, no E-Stop, power available\"},"
    "{\"id\":\"operation_idle\",\"name\":\"Operation Idle\",\"required\":true,\"met\":%s,\"status\":\"%s\",\"detail\":\"No active conveyor/classification/segregation/joystick command during readiness acquisition\"},"
    "{\"id\":\"physical_feedback\",\"name\":\"Physical Feedback Verification\",\"required\":false,\"met\":%s,\"status\":\"UNVERIFIED\",\"detail\":\"Current passive motor/servo wiring provides no electrical ACK\"}]",
    controllerInitialized?"true":"false", controllerInitialized?"READY":"WAITING",
    wifiRequired?"true":"false", wifiReady?"true":"false", wifiReady?"READY":(wifiRequired?"NOT READY":"NOT REQUIRED"),
    dashboardRequired?"true":"false", dashReady?"true":"false", dashReady?"READY":(dashboardRequired?"WAITING":"NOT REQUIRED"),
    CAMERA_REQUIRED_FOR_OPERATION?"true":"false", camReady?"true":"false", camReady?"READY":"NOT READY",
    sr04Fresh?"true":"false", sr04Fresh?"READY":"NOT READY",
    "false", servoPath?"UNVERIFIED":"WAITING",
    "false", servoPath?"UNVERIFIED":"WAITING",
    "false", motorDriverPathReady[0]?"UNVERIFIED":"WAITING",
    "false", motorDriverPathReady[1]?"UNVERIFIED":"WAITING",
    "false", motorDriverPathReady[2]?"UNVERIFIED":"WAITING",
    classificationReady?"true":"false", classificationReady?"READY":"NOT READY",
    faultsClear?"true":"false", faultsClear?"CLEAR":"FAULT",
    (operationalReady || operationIdle)?"true":"false", operationIdle?"IDLE":(operationalReady?"RUNNING":"BUSY"),
    physicalFeedback?"true":"false");
}

static void buildHardwareStatus(char* out, size_t n, uint32_t now) {
  const bool camReady = camSeen && camHbOk && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  const bool sr04Valid = hcSr04Ready && lastSr04ValidMs != 0 && ((uint32_t)(now - lastSr04ValidMs) <= READINESS_SR04_STALE_MS);
  const char* camStatus = camReady ? "READY" : (camSeen ? "FAULT" : "NOT DETECTED");
  const char* srStatus = (latchedFaults & R_SR04) ? "FAULT" : (sr04Valid ? "READY" : "WAITING");
  const char* servoStatus = (latchedFaults & R_SERVO) ? "FAULT" : ((servoPathReady[0] && servoPathReady[1]) ? "UNVERIFIED" : "WAITING");
  const char* motorL = motorDriverPathReady[0] ? "UNVERIFIED" : "WAITING";
  const char* motorR = motorDriverPathReady[1] ? "UNVERIFIED" : "WAITING";
  const char* motorC = motorDriverPathReady[2] ? "UNVERIFIED" : "WAITING";
  snprintf(out, n,
    "[{\"id\":\"esp32-main\",\"name\":\"ESP32 Main Controller\",\"status\":\"ACTIVE\",\"verified\":true,\"feedback\":true,\"detail\":\"Controller responding\"},"
    "{\"id\":\"esp32-cam\",\"name\":\"ESP32-CAM\",\"status\":\"%s\",\"verified\":%s,\"feedback\":true,\"detail\":\"UART heartbeat + model status\"},"
    "{\"id\":\"motor-left-driver\",\"name\":\"Left Propulsion Motor Driver\",\"status\":\"%s\",\"verified\":false,\"feedback\":false,\"detail\":\"Command path initialized; physical connection cannot be electrically verified\"},"
    "{\"id\":\"motor-right-driver\",\"name\":\"Right Propulsion Motor Driver\",\"status\":\"%s\",\"verified\":false,\"feedback\":false,\"detail\":\"Command path initialized; physical connection cannot be electrically verified\"},"
    "{\"id\":\"motor-conveyor-driver\",\"name\":\"Conveyor Motor Driver\",\"status\":\"%s\",\"verified\":false,\"feedback\":false,\"detail\":\"Command path initialized; physical connection cannot be electrically verified\"},"
    "{\"id\":\"servo1\",\"name\":\"Flap Servo 1\",\"status\":\"%s\",\"verified\":false,\"feedback\":false,\"detail\":\"PWM path initialized; physical position cannot be electrically verified\"},"
    "{\"id\":\"servo2\",\"name\":\"Flap Servo 2\",\"status\":\"%s\",\"verified\":false,\"feedback\":false,\"detail\":\"PWM path initialized; physical position cannot be electrically verified\"},"
    "{\"id\":\"hcsr04\",\"name\":\"HC-SR04\",\"status\":\"%s\",\"verified\":%s,\"feedback\":true,\"detail\":\"Ultrasonic distance input\"},"
    "{\"id\":\"classification\",\"name\":\"Classification System\",\"status\":\"%s\",\"verified\":%s,\"feedback\":true,\"detail\":\"CAM/Main inference path\"},"
    "{\"id\":\"dashboard\",\"name\":\"Dashboard Communication\",\"status\":\"%s\",\"verified\":%s,\"feedback\":true,\"detail\":\"REST/WebSocket supervision\"}]",
    camStatus, camReady?"true":"false",
    motorL, motorR, motorC, servoStatus, servoStatus,
    srStatus, sr04Valid?"true":"false",
    camReady?"READY":"WAITING", camReady?"true":"false",
    dashActive(now)?"READY":"WAITING", dashActive(now)?"true":"false");
}

static void readinessService(uint32_t now) {
  if ((uint32_t)(now - lastReadinessCheckMs) < READINESS_CHECK_MS) return;
  lastReadinessCheckMs = now;

  // A non-blocking startup sensor check: one valid echo is enough to prove the
  // input path works; stale data later makes readiness drop back to NOT_READY.
  {
    float d = readDistanceCm();
    if (d >= 0.0f) {
      hcSr04Ready = true;
      lastSr04ValidMs = now;
      lastDistCm = d;
    }
  }

  static char checks[2600];
  bool allReady = false;
  buildReadinessChecks(checks, sizeof(checks), now, &allReady);

  if (!allReady || safeMode || estop || !powerOn) {
    if (operationalReady) recordControllerEvent("MAIN", "READINESS", "SYSTEM", "OPERATION_READY", "LOST", "Operational readiness condition is no longer satisfied");
    operationalReady = false;
    readinessHealthySince = 0;
    return;
  }

  if (!operationalReady) {
    if (readinessHealthySince == 0) readinessHealthySince = now;
    if ((uint32_t)(now - readinessHealthySince) >= READINESS_HOLD_MS) {
      operationalReady = true;
      recordControllerEvent("MAIN", "READINESS", "SYSTEM", "OPERATION_READY", "READY", "All required verifiable readiness conditions satisfied");
      markDirty();
    }
  }
}

static void safetyService(uint32_t now) {
  uint8_t rec = 0;
  const bool camLost = CAMERA_REQUIRED_FOR_OPERATION &&
    (camSeen ? ((uint32_t)(now - lastHbMs) > CAM_HB_TIMEOUT_MS) : (now > BOOT_CAM_GRACE_MS));
  if (!stationaryMode && !remoteLinkAlive(now)) rec |= R_DASH;
  if (camLost) rec |= R_CAM_LOST;
  else if (CAMERA_REQUIRED_FOR_OPERATION && camSeen && !camHbOk) rec |= R_CAM_FAULT;
  recoverFlags = rec;

  // Critical fault is derived only from actual latched subsystem faults.
  // STALL remains false unless a future diagnostic path provides concrete evidence.
  criticalFault = (latchedFaults != 0);
  snprintf(exceptionState, sizeof(exceptionState), "%s", criticalFault ? "CRITICAL_FAULT" : (stallState ? "STALL" : "NORMAL"));

  // Startup NOT_READY does not automatically mean SAFE MODE. Safe Mode is an
  // operational-run-time reaction after the system has been armed for work.
  bool activeRun = conveyorOn || classificationEnabled || segState != SEG_IDLE ||
                   (!stationaryMode && (joyL != 0 || joyR != 0));
  if (activeRun) safetyArmed = true;
  if (!powerOn || estop) safetyArmed = false;

  bool remoteLinkLost = !stationaryMode && !remoteLinkAlive(now);
  bool want = safetyArmed && ((rec != 0) || (latchedFaults != 0) || remoteLinkLost);
  if (want) {
    safeClearSince = 0;
    if (!safeMode) {
      setErr(safeReasonStr()[0] ? safeReasonStr() : "CONTROLLER COMMUNICATION LOST");
      enterSafe();
    }
  } else if (safeMode) {
    bool healthy = (rec == 0) && (latchedFaults == 0) &&
                   (stationaryMode || remoteLinkAlive(now)) &&
                   (!CAMERA_REQUIRED_FOR_OPERATION || (camSeen && camHbOk && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS)));
    if (healthy) {
      if (safeClearSince == 0) safeClearSince = now;
      else if ((uint32_t)(now - safeClearSince) >= SAFE_CLEAR_HOLD_MS) {
        safeClearSince = 0;
        exitSafe();
      }
    } else safeClearSince = 0;
  }
}

/* =========================== ESTOP / MODE CONTROL ======================== */
static void doEstop(bool active) {
  if (active) {
    estop = true;
    operationalReady = false;
    readinessHealthySince = 0;

    stopAllOutputs();

    setErr("EMERGENCY STOP");
    Serial.println("[ESTOP] EMERGENCY STOP");
  } else {
    estop = false;

    // Clearing E-stop does not immediately restore operation. Readiness must
    // be reacquired by readinessService() before any outputs may run.
    operationalReady = false;
    readinessHealthySince = 0;

    // Operator reset clears the latched SR04/servo fault flags.
    latchedFaults = 0;

    if (!safeMode) {
      setErr("");
    }

    Serial.println("[ESTOP] cleared");
  }

  markDirty();
}

static void setStationary(bool on) {
  stationaryMode = on;
  stationaryAutoStartPending = on;

  // Stationary mode disables propulsion immediately.
  // Conveyor/classification remain governed by their normal safety logic.
  if (on) {
    hardStopPropulsion();
  }

  savePrefsSoon();
  markDirty();

  Serial.printf("[MODE] stationary %s\n", on ? "ON" : "OFF");
}

static void recordControllerEvent(const char* direction, const char* eventType, const char* component,
                                  const char* command, const char* result, const char* detail) {
  controllerEventSeq++;
  char safeDetail[180]; sanitize(safeDetail, detail ? detail : "", sizeof(safeDetail));
  char safeCommand[72]; sanitize(safeCommand, command ? command : "", sizeof(safeCommand));
  char safeComponent[72]; sanitize(safeComponent, component ? component : "", sizeof(safeComponent));
  char safeType[72]; sanitize(safeType, eventType ? eventType : "", sizeof(safeType));
  char safeResult[72]; sanitize(safeResult, result ? result : "", sizeof(safeResult));
  char safeDir[32]; sanitize(safeDir, direction ? direction : "", sizeof(safeDir));
  const uint32_t now = millis();
  const uint32_t ts = epochNow();
  snprintf(lastControllerEventSummary, sizeof(lastControllerEventSummary), "%s %s: %s (%s)", safeDir, safeComponent, safeResult, safeDetail);
  snprintf(lastControllerEventJson, sizeof(lastControllerEventJson),
           "{\"sequence\":%lu,\"direction\":\"%s\",\"eventType\":\"%s\",\"component\":\"%s\",\"command\":\"%s\",\"result\":\"%s\",\"detail\":\"%s\",\"operationalReady\":%s,\"safeMode\":%s,\"stationaryMode\":%s,\"stall\":%s,\"criticalFault\":%s,\"exceptionState\":\"%s\",\"timestamp\":%lu,\"uptimeMs\":%lu}",
           (unsigned long)controllerEventSeq, safeDir, safeType, safeComponent, safeCommand, safeResult, safeDetail,
           operationalReady?"true":"false", safeMode?"true":"false", stationaryMode?"true":"false", stallState?"true":"false", criticalFault?"true":"false", exceptionState, (unsigned long)ts, (unsigned long)now);
  String frame = String("{\"type\":\"controller_event\",\"data\":") + lastControllerEventJson + "}";
  wsBroadcast(frame.c_str());
  Serial.printf("[EVENT] %s\n", lastControllerEventSummary);
  lastControllerEventCloudAttemptMs = 0; // queue the latest event on the next cloud service pass
  markDirty();
}

static void buildRuntimeCloudPayload(String& payload) {
  uint32_t now=millis();
  static char checks[2600];
  static char hw[2600];
  bool all = false;
  buildReadinessChecks(checks, sizeof(checks), now, &all);
  buildHardwareStatus(hw,sizeof(hw),now);
  payload = "{";
  payload += "\"p_device_id\":\"SIBOAT-MAIN\",";
  payload += "\"p_boot_id\":\"" + discoveryJsonEscape(discoveryBootId) + "\",";
  payload += "\"p_operational_ready\":" + String(operationalReady ? "true":"false") + ",";
  payload += "\"p_operation_state\":\"" + String(stateStr()) + "\",";
  payload += "\"p_safe_mode\":" + String(safeMode ? "true":"false") + ",";
  payload += "\"p_safe_reason\":\"" + discoveryJsonEscape(safeMode ? safeReasonStr() : "") + "\",";
  payload += "\"p_stationary_mode\":" + String(stationaryMode ? "true":"false") + ",";
  payload += "\"p_stall_state\":" + String(stallState ? "true":"false") + ",";
  payload += "\"p_critical_fault\":" + String(criticalFault ? "true":"false") + ",";
  payload += "\"p_exception_state\":\"" + discoveryJsonEscape(exceptionState) + "\",";
  payload += "\"p_readiness_checks\":" + String(checks) + ",";
  payload += "\"p_hardware_status\":" + String(hw) + ",";
  payload += "\"p_last_event\":" + String(lastControllerEventJson[0]?lastControllerEventJson:"null");
  payload += "}";
}

static bool enqueueCloudRpc(uint8_t kind, const char* rpcPath, const String& payload, uint32_t sequence) {
  if (!cloudRpcQueue || WiFi.status() != WL_CONNECTED) return false;
  const size_t payloadLength = payload.length();
  if (!rpcPath || strlen(rpcPath) >= CLOUD_RPC_PATH_SIZE || payloadLength >= CLOUD_RPC_PAYLOAD_SIZE) {
    Serial.println("[CLOUD] RPC job rejected: path or payload exceeds queue limits");
    return false;
  }

  CloudRpcJob* job = (CloudRpcJob*)malloc(sizeof(CloudRpcJob));
  if (!job) {
    Serial.println("[CLOUD] RPC job rejected: insufficient heap");
    return false;
  }
  memset(job, 0, sizeof(*job));
  job->kind = kind;
  job->sequence = (kind == CLOUD_JOB_EVENT) ? controllerEventSeq :
                  ((kind == CLOUD_JOB_PERFORMANCE || kind == CLOUD_JOB_SESSION) ? sequence : 0);
  snprintf(job->path, sizeof(job->path), "%s", rpcPath);
  payload.toCharArray(job->payload, sizeof(job->payload));

  if (xQueueSend(cloudRpcQueue, &job, 0) != pdTRUE) {
    free(job);
    Serial.println("[CLOUD] RPC queue full; latest state will be retried on the next service interval");
    return false;
  }
  return true;
}

static void queuePerformanceEventService(uint32_t now) {
  // Apply a worker acknowledgement first so every LittleFS change happens on this task.
  if (perfAckReady) {
    const uint32_t ackSeq = perfAckSeq;
    perfAckReady = false;
    removeOutboxEvent(ackSeq);
    lastPerformanceEventCloudSeq = ackSeq;
    lastPerformanceEventQueuedSeq = ackSeq;
    lastPerformanceEventAttemptMs = 0;
  }
  if (WiFi.status() != WL_CONNECTED || !cloudRpcQueue || !outboxMounted) return;
  if (lastPerformanceEventQueuedSeq != lastPerformanceEventCloudSeq) return; // one in flight
  uint32_t gap = 750UL;                                                     // bounded back-off after failures
  if (perfFailStreak > 0) { gap = 2000UL << (perfFailStreak > 5 ? 5 : perfFailStreak); if (gap > 60000UL) gap = 60000UL; }
  if (lastPerformanceEventAttemptMs != 0 && (uint32_t)(now - lastPerformanceEventAttemptMs) < gap) return;

  EventRec pending;
  if (!readOldestOutboxEvent(pending)) return;
  char eventBuf[760];
  eventToJson(pending, eventBuf, sizeof(eventBuf), true);
  size_t evLen = strlen(eventBuf);
  if (evLen > 1 && eventBuf[evLen - 1] == '}') eventBuf[evLen - 1] = '\0';   // append sync metadata inside the object
  const bool sameBoot = discoveryBootId == String(pending.bootId);
  char meta[300];
  snprintf(meta, sizeof(meta),
           ",\"pendingAfter\":%lu,\"sameBoot\":%s,\"clock\":{\"uptimeMs\":%lu,\"epoch\":%lu,\"deploySeconds\":%lu,\"bootNumber\":%lu}}",
           (unsigned long)(outboxCount > 0 ? outboxCount - 1U : 0U), sameBoot ? "true" : "false",
           (unsigned long)millis(), (unsigned long)epochNow(), (unsigned long)deploySecNow(), (unsigned long)bootNumber);
  String payload = "{\"p_device_id\":\"SIBOAT-MAIN\",\"p_boot_id\":\"" +
                   discoveryJsonEscape(String(pending.bootId)) + "\",\"p_event\":" + eventBuf + meta + "}";
  lastPerformanceEventAttemptMs = now;
  lastPerformanceEventQueuedSeq = pending.seq;
  if (enqueueCloudRpc(CLOUD_JOB_PERFORMANCE, "/rest/v1/rpc/seabot_record_performance_event", payload, pending.seq)) {
    Serial.printf("[ANALYTICS] queued persistent item outcome #%lu for Supabase\n", (unsigned long)pending.seq);
  } else {
    lastPerformanceEventQueuedSeq = lastPerformanceEventCloudSeq;
  }
}

static void collectionSessionCloudService(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED || !cloudRpcQueue) return;
  const bool active = conveyorOn && classificationEnabled && operationAllowed();
  if (performanceSessionJobQueued) return;
  if (performanceSessionCloudKnown && performanceSessionCloudActive == active) return;
  if (lastPerformanceSessionAttemptMs != 0 && (uint32_t)(now - lastPerformanceSessionAttemptMs) < 5000UL) return;

  ensureDiscoveryBootId();
  String payload = "{\"p_device_id\":\"SIBOAT-MAIN\",\"p_boot_id\":\"" +
    discoveryJsonEscape(discoveryBootId) + "\",\"p_active\":" + String(active ? "true" : "false") +
    ",\"p_reason\":\"" + discoveryJsonEscape(active ? "conveyor and classification active" : "conveyor/classification stopped or readiness lost") + "\"}";
  lastPerformanceSessionAttemptMs = now;
  // Mark in-flight before enqueueing to avoid racing an immediate worker reply.
  performanceSessionJobQueued = true;
  if (enqueueCloudRpc(CLOUD_JOB_SESSION, "/rest/v1/rpc/seabot_set_collection_state", payload, active ? 1U : 0U)) {
    Serial.printf("[ANALYTICS] queued collection session state: %s\n", active ? "ACTIVE" : "INACTIVE");
  } else {
    performanceSessionJobQueued = false;
  }
}

static void cloudRpcWorker(void* parameter) {
  (void)parameter;
  CloudRpcJob* job = nullptr;
  for (;;) {
    if (xQueueReceive(cloudRpcQueue, &job, portMAX_DELAY) != pdTRUE || !job) continue;

    bool ok = false;
    int code = -1;
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClientSecure client;
      client.setInsecure(); // TODO: replace with CA validation before production deployment
      client.setTimeout((SIBOAT_DISCOVERY_TIMEOUT_MS + 999UL) / 1000UL);   // WiFiClientSecure takes SECONDS

      HTTPClient http;
      const String url = String(SIBOAT_SUPABASE_URL) + job->path;
      if (http.begin(client, url)) {
        http.setTimeout(SIBOAT_DISCOVERY_TIMEOUT_MS);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Accept", "application/json");
        http.addHeader("apikey", SIBOAT_SUPABASE_PUBLISHABLE_KEY);
        code = http.POST(job->payload);
        http.end();
        ok = code >= 200 && code < 300;
      }
    }

    if (ok) { backendReachable = true; lastBackendOkMs = millis(); backendFailStreak = 0; }
    else if (backendFailStreak < 250) { if (++backendFailStreak >= 2) backendReachable = false; }

    if (ok) {
      if (job->kind == CLOUD_JOB_DISCOVERY) {
        discoveryEverPublished = true;
        Serial.printf("[DISCOVERY] registry heartbeat accepted (HTTP %d)\n", code);
      } else if (job->kind == CLOUD_JOB_RUNTIME) {
        Serial.printf("[CLOUD] runtime state accepted (HTTP %d)\n", code);
      } else if (job->kind == CLOUD_JOB_EVENT) {
        lastControllerEventCloudSeq = job->sequence;
        Serial.printf("[CLOUD] controller event #%lu accepted (HTTP %d)\n", (unsigned long)job->sequence, code);
      } else if (job->kind == CLOUD_JOB_PERFORMANCE) {
        // Remove only after a successful 2xx response. If local removal fails,
        // the stable (device_id, boot_id, event_sequence) key makes a retry idempotent.
        // The main loop removes the file and clears the in-flight marker (all LittleFS access stays on one task).
        perfAckSeq = job->sequence;
        perfFailStreak = 0;
        perfAckReady = true;
        Serial.printf("[ANALYTICS] persistent item outcome #%lu acknowledged by Supabase (HTTP %d)\n", (unsigned long)job->sequence, code);
      } else if (job->kind == CLOUD_JOB_SESSION) {
        performanceSessionCloudActive = (job->sequence == 1U);
        performanceSessionCloudKnown = true;
        performanceSessionJobQueued = false;
        Serial.printf("[ANALYTICS] collection session state stored: %s (HTTP %d)\n", performanceSessionCloudActive ? "ACTIVE" : "INACTIVE", code);
      }
    } else {
      Serial.printf("[CLOUD] RPC failed: %s (HTTP/client result %d); local controller remains active\n", job->path, code);
      if (job->kind == CLOUD_JOB_PERFORMANCE) {
        // Do not advance the acknowledged sequence. Re-send this item after the
        // retry interval; SQL uniqueness makes any accepted duplicate harmless.
        lastPerformanceEventQueuedSeq = lastPerformanceEventCloudSeq;
        lastPerformanceEventAttemptMs = millis();
        if (perfFailStreak < 250) perfFailStreak++;
      } else if (job->kind == CLOUD_JOB_SESSION) {
        performanceSessionJobQueued = false;
        lastPerformanceSessionAttemptMs = millis();
      }
    }

    free(job);
    job = nullptr;
    vTaskDelay(1);
  }
}

static void runtimeCloudService(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) return;
  ensureDiscoveryBootId();

  if (lastRuntimeCloudMs == 0 || (uint32_t)(now - lastRuntimeCloudMs) >= RUNTIME_CLOUD_INTERVAL_MS) {
    lastRuntimeCloudMs = now;
    String payload;
    buildRuntimeCloudPayload(payload);
    enqueueCloudRpc(CLOUD_JOB_RUNTIME, "/rest/v1/rpc/seabot_update_device_runtime", payload);
  }

  if (lastControllerEventJson[0] && lastControllerEventCloudSeq != controllerEventSeq &&
      (lastControllerEventCloudAttemptMs == 0 ||
       (uint32_t)(now - lastControllerEventCloudAttemptMs) >= CONTROLLER_EVENT_CLOUD_MS)) {
    String eventPayload = "{";
    eventPayload += "\"p_device_id\":\"SIBOAT-MAIN\",\"p_boot_id\":\"" + discoveryJsonEscape(discoveryBootId) + "\",";
    eventPayload += "\"p_sequence\":" + String(controllerEventSeq) + ",";
    eventPayload += "\"p_event\":" + String(lastControllerEventJson) + "}";
    // Rate-limit attempts even when the queue is full or unavailable. Failed
    // sends are retried on the next interval; successful sends are sequence-gated.
    lastControllerEventCloudAttemptMs = now;
    enqueueCloudRpc(CLOUD_JOB_EVENT, "/rest/v1/rpc/seabot_record_controller_event", eventPayload);
  }

  collectionSessionCloudService(now);
  queuePerformanceEventService(now);
}

/* ============================ STATUS / TELEMETRY ========================== */
static const char* stateStr() {
  if (!powerOn) return "OFFLINE";
  if (estop) return "EMERGENCY";
  if (stallState) return "STALL";
  if (safeMode) return "SAFE MODE";
  if (segState != SEG_IDLE) return "SORTING";
  if (!operationalReady) return "NOT READY";
  return "READY";
}

static void buildStatus(char* out, size_t n) {
  uint32_t now = millis();
  bool camOnline = camSeen && ((uint32_t)(now - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  IPAddress ip = WiFi.localIP();
  char ipStr[20];
  snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
  unsigned long c1 = cat1Count, c2 = cat2Count, c3 = cat3Count, rj = rejectedCount;

  Jw j;
  jwInit(j, out, n);
  jwStr(j, "type", "status");
  jwStr(j, "deviceId", SIBOAT_DISCOVERY_DEVICE_ID);
  jwStr(j, "bootId", discoveryBootId.c_str());
  // fields read directly by the existing dashboard
  jwStr(j, "state", stateStr());
  jwStr(j, "nav", navStr);
  jwBool(j, "conveyor", conveyorOn);
  jwBool(j, "classification", classificationEnabled);
  jwBool(j, "outsideLights", outsideLights);
  jwBool(j, "insideLights", insideLights);
  jwNull(j, "battery");
  jwInt(j, "wifi", wifiPercent());
  jwInt(j, "devices", 1 + (camOnline ? 1 : 0) + wsClientCount());
  // final telemetry set
  jwStr(j, "systemState", stateStr());
  jwStr(j, "wifiStatus", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  jwStr(j, "wifiSsid", (wCred >= 0 && wCred < WIFI_CRED_COUNT && wifiCredConfigured(wCred)) ? WIFI_CREDS[wCred].ssid : "");
  jwStr(j, "mainIp", ipStr);
  jwStr(j, "cameraIp", camIp);
  jwInt(j, "cameraPort", 81);
  {
    char camStream[96];
    snprintf(camStream, sizeof(camStream), "http://%s:81/stream", camIp);
    jwStr(j, "cameraStream", camStream);
  }
  jwStr(j, "cameraHostname", "siboat-cam.local");
  jwBool(j, "cameraConnected", camOnline);
  jwBool(j, "cameraOnline", camOnline);
  jwBool(j, "classificationEnabled", classificationEnabled);
  jwStr(j, "lastCategory", lastCategory);
  jwFlt(j, "lastConfidence", lastConfidence, 2);
  jwStr(j, "lastLabel", lastLabel);
  jwStr(j, "servo1State", servoStateStr(0));
  jwStr(j, "servo2State", servoStateStr(1));
  if (lastDistCm >= 0.0f) jwFlt(j, "hcSr04Distance", lastDistCm, 1); else jwNull(j, "hcSr04Distance");
  jwUInt(j, "cat1Count", c1);
  jwUInt(j, "cat2Count", c2);
  jwUInt(j, "cat3Count", c3);
  jwUInt(j, "rejectedCount", rj);
  jwUInt(j, "category1Count", c1);
  jwUInt(j, "category2Count", c2);
  jwUInt(j, "category3Count", c3);
  jwUInt(j, "unknownCount", 0);
  jwUInt(j, "collectedCount", c1 + c2 + c3 + rj);
  jwUInt(j, "segregatedCount", c1 + c2 + c3);
  jwUInt(j, "classificationTimeoutCount", timeoutCount);
  jwUInt(j, "uartRetryCount", uartRetryCount);
  jwUInt(j, "retryCount", uartRetryCount);
  jwUInt(j, "uartErrorCount", uartErrorCount);
  jwStr(j, "uartStatus", uartStatusStr(now));
  jwStr(j, "lastError", lastError);
  jwUInt(j, "uptime", now / 1000UL);
  jwUInt(j, "lastClassificationUptimeMs", lastClassUpMs);
  jwBool(j, "stationaryMode", stationaryMode);
  jwBool(j, "safeMode", safeMode);
  jwStr(j, "safeReason", safeMode ? safeReasonStr() : "");
  jwBool(j, "emergency", estop);
  jwBool(j, "power", powerOn);
  jwBool(j, "operationalReady", operationalReady);
  jwStr(j, "operationStatus", operationalReady ? "READY" : "NOT READY");
  jwStr(j, "operationReadiness", operationalReady ? "OPERATIONALLY READY" : "NOT OPERATIONALLY READY");
  {
    static char checks[2600];
    static char hw[2600];
    bool allReady = false;
    buildReadinessChecks(checks, sizeof(checks), now, &allReady);
    buildHardwareStatus(hw,sizeof(hw),now);
    jwKey(j, "readinessChecks"); jwRaw(j, checks);
    jwKey(j, "hardwareStatus"); jwRaw(j, hw);
  }
  jwBool(j, "runtimeSafetyArmed", safetyArmed);
  jwBool(j, "stall", stallState);
  jwBool(j, "criticalFault", criticalFault);
  jwStr(j, "exceptionState", exceptionState);
  if (lastControllerEventJson[0]) { jwKey(j, "lastControllerEvent"); jwRaw(j, lastControllerEventJson); } else jwNull(j, "lastControllerEvent");
  jwStr(j, "lastControllerEventSummary", lastControllerEventSummary);
  jwUInt(j, "controllerEventSequence", controllerEventSeq);
  jwUInt(j, "eventSequence", eventSeq);
  jwBool(j, "outboxAvailable", outboxMounted && outboxStorageHealthy && !outboxStorageFault);
  jwBool(j, "outboxStorageFault", outboxStorageFault);
  jwBool(j, "outboxHasCorruptRecord", outboxHasCorruptRecord);
  jwUInt(j, "outboxWriteFailures", outboxWriteFailures);
  jwUInt(j, "outboxPendingCount", countOutboxRecords());
  jwUInt(j, "outboxUsedBytes", outboxUsedBytesCached(false));
  jwUInt(j, "outboxTotalBytes", outboxMounted ? (uint32_t)LittleFS.totalBytes() : 0U);
  jwUInt(j, "outboxQuarantined", outboxQuarantined);
  jwBool(j, "outboxReformatted", outboxReformatted);
  jwBool(j, "outboxFull", outboxFull);
  jwBool(j, "wifiConnected", WiFi.status() == WL_CONNECTED);
  jwBool(j, "backendReachable", backendReachable);
  jwStr(j, "syncState", syncStateStr(now));
  jwStr(j, "controlChannel", controlChannelStr(now));
  jwBool(j, "bleEnabled", blePinConfigured());
  jwBool(j, "bleLinkActive", bleLinkAlive(now));
  jwBool(j, "bleAuthorized", bleAuthorized);
  jwUInt(j, "bootNumber", bootNumber);
  jwUInt(j, "deploySeconds", deploySecNow());
  jwBool(j, "clockValid", epochNow() != 0);
  jwBool(j, "otaReady", otaStarted);
  jwUInt(j, "timestamp", epochNow());
  jwFlt(j, "baselineCm", baselineCm, 1);
  jwFlt(j, "objectDeltaCm", objectDeltaCm, 1);
  jwFlt(j, "triggerCm", triggerCm(), 1);
  jwUInt(j, "cameraFps", camFps);
  jwUInt(j, "cameraInferMs", camInferMs);
  jwStr(j, "firmware", FW_VERSION);
  jwEnd(j);
}

static void telemetryService(uint32_t now) {
  if (wsClientCount() == 0) { telemetryDirty = false; return; }
  uint32_t age = (uint32_t)(now - lastTelemetryMs);
  if (age >= 1000UL || (telemetryDirty && age >= 250UL)) {
    lastTelemetryMs = now;
    telemetryDirty = false;
    char* buf = g_statusBuf;
    buildStatus(buf, STATUS_BUF_SIZE);
    wsBroadcast(buf);
  }
}

/* ============================== WEBSOCKET ================================= */
static uint32_t rol32(uint32_t v, int b) { return (v << b) | (v >> (32 - b)); }

static void sha1Block(uint32_t h[5], const uint8_t* p) {
  uint32_t w[80];
  for (int t = 0; t < 16; t++) {
    w[t] = ((uint32_t)p[t * 4] << 24) | ((uint32_t)p[t * 4 + 1] << 16) |
           ((uint32_t)p[t * 4 + 2] << 8) | (uint32_t)p[t * 4 + 3];
  }
  for (int t = 16; t < 80; t++) w[t] = rol32(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
  for (int t = 0; t < 80; t++) {
    uint32_t f, k;
    if (t < 20)      { f = (b & c) | ((~b) & d);          k = 0x5A827999UL; }
    else if (t < 40) { f = b ^ c ^ d;                     k = 0x6ED9EBA1UL; }
    else if (t < 60) { f = (b & c) | (b & d) | (c & d);   k = 0x8F1BBCDCUL; }
    else             { f = b ^ c ^ d;                     k = 0xCA62C1D6UL; }
    uint32_t tmp = rol32(a, 5) + f + e + k + w[t];
    e = d; d = c; c = rol32(b, 30); b = a; a = tmp;
  }
  h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

static void sha1(const uint8_t* d, size_t n, uint8_t out[20]) {
  uint32_t h[5] = { 0x67452301UL, 0xEFCDAB89UL, 0x98BADCFEUL, 0x10325476UL, 0xC3D2E1F0UL };
  uint8_t tail[128];
  size_t full = n / 64;
  for (size_t i = 0; i < full; i++) sha1Block(h, d + i * 64);
  size_t rem = n - full * 64;
  memset(tail, 0, sizeof(tail));
  if (rem) memcpy(tail, d + full * 64, rem);
  tail[rem] = 0x80;
  size_t padLen = (rem < 56) ? 64 : 128;
  uint64_t bits = (uint64_t)n * 8ULL;
  for (int i = 0; i < 8; i++) tail[padLen - 1 - i] = (uint8_t)(bits >> (8 * i));
  sha1Block(h, tail);
  if (padLen == 128) sha1Block(h, tail + 64);
  for (int i = 0; i < 5; i++) {
    out[i * 4]     = (uint8_t)(h[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(h[i] >> 8);
    out[i * 4 + 3] = (uint8_t)(h[i]);
  }
}

static void base64Encode(const uint8_t* in, size_t n, char* out) {
  static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t o = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < n) v |= (uint32_t)in[i + 2];
    out[o++] = T[(v >> 18) & 63];
    out[o++] = T[(v >> 12) & 63];
    out[o++] = (i + 1 < n) ? T[(v >> 6) & 63] : '=';
    out[o++] = (i + 2 < n) ? T[v & 63] : '=';
  }
  out[o] = '\0';
}

static const char* findCI(const char* hay, const char* needle) {
  size_t nl = strlen(needle);
  if (nl == 0) return hay;
  for (const char* p = hay; *p; p++) {
    if (strncasecmp(p, needle, nl) == 0) return p;
  }
  return nullptr;
}

static void wsClose(WsClient& w) {
  w.c.stop();
  w.st = 0;
  w.len = 0;
}

static bool wsSendFrame(WsClient& w, uint8_t opcode, const uint8_t* payload, size_t len) {
  if (w.st != 2 || len > 65535) return false;
  uint8_t hdr[4];
  size_t hl;
  hdr[0] = 0x80 | (opcode & 0x0F);
  if (len < 126) { hdr[1] = (uint8_t)len; hl = 2; }
  else { hdr[1] = 126; hdr[2] = (uint8_t)(len >> 8); hdr[3] = (uint8_t)(len & 0xFF); hl = 4; }
  if (w.c.write(hdr, hl) != hl) return false;
  if (len > 0 && w.c.write(payload, len) != len) return false;
  return true;
}

static int wsClientCount() {
  int n = 0;
  for (int i = 0; i < WS_MAX_CLIENTS; i++) if (wsc[i].st == 2) n++;
  return n;
}

static void wsBroadcast(const char* msg) {
  size_t len = strlen(msg);
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (wsc[i].st != 2) continue;
    if (!wsSendFrame(wsc[i], 0x1, (const uint8_t*)msg, len)) wsClose(wsc[i]);
  }
}

static void wsOnText(char* msg) {
  char type[24];
  if (!jsonStr(msg, "type", type, sizeof(type))) return;
  if (strcmp(type, "joystick") == 0) {
    if (bleLinkAlive(millis())) return; // one active remote-control channel
    float x = 0, y = 0, s = (float)joySens;
    jsonNum(msg, "x", &x);
    jsonNum(msg, "y", &y);
    jsonNum(msg, "sensitivity", &s);
    joystickApply(x, y, s);
  } else if (strcmp(type, "stationary") == 0) {
    if (bleLinkAlive(millis())) return;
    int v = jsonBoolish(msg, "state");
    if (v < 0) v = jsonBoolish(msg, "enabled");
    if (v >= 0) setStationary(v == 1);
  } else if (strcmp(type, "estop") == 0) {
    int v = jsonBoolish(msg, "active");
    doEstop(v != 0);
  }
}

static bool wsHandshake(WsClient& w) {
  w.buf[w.len] = 0;
  const char* req = (const char*)w.buf;
  const char* keyH = findCI(req, "sec-websocket-key:");
  if (!keyH) {
    const char* r = "HTTP/1.1 426 Upgrade Required\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
    w.c.print(r);
    wsClose(w);
    return false;
  }
  keyH += strlen("sec-websocket-key:");
  while (*keyH == ' ') keyH++;
  char key[64];
  size_t k = 0;
  while (*keyH && *keyH != '\r' && *keyH != '\n' && k < 40) key[k++] = *keyH++;
  key[k] = '\0';
  static const char GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char cat[128];
  snprintf(cat, sizeof(cat), "%s%s", key, GUID);
  uint8_t dig[20];
  sha1((const uint8_t*)cat, strlen(cat), dig);
  char acc[40];
  base64Encode(dig, 20, acc);
  char resp[220];
  snprintf(resp, sizeof(resp),
           "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
           "Sec-WebSocket-Accept: %s\r\n\r\n", acc);
  w.c.print(resp);
  w.st = 2;
  w.len = 0;
  return true;
}

static void wsProcess(WsClient& w) {
  for (;;) {
    if (w.len < 2) return;
    uint8_t b0 = w.buf[0], b1 = w.buf[1];
    bool mask = (b1 & 0x80) != 0;
    uint32_t plen = b1 & 0x7F;
    size_t hdr = 2;
    if (plen == 126) {
      if (w.len < 4) return;
      plen = ((uint32_t)w.buf[2] << 8) | w.buf[3];
      hdr = 4;
    } else if (plen == 127) {
      wsClose(w);
      return;
    }
    size_t need = hdr + (mask ? 4 : 0) + plen;
    if (need + 1 > WS_BUF) { wsClose(w); return; }
    if (w.len < need) return;
    uint8_t* mk = w.buf + hdr;
    uint8_t* pl = w.buf + hdr + (mask ? 4 : 0);
    if (mask) for (uint32_t i = 0; i < plen; i++) pl[i] ^= mk[i & 3];
    uint8_t op = b0 & 0x0F;
    if (op == 0x1) {
      pl[plen] = 0;
      wsOnText((char*)pl);
    } else if (op == 0x8) {
      wsSendFrame(w, 0x8, pl, plen > 2 ? 2 : plen);
      wsClose(w);
      return;
    } else if (op == 0x9) {
      wsSendFrame(w, 0xA, pl, plen);
    }
    memmove(w.buf, w.buf + need, w.len - need);
    w.len -= (uint16_t)need;
  }
}

static void wsService(uint32_t now) {
  // accept
  WiFiClient nc = wsServer.available();
  if (nc) {
    int slot = -1;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (wsc[i].st == 0) { slot = i; break; }
    if (slot < 0) {
      // free the oldest handshake / drop newcomer
      nc.stop();
    } else {
      wsc[slot].c = nc;
      wsc[slot].st = 1;
      wsc[slot].len = 0;
      wsc[slot].t0 = now;
      wsc[slot].c.setNoDelay(true);
    }
  }
  // service
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    WsClient& w = wsc[i];
    if (w.st == 0) continue;
    if (!w.c.connected() && w.c.available() == 0) { wsClose(w); continue; }
    while (w.c.available() > 0 && w.len < WS_BUF - 1) {
      int ch = w.c.read();
      if (ch < 0) break;
      w.buf[w.len++] = (uint8_t)ch;
    }
    if (w.st == 1) {
      w.buf[w.len] = 0;
      if (strstr((const char*)w.buf, "\r\n\r\n")) wsHandshake(w);
      else if (w.len >= WS_BUF - 2 || (uint32_t)(now - w.t0) > 4000UL) wsClose(w);
    } else if (w.st == 2) {
      if (w.len >= WS_BUF - 1 && w.c.available() > 0) {
        wsProcess(w);
        if (w.st == 2 && w.len >= WS_BUF - 1) wsClose(w);
      } else {
        wsProcess(w);
      }
    }
  }
  // safety: the last dashboard link dropped -> stop the boat
  int cnt = wsClientCount();
  if (wsHadClients && cnt == 0) {
    hardStopPropulsion();
    Serial.println("[WS] last client left - propulsion stopped");
  }
  wsHadClients = (cnt > 0);
}


/* ======================= CLOUD IP DISCOVERY ================================ */
static String discoveryJsonEscape(const String& value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char ch = value[i];
    switch (ch) {
      case '\\': out += "\\\\"; break;
      case '"':  out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += ch; break;
    }
  }
  return out;
}

static void ensureDiscoveryBootId() {
  if (discoveryBootId.length() > 0) return;
  uint64_t chip = ESP.getEfuseMac();
  char b[64];
  snprintf(b, sizeof(b), "%08lX-%08lX-%08lX-%08lX",
           (unsigned long)(chip >> 32),
           (unsigned long)(chip & 0xFFFFFFFFULL),
           (unsigned long)esp_random(),          // unique per boot (millis() alone repeats between boots)
           (unsigned long)millis());
  discoveryBootId = String(b);
}

static bool publishDiscoveryHeartbeat(bool force) {
  if (WiFi.status() != WL_CONNECTED) return false;

  const uint32_t now = millis();
  if (!force && lastDiscoveryPublishMs != 0 &&
      (uint32_t)(now - lastDiscoveryPublishMs) < SIBOAT_DISCOVERY_INTERVAL_MS) {
    return discoveryEverPublished;
  }
  lastDiscoveryPublishMs = now;
  ensureDiscoveryBootId();

  IPAddress ip = WiFi.localIP();
  const String ipText = ip.toString();
  const String ssidText = WiFi.SSID();
  const String macText = WiFi.macAddress();

  String payload = "{";
  payload += "\"p_device_id\":\"" + discoveryJsonEscape(SIBOAT_DISCOVERY_DEVICE_ID) + "\",";
  payload += "\"p_device_type\":\"main\",";
  payload += "\"p_local_ip\":\"" + discoveryJsonEscape(ipText) + "\",";
  payload += "\"p_http_port\":80,\"p_ws_port\":81,";
  payload += "\"p_firmware_version\":\"" + discoveryJsonEscape(FW_VERSION) + "\",";
  payload += "\"p_ssid\":\"" + discoveryJsonEscape(ssidText) + "\",";
  payload += "\"p_mac\":\"" + discoveryJsonEscape(macText) + "\",";
  payload += "\"p_boot_id\":\"" + discoveryJsonEscape(discoveryBootId) + "\",";
  payload += "\"p_camera_ip\":\"" + discoveryJsonEscape(String(camIp)) + "\"}";

  const bool queued = enqueueCloudRpc(CLOUD_JOB_DISCOVERY, SIBOAT_DISCOVERY_RPC_PATH, payload);
  if (queued) {
    Serial.printf("[DISCOVERY] queued %s at %s:80 (WS 81) for background registration\n",
                  SIBOAT_DISCOVERY_DEVICE_ID, ipText.c_str());
  }
  return queued || discoveryEverPublished;
}

static void discoveryService(uint32_t now) {
  (void)now;
  if (WiFi.status() != WL_CONNECTED) return;
  publishDiscoveryHeartbeat(false);
}

/* ================================ REST API ================================ */
static void addCors() {
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  web.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  web.sendHeader("Access-Control-Allow-Private-Network", "true");
  web.sendHeader("Access-Control-Max-Age", "600");
  web.sendHeader("Cache-Control", "no-store");
}

static void sendJson(int code, const char* body) {
  addCors();
  web.send(code, "application/json", body);
}

static void sendOk() { sendJson(200, "{\"ok\":true}"); }

static void sendFail(int code, const char* why) {
  char b[120];
  snprintf(b, sizeof(b), "{\"ok\":false,\"error\":\"%s\"}", why);
  sendJson(code, b);
}

// true = request already answered (CORS preflight)
static bool preflight() {
  if (web.method() == HTTP_OPTIONS) {
    addCors();
    web.send(204, "text/plain", "");
    return true;
  }
  lastApiMs = millis();
  if (lastApiMs == 0) lastApiMs = 1;
  return false;
}

static void hStatus() {
  if (preflight()) return;
  char* buf = g_statusBuf;
  buildStatus(buf, STATUS_BUF_SIZE);
  sendJson(200, buf);
}

static void hReadiness() {
  if (preflight()) return;
  char* buf = g_statusBuf;
  buildStatus(buf, STATUS_BUF_SIZE);
  sendJson(200, buf);
}

static void hRoot() {
  if (preflight()) return;
  sendJson(200, "{\"ok\":true,\"device\":\"SIBOAT MAIN ESP32\",\"api\":\"/api/status\",\"readiness\":\"/api/readiness\",\"ws\":\"ws://<ip>:81/ws\"}");
}

static void hDevices() {
  if (preflight()) return;
  bool camOnline = camSeen && ((uint32_t)(millis() - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  int count = 1 + (camOnline ? 1 : 0) + wsClientCount();
  char b[80];
  snprintf(b, sizeof(b), "{\"ok\":true,\"count\":%d,\"camera\":%s}", count, camOnline ? "true" : "false");
  sendJson(200, b);
}

static void hJoystick() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  float x = 0, y = 0, s = (float)joySens;
  jsonNum(body.c_str(), "x", &x);
  jsonNum(body.c_str(), "y", &y);
  jsonNum(body.c_str(), "sensitivity", &s);
  joystickApply(x, y, s);
  if (propAllowed()) {
    if (x != 0.0f || y != 0.0f) safetyArmed = true;
    recordControllerEvent("DASHBOARD", "COMMAND", "Propulsion", "JOYSTICK", "APPLIED", "Joystick target accepted");
    sendOk();
  } else {
    const char* why = stationaryMode ? "STATIONARY MODE" : (safeMode ? safeReasonStr() : (!operationalReady ? "NOT OPERATIONALLY READY" : "PROPULSION BLOCKED"));
    if (x != 0.0f || y != 0.0f) recordControllerEvent("DASHBOARD", "COMMAND", "Propulsion", "JOYSTICK", "REJECTED", why);
    char rejected[160];
    snprintf(rejected, sizeof(rejected), "{\"ok\":false,\"applied\":false,\"error\":\"%s\"}", why);
    sendJson(409, rejected);
  }
}

static void hJoySpeed() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  float v;
  if (!jsonNum(body.c_str(), "value", &v)) { sendFail(400, "value missing"); return; }
  joySpeed = (int)constrain((int)v, 0, 100);
  savePrefsSoon();
  sendOk();
}

static void hJoySens() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  float v;
  if (!jsonNum(body.c_str(), "value", &v)) { sendFail(400, "value missing"); return; }
  joySens = (int)constrain((int)v, 0, 100);
  savePrefsSoon();
  sendOk();
}

static void hConveyor() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "state");
  if (v < 0) { sendFail(400, "state must be ON or OFF"); return; }
  if (v == 1) {
    if (!conveyorAllowed()) { const char* why = safeMode ? safeReasonStr() : (!powerOn ? "POWER OFF" : (estop ? "EMERGENCY STOP" : "NOT OPERATIONALLY READY")); recordControllerEvent("DASHBOARD", "COMMAND", "Conveyor", "ON", "REJECTED", why); sendFail(409, why[0] ? why : "not operationally ready"); return; }
    conveyorOn = true; safetyArmed = true;
    recordControllerEvent("DASHBOARD", "COMMAND", "Conveyor", "ON", "APPLIED", "Conveyor command path enabled");
  } else {
    conveyorOn = false;
    stationaryAutoStartPending = false;
    recordControllerEvent("DASHBOARD", "COMMAND", "Conveyor", "OFF", "APPLIED", "Conveyor command path disabled");
  }
  updateTargets();
  markDirty();
  sendJson(200, conveyorOn ? "{\"ok\":true,\"state\":\"ON\"}" : "{\"ok\":true,\"state\":\"OFF\"}");
}

static void hConveyorSpeed() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  float v;
  if (!jsonNum(body.c_str(), "value", &v)) { sendFail(400, "value missing"); return; }
  conveyorSpeed = (int)constrain((int)v, 0, 100);
  updateTargets();
  savePrefsSoon();
  sendOk();
}

static void hClassification() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "state");
  if (v < 0) { sendFail(400, "state must be ON or OFF"); return; }
  if (v == 1) {
    if (!operationAllowed()) { const char* why = safeMode ? safeReasonStr() : (!powerOn ? "POWER OFF" : (estop ? "EMERGENCY STOP" : "NOT OPERATIONALLY READY")); recordControllerEvent("DASHBOARD", "COMMAND", "Classification", "ON", "REJECTED", why); sendFail(409, why[0] ? why : "not operationally ready"); return; }
    classificationEnabled = true; safetyArmed = true;
    recordControllerEvent("DASHBOARD", "COMMAND", "Classification", "ON", "APPLIED", "Classification command path enabled");
  } else {
    classificationEnabled = false;
    stationaryAutoStartPending = false;
    recordControllerEvent("DASHBOARD", "COMMAND", "Classification", "OFF", "APPLIED", "Classification command path disabled");
  }
  markDirty();
  sendJson(200, classificationEnabled ? "{\"ok\":true,\"state\":\"ON\"}" : "{\"ok\":true,\"state\":\"OFF\"}");
}

static void hLightsOutside() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "state");
  if (v >= 0) outsideLights = (v == 1);
  markDirty();
  sendOk();
}

static void hLightsInside() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "state");
  if (v >= 0) insideLights = (v == 1);
  markDirty();
  sendOk();
}

static void hDeveloperVerify() {
  if (preflight()) return;
  sendJson(200, "{\"ok\":true,\"verified\":false}");
}

static void hPriority() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  sendOk();
}

static void hEstop() {
  if (preflight()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "active");
  doEstop(v != 0);
  sendOk();
}

static void hRestart() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  stopAllOutputs();
  pendingRestartAt = millis() + 800UL;
  if (pendingRestartAt == 0) pendingRestartAt = 1;
  sendOk();
}

static void hPower() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  String body = web.arg("plain");
  int v = jsonBoolish(body.c_str(), "state");
  if (v == 0) {
    stopAllOutputs();
    powerOn = false;
    operationalReady = false; readinessHealthySince = 0; safetyArmed = false;
    recordControllerEvent("DASHBOARD", "COMMAND", "Power", "OFF", "APPLIED", "Outputs stopped and controller powered off state");
  } else {
    powerOn = true;
    operationalReady = false; readinessHealthySince = 0;
    recordControllerEvent("DASHBOARD", "COMMAND", "Power", "ON", "APPLIED", "Power state on; readiness check required");
  }
  markDirty();
  sendOk();
}

static void hShutdown() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  stopAllOutputs();
  powerOn = false;
  operationalReady = false; readinessHealthySince = 0; safetyArmed = false;
  recordControllerEvent("DASHBOARD", "COMMAND", "Power", "SHUTDOWN", "APPLIED", "Outputs stopped");
  markDirty();
  sendOk();
}

static void hStationary() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  if (web.method() == HTTP_POST) {
    String body = web.arg("plain");
    int v = jsonBoolish(body.c_str(), "state");
    if (v < 0) v = jsonBoolish(body.c_str(), "enabled");
    if (v < 0) v = jsonBoolish(body.c_str(), "stationaryMode");
    if (v < 0) { sendFail(400, "state must be ON or OFF"); return; }
    setStationary(v == 1);
  }
  sendJson(200, stationaryMode ? "{\"ok\":true,\"stationaryMode\":true}" : "{\"ok\":true,\"stationaryMode\":false}");
}

static void hSafeClear() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  latchedFaults = 0;
  markDirty();
  sendOk();
}

static void hCounters() {
  if (preflight()) return;
  char b[240];
  Jw j;
  jwInit(j, b, sizeof(b));
  jwBool(j, "ok", true);
  jwUInt(j, "cat1Count", cat1Count);
  jwUInt(j, "cat2Count", cat2Count);
  jwUInt(j, "cat3Count", cat3Count);
  jwUInt(j, "rejectedCount", rejectedCount);
  jwUInt(j, "classificationTimeoutCount", timeoutCount);
  jwUInt(j, "eventSequence", eventSeq);
  jwEnd(j);
  sendJson(200, b);
}

static void hCountersReset() {
  if (preflight()) return;
  cat1Count = cat2Count = cat3Count = rejectedCount = timeoutCount = 0;
  uartRetryCount = uartErrorCount = 0;
  saveCounters();
  markDirty();
  sendOk();
}

static void hEvents() {
  if (preflight()) return;
  addCors();
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "application/json", "");
  web.sendContent("{\"ok\":true,\"events\":[");
  char buf[640];
  for (uint8_t k = 0; k < evtCount; k++) {
    int idx = (int)evtHead - 1 - (int)k;       // newest first
    while (idx < 0) idx += EVT_MAX;
    eventToJson(evts[idx], buf, sizeof(buf), false);
    if (k > 0) web.sendContent(",");
    web.sendContent(buf);
  }
  web.sendContent("]}");
  web.sendContent("");
}

static void hHcsr04() {
  if (preflight()) return;
  if (segState != SEG_IDLE) { sendFail(409, "segregation active"); return; }
  float d = readDistanceCm();
  char b[160];
  Jw j;
  jwInit(j, b, sizeof(b));
  jwBool(j, "ok", d >= 0.0f);
  if (d >= 0.0f) { lastDistCm = d; jwFlt(j, "distanceCm", d, 1); } else jwNull(j, "distanceCm");
  jwFlt(j, "baselineCm", baselineCm, 1);
  jwFlt(j, "triggerCm", triggerCm(), 1);
  jwEnd(j);
  sendJson(200, b);
}

static void hHcsr04Calibrate() {
  if (preflight()) return;
  if (segState != SEG_IDLE) { sendFail(409, "segregation active"); return; }
  float sum = 0.0f;
  int good = 0;
  for (int i = 0; i < 10; i++) {
    float d = readDistanceCm();
    if (d > 0.0f) { sum += d; good++; }
    delay(70);
  }
  if (good < 5) { sendFail(500, "sensor reading failed"); return; }
  baselineCm = sum / good;
  lastDistCm = baselineCm;
  saveSettings();
  markDirty();
  char b[120];
  Jw j;
  jwInit(j, b, sizeof(b));
  jwBool(j, "ok", true);
  jwFlt(j, "baselineCm", baselineCm, 2);
  jwFlt(j, "triggerCm", triggerCm(), 2);
  jwEnd(j);
  sendJson(200, b);
}

static void hConfig() {
  if (preflight()) return;
  if (httpControlBlockedByBle()) return;
  if (web.method() == HTTP_POST) {
    String body = web.arg("plain");
    float v;
    if (jsonNum(body.c_str(), "objectDeltaCm", &v)) { if (v >= 0.3f && v <= 15.0f) objectDeltaCm = v; }
    if (jsonNum(body.c_str(), "baselineCm", &v))    { if (v >= 5.0f && v <= 300.0f) baselineCm = v; }
    saveSettings();
    markDirty();
  }
  char b[160];
  Jw j;
  jwInit(j, b, sizeof(b));
  jwBool(j, "ok", true);
  jwFlt(j, "baselineCm", baselineCm, 2);
  jwFlt(j, "objectDeltaCm", objectDeltaCm, 2);
  jwFlt(j, "triggerCm", triggerCm(), 2);
  jwEnd(j);
  sendJson(200, b);
}

static void hNotFound() {
  if (web.method() == HTTP_OPTIONS) {
    addCors();
    web.send(204, "text/plain", "");
    return;
  }
  sendFail(404, "not found");
}

static void registerRoutes() {
  web.on("/", HTTP_ANY, hRoot);
  web.on("/api/status", HTTP_ANY, hStatus);
  web.on("/api/readiness", HTTP_ANY, hReadiness);
  web.on("/api/devices", HTTP_ANY, hDevices);
  web.on("/api/joystick", HTTP_ANY, hJoystick);
  web.on("/api/joystick/speed", HTTP_ANY, hJoySpeed);
  web.on("/api/joystick/sensitivity", HTTP_ANY, hJoySens);
  web.on("/api/conveyor", HTTP_ANY, hConveyor);
  web.on("/api/conveyor/speed", HTTP_ANY, hConveyorSpeed);
  web.on("/api/classification", HTTP_ANY, hClassification);
  web.on("/api/lights/outside", HTTP_ANY, hLightsOutside);
  web.on("/api/lights/inside", HTTP_ANY, hLightsInside);
  web.on("/api/developer/verify", HTTP_ANY, hDeveloperVerify);
  web.on("/api/priority/request", HTTP_ANY, hPriority);
  web.on("/api/priority/release", HTTP_ANY, hPriority);
  web.on("/api/estop", HTTP_ANY, hEstop);
  web.on("/api/restart", HTTP_ANY, hRestart);
  web.on("/api/power", HTTP_ANY, hPower);
  web.on("/api/shutdown", HTTP_ANY, hShutdown);
  web.on("/api/stationary", HTTP_ANY, hStationary);
  web.on("/api/safemode/clear", HTTP_ANY, hSafeClear);
  web.on("/api/counters", HTTP_ANY, hCounters);
  web.on("/api/counters/reset", HTTP_ANY, hCountersReset);
  web.on("/api/events", HTTP_ANY, hEvents);
  web.on("/api/hcsr04", HTTP_ANY, hHcsr04);
  web.on("/api/hcsr04/calibrate", HTTP_ANY, hHcsr04Calibrate);
  web.on("/api/config", HTTP_ANY, hConfig);
  web.onNotFound(hNotFound);
}

/* ================================== WI-FI ================================= */
static void otaSetup() {
  if (otaStarted) return;
  ArduinoOTA.setHostname(MAIN_HOSTNAME);
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setMdnsEnabled(false);            // siboat.local is advertised by the sketch itself
#ifdef SIBOAT_OTA_PASSWORD
  ArduinoOTA.setPassword(SIBOAT_OTA_PASSWORD);
#endif
  ArduinoOTA.onStart([]() {
    otaBusy = true;
    stopAllOutputs();                          // freeze motors and servos before flash is rewritten
    Serial.println("[OTA] update starting; all outputs stopped");
  });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] update finished; restarting"); });
  ArduinoOTA.onError([](ota_error_t error) {
    otaBusy = false;
    Serial.printf("[OTA] error %u; the controller keeps running\n", (unsigned)error);
  });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("[OTA] ready: %s.local / %s port 3232 (needs an OTA-capable partition scheme)\n", MAIN_HOSTNAME, WiFi.localIP().toString().c_str());
}

static void onWifiUp() {
  IPAddress ip = WiFi.localIP();
  Serial.printf("[WIFI] connected to '%s'  IP %u.%u.%u.%u  RSSI %d\n", WIFI_CREDS[wCred].ssid,
                (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3], WiFi.RSSI());
  if (!(ip == MAIN_STATIC_IP)) {
    Serial.println("[WIFI] NOTE: not on 10.90.102.50 - dashboard must use the address above");
  }
  MDNS.end();
  if (MDNS.begin(MAIN_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
#ifdef SIBOAT_OTA_PASSWORD
    MDNS.enableArduino(3232, true);
#else
    MDNS.enableArduino(3232, false);
#endif
    Serial.println("[MDNS] siboat.local ready");
  }
  otaSetup();
  perfFailStreak = 0; backendFailStreak = 0; lastPerformanceEventAttemptMs = 0;   // a fresh link retries immediately
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  if (!serversStarted) {
    web.begin();
    wsServer.begin();
    serversStarted = true;
    Serial.println("[NET] REST :80 and WebSocket :81/ws started");
  }
  publishDiscoveryHeartbeat(true);
  markDirty();
}

static void wifiBegin(int cred, bool useStatic) {
  cred = nextWifiCred(cred);
  if (cred < 0) {
    wState = W_BACKOFF;
    wNext = millis() + 2000UL;
    Serial.println("[WIFI] no configured Wi-Fi profiles are available");
    return;
  }
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_STA);
  if (useStatic && haveLearned) WiFi.config(MAIN_STATIC_IP, learnedGw, learnedMask, learnedGw);
  else WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  WiFi.setHostname(MAIN_HOSTNAME);
  WiFi.begin(WIFI_CREDS[cred].ssid, WIFI_CREDS[cred].pass);
  wCred = cred;
  wUseStatic = useStatic && haveLearned;
  wStart = millis();
  wState = W_CONNECTING;
}

static void wifiService(uint32_t now) {
  wl_status_t st = WiFi.status();

  if (wState == W_UP) {
    if (st != WL_CONNECTED) {
      wState = W_BACKOFF;
      wNext = now + 300UL;
      wifiLostCount++;
      Serial.println("[WIFI] link lost - recovering (local safety keeps running)");
      markDirty();
    }
    return;
  }

  if (wState == W_BACKOFF) {
    if ((int32_t)(now - wNext) >= 0) wifiBegin(wCred, haveLearned && !staticFailed);
    return;
  }

  // W_CONNECTING
  if (st == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    if (!wUseStatic && !staticFailed) {
      bool sameNet = (ip[0] == MAIN_STATIC_IP[0]) && (ip[1] == MAIN_STATIC_IP[1]) && (ip[2] == MAIN_STATIC_IP[2]);
      if (sameNet && !(ip == MAIN_STATIC_IP)) {
        learnedGw = WiFi.gatewayIP();
        learnedMask = WiFi.subnetMask();
        haveLearned = true;
        wifiBegin(wCred, true);       // re-join with the fixed address
        return;
      }
    }
    wState = W_UP;
    onWifiUp();
    return;
  }

  uint32_t el = now - wStart;
  bool bad = (st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED);
  if ((bad && el > 2500UL) || el > 10000UL) {
    if (wUseStatic) {
      staticFailed = true;            // fixed address refused: stay on DHCP
      Serial.println("[WIFI] static IP failed - falling back to DHCP");
      wifiBegin(wCred, false);
      return;
    }
    int nextCred = nextWifiCred((wCred + 1) % WIFI_CRED_COUNT);
    if (nextCred < 0) {
      wState = W_BACKOFF;
      wNext = now + 2000UL;
      return;
    }
    wCred = nextCred;
    staticFailed = false;
    haveLearned = false;
    wState = W_BACKOFF;
    wNext = now + ((wCred == 0) ? 2000UL : 50UL);
  }
}

static void stationaryAutoStartService(uint32_t now) {
  (void)now;
  if (!stationaryMode) { stationaryAutoStartPending = false; return; }
  if (!stationaryAutoStartPending || !operationAllowed()) return;
  const bool camReady = camSeen && camHbOk && ((uint32_t)(millis() - lastHbMs) <= CAM_HB_TIMEOUT_MS);
  const bool sr04Fresh = hcSr04Ready && lastSr04ValidMs != 0 && ((uint32_t)(millis() - lastSr04ValidMs) <= READINESS_SR04_STALE_MS);
  if (!powerOn || estop || safeMode || latchedFaults != 0 || !camReady || !sr04Fresh || segState != SEG_IDLE) return;

  hardStopPropulsion();
  conveyorOn = true;
  classificationEnabled = true;
  safetyArmed = true;
  stationaryAutoStartPending = false;
  recordControllerEvent("MAIN", "MODE", "Stationary Operation", "AUTO_START", "APPLIED",
                        "Stationary mode started onboard collection/classification after local readiness checks; no dashboard or external network required");
  Serial.println("[MODE] stationary autonomous collection/classification started after local readiness checks");
  markDirty();
}

/* ============================ SETUP AND LOOP ============================== */
static void setupPins() {
  // enables first, so a driver can never wake up energised
  pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, LOW);
  pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, LOW);
  pinMode(PIN_C_EN, OUTPUT); digitalWrite(PIN_C_EN, LOW);
  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  const uint8_t pwmPins[6] = { PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM, PIN_C_RPWM, PIN_C_LPWM };
  const uint8_t pwmCh[6]   = { CH_L_R, CH_L_L, CH_R_R, CH_R_L, CH_C_R, CH_C_L };
  pwmTimerInit(LEDC_TIMER_0, MOTOR_PWM_HZ, MOTOR_PWM_BITS);   // motors: 20 kHz, 8-bit
  pwmTimerInit(LEDC_TIMER_1, 50, 14);                          // servos: 50 Hz, 14-bit
  for (int i = 0; i < 6; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);
    pwmChannelInit(pwmCh[i], pwmPins[i], LEDC_TIMER_0);
  }
  pinMode(PIN_SERVO1, OUTPUT);
  pinMode(PIN_SERVO2, OUTPUT);
  pwmChannelInit(CH_SV1, PIN_SERVO1, LEDC_TIMER_1);
  pwmChannelInit(CH_SV2, PIN_SERVO2, LEDC_TIMER_1);
  servoHardZero();           // both flaps to 0 degrees at boot
  servoPathReady[0] = true;
  servoPathReady[1] = true;
  motorDriverPathReady[0] = true;
  motorDriverPathReady[1] = true;
  motorDriverPathReady[2] = true;
}

void setup() {
  setupPins();

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== SIBOAT MAIN ESP32 firmware " FW_VERSION " ===");

  cloudRpcQueue = xQueueCreate(CLOUD_RPC_QUEUE_DEPTH, sizeof(CloudRpcJob*));
  if (cloudRpcQueue) {
    const BaseType_t taskCreated = xTaskCreatePinnedToCore(
      cloudRpcWorker, "siboat-cloud-rpc", CLOUD_RPC_TASK_STACK,
      nullptr, 1, &cloudRpcTaskHandle, 0);
    if (taskCreated != pdPASS) {
      vQueueDelete(cloudRpcQueue);
      cloudRpcQueue = nullptr;
      cloudRpcTaskHandle = nullptr;
      Serial.println("[CLOUD] background worker unavailable; cloud sync disabled (local control remains active)");
    } else {
      Serial.println("[CLOUD] non-blocking Supabase worker started on Core 0");
    }
  } else {
    Serial.println("[CLOUD] queue allocation failed; cloud sync disabled (local control remains active)");
  }

  // Restore persisted counters and the boot/session identifier before touching
  // the filesystem queue. This prevents a new event from reusing a pending filename.
  loadPrefs();
  bootNumber = prefs.getUInt("bn", 0) + 1U;
  prefs.putUInt("bn", bootNumber);
  deployBaseSec = prefs.getUInt("dep", 0);
  ensureDiscoveryBootId();
  initializeOutbox();
  setupBleControl();
  stationaryAutoStartPending = stationaryMode;
  recordControllerEvent("MAIN", "BOOT", "ESP32 Main Controller", "STARTUP", "INITIALIZING", "Controller booted; local readiness checklist pending");
  Serial.printf("[BOOT] counters CAT1=%lu CAT2=%lu CAT3=%lu REJ=%lu seq=%lu  stationary=%s\n",
                (unsigned long)cat1Count, (unsigned long)cat2Count, (unsigned long)cat3Count,
                (unsigned long)rejectedCount, (unsigned long)eventSeq, stationaryMode ? "ON" : "OFF");
  Serial.printf("[BOOT] HC-SR04 baseline %.1f cm, trigger %.1f cm\n", (double)baselineCm, (double)triggerCm());

  // NOTE: never memset() the wsc[] array - it holds WiFiClient objects (virtual
  // functions) and wiping them crashes the board when the dashboard connects.
  for (int i = 0; i < WS_MAX_CLIENTS; i++) { wsc[i].st = 0; wsc[i].len = 0; }

  Serial2.setRxBufferSize(512);
  Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  registerRoutes();
  wState = W_BACKOFF;
  wNext = millis();

  controllerInitialized = true;
  Serial.println(stationaryMode ? "[BOOT] outputs safe; stationary auto-start pending local camera/sensor readiness (network optional)" : "[BOOT] outputs safe, remote readiness checklist active; waiting for local control link and camera");
}

void loop() {
  uint32_t now = millis();

  wifiService(now);
  if (serversStarted && wState == W_UP) {
    if (otaStarted) ArduinoOTA.handle();
    web.handleClient();
    wsService(now);
    discoveryService(now);
    runtimeCloudService(now);
  }
  uartPoll();
  readinessService(now);
  safetyService(now);
  stationaryAutoStartService(now);
  segregationService(now);
  servoService(now);
  motorService(now);
  stService(now);
  telemetryService(now);
  retryOutboxWriteService(now);
  bleCommandService(now);
  bleTxService(now);
  deployService(now);
  prefsService(now);

  if (pendingRestartAt != 0 && (int32_t)(now - pendingRestartAt) >= 0) {
    Serial.println("[SYS] restarting");
    delay(50);
    ESP.restart();
  }
  delay(1);
}
