/* ===========================================================================
   SIBOAT - ESP32-CAM SUPER FINAL FIRMWARE  (AI Thinker ESP32-CAM, OV3660, PSRAM)
   ---------------------------------------------------------------------------
   Board ........ AI Thinker ESP32-CAM      Core ...... Arduino-ESP32 2.x / 3.x
   Model ........ Edge Impulse FOMO int8 96x96 RGB, 22 labels
                  (library header: siboat_inferencing.h)
   Board settings: PSRAM = Enabled (required) and, if "Sketch too big" appears,
                   Partition Scheme = Huge APP (3MB).

   WHAT THIS SKETCH DOES
   - Joins the phone hotspot, static IP 10.90.102.51 (DHCP fallback).
   - MJPEG stream  http://10.90.102.51:81/stream   (QVGA 320x240, ~15 FPS).
     Also  /snapshot (one JPEG)  and  /  (text status).
   - FreeRTOS: camera+stream tasks on Core 1, AI inference task on Core 0.
     Every 5th captured frame is handed to the AI task.
   - Confidence >= 0.50, best (highest-confidence) valid object wins,
     2 consecutive samples of the same machine class (CAT1/CAT2/CAT3/REJECT)
     are required before anything is transmitted.
   - Sends UART packets to the Main ESP32 (115200, TX=GPIO14, RX=GPIO15):
        CAM -> MAIN  <CAT,1|2|3,0.84,SEQ,17,label>   (label = optional extra)
        CAM -> MAIN  <REJ,0.84,SEQ,20,label>
        CAM -> MAIN  <HB,ok,ip,fps,inferMs>           every 1000 ms
        MAIN -> CAM  ACK1 / ACK2 / ACK3 / REJECT / ERROR
        MAIN -> CAM  <ST,classificationEnabled,safeMode>   every ~1 s
   - ACK timeout 3 s, max 5 transmissions, retry cooldown 2 s.
   - Duplicate lockout 2 s, new-object delay 1 s after object disappears.
   - Classification only runs while the Main ESP32 says it is enabled.
   =========================================================================== */

// Edge Impulse library: the SIBOAT deployment (header siboat_inferencing.h).
// ArduinoDroid: Libraries > Import ZIP with the SIBOAT Edge Impulse Arduino ZIP.
#if defined(__has_include)
  #if !__has_include(<siboat_inferencing.h>)
    #error "siboat_inferencing.h not found - import the SIBOAT Edge Impulse Arduino library ZIP first."
  #endif
#endif
#include <siboat_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <strings.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#if defined(__has_include)
  #if __has_include("esp_random.h")
    #include "esp_random.h"
  #endif
#endif

/* =============================== USER CONFIG ============================== */

// Wi-Fi networks are tried in this order. Delete the ones you do not need.
struct WifiCred {
  const char* ssid;
  const char* pass;
};
static const WifiCred WIFI_CREDS[] = {
  { "SIBOAT",     "siboattrashcollector" },
  { "SIBOAT.DEV", "siboatdevteam" },
  { "siboat.dev", "siboatdevteam" }
};
static const int WIFI_CRED_COUNT = sizeof(WIFI_CREDS) / sizeof(WIFI_CREDS[0]);

static const IPAddress CAM_STATIC_IP(10, 90, 102, 51);
static const IPAddress CAM_DEFAULT_GW(10, 90, 102, 1);      // only used if hotspot gateway cannot be learned
static const IPAddress CAM_DEFAULT_MASK(255, 255, 255, 0);

#define CAM_HOSTNAME          "siboat-cam"
#define STREAM_PORT           81

// Image orientation (driver calls). OV3660 modules deliver an upside-down
// picture unless vflip=1, so 1 here gives an UPRIGHT image (= "no flip" as seen).
// If your stream looks upside-down set CAM_VFLIP to 0. Mirror: NO.
#define CAM_VFLIP             1
#define CAM_HMIRROR           0

#define JPEG_QUALITY          12
#define TARGET_FPS            15
#define FRAME_INTERVAL_MS     (1000 / TARGET_FPS)
#define AI_EVERY_N_FRAMES     5
#define FRAME_BUF_SIZE        (48 * 1024)

#define CONF_THRESHOLD        0.50f
#define CONSEC_REQUIRED       2
#define DUP_LOCKOUT_MS        2000UL
#define NEW_OBJECT_DELAY_MS   1000UL

#define UART_BAUD             115200
#define UART_TX_PIN           14
#define UART_RX_PIN           15
#define ACK_TIMEOUT_MS        3000UL
#define MAX_TX_ATTEMPTS       5
#define RETRY_COOLDOWN_MS     2000UL
#define HB_INTERVAL_MS        1000UL
#define MAIN_CMD_TIMEOUT_MS   5000UL

#define MAX_STREAM_CLIENTS    2

/* ============================ AI THINKER PINS ============================= */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

/* ========================= LABEL -> MACHINE CLASS ========================= */
// 1 = CAT1 Plastic Bottles & Containers, 2 = CAT2 Soft Plastics,
// 3 = CAT3 Biodegradables, 9 = REJECT (log only)
static const char* const LBL_NAME[] = {
  "Face-Mask", "PaperBag", "Plastic-Bag", "Plastic-Bottle", "aluminum",
  "cardboard", "egg shell", "facemask", "food wrapper", "fruit peels",
  "glass bottle", "left-over food", "paper", "pet bottle", "plastic bag",
  "plastic bottle", "plastic container", "plastic sachet", "plastic straw",
  "styrofoam containers", "treeleaves", "vegetable peels"
};
static const uint8_t LBL_CLS[] = {
  2, 3, 2, 1, 9,
  3, 3, 2, 2, 3,
  9, 9, 3, 1, 2,
  1, 1, 2, 2,
  2, 3, 3
};
static const int LBL_COUNT = sizeof(LBL_CLS) / sizeof(LBL_CLS[0]);

#define CLS_NONE    0
#define CLS_REJECT  9

/* ================================ TYPES =================================== */
struct TxMsg {
  uint8_t  cls;
  float    conf;
  uint32_t seq;
  char     label[32];
};

struct StreamSlot {
  WiFiClient c;
  uint8_t    mode;       // 0 empty, 1 waiting for request line, 2 streaming
  uint32_t   t0;
  char       req[96];
  uint8_t    reqLen;
};

/* ================================ GLOBALS ================================= */
#define AI_IDLE     0
#define AI_PENDING  1
#define AI_BUSY     2

#define TX_IDLE      0
#define TX_WAIT_ACK  1
#define TX_COOLDOWN  2

static volatile bool     camReady    = false;
static volatile bool     camFault    = false;
static volatile bool     modelFault  = false;
static volatile uint32_t frameCounter = 0;
static volatile uint32_t frameSeq     = 0;
static volatile uint32_t fpsValue     = 0;
static volatile uint32_t lastInferMs  = 0;

static uint8_t* frameBuf   = NULL;
static size_t   frameLen   = 0;
static uint8_t* aiJpeg     = NULL;
static volatile size_t aiJpegLen = 0;
static uint8_t* snapshotBuf = NULL;
static SemaphoreHandle_t frameMutex = NULL;
static volatile uint8_t aiState = AI_IDLE;

static QueueHandle_t txQueue = NULL;
static volatile uint8_t  txInFlight = 0;
static volatile uint32_t txDoneMs   = 0;

// Commands from Main
static volatile bool     stSeen   = false;
static volatile bool     mainCls  = false;
static volatile bool     mainSafe = true;
static volatile uint32_t lastStMs = 0;

// UART line parser
static char    rxBuf[160];
static uint8_t rxLen = 0;
static bool    rxInFrame = false;

// TX state machine
static uint8_t  txState = TX_IDLE;
static char     txPkt[160];
static char     txExpect[8];
static uint8_t  txAttempts = 0;
static uint32_t txDeadline = 0;
static uint32_t txSeqCounter = 1;

static uint32_t lastHbMs = 0;
static WiFiServer streamServer(STREAM_PORT);
static StreamSlot slots[MAX_STREAM_CLIENTS];
static volatile bool wifiUp = false;
static volatile bool serverStarted = false;

/* ============================== SMALL HELPERS ============================= */
static void* psAlloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = malloc(n);
  return p;
}

static void sanitizeLabel(char* dst, const char* src, size_t n) {
  size_t i = 0;
  if (n == 0) return;
  while (src && src[i] && i < n - 1) {
    char ch = src[i];
    bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
              (ch >= '0' && ch <= '9') || ch == ' ' || ch == '-' || ch == '_';
    dst[i] = ok ? ch : '_';
    i++;
  }
  dst[i] = '\0';
}

static uint8_t classOfLabel(const char* label) {
  for (int i = 0; i < LBL_COUNT; i++) {
    if (strcasecmp(label, LBL_NAME[i]) == 0) return LBL_CLS[i];
  }
  return CLS_NONE;
}

static void ipToStr(char* out, size_t n) {
  IPAddress ip = WiFi.localIP();
  snprintf(out, n, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

/* =============================== CAMERA =================================== */
static bool cameraInit() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  config.pin_sccb_sda = SIOD_GPIO_NUM;      // core 3.x (IDF 5.x)
  config.pin_sccb_scl = SIOC_GPIO_NUM;
#else
  config.pin_sscb_sda = SIOD_GPIO_NUM;      // core 2.x
  config.pin_sscb_scl = SIOC_GPIO_NUM;
#endif
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240
  config.jpeg_quality = JPEG_QUALITY;

  if (psramFound()) {
    config.fb_count    = 2;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2)
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode   = CAMERA_GRAB_LATEST;
#endif
  } else {
    config.fb_count    = 1;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2)
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
#endif
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed 0x%x\n", (unsigned)err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
    Serial.printf("[CAM] sensor PID 0x%x %s\n", (unsigned)s->id.PID,
                  (s->id.PID == OV3660_PID) ? "(OV3660)" : "(other)");
  }
  // warm-up: throw away the first frames
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(30);
  }
  camReady = true;
  Serial.println("[CAM] camera ready (QVGA JPEG)");
  return true;
}

static bool classifyAllowed() {
  if (modelFault || !camReady) return false;
  if (!stSeen) return false;
  if ((uint32_t)(millis() - lastStMs) > MAIN_CMD_TIMEOUT_MS) return false;
  return mainCls && !mainSafe;
}

// Core 1: capture frames, publish them to the streamer, hand every 5th to AI
static void camTask(void* arg) {
  uint32_t nextFrameMs = millis();
  uint32_t fpsCount = 0;
  uint32_t fpsT = millis();
  uint8_t failStreak = 0;

  for (;;) {
    if (!camReady) {
      if (!cameraInit()) {
        camFault = true;
        vTaskDelay(pdMS_TO_TICKS(10000));
        continue;
      }
      camFault = false;
    }

    uint32_t now = millis();
    if ((int32_t)(now - nextFrameMs) < 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    nextFrameMs += FRAME_INTERVAL_MS;
    if ((int32_t)(now - nextFrameMs) > (int32_t)(FRAME_INTERVAL_MS * 2)) nextFrameMs = now + FRAME_INTERVAL_MS;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      failStreak++;
      if (failStreak >= 10) {
        Serial.println("[CAM] capture failing - re-initialising camera");
        esp_camera_deinit();
        camReady = false;
        camFault = true;
        failStreak = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    failStreak = 0;
    camFault = false;
    frameCounter++;
    fpsCount++;

    if (fb->format == PIXFORMAT_JPEG && fb->len > 0 && fb->len <= FRAME_BUF_SIZE) {
      if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(frameBuf, fb->buf, fb->len);
        frameLen = fb->len;
        frameSeq++;
        xSemaphoreGive(frameMutex);
      }
      if ((frameCounter % AI_EVERY_N_FRAMES) == 0 && aiState == AI_IDLE && classifyAllowed()) {
        memcpy(aiJpeg, fb->buf, fb->len);
        aiJpegLen = fb->len;
        aiState = AI_PENDING;
      }
    }
    esp_camera_fb_return(fb);

    if ((uint32_t)(millis() - fpsT) >= 1000) {
      fpsValue = fpsCount;
      fpsCount = 0;
      fpsT = millis();
    }
  }
}

/* ============================ AI / INFERENCE ============================== */
static int aiGetData(size_t offset, size_t length, float* out_ptr) {
  // snapshotBuf holds RGB888 (byte order swapped, see esp32-camera issue 379)
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ix = 0;
  while (pixels_left != 0) {
    out_ptr[out_ix] = (snapshotBuf[pixel_ix + 2] << 16) + (snapshotBuf[pixel_ix + 1] << 8) + snapshotBuf[pixel_ix];
    out_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

// Returns false if decode / classifier failed. bestCls == 0 means "nothing valid".
static bool runInference(uint8_t* jpg, size_t len, uint8_t* bestCls, float* bestConf, char* bestLabel, size_t labelSize) {
  *bestCls = CLS_NONE;
  *bestConf = 0.0f;
  bestLabel[0] = '\0';

  bool ok = fmt2rgb888(jpg, len, PIXFORMAT_JPEG, snapshotBuf);
  if (!ok) return false;

  ei::image::processing::crop_and_interpolate_rgb888(
      snapshotBuf, 320, 240, snapshotBuf,
      EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &aiGetData;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("[AI] run_classifier error %d\n", (int)err);
    return false;
  }
  lastInferMs = (uint32_t)(result.timing.dsp + result.timing.classification);

#if defined(EI_CLASSIFIER_OBJECT_DETECTION) && (EI_CLASSIFIER_OBJECT_DETECTION == 1)
  // FOMO / object detection model: one entry per detected object
  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;                 // empty slot
    if (bb.value < CONF_THRESHOLD) continue;     // below threshold -> UNKNOWN
    uint8_t c = classOfLabel(bb.label);
    if (c == CLS_NONE) continue;                 // unmapped label -> UNKNOWN
    if (bb.value > *bestConf) {
      *bestConf = bb.value;
      *bestCls = c;
      sanitizeLabel(bestLabel, bb.label, labelSize);
    }
  }
#else
  // plain image-classification model: one score per label, best valid one wins
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float v = result.classification[i].value;
    if (v < CONF_THRESHOLD) continue;
    uint8_t c = classOfLabel(result.classification[i].label);
    if (c == CLS_NONE) continue;
    if (v > *bestConf) {
      *bestConf = v;
      *bestCls = c;
      sanitizeLabel(bestLabel, result.classification[i].label, labelSize);
    }
  }
#endif
  return true;
}

// Core 0: inference + temporal filtering
static void aiTask(void* arg) {
  uint8_t  prevCls = CLS_NONE;
  uint8_t  consec = 0;
  uint32_t absentSince = 0;
  bool     needAbsence = false;
  uint8_t  failStreak = 0;
  bool     wasAllowed = false;

  for (;;) {
    if (aiState != AI_PENDING) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    aiState = AI_BUSY;

    if (!classifyAllowed()) {
      prevCls = CLS_NONE; consec = 0; absentSince = 0; needAbsence = false;
      wasAllowed = false;
      aiState = AI_IDLE;
      continue;
    }
    if (!wasAllowed) {           // classification was just (re)enabled: start clean
      prevCls = CLS_NONE; consec = 0; absentSince = 0; needAbsence = false;
      wasAllowed = true;
    }

    uint8_t cls; float conf; char label[32];
    bool ok = runInference(aiJpeg, aiJpegLen, &cls, &conf, label, sizeof(label));
    uint32_t now = millis();

    if (!ok) {
      failStreak++;
      if (failStreak >= 3) {
        if (!modelFault) Serial.println("[AI] MODEL/DECODE FAULT");
        modelFault = true;
      }
      aiState = AI_IDLE;
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    failStreak = 0;
    modelFault = false;

    // object presence tracking (for the 1 s new-object delay):
    // after a packet was sent, the scene must be empty for NEW_OBJECT_DELAY_MS
    // before another classification may be transmitted.
    if (cls != CLS_NONE) {
      absentSince = 0;
    } else {
      if (absentSince == 0) absentSince = now;
      if (needAbsence && (uint32_t)(now - absentSince) >= NEW_OBJECT_DELAY_MS) needAbsence = false;
    }

    // consecutive same-class filter
    if (cls == CLS_NONE) {
      prevCls = CLS_NONE; consec = 0;
    } else if (cls == prevCls) {
      if (consec < 250) consec++;
    } else {
      prevCls = cls; consec = 1;
    }

    if (cls != CLS_NONE) {
      Serial.printf("[AI] %s %.2f class=%u consec=%u (%ums)\n", label, conf, cls, consec, (unsigned)lastInferMs);
    }

    if (cls != CLS_NONE && consec >= CONSEC_REQUIRED && txInFlight == 0) {
      bool lockOk = (txDoneMs == 0) || ((uint32_t)(now - txDoneMs) >= DUP_LOCKOUT_MS);
      if (lockOk && !needAbsence) {
        TxMsg m;
        m.cls = cls;
        m.conf = conf;
        m.seq = txSeqCounter++;
        strncpy(m.label, label, sizeof(m.label) - 1);
        m.label[sizeof(m.label) - 1] = '\0';
        if (xQueueSend(txQueue, &m, 0) == pdTRUE) {
          txInFlight = 1;
          needAbsence = true;
          consec = 0;
        }
      }
    }
    aiState = AI_IDLE;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ============================ MJPEG STREAM ================================ */
static void closeSlot(int i) {
  slots[i].c.stop();
  slots[i].mode = 0;
  slots[i].reqLen = 0;
}

static bool sendAll(WiFiClient& c, const uint8_t* data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    size_t chunk = len - sent;
    if (chunk > 1436) chunk = 1436;
    size_t w = c.write(data + sent, chunk);
    if (w == 0) return false;
    sent += w;
  }
  return true;
}

static void handleRequest(int i, uint8_t* sendBuf) {
  char* r = slots[i].req;
  WiFiClient& c = slots[i].c;
  if (strncmp(r, "GET /stream", 11) == 0) {
    const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Connection: close\r\n\r\n";
    if (!sendAll(c, (const uint8_t*)hdr, strlen(hdr))) { closeSlot(i); return; }
    c.setNoDelay(true);
    slots[i].mode = 2;
    return;
  }
  if (strncmp(r, "GET /snapshot", 13) == 0) {
    size_t len = 0;
    if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      len = frameLen;
      if (len > 0) memcpy(sendBuf, frameBuf, len);
      xSemaphoreGive(frameMutex);
    }
    if (len == 0) {
      const char* nf = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n";
      sendAll(c, (const uint8_t*)nf, strlen(nf));
    } else {
      char hdr[200];
      int hl = snprintf(hdr, sizeof(hdr),
                        "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: image/jpeg\r\n"
                        "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)len);
      if (sendAll(c, (const uint8_t*)hdr, hl)) sendAll(c, sendBuf, len);
    }
    closeSlot(i);
    return;
  }
  if (strncmp(r, "GET / ", 6) == 0 || strncmp(r, "GET /health", 11) == 0) {
    char ip[20];
    ipToStr(ip, sizeof(ip));
    char body[220];
    int bl = snprintf(body, sizeof(body),
                      "SIBOAT ESP32-CAM\r\nip: %s\r\nstream: http://%s:%d/stream\r\nfps: %u\r\nmodel_ok: %d\r\n",
                      ip, ip, STREAM_PORT, (unsigned)fpsValue, modelFault ? 0 : 1);
    char hdr[160];
    int hl = snprintf(hdr, sizeof(hdr),
                      "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: text/plain\r\n"
                      "Content-Length: %d\r\nConnection: close\r\n\r\n", bl);
    if (sendAll(c, (const uint8_t*)hdr, hl)) sendAll(c, (const uint8_t*)body, bl);
    closeSlot(i);
    return;
  }
  const char* nf = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
  sendAll(c, (const uint8_t*)nf, strlen(nf));
  closeSlot(i);
}

// Core 1: HTTP server + MJPEG push. Slow clients only stall this task.
static void streamTask(void* arg) {
  uint8_t* sendBuf = (uint8_t*)psAlloc(FRAME_BUF_SIZE);
  uint32_t lastSeq = 0;
  if (!sendBuf) {
    Serial.println("[STREAM] out of memory");
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    if (serverStarted && WiFi.status() == WL_CONNECTED) {
      WiFiClient nc = streamServer.available();
      if (nc) {
        int free_i = -1;
        for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
          if (slots[i].mode == 0) { free_i = i; break; }
        }
        if (free_i < 0) {
          nc.stop();
        } else {
          slots[free_i].c = nc;
          slots[free_i].mode = 1;
          slots[free_i].t0 = millis();
          slots[free_i].reqLen = 0;
          slots[free_i].req[0] = '\0';
        }
      }
    }

    bool anyStreaming = false;
    for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
      if (slots[i].mode == 0) continue;
      if (!slots[i].c.connected() && slots[i].c.available() == 0) { closeSlot(i); continue; }

      if (slots[i].mode == 1) {
        while (slots[i].c.available() > 0 && slots[i].mode == 1) {
          int ch = slots[i].c.read();
          if (ch < 0) break;
          if (ch == '\n') {
            slots[i].req[slots[i].reqLen] = '\0';
            handleRequest(i, sendBuf);
          } else if (ch != '\r' && slots[i].reqLen < sizeof(slots[i].req) - 1) {
            slots[i].req[slots[i].reqLen++] = (char)ch;
          }
        }
        if (slots[i].mode == 1 && (uint32_t)(millis() - slots[i].t0) > 3000) closeSlot(i);
      }
      if (slots[i].mode == 2) anyStreaming = true;
    }

    if (anyStreaming && frameSeq != lastSeq) {
      size_t len = 0;
      if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        len = frameLen;
        lastSeq = frameSeq;
        if (len > 0) memcpy(sendBuf, frameBuf, len);
        xSemaphoreGive(frameMutex);
      }
      if (len > 0) {
        char part[96];
        int pl = snprintf(part, sizeof(part),
                          "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)len);
        for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
          if (slots[i].mode != 2) continue;
          bool ok = sendAll(slots[i].c, (const uint8_t*)part, pl) &&
                    sendAll(slots[i].c, sendBuf, len) &&
                    sendAll(slots[i].c, (const uint8_t*)"\r\n", 2);
          if (!ok) closeSlot(i);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

/* ================================ WI-FI =================================== */
static bool wifiTry(const char* ssid, const char* pass, bool useStatic, IPAddress gw, IPAddress mask, uint32_t timeoutMs) {
  WiFi.disconnect(false, false);
  vTaskDelay(pdMS_TO_TICKS(150));
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(CAM_HOSTNAME);
  WiFi.setSleep(false);
  if (useStatic) WiFi.config(CAM_STATIC_IP, gw, mask, gw);
  else WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  WiFi.begin(ssid, pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < timeoutMs) {
    wl_status_t st = WiFi.status();
    // SSID not visible / wrong password: give up on this network quickly
    if ((st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED) && (uint32_t)(millis() - t0) > 2500) break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return WiFi.status() == WL_CONNECTED;
}

// Learn the hotspot's gateway with DHCP, then lock the static address.
static bool connectStation() {
  for (int k = 0; k < WIFI_CRED_COUNT; k++) {
    const char* ssid = WIFI_CREDS[k].ssid;
    const char* pass = WIFI_CREDS[k].pass;
    Serial.printf("[WIFI] trying '%s' ...\n", ssid);
    if (!wifiTry(ssid, pass, false, CAM_DEFAULT_GW, CAM_DEFAULT_MASK, 10000)) continue;

    IPAddress got = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    IPAddress mask = WiFi.subnetMask();
    Serial.printf("[WIFI] DHCP gave %u.%u.%u.%u gw %u.%u.%u.%u\n", got[0], got[1], got[2], got[3], gw[0], gw[1], gw[2], gw[3]);

    bool sameNet = (got[0] == CAM_STATIC_IP[0]) && (got[1] == CAM_STATIC_IP[1]) && (got[2] == CAM_STATIC_IP[2]);
    if (!sameNet) {
      Serial.println("[WIFI] WARNING: hotspot subnet differs from 10.90.102.x - keeping DHCP address");
      return true;
    }
    if (got == CAM_STATIC_IP) return true;

    if (wifiTry(ssid, pass, true, gw, mask, 10000)) return true;
    Serial.println("[WIFI] static IP failed - back to DHCP");
    if (wifiTry(ssid, pass, false, gw, mask, 10000)) return true;
  }
  return false;
}

static void wifiTask(void* arg) {
  bool wasUp = false;
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wasUp) {
        IPAddress ip = WiFi.localIP();
        Serial.printf("[WIFI] connected %u.%u.%u.%u  stream: http://%u.%u.%u.%u:%d/stream\n",
                      ip[0], ip[1], ip[2], ip[3], ip[0], ip[1], ip[2], ip[3], STREAM_PORT);
      }
      wasUp = true;
      wifiUp = true;
      if (!serverStarted) {
        streamServer.begin();
        serverStarted = true;
        Serial.printf("[STREAM] server listening on port %d\n", STREAM_PORT);
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    wifiUp = false;
    if (wasUp) {
      wasUp = false;
      Serial.println("[WIFI] link lost - reconnecting");
      WiFi.reconnect();
      uint32_t t = millis();
      while ((uint32_t)(millis() - t) < 10000 && WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(200));
      if (WiFi.status() == WL_CONNECTED) continue;
    }
    if (!connectStation()) {
      Serial.println("[WIFI] no network yet - retrying");
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
  }
}

/* ================================= UART =================================== */
static void uartSendLine(const char* s) {
  Serial2.print(s);
  Serial2.print('\n');
}

static void uartHandleLine(char* line) {
  // strip spaces / CR
  while (*line == ' ' || *line == '\r') line++;
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\r')) { line[--n] = '\0'; }
  if (n == 0) return;

  if (line[0] == '<') {                       // framed command from Main
    if (strncmp(line, "<ST,", 4) == 0) {
      int c = 0, s = 1;
      if (sscanf(line, "<ST,%d,%d", &c, &s) == 2) {
        mainCls = (c != 0);
        mainSafe = (s != 0);
        stSeen = true;
        lastStMs = millis();
      }
    }
    return;
  }

  // plain replies
  if (txState == TX_WAIT_ACK) {
    if (strcmp(line, txExpect) == 0) {
      Serial.printf("[UART] %s received after %u attempt(s)\n", line, txAttempts);
      txState = TX_IDLE;
      txDoneMs = millis();
      txInFlight = 0;
    } else if (strcmp(line, "ERROR") == 0) {
      Serial.println("[UART] Main replied ERROR");
      txDeadline = millis() + RETRY_COOLDOWN_MS;   // treated like a failed attempt
      txState = TX_COOLDOWN;
    }
  }
}

static void uartPoll() {
  while (Serial2.available() > 0) {
    int ch = Serial2.read();
    if (ch < 0) break;
    if (ch == '<') { rxInFrame = true; rxLen = 0; rxBuf[rxLen++] = '<'; continue; }
    if (rxInFrame) {
      if (rxLen < sizeof(rxBuf) - 2) rxBuf[rxLen++] = (char)ch;
      if (ch == '>') {
        rxBuf[rxLen] = '\0';
        rxInFrame = false;
        rxLen = 0;
        uartHandleLine(rxBuf);
      } else if (ch == '\n' || rxLen >= sizeof(rxBuf) - 2) {
        rxInFrame = false;
        rxLen = 0;
      }
      continue;
    }
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      rxLen = 0;
      uartHandleLine(rxBuf);
    } else if (rxLen < sizeof(rxBuf) - 2) {
      rxBuf[rxLen++] = (char)ch;
    } else {
      rxLen = 0;
    }
  }
}

static void txService() {
  uint32_t now = millis();
  if (txState == TX_IDLE) {
    TxMsg m;
    if (xQueueReceive(txQueue, &m, 0) == pdTRUE) {
      if (m.cls == CLS_REJECT) {
        snprintf(txPkt, sizeof(txPkt), "<REJ,%.2f,SEQ,%lu,%s>", m.conf, (unsigned long)m.seq, m.label);
        strcpy(txExpect, "REJECT");
      } else {
        snprintf(txPkt, sizeof(txPkt), "<CAT,%u,%.2f,SEQ,%lu,%s>", (unsigned)m.cls, m.conf, (unsigned long)m.seq, m.label);
        snprintf(txExpect, sizeof(txExpect), "ACK%u", (unsigned)m.cls);
      }
      txAttempts = 1;
      uartSendLine(txPkt);
      Serial.printf("[UART] TX %s (attempt 1/%d)\n", txPkt, MAX_TX_ATTEMPTS);
      txDeadline = now + ACK_TIMEOUT_MS;
      txState = TX_WAIT_ACK;
    }
    return;
  }

  if (txState == TX_WAIT_ACK) {
    if ((int32_t)(now - txDeadline) >= 0) {
      Serial.println("[UART] ACK timeout");
      txDeadline = now + RETRY_COOLDOWN_MS;
      txState = TX_COOLDOWN;
    }
    return;
  }

  if (txState == TX_COOLDOWN) {
    if ((int32_t)(now - txDeadline) < 0) return;
    if (txAttempts >= MAX_TX_ATTEMPTS) {
      Serial.printf("[UART] GIVING UP after %u attempts: %s\n", txAttempts, txPkt);
      txState = TX_IDLE;
      txDoneMs = now;
      txInFlight = 0;
      return;
    }
    txAttempts++;
    uartSendLine(txPkt);
    Serial.printf("[UART] TX %s (attempt %u/%d)\n", txPkt, txAttempts, MAX_TX_ATTEMPTS);
    txDeadline = now + ACK_TIMEOUT_MS;
    txState = TX_WAIT_ACK;
  }
}

static void hbService() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastHbMs) < HB_INTERVAL_MS) return;
  lastHbMs = now;
  char ip[20];
  ipToStr(ip, sizeof(ip));
  bool ok = camReady && !camFault && !modelFault && (snapshotBuf != NULL);
  char pkt[80];
  snprintf(pkt, sizeof(pkt), "<HB,%d,%s,%u,%u>", ok ? 1 : 0, ip, (unsigned)fpsValue, (unsigned)lastInferMs);
  uartSendLine(pkt);
}

/* ============================ SETUP / LOOP ================================ */
void setup() {
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== SIBOAT ESP32-CAM SUPER FINAL firmware ===");

  Serial2.setRxBufferSize(512);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.printf("[MEM] PSRAM %s, free PSRAM %u, free heap %u\n", psramFound() ? "found" : "NOT FOUND",
                (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap());

  frameMutex = xSemaphoreCreateMutex();
  txQueue = xQueueCreate(1, sizeof(TxMsg));
  frameBuf = (uint8_t*)psAlloc(FRAME_BUF_SIZE);
  aiJpeg = (uint8_t*)psAlloc(FRAME_BUF_SIZE);
  snapshotBuf = (uint8_t*)psAlloc(320 * 240 * 3);
  bool memOk = (frameMutex && txQueue && frameBuf && aiJpeg && snapshotBuf);
  if (!memOk) {
    Serial.println("[MEM] allocation FAILED - fault flagged, camera tasks not started");
    modelFault = true;
  }
  for (int i = 0; i < MAX_STREAM_CLIENTS; i++) { slots[i].mode = 0; slots[i].reqLen = 0; }

  // random starting sequence number so a camera reboot never repeats an old SEQ
  txSeqCounter = 1 + (esp_random() % 50000);

  // model label sanity check
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (classOfLabel(ei_classifier_inferencing_categories[i]) == CLS_NONE) {
      Serial.printf("[AI] WARNING: model label '%s' has no machine class (will be ignored)\n",
                    ei_classifier_inferencing_categories[i]);
    }
  }

  // (the HTTP server is started by wifiTask once Wi-Fi is up - starting a
  //  server before the TCP/IP stack exists makes the ESP32 reboot)

  // Initialise the camera here (setup has an 8 KB stack), before Wi-Fi starts.
  if (memOk) {
    if (!cameraInit()) camFault = true;
  }

  xTaskCreatePinnedToCore(wifiTask,   "wifi",   6144,  NULL, 1, NULL, 1);
  if (memOk) {
    xTaskCreatePinnedToCore(camTask,    "cam",    8192,  NULL, 3, NULL, 1);   // Core 1: capture + stream
    xTaskCreatePinnedToCore(streamTask, "stream", 8192,  NULL, 2, NULL, 1);   // Core 1: HTTP MJPEG
    xTaskCreatePinnedToCore(aiTask,     "ai",     16384, NULL, 1, NULL, 0);   // Core 0: inference
  }

  Serial.println("[BOOT] tasks started; classification waits for Main ESP32 command");
}

void loop() {
  uartPoll();
  txService();
  hbService();
  delay(2);
}
