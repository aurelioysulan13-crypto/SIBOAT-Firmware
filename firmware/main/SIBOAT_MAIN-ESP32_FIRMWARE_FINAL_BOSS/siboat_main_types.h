#ifndef SIBOAT_MAIN_TYPES_H
#define SIBOAT_MAIN_TYPES_H

// Shared Main ESP32 firmware types are kept in a header so they are declared
// before Arduino's automatic prototype generation processes the .ino file.
// Existing member order and field types are preserved intentionally.

#include <Arduino.h>
#include <WiFi.h>

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

#endif  // SIBOAT_MAIN_TYPES_H
