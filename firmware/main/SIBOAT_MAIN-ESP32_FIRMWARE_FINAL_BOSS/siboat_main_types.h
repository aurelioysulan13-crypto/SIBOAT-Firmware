#ifndef SIBOAT_MAIN_TYPES_H
#define SIBOAT_MAIN_TYPES_H

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
  uint8_t  timeSrc;
  uint32_t bootNum;
  uint32_t deploySec;
  uint32_t crc;
};

#define WS_MAX_CLIENTS 3
#define WS_BUF         640

struct WsClient {
  WiFiClient c;
  uint8_t    st;
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

#endif
