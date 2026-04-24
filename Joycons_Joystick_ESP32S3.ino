// ============================================================================
//  DUAL JOY-CON + USB HID GAMEPAD (TinyUSB) + LED WS2812 + LOG opzionale
//  ESP32-S3 Zero (Waveshare) – Joy-Con sinistro + destro su 1 Mbps UART IDF
//  HID layout tipo Nintendo Pro Controller, con L3/R3 e D-pad come Hat.
//  LED WS2812 su GPIO 21
// ============================================================================

#define ENABLE_SERIAL_LOG 0  // 1 = abilita log seriale, 0 = silenzioso

#include <Arduino.h>
#include <USB.h>
#include <USBHIDGamepad.h>
#include <Adafruit_NeoPixel.h>
#include "driver/uart.h"

// ------------ LED WS2812 integrato ------------
#define LED_PIN 21
#define LED_COUNT 1
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

static inline void setLedRGB(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ------------ HID Gamepad ------------
USBHIDGamepad gamepad;

// ------------ Pinout Joy-Con ------------
#define JCL_UART UART_NUM_1
#define JCL_RX_PIN 8
#define JCL_TX_PIN 9
#define JCL_JRST_PIN 12

#define JCR_UART UART_NUM_2
#define JCR_RX_PIN 1
#define JCR_TX_PIN 2
#define JCR_JRST_PIN 4

// UART + timing
#define BAUD_1M 1000000
#define RX_BUF_SIZE 4096
#define POLL_MS 15
#define WATCHDOG_MS 5000
#define DETECT_RETRY_MS 100
#define HS_WINDOW_MS 3000
#define PLEN_MIN 7
#define PLEN_MAX 128

// ------------ Packets Joy-Con ------------
static const uint8_t PKT_STARTSEQ[4] = { 0xA1, 0xA2, 0xA3, 0xA4 };
static const uint8_t PKT_HS_A5[12] = { 0x19, 0x01, 0x03, 0x07, 0x00, 0xA5, 0x02, 0x01, 0x7E, 0x00, 0x00, 0x00 };
static const uint8_t PKT_GET_MAC[12] = { 0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x01, 0x00, 0x00, 0x00, 0x00, 0x24 };
static const uint8_t PKT_CMD_11[12] = { 0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x11, 0x00, 0x00, 0x00, 0x00, 0x0E };
static const uint8_t PKT_CMD_10[12] = { 0x19, 0x01, 0x03, 0x07, 0x00, 0x91, 0x10, 0x00, 0x00, 0x00, 0x00, 0x3D };
static const uint8_t PKT_CMD_12[16] = { 0x19, 0x01, 0x03, 0x0B, 0x00, 0x91, 0x12, 0x04, 0x00, 0x00, 0x12, 0xA6, 0x0F, 0x00, 0x00, 0x00 };
static const uint8_t PKT_STATUS[13] = { 0x19, 0x01, 0x03, 0x08, 0x00, 0x92, 0x00, 0x01, 0x00, 0x00, 0x69, 0x2D, 0x1F };

// ------------ Frame ------------
typedef struct {
  uint8_t data[256];
  int len;
} FrameMsg;

// ------------ Struttura JoyCon ------------
typedef struct {
  const char* tag;
  uart_port_t uart;
  int rxPin, txPin, jrstPin;
  bool isRight;

  uint8_t last_btn_main;
  uint8_t last_btn_shared;
  uint16_t last_X, last_Y;
  bool haveStickBase;

  QueueHandle_t q;

  enum { DETECT,
         HS,
         DO_CMDS,
         POLL } state;
  uint32_t tDetectStart, tLastDetectTx;
  uint32_t tLastPoll, tLastFrame;

} JoyCon;

// ------------ Inizializzatori ------------
JoyCon L = { "LEFT ", JCL_UART, JCL_RX_PIN, JCL_TX_PIN, JCL_JRST_PIN, false,
             0, 0, 0, 0, false, NULL, JoyCon::DETECT, 0, 0, 0, 0 };

JoyCon R = { "RIGHT", JCR_UART, JCR_RX_PIN, JCR_TX_PIN, JCR_JRST_PIN, true,
             0, 0, 0, 0, false, NULL, JoyCon::DETECT, 0, 0, 0, 0 };

// ------------ Prototipi per bloccare l'auto-prototipo Arduino ------------
bool parseFrame(uart_port_t uart, FrameMsg* out);
bool waitAck(JoyCon& jc, uint8_t cmd, uint32_t to_ms);
void logButtons(JoyCon& jc, uint8_t m, uint8_t s);
void logStick(JoyCon& jc, uint16_t X, uint16_t Y);
void processJoyCon(JoyCon& jc);
void sendHIDState();

// ------------ Utility ------------
static inline bool isHostEcho(const uint8_t* f, int len) {
  return (len >= 2) && ((f[1] & 0x80) == 0);
}

void jrReset(JoyCon& jc, bool full = true) {
  digitalWrite(jc.jrstPin, HIGH);
  delay(full ? 200 : 100);
  digitalWrite(jc.jrstPin, LOW);
  delay(350);
}

// ------------ Parser frame ------------
bool parseFrame(uart_port_t uart, FrameMsg* out) {
  uint8_t b;
  int r = uart_read_bytes(uart, &b, 1, pdMS_TO_TICKS(20));
  if (r != 1 || b != 0x19) return false;

  uint8_t hdr[4];
  r = uart_read_bytes(uart, hdr, 4, pdMS_TO_TICKS(20));
  if (r != 4) return false;

  uint8_t b1 = hdr[0], b2 = hdr[1], lenL = hdr[2], lenH = hdr[3];
  if (!((b1 == 0x01 || b1 == 0x81) && b2 == 0x03)) return false;

  uint16_t plen = lenL | (lenH << 8);
  if (plen < PLEN_MIN || plen > PLEN_MAX) return false;

  out->data[0] = 0x19;
  out->data[1] = b1;
  out->data[2] = 0x03;
  out->data[3] = lenL;
  out->data[4] = lenH;

  r = uart_read_bytes(uart, out->data + 5, plen, pdMS_TO_TICKS(20));
  if (r != plen) return false;

  if (isHostEcho(out->data, 5 + plen)) return false;

  out->len = 5 + plen;
  return true;
}

// ------------ Task RX ------------
void taskUART_Left(void*) {
  FrameMsg m;
  for (;;) {
    if (parseFrame(L.uart, &m)) xQueueSend(L.q, &m, 0);
    taskYIELD();
  }
}

void taskUART_Right(void*) {
  FrameMsg m;
  for (;;) {
    if (parseFrame(R.uart, &m)) xQueueSend(R.q, &m, 0);
    taskYIELD();
  }
}

// ------------ waitAck ------------
bool waitAck(JoyCon& jc, uint8_t cmd, uint32_t to_ms) {
  FrameMsg m;
  uint32_t t0 = millis();
  while (millis() - t0 < to_ms) {
    if (xQueueReceive(jc.q, &m, pdMS_TO_TICKS(20))) {
      if (m.data[5] == 0x94 && m.data[6] == cmd) return true;
    }
  }
  return false;
}

// ------------ Log pulsanti ------------
void logButtons(JoyCon& jc, uint8_t m, uint8_t s) {
#if ENABLE_SERIAL_LOG
  uint8_t pM = m & ~jc.last_btn_main;
  uint8_t rM = jc.last_btn_main & ~m;
  uint8_t pS = s & ~jc.last_btn_shared;
  uint8_t rS = jc.last_btn_shared & ~s;

  if (pM | rM | pS | rS) {
    Serial.printf("[%s] BTN ", jc.tag);
    if (pM) Serial.printf("P_MAIN=%02X ", pM);
    if (rM) Serial.printf("R_MAIN=%02X ", rM);
    if (pS) Serial.printf("P_SH=%02X ", pS);
    if (rS) Serial.printf("R_SH=%02X ", rS);
    Serial.println();
  }
#endif

  jc.last_btn_main = m;
  jc.last_btn_shared = s;
}

// ------------ Log stick ------------
void logStick(JoyCon& jc, uint16_t X, uint16_t Y) {
#if ENABLE_SERIAL_LOG
  const uint16_t DZ = 6;
  if (!jc.haveStickBase) {
    jc.haveStickBase = true;
    jc.last_X = X;
    jc.last_Y = Y;
    Serial.printf("[%s] STICK BASE (%u,%u)\n", jc.tag, X, Y);
    return;
  }
  if (abs((int)X - (int)jc.last_X) >= DZ || abs((int)Y - (int)jc.last_Y) >= DZ) {
    jc.last_X = X;
    jc.last_Y = Y;
    Serial.printf("[%s] STICK (%u,%u)\n", jc.tag, X, Y);
  }
#endif
  if (!jc.haveStickBase) {
    jc.haveStickBase = true;
    jc.last_X = X;
    jc.last_Y = Y;
  } else {
    jc.last_X = X;
    jc.last_Y = Y;
  }
}

// ------------ State machine ------------
static uint32_t g_lastErrorMs = 0;

void processJoyCon(JoyCon& jc) {
  FrameMsg msg;
  uint32_t now = millis();

  switch (jc.state) {
    case JoyCon::DETECT:
      {
        if (now - jc.tLastDetectTx >= DETECT_RETRY_MS) {
          jc.tLastDetectTx = now;
          uart_write_bytes(jc.uart, (const char*)PKT_STARTSEQ, 4);
          uart_write_bytes(jc.uart, (const char*)PKT_HS_A5, 12);
        }

        if (xQueueReceive(jc.q, &msg, pdMS_TO_TICKS(10))) {
          if (msg.data[5] == 0xA5) {
#if ENABLE_SERIAL_LOG
            Serial.printf("[%s] Detect A5 OK\n", jc.tag);
#endif
            jc.state = JoyCon::HS;
          }
        }

        if (now - jc.tDetectStart > HS_WINDOW_MS) {
#if ENABLE_SERIAL_LOG
          Serial.printf("[%s] Detect timeout\n", jc.tag);
#endif
          jrReset(jc, true);
          jc.tDetectStart = millis();
          jc.tLastDetectTx = 0;
        }
      }
      break;

    case JoyCon::HS:
      {
        uart_write_bytes(jc.uart, (const char*)PKT_GET_MAC, 12);
        (void)waitAck(jc, 0x01, 400);

        uart_write_bytes(jc.uart, (const char*)PKT_CMD_11, 12);
        if (!waitAck(jc, 0x11, 400)) {
          jc.state = JoyCon::DETECT;
          jc.tDetectStart = millis();
          break;
        }

        uart_write_bytes(jc.uart, (const char*)PKT_CMD_10, 12);
        if (!waitAck(jc, 0x10, 400)) {
          jc.state = JoyCon::DETECT;
          jc.tDetectStart = millis();
          break;
        }

        uart_write_bytes(jc.uart, (const char*)PKT_CMD_12, 16);
        if (!waitAck(jc, 0x12, 1000)) {
          jc.state = JoyCon::DETECT;
          jc.tDetectStart = millis();
          break;
        }

#if ENABLE_SERIAL_LOG
        Serial.printf("[%s] Handshake COMPLETO\n", jc.tag);
#endif

        jc.state = JoyCon::DO_CMDS;
      }
      break;

    case JoyCon::DO_CMDS:
      {
        jc.haveStickBase = false;
        jc.last_btn_main = 0;
        jc.last_btn_shared = 0;
        jc.tLastPoll = now;
        jc.tLastFrame = now;

#if ENABLE_SERIAL_LOG
        Serial.printf("[%s] Entering POLL...\n", jc.tag);
#endif

        jc.state = JoyCon::POLL;
      }
      break;

    case JoyCon::POLL:
      {
        if (now - jc.tLastPoll >= POLL_MS) {
          jc.tLastPoll = now;
          uart_write_bytes(jc.uart, (const char*)PKT_STATUS, 13);
        }

        while (xQueueReceive(jc.q, &msg, 0)) {
          if (msg.data[5] != 0x92) continue;
          const uint8_t* d = msg.data + 12;
          if (d[0] != 0x30) continue;

          jc.tLastFrame = now;

          uint8_t btnM, btnS;
          uint16_t X, Y;

          if (jc.isRight) {
            btnM = d[3];
            btnS = d[4] & 0x3F;
            X = d[9] | ((d[10] & 0x0F) << 8);
            Y = (d[10] >> 4) | (d[11] << 4);
          } else {
            btnM = d[5];
            btnS = d[4] & 0x3F;
            X = d[6] | ((d[7] & 0x0F) << 8);
            Y = (d[7] >> 4) | (d[8] << 4);
          }

          logButtons(jc, btnM, btnS);
          logStick(jc, X, Y);
        }

        if (now - jc.tLastFrame > WATCHDOG_MS) {
#if ENABLE_SERIAL_LOG
          Serial.printf("[%s] Watchdog!\n", jc.tag);
#endif
          jc.state = JoyCon::DETECT;
          jc.tDetectStart = millis();
          jc.tLastDetectTx = 0;
          g_lastErrorMs = now;
        }
      }
      break;
  }
}

void sendHIDState() {
  auto to_i8 = [](uint16_t raw) -> int8_t {
    int32_t v = (int32_t)raw - 2048;
    int32_t out = (v * 127) / 2047;
    if (out < -127) out = -127;
    if (out > 127) out = 127;
    return (int8_t)out;
  };

  // ====== STICK ======
  int8_t lx = to_i8(L.last_X);
  int8_t ly = -to_i8(L.last_Y);
  int8_t rx_stick = to_i8(R.last_X);
  int8_t ry_stick = -to_i8(R.last_Y);

  // TRIGGER ANALOGICI - RANGE COMPLETO -128 a 127
  static int8_t z_trig = -128;
  static int8_t rz_trig = -128;

  int8_t z_target = (L.last_btn_main & 0x80) ? 127 : -128;
  int8_t rz_target = (R.last_btn_main & 0x80) ? 127 : -128;

  const int8_t step = 50;

  // ZL con cast esplicito
  if (z_trig != z_target) {
    if (z_trig < z_target) {
      z_trig = (int8_t)min((int)z_trig + (int)step, (int)z_target);
    } else {
      z_trig = (int8_t)max((int)z_trig - (int)step, (int)z_target);
    }
  }

  // ZR con cast esplicito
  if (rz_trig != rz_target) {
    if (rz_trig < rz_target) {
      rz_trig = (int8_t)min((int)rz_trig + (int)step, (int)rz_target);
    } else {
      rz_trig = (int8_t)max((int)rz_trig - (int)step, (int)rz_target);
    }
  }

  // ====== HAT (D-PAD) ======
  bool up = L.last_btn_main & 0x02;
  bool down = L.last_btn_main & 0x01;
  bool left = L.last_btn_main & 0x08;
  bool right = L.last_btn_main & 0x04;

  uint8_t hat = 0;
  if (up && right) hat = 2;
  else if (right && down) hat = 4;
  else if (down && left) hat = 6;
  else if (left && up) hat = 8;
  else if (up) hat = 1;
  else if (right) hat = 3;
  else if (down) hat = 5;
  else if (left) hat = 7;

  // ====== BUTTONS - LAYOUT STANDARD + TRIGGER DIGITALI ======
  uint32_t b = 0;

  // Face buttons (ABXY)
  if (R.last_btn_main & 0x04) b |= (1u << 0);  // B → A/South
  if (R.last_btn_main & 0x08) b |= (1u << 1);  // A → B/East
  if (R.last_btn_main & 0x01) b |= (1u << 3);  // Y → X/North
  if (R.last_btn_main & 0x02) b |= (1u << 4);  // X → Y/West

  // Shoulder buttons (L/R)
  if (L.last_btn_main & 0x40) b |= (1u << 6);  // L → TL/LB
  if (R.last_btn_main & 0x40) b |= (1u << 7);  // R → TR/RB

  // TRIGGER DIGITALI (compatibilità massima!)
  if (L.last_btn_main & 0x80) b |= (1u << 8);  // ZL → TL2/LT button
  if (R.last_btn_main & 0x80) b |= (1u << 9);  // ZR → TR2/RT button

  // System buttons
  if (L.last_btn_shared & 0x01) b |= (1u << 10);  // Minus → SELECT/Back
  if (R.last_btn_shared & 0x02) b |= (1u << 11);  // Plus → START
  if (R.last_btn_shared & 0x10) b |= (1u << 12);  // Home → MODE/Guide

  // Stick clicks
  if (L.last_btn_shared & 0x08) b |= (1u << 13);  // L3 → THUMBL
  if (R.last_btn_shared & 0x04) b |= (1u << 14);  // R3 → THUMBR

  // Extra (Capture)
  if (L.last_btn_shared & 0x20) b |= (1u << 2);  // Capture → button C

  // ====== SEND - ORDINE STANDARD ======
  gamepad.send(
    lx, ly,    // Left stick
    rx_stick,  // Right stick X
    rz_trig,   // Right trigger analog
    z_trig,    // Left trigger analog
    ry_stick,  // Right stick Y
    hat,       // D-pad
    b          // Buttons
  );
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {

  delay(1500);

#if ENABLE_SERIAL_LOG
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== DUAL JOYCON + HID READY ===");
#endif

  // USB HID
  gamepad.begin();
  USB.begin();


  // LED
  led.begin();
  led.setBrightness(32);
  setLedRGB(35, 0, 0);

  // JRST pin
  pinMode(JCL_JRST_PIN, OUTPUT);
  digitalWrite(JCL_JRST_PIN, LOW);
  pinMode(JCR_JRST_PIN, OUTPUT);
  digitalWrite(JCR_JRST_PIN, LOW);

  // UART
  uart_config_t cfg = {};
  cfg.baud_rate = BAUD_1M;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.rx_flow_ctrl_thresh = 0;
  cfg.source_clk = UART_SCLK_APB;

  // Left
  uart_param_config(L.uart, &cfg);
  uart_set_pin(L.uart, L.txPin, L.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(L.uart, RX_BUF_SIZE, RX_BUF_SIZE, 0, NULL, 0);
  uart_set_line_inverse(L.uart, UART_SIGNAL_TXD_INV);

  // Right
  uart_param_config(R.uart, &cfg);
  uart_set_pin(R.uart, R.txPin, R.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(R.uart, RX_BUF_SIZE, RX_BUF_SIZE, 0, NULL, 0);
  uart_set_line_inverse(R.uart, UART_SIGNAL_TXD_INV);

  // Queue
  L.q = xQueueCreate(60, sizeof(FrameMsg));
  R.q = xQueueCreate(60, sizeof(FrameMsg));

  // Task
  xTaskCreatePinnedToCore(taskUART_Left, "uartL", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(taskUART_Right, "uartR", 4096, NULL, 3, NULL, 1);

  L.state = JoyCon::DETECT;
  L.tDetectStart = millis();
  L.tLastDetectTx = 0;
  R.state = JoyCon::DETECT;
  R.tDetectStart = millis();
  R.tLastDetectTx = 0;
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {

  processJoyCon(L);
  processJoyCon(R);

  // LED STATE
  uint32_t now = millis();
  bool leftOk = (L.state == JoyCon::POLL);
  bool rightOk = (R.state == JoyCon::POLL);

  bool handshake =
    (L.state != JoyCon::POLL) || (R.state != JoyCon::POLL);

  bool recentError = (now - g_lastErrorMs) < 1500;

  if (recentError) setLedRGB(255, 0, 0);
  else if (handshake) setLedRGB(220, 180, 0);
  else if (leftOk && rightOk) setLedRGB(120, 0, 120);
  else if (leftOk) setLedRGB(0, 0, 160);
  else if (rightOk) setLedRGB(0, 160, 0);
  else setLedRGB(35, 0, 0);

  // ---- HID SEND ----
  sendHIDState();

  delay(1);
}
