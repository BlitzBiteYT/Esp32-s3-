/*
 * ESP32-S3 hardware diagnostics: nRF24L01(+) and microSD module
 * Pins match the Bruce diy-s3-st7735 build and the wiring diagram.
 *
 * ARDUINO IDE SETTINGS (Tools menu)
 *   Board ............ ESP32S3 Dev Module
 *   USB CDC On Boot .. Disabled   (then Serial = the COM/CH343 port)
 *   Flash Size ....... 16MB (128Mb)
 *   PSRAM ............ OPI PSRAM
 *   Upload Speed ..... 921600 (or 115200 if uploads fail)
 * Open the serial terminal at 115200 baud. No extra libraries are needed.
 * If you leave USB CDC On Boot enabled, this sketch prints to BOTH ports.
 *
 * PLATFORMIO: copy this file to src/main.cpp and build as usual.
 */
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <esp_system.h>
#include <stdarg.h>
#include <string.h>

#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT == 1)
#define DUAL_PORT 1
HardwareSerial uart0(0);
#else
#define DUAL_PORT 0
#endif

#define TFT_CS  7
#define SD_CS   3
#define SD_SCK  18
#define SD_MISO 8
#define SD_MOSI 17
#define R_SCK   13
#define R_MISO  11
#define R_MOSI  12
#define NRF_CSN 14
#define NRF_CE  21
#define CC_CSN  46

SPIClass sdspi(HSPI);
static uint32_t radioHz = 1000000;
static uint8_t nrfStatus = 0;

static void out(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
#if DUAL_PORT
  uart0.print(buf);
#endif
}

static bool keyPressed() {
  bool k = false;
  while (Serial.available()) { Serial.read(); k = true; }
#if DUAL_PORT
  while (uart0.available()) { uart0.read(); k = true; }
#endif
  return k;
}

static void busesBegin() {
  SPI.begin(R_SCK, R_MISO, R_MOSI, -1);
  sdspi.begin(SD_SCK, SD_MISO, SD_MOSI, -1);
}

static void idleSelects() {
  const int cs[] = {TFT_CS, SD_CS, NRF_CSN, CC_CSN};
  for (int i = 0; i < 4; i++) { pinMode(cs[i], OUTPUT); digitalWrite(cs[i], HIGH); }
  pinMode(NRF_CE, OUTPUT);
  digitalWrite(NRF_CE, LOW);
}

static const char *resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_EXT:      return "reset button";
    case ESP_RST_SW:       return "software reset";
    case ESP_RST_PANIC:    return "CRASH (panic)";
    case ESP_RST_INT_WDT:  return "watchdog";
    case ESP_RST_TASK_WDT: return "watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power dipped, fix the supply!)";
    default:               return "other";
  }
}

/* ---------------- nRF24 low level ---------------- */
static uint8_t nrfRead(uint8_t reg) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(reg & 0x1F);
  uint8_t v = SPI.transfer(0xFF);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
  return v;
}

static void nrfWrite(uint8_t reg, uint8_t v) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(0x20 | (reg & 0x1F));
  SPI.transfer(v);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
}

static void nrfReadBuf(uint8_t reg, uint8_t *b, uint8_t n) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(reg & 0x1F);
  for (uint8_t i = 0; i < n; i++) b[i] = SPI.transfer(0xFF);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
}

static void nrfWriteBuf(uint8_t reg, const uint8_t *b, uint8_t n) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(0x20 | (reg & 0x1F));
  for (uint8_t i = 0; i < n; i++) SPI.transfer(b[i]);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
}

static void nrfCmd(uint8_t c) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(c);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
}

static void nrfWritePayload(const uint8_t *b, uint8_t n) {
  digitalWrite(NRF_CSN, LOW);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  nrfStatus = SPI.transfer(0xA0);
  for (uint8_t i = 0; i < n; i++) SPI.transfer(b[i]);
  SPI.endTransaction();
  digitalWrite(NRF_CSN, HIGH);
}

/* ---------------- CC1101 (quick ID only) ---------------- */
static uint8_t ccStatusReg(uint8_t addr) {
  digitalWrite(CC_CSN, LOW);
  delayMicroseconds(20);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  SPI.transfer(0xC0 | addr);
  uint8_t v = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(CC_CSN, HIGH);
  return v;
}

static void ccReset() {
  digitalWrite(CC_CSN, HIGH); delay(2);
  digitalWrite(CC_CSN, LOW);  delay(2);
  digitalWrite(CC_CSN, HIGH); delay(2);
  digitalWrite(CC_CSN, LOW);  delay(2);
  SPI.beginTransaction(SPISettings(radioHz, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x30);
  SPI.endTransaction();
  delay(5);
  digitalWrite(CC_CSN, HIGH); delay(5);
}

/* ---------------- results kept for the summary ---------------- */
static int pinProblems = 0;
static bool nrfAnswers = false, nrfWritesOk = false, nrfTxOk = false, nrfRxHeard = false;
static int nrfTxCount = 0, nrfRxTotal = 0;
static bool sdRawOk = false, sdMountOk = false, sdReadOk = false;
static uint8_t sdCmd0 = 0xFF;
static uint32_t sdBestHz = 0;

/* ---------------- [2] wire check ---------------- */
struct PinDef { int pin; const char *name; };
static const PinDef PINS[] = {
  {NRF_CE, "nRF24 CE"}, {NRF_CSN, "nRF24 CSN"}, {R_SCK, "radio SCK"}, {R_MOSI, "radio MOSI"}, {R_MISO, "radio MISO"},
  {SD_CS, "SD CS"}, {SD_SCK, "SD SCK"}, {SD_MOSI, "SD MOSI"}, {SD_MISO, "SD MISO"},
  {CC_CSN, "CC1101 CSN"}, {TFT_CS, "TFT CS"}
};

static void pinCheck() {
  out("\n[2] WIRE CHECK (idle levels of every wire)\n");
  SPI.end();
  sdspi.end();
  pinProblems = 0;
  for (unsigned i = 0; i < sizeof(PINS) / sizeof(PINS[0]); i++) {
    pinMode(PINS[i].pin, INPUT_PULLUP);
    delay(3);
    int up = digitalRead(PINS[i].pin);
    pinMode(PINS[i].pin, INPUT_PULLDOWN);
    delay(3);
    int dn = digitalRead(PINS[i].pin);
    const char *v;
    if (up == 0 && dn == 0)      { v = "HELD LOW: short to GND or driven low  <-- suspect"; pinProblems++; }
    else if (up == 1 && dn == 1) { v = "held high (module pull-up is normal; short to 3V3 if not)"; }
    else if (up == 1 && dn == 0) { v = "floating/free (ok)"; }
    else                         { v = "odd reading (loose or noisy wire)"; pinProblems++; }
    out("  GPIO %2d %-11s %s\n", PINS[i].pin, PINS[i].name, v);
  }
  idleSelects();
  busesBegin();
}

static bool loopTest(SPIClass &s) {
  s.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  int ok = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t t = 0xA5 ^ (i * 37);
    if (s.transfer(t) == t) ok++;
  }
  s.endTransaction();
  return ok == 8;
}

/* ---------------- [3] nRF24 ---------------- */
static void nrfDump() {
  struct R { uint8_t reg; const char *n; uint8_t def; };
  static const R regs[] = {
    {0x00, "CONFIG", 0x08}, {0x01, "EN_AA", 0x3F}, {0x02, "EN_RXADDR", 0x03}, {0x03, "SETUP_AW", 0x03},
    {0x04, "SETUP_RETR", 0x03}, {0x05, "RF_CH", 0x02}, {0x06, "RF_SETUP", 0x0E}, {0x07, "STATUS", 0x0E},
    {0x17, "FIFO_STATUS", 0x11}
  };
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    uint8_t v = nrfRead(regs[i].reg);
    out("  %-11s = 0x%02X   (fresh chip: 0x%02X)%s\n", regs[i].n, v, regs[i].def, v == regs[i].def ? "  ok" : "  differs");
  }
}

static void nrfSpeedTest() {
  const uint32_t hz[] = {1000000, 2000000, 4000000, 8000000};
  for (int i = 0; i < 4; i++) {
    radioHz = hz[i];
    int ok = 0;
    for (int k = 0; k < 50; k++) if (nrfRead(0x03) == 0x03) ok++;
    out("  %lu MHz: %d/50 reads correct\n", (unsigned long)(hz[i] / 1000000), ok);
  }
  radioHz = 1000000;
}

static bool nrfWriteTest() {
  uint8_t oldCh = nrfRead(0x05);
  nrfWrite(0x05, 0x4C);
  uint8_t rb = nrfRead(0x05);
  nrfWrite(0x05, (oldCh == 0xFF || oldCh == 0x00) ? 0x02 : oldCh);
  const uint8_t pat[5] = {0xA5, 0x5A, 0xA5, 0x5A, 0xC3};
  const uint8_t def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t rd[5] = {0, 0, 0, 0, 0};
  nrfWriteBuf(0x10, pat, 5);
  nrfReadBuf(0x10, rd, 5);
  nrfWriteBuf(0x10, def, 5);
  bool a = (rb == 0x4C);
  bool b = (memcmp(pat, rd, 5) == 0);
  out("  write RF_CH=0x4C, read back 0x%02X ........ %s\n", rb, a ? "ok" : "FAIL");
  out("  write TX_ADDR A5 5A A5 5A C3, read back %02X %02X %02X %02X %02X ... %s\n", rd[0], rd[1], rd[2], rd[3], rd[4], b ? "ok" : "FAIL");
  return a && b;
}

static int nrfTxBurst() {
  nrfWrite(0x01, 0x00);
  nrfWrite(0x04, 0x00);
  nrfWrite(0x05, 40);
  nrfWrite(0x06, 0x06);
  nrfWrite(0x00, 0x0A);
  delay(5);
  nrfCmd(0xE1);
  nrfWrite(0x07, 0x70);
  uint8_t pl[32];
  memset(pl, 0xA5, sizeof(pl));
  int ok = 0;
  for (int i = 0; i < 20; i++) {
    nrfWritePayload(pl, 32);
    digitalWrite(NRF_CE, HIGH);
    delayMicroseconds(20);
    digitalWrite(NRF_CE, LOW);
    delay(3);
    uint8_t st = nrfRead(0x07);
    if (st & 0x20) ok++;
    if (st & 0x10) nrfCmd(0xE1);
    nrfWrite(0x07, 0x70);
  }
  nrfWrite(0x00, 0x08);
  return ok;
}

static int nrfScan() {
  nrfWrite(0x01, 0x00);
  nrfWrite(0x06, 0x06);
  nrfWrite(0x00, 0x0B);
  delay(5);
  uint16_t hits[126];
  memset(hits, 0, sizeof(hits));
  for (int sweep = 0; sweep < 15; sweep++) {
    for (int ch = 0; ch < 126; ch++) {
      nrfWrite(0x05, ch);
      digitalWrite(NRF_CE, HIGH);
      delayMicroseconds(200);
      uint8_t r = nrfRead(0x09);
      digitalWrite(NRF_CE, LOW);
      if (r & 1) hits[ch]++;
    }
  }
  nrfWrite(0x00, 0x08);
  int total = 0;
  out("  signal hits per 10-channel group (15 sweeps):\n  ");
  for (int g = 0; g < 126; g += 10) {
    int s = 0;
    for (int c = g; c < g + 10 && c < 126; c++) s += hits[c];
    total += s;
    out("%d-%d:%d  ", g, (g + 9 > 125) ? 125 : g + 9, s);
  }
  out("\n");
  return total;
}

static void nrfTests() {
  out("\n[3] nRF24L01 TEST (CE=%d CSN=%d SCK=%d MOSI=%d MISO=%d)\n", NRF_CE, NRF_CSN, R_SCK, R_MOSI, R_MISO);
  digitalWrite(CC_CSN, HIGH);
  digitalWrite(NRF_CE, LOW);
  radioHz = 1000000;
  uint8_t aw = nrfRead(0x03);
  uint8_t st = nrfStatus;
  out("  first reply byte (STATUS) = 0x%02X, SETUP_AW = 0x%02X (want 0x03)\n", st, aw);
  nrfAnswers = (aw == 0x03);
  if (aw == 0xFF && st == 0xFF) {
    out("  >> NO ANSWER, line reads all 1s: MISO(11) not connected, CSN(14) wrong/loose, module has no power, or GND missing.\n");
  } else if (aw == 0x00 && st == 0x00) {
    out("  >> NO ANSWER, line reads all 0s: MISO(11) shorted to GND, or module unpowered/ground missing.\n");
  } else if (!nrfAnswers) {
    out("  >> Wrong data: MOSI(12)/MISO(11) swapped or loose, or the CC1101 is fighting for MISO (unplug it).\n");
  } else {
    out("  >> The chip answers like a real nRF24.\n");
  }
  out("  register dump:\n");
  nrfDump();
  if (!nrfAnswers) {
    nrfWritesOk = false; nrfTxOk = false; nrfRxHeard = false;
    out("  (skipping the other nRF24 tests until it answers)\n");
    return;
  }
  out("  SPI speed margin (longer or noisy wires fail at higher speeds):\n");
  nrfSpeedTest();
  out("  write and read-back test:\n");
  nrfWritesOk = nrfWriteTest();
  out("  transmit burst, 20 packets at max power (also stresses the 3V3 supply):\n");
  nrfTxCount = nrfTxBurst();
  nrfTxOk = (nrfTxCount == 20);
  out("  packets sent: %d/20 %s\n", nrfTxCount, nrfTxOk ? "ok" : "<-- radio or power problem (add 100 uF at the module)");
  out("  receive scan of the 2.4 GHz band (WiFi and other traffic should show up):\n");
  nrfRxTotal = nrfScan();
  nrfRxHeard = (nrfRxTotal > 0);
  out("  total hits: %d %s\n", nrfRxTotal, nrfRxHeard ? "(receiver works)" : "(nothing heard: quiet air, or antenna/receiver problem)");
}

static void ccQuick() {
  out("\n[4] CC1101 QUICK CHECK (CSN=%d)\n", CC_CSN);
  ccReset();
  uint8_t pn = ccStatusReg(0x30), ver = ccStatusReg(0x31);
  out("  PARTNUM=0x%02X VERSION=0x%02X (usually 0x14) -> %s\n", pn, ver, (ver == 0x00 || ver == 0xFF) ? "NO ANSWER (or not connected)" : "answers");
}

/* ---------------- SD raw SPI-mode test ---------------- */
static uint8_t sdX(uint8_t b) { return sdspi.transfer(b); }

static uint8_t sdCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
  sdX(0xFF);
  sdX(0x40 | cmd);
  sdX((uint8_t)(arg >> 24));
  sdX((uint8_t)(arg >> 16));
  sdX((uint8_t)(arg >> 8));
  sdX((uint8_t)arg);
  sdX(crc);
  uint8_t r = 0xFF;
  for (int i = 0; i < 10; i++) { r = sdX(0xFF); if (!(r & 0x80)) break; }
  return r;
}

static bool sdRaw() {
  out("\n[5] SD RAW TEST (400 kHz, talks to the card directly, CS=%d SCK=%d MOSI=%d MISO=%d)\n", SD_CS, SD_SCK, SD_MOSI, SD_MISO);
  digitalWrite(SD_CS, HIGH);
  sdspi.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 10; i++) sdX(0xFF);
  uint8_t idle[4];
  for (int i = 0; i < 4; i++) idle[i] = sdX(0xFF);
  out("  MISO idle bytes (CS high): %02X %02X %02X %02X\n", idle[0], idle[1], idle[2], idle[3]);

  uint8_t r1 = 0xFF;
  int tries = 0;
  for (; tries < 5; tries++) {
    digitalWrite(SD_CS, LOW);
    r1 = sdCmd(0, 0, 0x95);
    digitalWrite(SD_CS, HIGH);
    sdX(0xFF);
    if (r1 == 0x01) break;
    delay(10);
  }
  sdCmd0 = r1;
  out("  CMD0 (reset)   -> 0x%02X (want 0x01) after %d tries\n", r1, (r1 == 0x01) ? tries + 1 : 5);
  if (r1 != 0x01) {
    if (r1 == 0xFF)      out("  >> NO RESPONSE, MISO reads all 1s: card missing or not seated, module unpowered (5V vs 3V3 type?), MISO(8) wire loose, CS(3) wire wrong, or SCK/MOSI wires loose.\n");
    else if (r1 == 0x00) out("  >> MISO stuck LOW: MISO(8) shorted to GND or the module is damaged.\n");
    else                 out("  >> Unexpected reply: noisy line, wrong pin order, or a 5V module without a level shifter.\n");
    sdspi.endTransaction();
    return false;
  }

  digitalWrite(SD_CS, LOW);
  uint8_t r8 = sdCmd(8, 0x1AA, 0x87);
  uint8_t echo[4] = {0, 0, 0, 0};
  bool v2 = (r8 == 0x01);
  if (v2) for (int i = 0; i < 4; i++) echo[i] = sdX(0xFF);
  digitalWrite(SD_CS, HIGH);
  sdX(0xFF);
  if (v2) out("  CMD8 (voltage) -> 0x%02X, echo %02X%02X%02X%02X (want 000001AA)\n", r8, echo[0], echo[1], echo[2], echo[3]);
  else    out("  CMD8 (voltage) -> 0x%02X (old SD v1 or MMC card)\n", r8);

  uint32_t t0 = millis();
  uint8_t ra = 0xFF;
  do {
    digitalWrite(SD_CS, LOW);  sdCmd(55, 0, 0x65);                       digitalWrite(SD_CS, HIGH); sdX(0xFF);
    digitalWrite(SD_CS, LOW);  ra = sdCmd(41, v2 ? 0x40000000UL : 0, 0x77); digitalWrite(SD_CS, HIGH); sdX(0xFF);
    if (ra == 0x00) break;
    delay(5);
  } while (millis() - t0 < 1500);
  out("  ACMD41 (init)  -> 0x%02X (want 0x00) in %lu ms\n", ra, (unsigned long)(millis() - t0));
  if (ra != 0x00) {
    out("  >> Card answered but would not finish starting up: weak 3V3/5V supply, or a worn/fake card. Try another card.\n");
    sdspi.endTransaction();
    return false;
  }

  uint8_t ocr[4] = {0, 0, 0, 0};
  digitalWrite(SD_CS, LOW);
  uint8_t r58 = sdCmd(58, 0, 0xFD);
  if (r58 == 0x00) for (int i = 0; i < 4; i++) ocr[i] = sdX(0xFF);
  digitalWrite(SD_CS, HIGH);
  sdX(0xFF);
  out("  CMD58 (OCR)    -> 0x%02X, OCR %02X%02X%02X%02X, card type %s\n", r58, ocr[0], ocr[1], ocr[2], ocr[3], (ocr[0] & 0x40) ? "SDHC/SDXC" : "SDSC");

  uint8_t csd[16];
  memset(csd, 0, sizeof(csd));
  digitalWrite(SD_CS, LOW);
  uint8_t r9 = sdCmd(9, 0, 0xAF);
  bool gotCsd = false;
  if (r9 == 0x00) {
    for (int i = 0; i < 4000; i++) { if (sdX(0xFF) == 0xFE) { gotCsd = true; break; } }
    if (gotCsd) { for (int i = 0; i < 16; i++) csd[i] = sdX(0xFF); sdX(0xFF); sdX(0xFF); }
  }
  digitalWrite(SD_CS, HIGH);
  sdX(0xFF);
  if (gotCsd && (csd[0] >> 6) == 1) {
    uint32_t csize = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
    out("  CMD9 (size)    -> capacity about %lu MB\n", (unsigned long)((csize + 1) / 2));
  } else {
    out("  CMD9 (size)    -> 0x%02X %s\n", r9, gotCsd ? "(older card, size not decoded)" : "(no data token)");
  }
  uint8_t sec[512];
  memset(sec, 0, sizeof(sec));
  digitalWrite(SD_CS, LOW);
  uint8_t r17 = sdCmd(17, 0, 0xFF);
  bool gotSec = false;
  if (r17 == 0x00) {
    for (int i = 0; i < 20000; i++) { if (sdX(0xFF) == 0xFE) { gotSec = true; break; } }
    if (gotSec) { for (int i = 0; i < 512; i++) sec[i] = sdX(0xFF); sdX(0xFF); sdX(0xFF); }
  }
  digitalWrite(SD_CS, HIGH);
  sdX(0xFF);
  sdReadOk = gotSec;
  if (gotSec) out("  CMD17 (read sector 0) -> ok, signature bytes %02X %02X (want 55 AA)%s\n", sec[510], sec[511], (sec[510] == 0x55 && sec[511] == 0xAA) ? "" : "  <-- card looks unformatted: format it as FAT32");
  else        out("  CMD17 (read sector 0) -> 0x%02X, no data token\n", r17);
  sdspi.endTransaction();
  out("  >> The card talks to the ESP32 correctly. Wiring and power are fine.\n");
  return true;
}

/* ---------------- SD library mount test ---------------- */
static const char *cardName(uint8_t t) {
  switch (t) {
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SD";
    case CARD_SDHC: return "SDHC";
    case CARD_NONE: return "none";
    default:        return "unknown";
  }
}

static void sdMount() {
  out("\n[6] SD LIBRARY MOUNT TEST (the FAT filesystem Bruce uses; Bruce runs at 4000 kHz)\n");
  const uint32_t f[] = {400000, 1000000, 4000000, 10000000, 20000000};
  sdMountOk = false;
  sdBestHz = 0;
  bool shown = false;
  for (int i = 0; i < 5; i++) {
    SD.end();
    bool ok = SD.begin(SD_CS, sdspi, f[i]);
    out("  %5lu kHz: %s\n", (unsigned long)(f[i] / 1000), ok ? "MOUNTED" : "failed");
    if (ok) {
      sdMountOk = true;
      sdBestHz = f[i];
      if (!shown) {
        shown = true;
        out("    type %s, size %llu MB\n", cardName(SD.cardType()), (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
      }
    }
  }
  SD.end();
}

/* ---------------- run everything ---------------- */
static void runAll() {
  out("\n=========================================================\n");
  out(" ESP32-S3 DIAGNOSTICS: nRF24 + microSD   (uptime %lu s)\n", (unsigned long)(millis() / 1000));
  out("=========================================================\n");
  out("[1] BOARD\n");
  out("  chip %s, %lu MHz, flash %lu MB, PSRAM %lu MB, free heap %lu KB\n", ESP.getChipModel(), (unsigned long)ESP.getCpuFreqMHz(),
      (unsigned long)(ESP.getFlashChipSize() / (1024UL * 1024UL)), (unsigned long)(ESP.getPsramSize() / (1024UL * 1024UL)), (unsigned long)(ESP.getFreeHeap() / 1024UL));
  esp_reset_reason_t rr = esp_reset_reason();
  out("  last reset reason: %s\n", resetName(rr));

  pinCheck();
  bool lb1 = loopTest(SPI);
  bool lb2 = loopTest(sdspi);
  out("  SPI loopback (true only if you jumpered MOSI to MISO with modules unplugged): radio bus %s, SD bus %s\n", lb1 ? "YES" : "no", lb2 ? "YES" : "no");

  nrfTests();
  ccQuick();
  sdRawOk = sdRaw();
  sdMount();

  out("\n[7] SUMMARY\n");
  out("  Wires held low/odd ........ %d %s\n", pinProblems, pinProblems ? "<-- fix first" : "(none)");
  out("  nRF24 answers ............. %s\n", nrfAnswers ? "YES" : "NO");
  if (nrfAnswers) {
    out("  nRF24 write/read-back ..... %s\n", nrfWritesOk ? "ok" : "FAIL (MOSI wire?)");
    out("  nRF24 transmit burst ...... %d/20 %s\n", nrfTxCount, nrfTxOk ? "ok" : "(power problem?)");
    out("  nRF24 receive scan ........ %s\n", nrfRxHeard ? "heard signals" : "heard nothing");
  }
  out("  SD card talks (raw) ....... %s (CMD0 reply 0x%02X)\n", sdRawOk ? "YES" : "NO", sdCmd0);
  if (sdRawOk) out("  SD sector read ............ %s\n", sdReadOk ? "ok" : "FAIL");
  out("  SD filesystem mounts ...... %s", sdMountOk ? "YES" : "NO");
  if (sdMountOk) out(" (up to %lu kHz)", (unsigned long)(sdBestHz / 1000));
  out("\n");
  if (rr == ESP_RST_BROWNOUT) out("  !! The board browned out at the last reset: use a 5V 1A+ wall charger, short thick cable, 470-1000 uF across 5V and GND.\n");
  if (sdRawOk && !sdMountOk) out("  !! Card talks but FAT will not mount: reformat it as FAT32 (32 GB or smaller).\n");
  if (sdMountOk && sdBestHz < 4000000) out("  !! SD only works at low speed: shorten and tidy the SD wires. Bruce needs 4000 kHz.\n");
  if (nrfAnswers && !nrfTxOk) out("  !! nRF24 answers but fails the transmit burst: add 100 uF across its VCC/GND and use a stronger 3V3 source.\n");
  out("\nSend any key to rerun now. It reruns by itself in 20 s so you can wiggle wires.\n");
}

void setup() {
  Serial.begin(115200);
#if DUAL_PORT
  uart0.begin(115200, SERIAL_8N1, 44, 43);
#endif
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);
  idleSelects();
  busesBegin();
}

void loop() {
  runAll();
  uint32_t t = millis();
  while (millis() - t < 20000) {
    if (keyPressed()) break;
    delay(20);
  }
}
