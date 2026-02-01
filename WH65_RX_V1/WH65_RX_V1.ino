/*
 * WH65 Weather Station Receiver V1
 * ESP32-C3 Super-mini + CC1101 + PMS7003
 *
 * CC1101 Pins: MISO=GPIO21, MOSI=GPIO20, CLK=GPIO2, CSN=GPIO3
 * PMS7003:     UART RX=GPIO4
 * RGB LED:     GPIO8 (built-in WS2812)
 * Board:       esp32:esp32:esp32c3
 */

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
// neopixelWrite() is built into ESP32 Arduino core

// ─── Pin Definitions ────────────────────────────────────────────────
#define CC1101_MISO   21
#define CC1101_MOSI   20
#define CC1101_CLK    2
#define CC1101_CSN    3
#define PMS_RX_PIN    4
#define RGB_LED_PIN   8

// ─── CC1101 Register Addresses ──────────────────────────────────────
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_SYNC1    0x04
#define CC1101_SYNC0    0x05
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_ADDR     0x09
#define CC1101_CHANNR   0x0A
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM2    0x16
#define CC1101_MCSM1    0x17
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E

// CC1101 Strobe Commands
#define CC1101_SRES     0x30
#define CC1101_SRX      0x34
#define CC1101_SIDLE    0x36
#define CC1101_SFRX     0x3A

// CC1101 Status Registers
#define CC1101_RXBYTES  0x3B
#define CC1101_MARCSTATE 0x35

// CC1101 FIFO
#define CC1101_RXFIFO   0x3F

// ─── Global Data Structures ────────────────────────────────────────
struct WH65Data {
  float tempC;
  uint8_t humidity;
  uint16_t windDir;
  float windSpeed;
  float windGust;
  float rainMM;
  uint8_t uvIndex;
  uint32_t lux;
  uint8_t battery;
  bool valid;
  uint32_t lastUpdate;
};

struct PMSData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
  uint16_t p03;
  uint16_t p05;
  uint16_t p10;
  uint16_t p25;
  uint16_t p50;
  uint16_t p100;
  bool valid;
  uint32_t lastUpdate;
};

struct Config {
  char ssid[33];
  char pass[65];
  char mqttHost[65];
  uint16_t mqttPort;
  char mqttUser[33];
  char mqttPass[65];
  char mqttTopic[65];
  uint16_t freq;  // MHz (915 default)
};

static WH65Data wh65;
static PMSData pms;
static Config cfg;

// ─── Objects ────────────────────────────────────────────────────────
static SPIClass spiCC(FSPI);
static WebServer server(80);
static DNSServer dnsServer;
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static Preferences prefs;

static bool apMode = false;
static uint8_t ledColorIdx = 0;
static uint32_t lastMqttReconnect = 0;

// ─── RGB LED (WS2812 via built-in neopixelWrite) ────────────────────
static void setLED(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_LED_PIN, r, g, b);
}

static void ledOff() { setLED(0, 0, 0); }

// 8-color cycle for WH65 packets
static void ledCycleColor() {
  static const uint8_t colors[][3] = {
    {30,0,0},{0,30,0},{0,0,30},{30,30,0},
    {30,0,30},{0,30,30},{30,15,0},{15,0,30}
  };
  setLED(colors[ledColorIdx][0], colors[ledColorIdx][1], colors[ledColorIdx][2]);
  ledColorIdx = (ledColorIdx + 1) & 7;
}

// Brief green pulse for PMS
static void ledPMSPulse() {
  setLED(0, 30, 0);
}

// ─── CC1101 SPI Driver ─────────────────────────────────────────────
static void cc1101Select()   { digitalWrite(CC1101_CSN, LOW); }
static void cc1101Deselect() { digitalWrite(CC1101_CSN, HIGH); }

static uint8_t cc1101Strobe(uint8_t strobe) {
  cc1101Select();
  uint8_t s = spiCC.transfer(strobe);
  cc1101Deselect();
  return s;
}

static void cc1101WriteReg(uint8_t addr, uint8_t val) {
  cc1101Select();
  spiCC.transfer(addr);
  spiCC.transfer(val);
  cc1101Deselect();
}

static uint8_t cc1101ReadReg(uint8_t addr) {
  cc1101Select();
  spiCC.transfer(addr | 0x80);
  uint8_t val = spiCC.transfer(0);
  cc1101Deselect();
  return val;
}

static uint8_t cc1101ReadStatus(uint8_t addr) {
  cc1101Select();
  spiCC.transfer(addr | 0xC0);
  uint8_t val = spiCC.transfer(0);
  cc1101Deselect();
  return val;
}

static void cc1101ReadBurst(uint8_t addr, uint8_t *buf, uint8_t len) {
  cc1101Select();
  spiCC.transfer(addr | 0xC0);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = spiCC.transfer(0);
  }
  cc1101Deselect();
}

static bool cc1101Init() {
  pinMode(CC1101_CSN, OUTPUT);
  cc1101Deselect();
  delay(1);
  cc1101Select();
  delay(1);
  cc1101Deselect();
  delay(40);

  cc1101Strobe(CC1101_SRES);
  delay(10);

  // Verify chip: read IOCFG2 default = 0x29
  uint8_t ver = cc1101ReadReg(CC1101_IOCFG2);
  if (ver != 0x29) {
    Serial.printf("CC1101 not found (IOCFG2=0x%02X)\n", ver);
    return false;
  }

  // Configure for Fine Offset WH65: 2-FSK, 17.24kbps, deviation ~40kHz
  // Frequency: configurable (default 915MHz)
  // 915MHz: FREQ2=0x23, FREQ1=0x31, FREQ0=0x3B
  // 868MHz: FREQ2=0x21, FREQ1=0x62, FREQ0=0x76
  uint8_t f2, f1, f0;
  if (cfg.freq == 868) {
    f2 = 0x21; f1 = 0x62; f0 = 0x76;
  } else {
    f2 = 0x23; f1 = 0x31; f0 = 0x3B;
  }

  cc1101WriteReg(CC1101_IOCFG2,   0x01);  // GDO2: asserts on sync, deasserts on pkt
  cc1101WriteReg(CC1101_IOCFG0,   0x06);  // GDO0: asserts on pkt received
  cc1101WriteReg(CC1101_FIFOTHR,  0x47);  // RX FIFO threshold
  cc1101WriteReg(CC1101_SYNC1,    0x2D);  // Sync word high
  cc1101WriteReg(CC1101_SYNC0,    0xD4);  // Sync word low
  cc1101WriteReg(CC1101_PKTLEN,   0x50);  // Max packet length 80
  cc1101WriteReg(CC1101_PKTCTRL1, 0x00);  // No addr check, no append
  cc1101WriteReg(CC1101_PKTCTRL0, 0x00);  // Fixed length off, no CRC auto
  cc1101WriteReg(CC1101_ADDR,     0x00);
  cc1101WriteReg(CC1101_CHANNR,   0x00);
  cc1101WriteReg(CC1101_FSCTRL1,  0x06);  // IF frequency
  cc1101WriteReg(CC1101_FSCTRL0,  0x00);
  cc1101WriteReg(CC1101_FREQ2,    f2);
  cc1101WriteReg(CC1101_FREQ1,    f1);
  cc1101WriteReg(CC1101_FREQ0,    f0);
  cc1101WriteReg(CC1101_MDMCFG4,  0x88);  // BW 101.6kHz, dRate exp=8
  cc1101WriteReg(CC1101_MDMCFG3,  0x4B);  // dRate mantissa → 17.24kbps
  cc1101WriteReg(CC1101_MDMCFG2,  0x02);  // 2-FSK, 16/16 sync
  cc1101WriteReg(CC1101_MDMCFG1,  0x02);  // 2 preamble bytes
  cc1101WriteReg(CC1101_MDMCFG0,  0xF8);
  cc1101WriteReg(CC1101_DEVIATN,  0x44);  // Deviation ~40kHz
  cc1101WriteReg(CC1101_MCSM2,    0x07);
  cc1101WriteReg(CC1101_MCSM1,    0x00);  // Next state after RX: IDLE
  cc1101WriteReg(CC1101_MCSM0,    0x18);  // Auto calibrate on IDLE→RX
  cc1101WriteReg(CC1101_FOCCFG,   0x16);
  cc1101WriteReg(CC1101_BSCFG,    0x6C);
  cc1101WriteReg(CC1101_AGCCTRL2, 0x03);
  cc1101WriteReg(CC1101_AGCCTRL1, 0x40);
  cc1101WriteReg(CC1101_AGCCTRL0, 0x91);
  cc1101WriteReg(CC1101_FREND1,   0x56);
  cc1101WriteReg(CC1101_FREND0,   0x10);
  cc1101WriteReg(CC1101_FSCAL3,   0xE9);
  cc1101WriteReg(CC1101_FSCAL2,   0x2A);
  cc1101WriteReg(CC1101_FSCAL1,   0x00);
  cc1101WriteReg(CC1101_FSCAL0,   0x1F);
  cc1101WriteReg(CC1101_TEST2,    0x81);
  cc1101WriteReg(CC1101_TEST1,    0x35);
  cc1101WriteReg(CC1101_TEST0,    0x09);

  // Enter RX mode
  cc1101Strobe(CC1101_SFRX);
  cc1101Strobe(CC1101_SRX);

  Serial.printf("CC1101 init OK (%dMHz)\n", cfg.freq);
  return true;
}

static void cc1101StartRX() {
  cc1101Strobe(CC1101_SIDLE);
  cc1101Strobe(CC1101_SFRX);
  cc1101Strobe(CC1101_SRX);
}

// ─── CRC-8 (polynomial 0x31, init 0x00) ────────────────────────────
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// ─── WH65 Decoder ──────────────────────────────────────────────────
/*
 * Fine Offset WH65 packet (after sync 2DD4):
 * Byte 0:    Family code (0x24 for WH65)
 * Byte 1-3:  Device ID (24-bit)
 * Byte 4:    Flags / battery
 * Byte 5-6:  Temperature (raw, 10ths °C offset)
 * Byte 7:    Humidity
 * Byte 8-9:  Wind direction (degrees)
 * Byte 10:   Wind speed (0.1 m/s)
 * Byte 11:   Wind gust (0.1 m/s)
 * Byte 12-14:Rain (0.1 mm)
 * Byte 15:   UV index
 * Byte 16-19:Light / lux
 * Byte N-1:  CRC-8
 */
static bool decodeWH65(const uint8_t *buf, uint8_t len) {
  if (len < 17) return false;

  // Check CRC on all bytes except the last
  uint8_t calcCrc = crc8(buf, len - 1);
  if (calcCrc != buf[len - 1]) {
    Serial.printf("WH65 CRC fail: calc=0x%02X got=0x%02X\n", calcCrc, buf[len-1]);
    return false;
  }

  // Family code check
  if (buf[0] != 0x24) {
    Serial.printf("WH65 unknown family: 0x%02X\n", buf[0]);
    // Still try to decode - some variants differ
  }

  // Battery: bit 2 of byte 4
  wh65.battery = (buf[4] & 0x04) ? 0 : 1;

  // Temperature: bytes 5-6, offset by 400, in 0.1°C
  int16_t tempRaw = ((buf[5] & 0x0F) << 8) | buf[6];
  wh65.tempC = (tempRaw - 400) / 10.0f;

  // Humidity
  wh65.humidity = buf[7];

  // Wind direction
  wh65.windDir = ((uint16_t)buf[8] << 8) | buf[9];
  if (wh65.windDir > 360) wh65.windDir = 0;

  // Wind speed (m/s → km/h)
  wh65.windSpeed = buf[10] * 0.36f;  // 0.1 m/s * 3.6

  // Wind gust
  wh65.windGust = buf[11] * 0.36f;

  // Rain accumulation (0.1mm)
  uint32_t rainRaw = ((uint32_t)buf[12] << 16) | ((uint32_t)buf[13] << 8) | buf[14];
  wh65.rainMM = rainRaw * 0.1f;

  // UV index
  wh65.uvIndex = buf[15];

  // Lux (4 bytes if available)
  if (len >= 20) {
    wh65.lux = ((uint32_t)buf[16] << 24) | ((uint32_t)buf[17] << 16) |
               ((uint32_t)buf[18] << 8) | buf[19];
  } else {
    wh65.lux = 0;
  }

  wh65.valid = true;
  wh65.lastUpdate = millis();

  Serial.printf("WH65: %.1fC %d%% wind=%d°@%.1fkm/h gust=%.1f rain=%.1f UV=%d lux=%lu bat=%d\n",
    wh65.tempC, wh65.humidity, wh65.windDir, wh65.windSpeed,
    wh65.windGust, wh65.rainMM, wh65.uvIndex, wh65.lux, wh65.battery);

  return true;
}

// ─── CC1101 Receive Check ───────────────────────────────────────────
static void cc1101CheckRX() {
  uint8_t rxBytes = cc1101ReadStatus(CC1101_RXBYTES);
  if (rxBytes & 0x80) {
    // Overflow
    cc1101StartRX();
    return;
  }
  if (rxBytes == 0) return;

  // Wait briefly for full packet
  delay(10);
  rxBytes = cc1101ReadStatus(CC1101_RXBYTES);
  if (rxBytes == 0 || (rxBytes & 0x80)) {
    cc1101StartRX();
    return;
  }

  if (rxBytes > 80) rxBytes = 80;

  uint8_t buf[80];
  cc1101ReadBurst(CC1101_RXFIFO, buf, rxBytes);

  Serial.printf("RX %d bytes:", rxBytes);
  for (uint8_t i = 0; i < rxBytes; i++) Serial.printf(" %02X", buf[i]);
  Serial.println();

  if (decodeWH65(buf, rxBytes)) {
    ledCycleColor();
  }

  cc1101StartRX();
}

// ─── PMS7003 Reader ─────────────────────────────────────────────────
static HardwareSerial pmsSerial(1);
static uint8_t pmsBuf[32];
static uint8_t pmsIdx = 0;

static void pmsInit() {
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, -1);
  Serial.println("PMS7003 UART init on GPIO4");
}

static void pmsRead() {
  while (pmsSerial.available()) {
    uint8_t c = pmsSerial.read();

    if (pmsIdx == 0 && c != 0x42) continue;
    if (pmsIdx == 1 && c != 0x4D) { pmsIdx = 0; continue; }

    pmsBuf[pmsIdx++] = c;

    if (pmsIdx == 32) {
      pmsIdx = 0;

      // Verify checksum
      uint16_t ckSum = 0;
      for (int i = 0; i < 30; i++) ckSum += pmsBuf[i];
      uint16_t rxCk = ((uint16_t)pmsBuf[30] << 8) | pmsBuf[31];

      if (ckSum != rxCk) {
        Serial.println("PMS checksum fail");
        return;
      }

      // Standard particle (atmospheric environment)
      pms.pm1_0 = ((uint16_t)pmsBuf[10] << 8) | pmsBuf[11];
      pms.pm2_5 = ((uint16_t)pmsBuf[12] << 8) | pmsBuf[13];
      pms.pm10  = ((uint16_t)pmsBuf[14] << 8) | pmsBuf[15];

      // Particle counts per 0.1L
      pms.p03  = ((uint16_t)pmsBuf[16] << 8) | pmsBuf[17];
      pms.p05  = ((uint16_t)pmsBuf[18] << 8) | pmsBuf[19];
      pms.p10  = ((uint16_t)pmsBuf[20] << 8) | pmsBuf[21];
      pms.p25  = ((uint16_t)pmsBuf[22] << 8) | pmsBuf[23];
      pms.p50  = ((uint16_t)pmsBuf[24] << 8) | pmsBuf[25];
      pms.p100 = ((uint16_t)pmsBuf[26] << 8) | pmsBuf[27];

      pms.valid = true;
      pms.lastUpdate = millis();

      Serial.printf("PMS: PM1=%d PM2.5=%d PM10=%d\n", pms.pm1_0, pms.pm2_5, pms.pm10);
      ledPMSPulse();
    }
  }
}

// ─── NVS Config ─────────────────────────────────────────────────────
static void loadConfig() {
  prefs.begin("wstation", true);
  strlcpy(cfg.ssid, prefs.getString("ssid", "").c_str(), sizeof(cfg.ssid));
  strlcpy(cfg.pass, prefs.getString("pass", "").c_str(), sizeof(cfg.pass));
  strlcpy(cfg.mqttHost, prefs.getString("mqttHost", "").c_str(), sizeof(cfg.mqttHost));
  cfg.mqttPort = prefs.getUShort("mqttPort", 1883);
  strlcpy(cfg.mqttUser, prefs.getString("mqttUser", "").c_str(), sizeof(cfg.mqttUser));
  strlcpy(cfg.mqttPass, prefs.getString("mqttPass", "").c_str(), sizeof(cfg.mqttPass));
  strlcpy(cfg.mqttTopic, prefs.getString("mqttTopic", "weather/wh65").c_str(), sizeof(cfg.mqttTopic));
  cfg.freq = prefs.getUShort("freq", 915);
  prefs.end();
}

static void saveConfig() {
  prefs.begin("wstation", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putString("mqttHost", cfg.mqttHost);
  prefs.putUShort("mqttPort", cfg.mqttPort);
  prefs.putString("mqttUser", cfg.mqttUser);
  prefs.putString("mqttPass", cfg.mqttPass);
  prefs.putString("mqttTopic", cfg.mqttTopic);
  prefs.putUShort("freq", cfg.freq);
  prefs.end();
}

// ─── Web Dashboard HTML (PROGMEM) ──────────────────────────────────
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Weather Station</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',system-ui,sans-serif;background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);
color:#e0e0e0;min-height:100vh;padding:16px}
h1{text-align:center;font-size:1.5em;margin-bottom:12px;
background:linear-gradient(90deg,#00d2ff,#3a7bd5);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:12px;max-width:900px;margin:0 auto}
.card{background:rgba(255,255,255,0.08);backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.12);
border-radius:12px;padding:14px;text-align:center;transition:transform 0.2s}
.card:hover{transform:translateY(-2px)}
.card .label{font-size:0.75em;color:#aaa;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px}
.card .value{font-size:1.6em;font-weight:700}
.card .unit{font-size:0.7em;color:#888;margin-left:2px}
.temp{color:#ff6b6b}.hum{color:#48dbfb}.wind{color:#0abde3}.rain{color:#54a0ff}
.uv{color:#feca57}.lux{color:#ff9ff3}.pm1{color:#1dd1a1}.pm25{color:#f368e0}.pm10{color:#ee5a24}
.bat{color:#2ecc71}.cnt{color:#a29bfe}
.nav{text-align:center;margin:14px 0}
.nav a{color:#48dbfb;text-decoration:none;margin:0 12px;font-size:0.85em}
.status{text-align:center;font-size:0.7em;color:#666;margin-top:12px}
#age{color:#888}
</style></head><body>
<h1>Weather Station</h1>
<div class="nav"><a href="/">Dashboard</a><a href="/settings">Settings</a></div>
<div class="grid" id="g"></div>
<div class="status">
<span id="age">--</span> | Heap: <span id="heap">--</span> | <span id="wifi">--</span>
</div>
<script>
function u(){fetch('/api/data').then(r=>r.json()).then(d=>{
let h='';
if(d.wh65){let w=d.wh65;
h+=c('Temperature',w.tempC.toFixed(1),'°C','temp');
h+=c('Humidity',w.hum,'%','hum');
h+=c('Wind Dir',w.wdir,'°','wind');
h+=c('Wind Speed',w.wspd.toFixed(1),'km/h','wind');
h+=c('Wind Gust',w.wgst.toFixed(1),'km/h','wind');
h+=c('Rain',w.rain.toFixed(1),'mm','rain');
h+=c('UV Index',w.uv,'','uv');
h+=c('Light',w.lux,'lux','lux');
h+=c('Battery',w.bat?'OK':'LOW','','bat');}
if(d.pms){let p=d.pms;
h+=c('PM 1.0',p.pm1,'µg/m³','pm1');
h+=c('PM 2.5',p.pm25,'µg/m³','pm25');
h+=c('PM 10',p.pm10,'µg/m³','pm10');
h+=c('>0.3µm',p.p03,'/0.1L','cnt');
h+=c('>0.5µm',p.p05,'/0.1L','cnt');
h+=c('>1.0µm',p.p10,'/0.1L','cnt');
h+=c('>2.5µm',p.p25,'/0.1L','cnt');
h+=c('>5.0µm',p.p50,'/0.1L','cnt');
h+=c('>10µm',p.p100,'/0.1L','cnt');}
document.getElementById('g').innerHTML=h;
document.getElementById('heap').textContent=d.heap;
document.getElementById('wifi').textContent=d.ip||'AP Mode';
if(d.wAge!==undefined){let a=d.wAge;
document.getElementById('age').textContent=a<60?a+'s ago':Math.floor(a/60)+'m ago';}
}).catch(e=>console.error(e));}
function c(l,v,un,cl){return '<div class="card"><div class="label">'+l+'</div><div class="value '+cl+'">'+v+'<span class="unit">'+un+'</span></div></div>';}
u();setInterval(u,5000);
</script></body></html>)rawliteral";

// ─── Settings Page HTML (PROGMEM) ──────────────────────────────────
static const char SETTINGS_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Settings</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',system-ui,sans-serif;background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);
color:#e0e0e0;min-height:100vh;padding:16px;display:flex;flex-direction:column;align-items:center}
h1{font-size:1.4em;margin-bottom:12px;
background:linear-gradient(90deg,#00d2ff,#3a7bd5);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
form{background:rgba(255,255,255,0.08);backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.12);
border-radius:12px;padding:20px;width:100%;max-width:400px}
label{display:block;font-size:0.8em;color:#aaa;margin:10px 0 3px;text-transform:uppercase;letter-spacing:1px}
input,select{width:100%;padding:8px 10px;border-radius:6px;border:1px solid rgba(255,255,255,0.2);
background:rgba(0,0,0,0.3);color:#fff;font-size:0.9em}
input:focus{outline:none;border-color:#48dbfb}
button{margin-top:16px;width:100%;padding:10px;border:none;border-radius:8px;
background:linear-gradient(90deg,#00d2ff,#3a7bd5);color:#fff;font-size:1em;font-weight:600;cursor:pointer}
button:hover{opacity:0.9}
.nav{margin:14px 0}
.nav a{color:#48dbfb;text-decoration:none;margin:0 12px;font-size:0.85em}
.msg{text-align:center;color:#1dd1a1;margin-top:10px;font-size:0.85em}
h2{font-size:1em;color:#aaa;margin-top:16px;border-top:1px solid rgba(255,255,255,0.1);padding-top:12px}
</style></head><body>
<h1>Settings</h1>
<div class="nav"><a href="/">Dashboard</a><a href="/settings">Settings</a></div>
<form method="POST" action="/settings">
<h2>WiFi</h2>
<label>SSID</label><input name="ssid" value="{{SSID}}" maxlength="32">
<label>Password</label><input name="pass" type="password" value="{{PASS}}" maxlength="64">
<h2>MQTT</h2>
<label>Host</label><input name="mqttHost" value="{{MQTT_HOST}}" maxlength="64">
<label>Port</label><input name="mqttPort" type="number" value="{{MQTT_PORT}}">
<label>User</label><input name="mqttUser" value="{{MQTT_USER}}" maxlength="32">
<label>Password</label><input name="mqttPass" type="password" value="{{MQTT_PASS}}" maxlength="64">
<label>Topic</label><input name="mqttTopic" value="{{MQTT_TOPIC}}" maxlength="64">
<h2>Radio</h2>
<label>Frequency</label>
<select name="freq"><option value="915" {{SEL915}}>915 MHz</option><option value="868" {{SEL868}}>868 MHz</option></select>
<button type="submit">Save & Reboot</button>
</form>
<div class="msg" id="msg"></div>
<script>
if(location.search==='?saved=1')document.getElementById('msg').textContent='Settings saved! Rebooting...';
</script></body></html>)rawliteral";

// ─── Captive Portal Page (PROGMEM) ─────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Weather Station Setup</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:#1a1a2e;color:#e0e0e0;
display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;padding:20px}
h1{color:#48dbfb;margin-bottom:20px}
form{background:rgba(255,255,255,0.08);border-radius:12px;padding:24px;width:100%;max-width:350px}
label{display:block;font-size:0.8em;color:#aaa;margin:10px 0 3px}
input{width:100%;padding:8px 10px;border-radius:6px;border:1px solid rgba(255,255,255,0.2);
background:rgba(0,0,0,0.3);color:#fff;font-size:0.9em}
button{margin-top:16px;width:100%;padding:10px;border:none;border-radius:8px;
background:linear-gradient(90deg,#00d2ff,#3a7bd5);color:#fff;font-size:1em;font-weight:600;cursor:pointer}
</style></head><body>
<h1>Weather Station Setup</h1>
<form method="POST" action="/save">
<label>WiFi SSID</label><input name="ssid" maxlength="32" required>
<label>WiFi Password</label><input name="pass" type="password" maxlength="64">
<button type="submit">Connect</button>
</form></body></html>)rawliteral";

// ─── Web Handlers ───────────────────────────────────────────────────

// Serve dashboard
static void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

// JSON API
static void handleAPI() {
  char buf[512];
  int len = 0;
  len += snprintf(buf + len, sizeof(buf) - len, "{\"heap\":%lu", (unsigned long)ESP.getFreeHeap());

  if (!apMode) {
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), "%s", WiFi.localIP().toString().c_str());
    len += snprintf(buf + len, sizeof(buf) - len, ",\"ip\":\"%s\"", ipStr);
  }

  if (wh65.valid) {
    uint32_t age = (millis() - wh65.lastUpdate) / 1000;
    len += snprintf(buf + len, sizeof(buf) - len,
      ",\"wAge\":%lu,\"wh65\":{\"tempC\":%.1f,\"hum\":%d,\"wdir\":%d,\"wspd\":%.1f,"
      "\"wgst\":%.1f,\"rain\":%.1f,\"uv\":%d,\"lux\":%lu,\"bat\":%d}",
      (unsigned long)age, wh65.tempC, wh65.humidity, wh65.windDir,
      wh65.windSpeed, wh65.windGust, wh65.rainMM, wh65.uvIndex,
      (unsigned long)wh65.lux, wh65.battery);
  }

  if (pms.valid) {
    len += snprintf(buf + len, sizeof(buf) - len,
      ",\"pms\":{\"pm1\":%d,\"pm25\":%d,\"pm10\":%d,"
      "\"p03\":%d,\"p05\":%d,\"p10\":%d,\"p25\":%d,\"p50\":%d,\"p100\":%d}",
      pms.pm1_0, pms.pm2_5, pms.pm10,
      pms.p03, pms.p05, pms.p10, pms.p25, pms.p50, pms.p100);
  }

  len += snprintf(buf + len, sizeof(buf) - len, "}");
  server.send(200, "application/json", buf);
}

// Settings page with template replacement
static void handleSettings() {
  // Build settings page from PROGMEM with substitutions
  // Read PROGMEM into a temporary buffer since we need to do replacements
  size_t pgmLen = strlen_P(SETTINGS_HTML);
  // Use a stack buffer large enough for the template + values
  char *page = (char *)malloc(pgmLen + 256);
  if (!page) {
    server.send(500, "text/plain", "OOM");
    return;
  }
  memcpy_P(page, SETTINGS_HTML, pgmLen + 1);

  // Simple replace helper using memmove
  auto replaceTag = [](char *html, const char *tag, const char *val) {
    char *pos = strstr(html, tag);
    if (!pos) return;
    size_t tagLen = strlen(tag);
    size_t valLen = strlen(val);
    size_t tailLen = strlen(pos + tagLen);
    memmove(pos + valLen, pos + tagLen, tailLen + 1);
    memcpy(pos, val, valLen);
  };

  char portStr[6];
  snprintf(portStr, sizeof(portStr), "%d", cfg.mqttPort);

  replaceTag(page, "{{SSID}}", cfg.ssid);
  replaceTag(page, "{{PASS}}", cfg.pass);
  replaceTag(page, "{{MQTT_HOST}}", cfg.mqttHost);
  replaceTag(page, "{{MQTT_PORT}}", portStr);
  replaceTag(page, "{{MQTT_USER}}", cfg.mqttUser);
  replaceTag(page, "{{MQTT_PASS}}", cfg.mqttPass);
  replaceTag(page, "{{MQTT_TOPIC}}", cfg.mqttTopic);
  replaceTag(page, "{{SEL915}}", cfg.freq == 915 ? "selected" : "");
  replaceTag(page, "{{SEL868}}", cfg.freq == 868 ? "selected" : "");

  server.send(200, "text/html", page);
  free(page);
}

// Save settings from POST
static void handleSettingsSave() {
  if (server.hasArg("ssid")) strlcpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid));
  if (server.hasArg("pass")) strlcpy(cfg.pass, server.arg("pass").c_str(), sizeof(cfg.pass));
  if (server.hasArg("mqttHost")) strlcpy(cfg.mqttHost, server.arg("mqttHost").c_str(), sizeof(cfg.mqttHost));
  if (server.hasArg("mqttPort")) cfg.mqttPort = server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser")) strlcpy(cfg.mqttUser, server.arg("mqttUser").c_str(), sizeof(cfg.mqttUser));
  if (server.hasArg("mqttPass")) strlcpy(cfg.mqttPass, server.arg("mqttPass").c_str(), sizeof(cfg.mqttPass));
  if (server.hasArg("mqttTopic")) strlcpy(cfg.mqttTopic, server.arg("mqttTopic").c_str(), sizeof(cfg.mqttTopic));
  if (server.hasArg("freq")) cfg.freq = server.arg("freq").toInt();

  saveConfig();
  server.sendHeader("Location", "/settings?saved=1");
  server.send(302);
  delay(1000);
  ESP.restart();
}

// Captive portal
static void handlePortal() {
  server.send_P(200, "text/html", PORTAL_HTML);
}

static void handlePortalSave() {
  if (server.hasArg("ssid")) strlcpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid));
  if (server.hasArg("pass")) strlcpy(cfg.pass, server.arg("pass").c_str(), sizeof(cfg.pass));
  saveConfig();
  server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#fff;text-align:center;padding:40px'>"
    "<h2>Saved! Rebooting...</h2></body></html>");
  delay(1500);
  ESP.restart();
}

// Redirect all unknown to portal in AP mode
static void handleNotFound() {
  if (apMode) {
    server.sendHeader("Location", "http://192.168.4.1/");
    server.send(302);
  } else {
    server.send(404, "text/plain", "Not Found");
  }
}

// ─── MQTT ───────────────────────────────────────────────────────────
static void mqttPublish() {
  if (cfg.mqttHost[0] == '\0') return;
  if (!mqtt.connected()) return;

  char buf[512];
  int len = 0;
  len += snprintf(buf + len, sizeof(buf) - len, "{");

  if (wh65.valid) {
    len += snprintf(buf + len, sizeof(buf) - len,
      "\"tempC\":%.1f,\"humidity\":%d,\"windDir\":%d,\"windSpeed\":%.1f,"
      "\"windGust\":%.1f,\"rain\":%.1f,\"uvIndex\":%d,\"lux\":%lu,\"battery\":%d",
      wh65.tempC, wh65.humidity, wh65.windDir, wh65.windSpeed,
      wh65.windGust, wh65.rainMM, wh65.uvIndex, (unsigned long)wh65.lux, wh65.battery);
  }

  if (pms.valid) {
    if (wh65.valid) len += snprintf(buf + len, sizeof(buf) - len, ",");
    len += snprintf(buf + len, sizeof(buf) - len,
      "\"pm1_0\":%d,\"pm2_5\":%d,\"pm10\":%d",
      pms.pm1_0, pms.pm2_5, pms.pm10);
  }

  len += snprintf(buf + len, sizeof(buf) - len, "}");
  mqtt.publish(cfg.mqttTopic, buf);
}

static void mqttReconnect() {
  if (cfg.mqttHost[0] == '\0') return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttReconnect < 10000) return;
  lastMqttReconnect = millis();

  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  bool ok;
  if (cfg.mqttUser[0] != '\0') {
    ok = mqtt.connect("WeatherStation", cfg.mqttUser, cfg.mqttPass);
  } else {
    ok = mqtt.connect("WeatherStation");
  }
  Serial.printf("MQTT %s: %s\n", ok ? "connected" : "failed", cfg.mqttHost);
}

// ─── WiFi Setup ─────────────────────────────────────────────────────
static bool wifiConnect() {
  if (cfg.ssid[0] == '\0') return false;

  Serial.printf("WiFi connecting to: %s\n", cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid, cfg.pass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("WiFi connection failed");
  return false;
}

static void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Weather_Station_1");
  delay(100);
  Serial.printf("AP mode: %s\n", WiFi.softAPIP().toString().c_str());

  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handlePortal);
  server.on("/save", HTTP_POST, handlePortalSave);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_POST, handleSettingsSave);
  server.on("/api/data", handleAPI);
  server.onNotFound(handleNotFound);
  server.begin();
}

static void startSTA() {
  apMode = false;
  server.on("/", handleRoot);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_POST, handleSettingsSave);
  server.on("/api/data", handleAPI);
  server.onNotFound(handleNotFound);
  server.begin();

  // OTA
  ArduinoOTA.setHostname("Weather_Station_1");
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

// ─── Setup ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for USB CDC serial
  Serial.println("\n=== WH65 Weather Station RX V1 ===");

  // LED init
  setLED(0, 0, 30);  // Blue on boot

  // Load config from NVS
  loadConfig();
  Serial.printf("Config: SSID='%s' MQTT='%s' Freq=%dMHz\n", cfg.ssid, cfg.mqttHost, cfg.freq);

  // SPI for CC1101
  spiCC.begin(CC1101_CLK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
  spiCC.setFrequency(4000000);
  spiCC.setDataMode(SPI_MODE0);

  // CC1101 init
  if (!cc1101Init()) {
    Serial.println("CC1101 init FAILED - check wiring");
    setLED(30, 0, 0);  // Red = error
  }

  // PMS7003 init
  pmsInit();

  // WiFi
  if (wifiConnect()) {
    startSTA();
    setLED(0, 30, 0);  // Green = connected
  } else {
    startAP();
    setLED(30, 15, 0);  // Orange = AP mode
  }

  delay(1000);
  ledOff();

  Serial.printf("Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("Ready.");
}

// ─── Main Loop ──────────────────────────────────────────────────────
static uint32_t lastLedOff = 0;
static uint32_t lastMqttPub = 0;

void loop() {
  // CC1101 receive check
  cc1101CheckRX();

  // PMS7003 read
  pmsRead();

  // Web server
  server.handleClient();

  // DNS (captive portal in AP mode)
  if (apMode) {
    dnsServer.processNextRequest();
  }

  // OTA
  if (!apMode) {
    ArduinoOTA.handle();
  }

  // MQTT
  if (!apMode && cfg.mqttHost[0] != '\0') {
    mqttReconnect();
    mqtt.loop();

    // Publish every 30 seconds if we have data
    if (millis() - lastMqttPub > 30000 && (wh65.valid || pms.valid)) {
      mqttPublish();
      lastMqttPub = millis();
    }
  }

  // Turn off LED after 500ms
  if (lastLedOff != 0 && millis() - lastLedOff > 500) {
    ledOff();
    lastLedOff = 0;
  }

  // Track LED timing from cycle/pulse
  static uint32_t prevWH65Update = 0;
  static uint32_t prevPMSUpdate = 0;
  if (wh65.lastUpdate != prevWH65Update) {
    prevWH65Update = wh65.lastUpdate;
    lastLedOff = millis();
    // MQTT publish on new WH65 data
    if (!apMode && mqtt.connected()) mqttPublish();
  }
  if (pms.lastUpdate != prevPMSUpdate) {
    prevPMSUpdate = pms.lastUpdate;
    lastLedOff = millis();
    // MQTT publish on new PMS data
    if (!apMode && mqtt.connected()) mqttPublish();
  }
}
