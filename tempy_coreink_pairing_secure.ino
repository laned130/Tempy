/* tempy_coreink_pairing_secure.ino
   CoreInk thermostat:
   - SHT40 high precision
   - Pairing via WiFi (hold BtnB during boot to pair)
   - Generates LMK on pairing; POSTs to Atom pairing endpoint
   - Uses PMK + per-peer LMK for ESP-NOW encryption after pairing
   - Includes 32-bit message counters for replay protection
   - UI: large temp, small humidity, battery icon (top-left), heating indicator H/?/blank (top-right)
   - 200 ms ESP-NOW reply timeout
   - Setpoint mode: inverted display, 5s timeout, A = -1C, C = +1C
*/

#include "secrets.h"
#include <M5CoreInk.h>
#include <Adafruit_SHT4x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <Preferences.h>

// ---------- CONFIG ----------
#define SETPOINT_DEFAULT 20
#define SETPOINT_TIMEOUT_MS 5000
#define HEATING_REPLY_TIMEOUT_MS 200
#define WAKE_SECONDS 60

#define BAT_ADC_PIN 35
#define ADC_MAX 4095.0
#define VREF 3.3
#define VOLT_DIV 2.0

// pairing endpoint on Atom
#define ATOM_PAIRING_URL_FORMAT "http://%s/pair"  // POST JSON: { "mac": "aa:bb:cc:dd:ee:ff", "lmk": "hex32" }

Preferences prefs;
Adafruit_SHT4x sht;
uint8_t atomMac[6]; // will be filled after pairing or set with placeholder
uint8_t peerLMK[16]; // per-peer LMK (persisted)
bool haveLMK = false;

// RTC-stored setpoint and last heating
RTC_DATA_ATTR int targetTemp = SETPOINT_DEFAULT;
RTC_DATA_ATTR bool lastHeatingState = false;

// Message counters (persisted)
uint32_t sendCounter = 0;
uint32_t recvCounter = 0;

// runtime
float temperatureVal = NAN;
float humidityVal = NAN;
bool inSetpointMode = false;
unsigned long setpointStartMs = 0;
volatile bool heatingReplyReceived = false;
volatile bool heatingActive = false;

// message struct
typedef struct __attribute__((packed)) {
  uint8_t type;   // 1=request, 2=target_update, 3=heating_state (reply)
  uint8_t id;     // thermostat id (0 for single)
  int8_t target;
  uint8_t heating;
  uint32_t counter; // little-endian
} esp_msg_t;

// ---------- utils ----------
String macToString(const uint8_t *mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}

void hexToBytes(const char *hex, uint8_t *out, size_t outlen) {
  for (size_t i=0;i<outlen;i++) {
    unsigned int v;
    sscanf(hex + 2*i, "%02x", &v);
    out[i] = (uint8_t)v;
  }
}

String bytesToHex(const uint8_t *in, size_t len) {
  String s;
  s.reserve(len*2+1);
  for (size_t i=0;i<len;i++) {
    char buf[3];
    sprintf(buf,"%02x", in[i]);
    s += buf;
  }
  return s;
}

void printHex(const uint8_t *p, size_t n) {
  for (size_t i=0;i<n;i++) { if (i) Serial.print(':'); Serial.printf("%02X", p[i]); }
  Serial.println();
}

float readBatteryVoltage() {
  int raw = analogRead(BAT_ADC_PIN);
  return (raw / ADC_MAX) * VREF * VOLT_DIV;
}

// ---------- ESP-NOW callbacks ----------
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < (int)sizeof(esp_msg_t)) return;
  esp_msg_t msg;
  memcpy(&msg, data, sizeof(msg));

  // verify counter monotonic
  uint32_t incoming = msg.counter;
  if (incoming <= recvCounter) {
    Serial.printf("Replay or out-of-order message (incoming %u <= saved %u) - ignore\n", incoming, recvCounter);
    return;
  }
  recvCounter = incoming;
  prefs.begin("counters", false);
  prefs.putUInt("recv", recvCounter);
  prefs.end();

  if (msg.type == 3) {
    heatingActive = (msg.heating != 0);
    heatingReplyReceived = true;
    Serial.printf("Received heating_state: %d counter %u\n", (int)heatingActive, incoming);
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // nothing to do here
}

// build peer info and add with LMK
bool espNowAddPeerWithLMK(const uint8_t *peerMac, const uint8_t *lmk) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = true;
  memcpy(peerInfo.lmk, lmk, 16);
  esp_err_t r = esp_now_add_peer(&peerInfo);
  if (r != ESP_OK && r != ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.printf("esp_now_add_peer failed: %d\n", r);
    return false;
  }
  return true;
}

// send message (increments counter and persists)
bool sendEspNow(const uint8_t *peer, esp_msg_t *m) {
  sendCounter++;
  m->counter = sendCounter;
  prefs.begin("counters", false);
  prefs.putUInt("send", sendCounter);
  prefs.end();

  esp_err_t r = esp_now_send(peer, (uint8_t*)m, sizeof(*m));
  if (r != ESP_OK) {
    Serial.printf("esp_now_send error %d\n", r);
    return false;
  }
  return true;
}

// request heating state and wait up to timeout_ms
bool requestHeatingStateWait(uint32_t timeout_ms) {
  heatingReplyReceived = false;
  esp_msg_t req = {};
  req.type = 1;
  req.id = 0;
  req.target = 0;
  req.heating = 0;
  req.counter = 0;
  if (!sendEspNow(atomMac, &req)) return false;

  unsigned long start = millis();
  while (!heatingReplyReceived && (millis() - start) < timeout_ms) {
    delay(2);
  }
  return heatingReplyReceived;
}

// send target_update then query heating (fire-and-forget send)
void sendTargetUpdateAndQuery() {
  esp_msg_t upd = {};
  upd.type = 2;
  upd.id = 0;
  upd.target = (int8_t)targetTemp;
  upd.heating = 0;
  sendEspNow(atomMac, &upd);
  // small delay then query
  delay(20);
  requestHeatingStateWait(HEATING_REPLY_TIMEOUT_MS);
}

// ---------- Pairing (Thermostat side) ----------
// Thermostat generates LMK here, sends to Atom pairing endpoint via HTTP POST
bool doPairingProcedure() {
  Serial.println("Pairing mode: connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis()-start) < 15000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed for pairing.");
    return false;
  }
  Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());

  // generate a random LMK (16 bytes) and persist in prefs
  uint8_t generatedLMK[16];
  for (int i=0;i<16;i++) {
    uint32_t r = esp_random();
    generatedLMK[i] = (uint8_t)(r & 0xFF);
  }
  String lmkHex = bytesToHex(generatedLMK, 16);

  // get our MAC
  uint8_t mymac[6];
  esp_read_mac(mymac, ESP_MAC_WIFI_STA);
  String macStr = macToString(mymac);

  // Build JSON
  String payload = "{\"mac\":\"" + macStr + "\",\"lmk\":\"" + lmkHex + "\"}";

  // POST to Atom pairing endpoint
  char url[128];
  snprintf(url, sizeof(url), ATOM_PAIRING_URL_FORMAT, ATOM_PAIRING_IP);
  Serial.printf("Posting pairing to %s\n", url);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(payload);
  if (httpCode != 200) {
    Serial.printf("Pairing POST failed code %d\n", httpCode);
    http.end();
    WiFi.disconnect(true);
    return false;
  }
  String resp = http.getString();
  http.end();
  Serial.printf("Pairing response: %s\n", resp.c_str());

  // Expect JSON like: { "atom_mac":"aa:bb:cc:dd:ee:ff" }
  // For simplicity, parse atom_mac
  int idx = resp.indexOf("atom_mac");
  if (idx == -1) {
    Serial.println("Pairing response missing atom_mac");
    WiFi.disconnect(true);
    return false;
  }
  int q1 = resp.indexOf('"', idx+9);
  int q2 = resp.indexOf('"', q1+1);
  int q3 = resp.indexOf('"', q2+1);
  int q4 = resp.indexOf('"', q3+1);
  String atomMacStr = resp.substring(q3+1, q4);
  Serial.printf("Atom MAC string: %s\n", atomMacStr.c_str());

  // parse atomMacStr into bytes
  int vals[6]; sscanf(atomMacStr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
                     &vals[0],&vals[1],&vals[2],&vals[3],&vals[4],&vals[5]);
  for (int i=0;i<6;i++) atomMac[i] = (uint8_t)vals[i];

  // persist LMK and atomMac
  prefs.begin("pair", false);
  prefs.putBytes("lmk", generatedLMK, 16);
  prefs.putBytes("atommac", atomMac, 6);
  prefs.end();

  memcpy(peerLMK, generatedLMK, 16);
  haveLMK = true;

  // Disconnect WiFi to save power; pairing is complete
  WiFi.disconnect(true);
  delay(200);

  Serial.println("Pairing complete.");
  return true;
}

// ---------- Setup & Loop ----------
void setup() {
  Serial.begin(115200);
  M5.begin();
  M5.Ink.begin();
  M5.Ink.createSprite(250,300);

  // prefs load
  prefs.begin("pair", false);
  if (prefs.isKey("lmk")) {
    prefs.getBytes("lmk", peerLMK, 16);
    haveLMK = true;
    prefs.getBytes("atommac", atomMac, 6);
    Serial.print("Loaded LMK and Atom MAC from prefs. Atom MAC: ");
    printHex(atomMac,6);
  } else {
    haveLMK = false;
  }
  prefs.end();

  // counters
  prefs.begin("counters", false);
  sendCounter = prefs.getUInt("send", 0);
  recvCounter = prefs.getUInt("recv", 0);
  prefs.end();

  // SHT40
  if (!sht.begin()) { Serial.println("SHT40 not found"); }
  sht.setPrecision(SHT4X_HIGH);

  // Load target temp
  prefs.begin("tempy", false);
  targetTemp = prefs.getInt("target", SETPOINT_DEFAULT);
  prefs.end();

  // If button B held at boot -> pairing mode
  M5.update();
  bool pairingPressed = M5.BtnB.isPressed(); // hold while powering
  if (pairingPressed) {
    if (doPairingProcedure()) {
      // after pairing, init ESP-NOW with keys
      Serial.println("Pairing done; configuring ESP-NOW with PMK + LMK");
    } else {
      Serial.println("Pairing failed or aborted");
    }
  }

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // set PMK if defined
  esp_now_set_pmk(PMK);

  // If have LMK and atomMac, add peer entry
  if (haveLMK) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, atomMac, 6);
    peer.channel = 0;
    peer.encrypt = true;
    memcpy(peer.lmk, peerLMK, 16);
    if (esp_now_is_peer_exist(atomMac) == false) {
      if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add Atom peer (esp_now_add_peer)");
      } else {
        Serial.println("Added Atom as ESP-NOW peer with LMK");
      }
    }
  } else {
    Serial.println("No LMK yet; must pair before secure ESP-NOW communication");
  }

  // initial sensor read & heating request
  sensors_event_t h, t;
  sht.getEvent(&h, &t);
  temperatureVal = t.temperature;
  humidityVal = h.relative_humidity;

  // request heating state (wait up to 200ms). If no reply, heatingReplyReceived stays false.
  if (haveLMK) requestHeatingStateWait(HEATING_REPLY_TIMEOUT_MS);

  // initial display
  // draw normal screen
  M5.Ink.clear();
  M5.Ink.setTextSize(6);
  if (!isnan(temperatureVal)) M5.Ink.printf("%.1fC", temperatureVal);
  else M5.Ink.printf("--.-C");
  M5.Ink.setTextSize(2);
  M5.Ink.setCursor(10,140);
  M5.Ink.printf("Target: %dC", targetTemp);
  M5.Ink.setTextSize(1);
  M5.Ink.setCursor(10,200);
  if (!isnan(humidityVal)) M5.Ink.printf("Hum: %d%%", (int)round(humidityVal));
  else M5.Ink.printf("Hum: --%%");
  // battery icon
  float batt = readBatteryVoltage();
  M5.Ink.setCursor(4,6);
  if (batt > 3.90) M5.Ink.printf("[####]");
  else if (batt > 3.75) M5.Ink.printf("[### ]");
  else if (batt > 3.60) M5.Ink.printf("[##  ]");
  else M5.Ink.printf("[!   ]");
  // heating indicator
  M5.Ink.setCursor(180,6);
  char c = ' ';
  if (!heatingReplyReceived) c = '?';
  else if (heatingActive) c = 'H';
  M5.Ink.setTextSize(2);
  M5.Ink.printf("%c", c);

  M5.Ink.pushSprite();
  delay(200);
}

void goToSleep() {
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {
  M5.update();

  bool anyPressed = M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || M5.BtnC.wasPressed();
  if (anyPressed) {
    if (!inSetpointMode) {
      inSetpointMode = true;
      setpointStartMs = millis();
      // show inverted setpoint
      M5.Ink.clear();
      M5.Ink.fillRect(0,0,250,300,INK_COLOR_BLACK);
      M5.Ink.setTextColor(INK_COLOR_WHITE);
      M5.Ink.setTextSize(3);
      M5.Ink.setCursor(10,30);
      M5.Ink.printf("Set Target");
      M5.Ink.setTextSize(6);
      M5.Ink.setCursor(10,80);
      M5.Ink.printf("%dC", targetTemp);
      M5.Ink.setTextSize(2);
      M5.Ink.setCursor(10,200);
      M5.Ink.printf("A = -1C  C = +1C");
      M5.Ink.pushSprite();
      M5.Ink.setTextColor(INK_COLOR_BLACK);
      // query heating while in setpoint mode
      if (haveLMK) requestHeatingStateWait(HEATING_REPLY_TIMEOUT_MS);
    } else {
      // already in setpoint mode: adjust
      bool changed = false;
      if (M5.BtnA.wasPressed()) { targetTemp--; changed = true; }
      if (M5.BtnC.wasPressed()) { targetTemp++; changed = true; }
      if (changed) {
        // persist target
        prefs.begin("tempy", false);
        prefs.putInt("target", targetTemp);
        prefs.end();
        // send update and ask for heating state
        if (haveLMK) sendTargetUpdateAndQuery();
        // refresh setpoint screen
        M5.Ink.clear();
        M5.Ink.fillRect(0,0,250,300,INK_COLOR_BLACK);
        M5.Ink.setTextColor(INK_COLOR_WHITE);
        M5.Ink.setTextSize(6);
        M5.Ink.setCursor(10,80);
        M5.Ink.printf("%dC", targetTemp);
        M5.Ink.pushSprite();
        M5.Ink.setTextColor(INK_COLOR_BLACK);
      }
      setpointStartMs = millis();
    }
  }

  if (inSetpointMode) {
    if (millis() - setpointStartMs >= SETPOINT_TIMEOUT_MS) {
      inSetpointMode = false;
      // on exit request heating state and display normal screen then sleep
      if (haveLMK) requestHeatingStateWait(HEATING_REPLY_TIMEOUT_MS);
      // read sensors quickly
      sensors_event_t h, t;
      sht.getEvent(&h,&t);
      temperatureVal = t.temperature;
      humidityVal = h.relative_humidity;
      // display
      M5.Ink.clear();
      M5.Ink.setTextSize(6);
      M5.Ink.setCursor(10,40);
      M5.Ink.printf("%.1fC", temperatureVal);
      M5.Ink.setTextSize(2);
      M5.Ink.setCursor(10,140);
      M5.Ink.printf("Target: %dC", targetTemp);
      M5.Ink.setTextSize(1);
      M5.Ink.setCursor(10,200);
      M5.Ink.printf("Hum: %d%%", (int)round(humidityVal));
      M5.Ink.setCursor(4,6);
      float batt = readBatteryVoltage();
      if (batt > 3.90) M5.Ink.printf("[####]");
      else if (batt > 3.75) M5.Ink.printf("[### ]");
      else if (batt > 3.60) M5.Ink.printf("[##  ]");
      else M5.Ink.printf("[!   ]");
      M5.Ink.setCursor(180,6);
      char c = ' ';
      if (!heatingReplyReceived) c = '?';
      else if (heatingActive) c = 'H';
      M5.Ink.setTextSize(2);
      M5.Ink.printf("%c", c);
      M5.Ink.pushSprite();
      delay(200);
      goToSleep();
    } else {
      delay(10);
    }
  } else {
    // no setpoint activity -> sleep
    delay(250);
    goToSleep();
  }
}
