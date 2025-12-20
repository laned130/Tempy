/* tempy_atom_pairing_secure.ino
   M5 Atom:
   - Connects to WiFi (secrets.h)
   - Connects to MQTT broker (secrets.h) and publishes/subscribes per thermostat
   - Exposes pairing endpoint at /pair to accept LMK from thermostats (POST JSON)
   - Stores LMK + MAC per thermostat in Preferences and adds encrypted ESP-NOW peers
   - Handles up to 5 thermostats (N_THERM)
   - Responds to heating_request and target_update messages over ESP-NOW with heating_state reply
   - Validates message counters (rejects non-increasing counters)
*/

#include "secrets.h"
#include <M5Atom.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

Preferences prefs;
WebServer server(80);

// MQTT client
WiFiClient espClient;
PubSubClient mqtt(espClient);

#define N_THERM 5
#define MQTT_BASE "tempy"   // base topic e.g. tempy/<id>/...

// placeholders - persisted during pairing
uint8_t thermMacs[N_THERM][6];
uint8_t lmks[N_THERM][16];
bool peerConfigured[N_THERM];

// relay pin mapping - adjust to your wiring
int relayPins[N_THERM] = {12,13,14,15,16}; // TODO: set your real pins

int lastTargets[N_THERM];
bool relayState[N_THERM];
uint32_t lastRecvCounter[N_THERM];

// PMK must match thermostats
static const uint8_t LOCAL_PMK[16] = {
  PMK[0],PMK[1],PMK[2],PMK[3],PMK[4],PMK[5],PMK[6],PMK[7],
  PMK[8],PMK[9],PMK[10],PMK[11],PMK[12],PMK[13],PMK[14],PMK[15]
};

// message struct
typedef struct __attribute__((packed)) {
  uint8_t type;
  uint8_t id;
  int8_t target;
  uint8_t heating;
  uint32_t counter;
} esp_msg_t;

// ---------- helpers ----------
void printHex(const uint8_t *p, size_t n) {
  for (size_t i=0;i<n;i++) { if (i) Serial.print(':'); Serial.printf("%02X", p[i]); }
  Serial.println();
}

String macToString(const uint8_t *mac) {
  char buf[18];
  sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}

String prefKey(const char *prefix, int idx) {
  String key(prefix);
  key += idx;
  return key;
}

void addEncryptedPeerFromStored(int idx) {
  if (!peerConfigured[idx]) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, thermMacs[idx], 6);
  peer.channel = 0;
  peer.encrypt = true;
  memcpy(peer.lmk, lmks[idx], 16);
  if (esp_now_is_peer_exist(thermMacs[idx]) == false) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.printf("Failed to add peer %d\n", idx);
    } else {
      Serial.printf("Added encrypted peer %d\n", idx);
    }
  }
}

void persistPeer(int idx) {
  prefs.begin("peers", false);
  String macKey = prefKey("mac", idx);
  String lmkKey = prefKey("lmk", idx);
  prefs.putBytes(macKey.c_str(), thermMacs[idx], 6);
  prefs.putBytes(lmkKey.c_str(), lmks[idx], 16);
  prefs.end();
}

// ---------- ESP-NOW callbacks ----------
void sendHeatingStateTo(const uint8_t *peerMac, uint8_t peerId) {
  esp_msg_t out = {};
  out.type = 3;
  out.id = peerId;
  out.target = (int8_t)lastTargets[peerId];
  out.heating = relayState[peerId] ? 1 : 0;
  // include counter? not needed for reply but include for monotonicity from Atom to thermostat
  static uint32_t localSendCounter = 0;
  localSendCounter++;
  out.counter = localSendCounter;
  esp_now_send((uint8_t*)peerMac, (uint8_t*)&out, sizeof(out));
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < (int)sizeof(esp_msg_t)) return;
  esp_msg_t msg;
  memcpy(&msg, data, sizeof(msg));

  // find index for peer
  int idx = -1;
  for (int i=0;i<N_THERM;i++) {
    bool same = true;
    for (int b=0;b<6;b++) if (mac_addr[b] != thermMacs[i][b]) { same = false; break; }
    if (same) { idx = i; break; }
  }
  if (idx < 0) {
    Serial.println("Received ESP-NOW from unknown MAC");
    return;
  }

  // validate counter
  if (msg.counter <= lastRecvCounter[idx]) {
    Serial.printf("Dropping out-of-order/replay for idx %d (incoming %u <= last %u)\n", idx, msg.counter, lastRecvCounter[idx]);
    return;
  }
  lastRecvCounter[idx] = msg.counter;
  prefs.begin("counters", false);
  String counterKey = prefKey("c_r", idx);
  prefs.putUInt(counterKey.c_str(), lastRecvCounter[idx]);
  prefs.end();

  if (msg.type == 1) {
    // heating_request -> reply with heating state
    sendHeatingStateTo(mac_addr, idx);
  }
  else if (msg.type == 2) {
    // target_update
    lastTargets[idx] = (int)msg.target;
    prefs.begin("tempy", false);
    String targetKey = prefKey("t", idx);
    prefs.putInt(targetKey.c_str(), lastTargets[idx]);
    prefs.end();

    // reply current heating state
    sendHeatingStateTo(mac_addr, idx);

    // publish to MQTT for HA visibility
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/%d/state", MQTT_BASE, idx);
    StaticJsonDocument<200> doc;
    doc["target"] = lastTargets[idx];
    doc["relay"] = relayState[idx];
    doc["lastSeen"] = millis();
    char buf[256];
    size_t n = serializeJson(doc, buf);
    mqtt.publish(topic, buf, n);
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  // optional logging
}

// ---------- Pairing HTTP endpoint ----------
// POST /pair  JSON { "mac":"aa:bb:cc:dd:ee:ff", "lmk":"hex32" }
// Response: { "ok": true, "atom_mac": "aa:bb:.." }

void handlePair() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"ok\":false,\"err\":\"Method\"}");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
    return;
  }
  const char *macs = doc["mac"];
  const char *lmkHex = doc["lmk"];
  if (!macs || !lmkHex) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"missing\"}");
    return;
  }

  // parse mac
  uint8_t macBytes[6];
  int vals[6];
  if (sscanf(macs, "%02x:%02x:%02x:%02x:%02x:%02x",
             &vals[0],&vals[1],&vals[2],&vals[3],&vals[4],&vals[5]) != 6) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"mac\"}");
    return;
  }
  for (int i=0;i<6;i++) macBytes[i] = (uint8_t)vals[i];

  // parse lmk hex (32 chars)
  if (strlen(lmkHex) != 32) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"lmklen\"}");
    return;
  }
  uint8_t lmk[16];
  for (int i=0;i<16;i++) {
    unsigned int v;
    sscanf(lmkHex + 2*i, "%02x", &v);
    lmk[i] = (uint8_t)v;
  }

  // find free slot or existing slot for this MAC
  int slot = -1;
  for (int i=0;i<N_THERM;i++) {
    bool same = true;
    for (int b=0;b<6;b++) if (thermMacs[i][b] != 0 && thermMacs[i][b] != macBytes[b]) { same = false; break; }
    if (same) { slot = i; break; }
  }
  if (slot == -1) {
    // find empty slot
    for (int i=0;i<N_THERM;i++) {
      bool empty = true;
      for (int b=0;b<6;b++) if (thermMacs[i][b] != 0) { empty = false; break; }
      if (empty) { slot = i; break; }
    }
  }
  if (slot == -1) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"no_slot\"}");
    return;
  }

  // store
  memcpy(thermMacs[slot], macBytes, 6);
  memcpy(lmks[slot], lmk, 16);
  peerConfigured[slot] = true;
  persistPeer(slot);
  addEncryptedPeerFromStored(slot);

  // respond with atom mac
  uint8_t mymac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mymac);
  char amac[32];
  sprintf(amac, "%02x:%02x:%02x:%02x:%02x:%02x",
          mymac[0],mymac[1],mymac[2],mymac[3],mymac[4],mymac[5]);

  StaticJsonDocument<128> out;
  out["ok"] = true;
  out["atom_mac"] = amac;
  char buf[128];
  size_t n = serializeJson(out, buf);
  server.send(200, "application/json", buf);
  Serial.printf("Paired thermostat into slot %d mac %s\n", slot, macs);
}

// ---------- MQTT callbacks ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // topic: tempy/<id>/relay/set or tempy/<id>/cmd etc
  String t = String(topic);
  // parse id from topic
  int slash1 = t.indexOf('/');
  int slash2 = t.indexOf('/', slash1+1);
  if (slash1 < 0 || slash2 < 0) return;
  String prefix = t.substring(0, slash1);
  if (prefix != String(MQTT_BASE)) return;
  int id = t.substring(slash1+1, slash2).toInt();
  String action = t.substring(slash2+1);
  String payloadStr;
  for (unsigned int i=0;i<length;i++) payloadStr += (char)payload[i];

  if (action == "relay" || action == "relay/set") {
    bool v = (payloadStr == "1" || payloadStr == "true");
    if (id >=0 && id < N_THERM) {
      relayState[id] = v;
      digitalWrite(relayPins[id], relayState[id] ? HIGH : LOW);
      // ack
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "%s/%d/state", MQTT_BASE, id);
      StaticJsonDocument<128> doc;
      doc["relay"] = relayState[id];
      char buf[128];
      size_t n = serializeJson(doc, buf);
      mqtt.publish(stateTopic, buf, n);
    }
  }
}

// ---------- Setup & loop ----------
bool initWifiAndMQTT() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to WiFi %s ...\n", WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed");
    return false;
  }
  Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  Serial.println("Connecting MQTT...");
  if (!mqtt.connect("tempy_atom", MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("MQTT connect failed");
    return false;
  }
  Serial.println("MQTT connected");

  // subscribe to per-thermostat relay topics
  for (int i=0;i<N_THERM;i++) {
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/%d/relay/set", MQTT_BASE, i);
    mqtt.subscribe(topic);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  M5.begin(true, false, true);
  prefs.begin("peers", false);

  // init arrays from prefs
  for (int i=0;i<N_THERM;i++) {
    // try to load mac and lmk
    String macKey = prefKey("mac", i);
    String lmkKey = prefKey("lmk", i);
    if (prefs.isKey(macKey.c_str()) && prefs.isKey(lmkKey.c_str())) {
      prefs.getBytes(macKey.c_str(), thermMacs[i], 6);
      prefs.getBytes(lmkKey.c_str(), lmks[i], 16);
      peerConfigured[i] = true;
      Serial.printf("Loaded peer %d: ", i); printHex(thermMacs[i],6);
    } else {
      memset(thermMacs[i], 0, 6);
      memset(lmks[i], 0, 16);
      peerConfigured[i] = false;
    }
    String targetKey = prefKey("t", i);
    String counterKey = prefKey("c_r", i);
    lastTargets[i] = prefs.getInt(targetKey.c_str(), 20);
    lastRecvCounter[i] = prefs.getUInt(counterKey.c_str(), 0);
    relayState[i] = false;
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  // init ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
  }
  esp_now_set_pmk(LOCAL_PMK);
  esp_now_register_recv_cb((esp_now_recv_cb_t)onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // add stored peers
  for (int i=0;i<N_THERM;i++) if (peerConfigured[i]) addEncryptedPeerFromStored(i);

  // Start webserver for pairing
  server.on("/pair", HTTP_POST, handlePair);
  server.begin();
  Serial.println("Pairing HTTP server started on port 80");

  // connect to WiFi and MQTT (used for HA integration)
  if (!initWifiAndMQTT()) {
    Serial.println("WiFi/MQTT init failed; pairing still available via AP mode if needed.");
  }
}

unsigned long lastMqttLoop = 0;

void loop() {
  server.handleClient();
  if (mqtt.connected()) {
    mqtt.loop();
    unsigned long now = millis();
    if (now - lastMqttLoop > 5000) {
      // publish per-thermostat state periodically
      for (int i=0;i<N_THERM;i++) {
        char topic[64];
        snprintf(topic, sizeof(topic), "%s/%d/state", MQTT_BASE, i);
        StaticJsonDocument<256> doc;
        doc["target"] = lastTargets[i];
        doc["relay"] = relayState[i];
        doc["lastSeen"] = lastRecvCounter[i]; // or lastSeen time if tracked
        char buf[256];
        size_t n = serializeJson(doc, buf);
        mqtt.publish(topic, buf, n);
      }
      lastMqttLoop = now;
    }
  } else {
    // try reconnect
    if (WiFi.status() == WL_CONNECTED) {
      if (mqtt.connect("tempy_atom", MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("MQTT reconnected");
        for (int i=0;i<N_THERM;i++) {
          char topic[64];
          snprintf(topic, sizeof(topic), "%s/%d/relay/set", MQTT_BASE, i);
          mqtt.subscribe(topic);
        }
      }
    } else {
      // attempt to reconnect WiFi
      WiFi.reconnect();
      delay(1000);
    }
  }

  delay(10);
}
