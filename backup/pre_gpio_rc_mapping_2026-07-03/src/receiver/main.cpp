#include <Arduino.h>
#include <MAVLink.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "../common/remote_protocol.h"

#define FC_RX_PIN 2
#define FC_TX_PIN 1

constexpr uint16_t CONTROL_FAILSAFE_MS = 500;
constexpr int8_t WIFI_MAX_TX_POWER = 78; // 19.5 dBm in 0.25 dBm units.

HardwareSerial fcSerial(1);

// Valori channels RC (1000-2000us)
uint16_t ch_throttle = 1000;
uint16_t ch_steering = 1500;
uint16_t ch_arm      = 1000;
uint16_t ch_mode     = 1000;

unsigned long lastRCOverride    = 0;
unsigned long lastHeartbeat     = 0;
unsigned long lastTelemetryPrint = 0;
unsigned long lastEspNowTelemetry = 0;
unsigned long lastControlPacket = 0;

// Telemetrie
float gps_lat, gps_lon, gps_speed, gps_heading;
uint8_t gps_satellites;
float battery_voltage;
uint8_t battery_remaining;
bool armed = false;

uint8_t txMac[6] = {};
bool txKnown = false;
uint16_t rxControlSeq = 0;
uint16_t txTelemetrySeq = 0;
uint8_t lastArmToggleCount = 0;
bool lastArmRequested = false;
bool controlFailsafeActive = false;
bool hasReceivedControl = false;
uint32_t controlPackets = 0;
uint32_t missedPackets = 0;
int lastRssi = 0;

void sendHeartbeat();
void sendRCOverride();
void receiveTelemetry();
void printTelemetry();
void sendArmCommand(bool arm);
void initEspNow();
void sendTelemetryPacket();
void onControlRecv(const uint8_t *mac, const uint8_t *data, int len);

uint16_t mapThrottleToPwm(uint8_t throttle) {
  return map(constrain(throttle, 0, 100), 0, 100, 1000, 2000);
}

uint16_t mapSteeringToPwm(int8_t steering) {
  return map(constrain(steering, -100, 100), -100, 100, 1000, 2000);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 MAVLink Receiver pornit");

  fcSerial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  Serial.println("Serial catre FC initializat");
  initEspNow();
}

void loop() {
  // Trimite heartbeat la 1Hz (obligatoriu pentru MAVLink)
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  // Trimite RC override la 25Hz
  if (millis() - lastRCOverride >= 40) {
    lastRCOverride = millis();
    sendRCOverride();
  }

  // Citeste telemetrie de la FC
  receiveTelemetry();

  if (hasReceivedControl && millis() - lastControlPacket > CONTROL_FAILSAFE_MS &&
      !controlFailsafeActive) {
    ch_throttle = 1000;
    ch_steering = 1500;
    lastArmRequested = false;
    controlFailsafeActive = true;
    sendArmCommand(false);
    Serial.println("CONTROL FAILSAFE: neutral + DISARM");
  }

  if (millis() - lastEspNowTelemetry >= 100) {
    lastEspNowTelemetry = millis();
    sendTelemetryPacket();
  }

  // Printeaza telemetrie la fiecare 500ms
  if (millis() - lastTelemetryPrint >= 500) {
    lastTelemetryPrint = millis();
    printTelemetry();
  }
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    1,                        // system ID (al nostru)
    MAV_COMP_ID_PERIPHERAL,   // component ID
    &msg,
    MAV_TYPE_GCS,             // tip: ground control station
    MAV_AUTOPILOT_INVALID,
    0, 0, MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  fcSerial.write(buf, len);
  Serial.println("Heartbeat trimis");
}

void sendRCOverride() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE
  // target_system=1 (FC), target_component=1
  mavlink_msg_rc_channels_override_pack(
    255,  // GCS system ID
    MAV_COMP_ID_MISSIONPLANNER,
    &msg,
    1,    // target system (FC)
    1,    // target component
    ch_throttle,  // ch1
    ch_steering,  // ch2
    1500,         // ch3
    ch_arm,       // ch4  
    ch_mode,      // ch5
    1500,         // ch6
    1500,         // ch7
    1500,         // ch8
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // ch9-18 (0 = ignore)
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  fcSerial.write(buf, len);
}

void sendArmCommand(bool arm) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    255,
    MAV_COMP_ID_MISSIONPLANNER,
    &msg,
    1,
    1,
    MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    arm ? 1.0f : 0.0f,
    0, 0, 0, 0, 0, 0
  );

  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  fcSerial.write(buf, len);
  Serial.printf("ARM command trimis: %s\n", arm ? "ARM" : "DISARM");
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  const esp_err_t powerResult = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL");
    return;
  }

  esp_now_register_recv_cb(onControlRecv);
  Serial.printf("WiFi TX power: %s (requested 19.5 dBm)\n",
                powerResult == ESP_OK ? "OK" : "FAIL");
  Serial.print("Receiver ESP-NOW MAC: ");
  Serial.println(WiFi.macAddress());
}

void onControlRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(RemoteControlPacket)) {
    return;
  }

  RemoteControlPacket pkt = {};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != REMOTE_PACKET_MAGIC || pkt.type != PKT_REMOTE_CONTROL) {
    return;
  }

  if (!txKnown || memcmp(txMac, mac, 6) != 0) {
    memcpy(txMac, mac, 6);
    txKnown = true;

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.printf("TX peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  txMac[0], txMac[1], txMac[2], txMac[3], txMac[4], txMac[5]);
  }

  const bool syncAfterFailsafe = !hasReceivedControl || controlFailsafeActive;

  lastRssi = 0;
  if (controlPackets > 0 && pkt.seq != static_cast<uint16_t>(rxControlSeq + 1)) {
    missedPackets++;
  }
  rxControlSeq = pkt.seq;
  controlPackets++;
  lastControlPacket = millis();
  hasReceivedControl = true;

  ch_throttle = mapThrottleToPwm(pkt.throttle);
  ch_steering = mapSteeringToPwm(pkt.steering);

  if (syncAfterFailsafe) {
    lastArmToggleCount = pkt.armToggleCount;
    lastArmRequested = false;
    controlFailsafeActive = false;
    return;
  }

  if (pkt.armToggleCount != lastArmToggleCount) {
    lastArmToggleCount = pkt.armToggleCount;
    lastArmRequested = (pkt.flags & CONTROL_FLAG_ARM_REQUESTED) != 0;
    sendArmCommand(lastArmRequested);
  }
}

void sendTelemetryPacket() {
  if (!txKnown) {
    return;
  }

  const uint32_t total = controlPackets + missedPackets;
  uint8_t lq = 0;
  if (total > 0) {
    const uint32_t percent = (controlPackets * 100UL) / total;
    lq = static_cast<uint8_t>(percent > 100 ? 100 : percent);
  }

  RemoteTelemetryPacket pkt = {};
  pkt.magic = REMOTE_PACKET_MAGIC;
  pkt.type = PKT_REMOTE_TELEMETRY;
  pkt.seq = txTelemetrySeq++;
  pkt.rxMillis = millis();
  pkt.armed = armed;
  pkt.satellites = gps_satellites;
  pkt.mainBatteryPercent = battery_remaining <= 100 ? battery_remaining : 0;
  pkt.remoteBatteryPercent = 100;
  pkt.linkQuality = lq;
  pkt.rssi = static_cast<int8_t>(constrain(lastRssi, -128, 127));
  pkt.batteryVoltage = battery_voltage;
  pkt.groundSpeed = gps_speed;
  pkt.heading = gps_heading;

  esp_now_send(txMac, reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
}

void receiveTelemetry() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (fcSerial.available()) {
    uint8_t c = fcSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
          break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&msg, &gps);
          gps_lat        = gps.lat / 1e7;
          gps_lon        = gps.lon / 1e7;
          gps_speed      = gps.vel / 100.0;  // cm/s -> m/s
          gps_satellites = gps.satellites_visible;
          break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          gps_speed   = hud.groundspeed;
          gps_heading = hud.heading;
          break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_sys_status_t sys;
          mavlink_msg_sys_status_decode(&msg, &sys);
          battery_voltage   = sys.voltage_battery / 1000.0;
          battery_remaining = sys.battery_remaining;
          break;
        }
      }
    }
  }
}

void printTelemetry() {
  Serial.println("--- Telemetrie ---");
  Serial.print("Armed: ");       Serial.println(armed ? "DA" : "NU");
  Serial.print("Speed: ");       Serial.print(gps_speed); Serial.println(" m/s");
  Serial.print("Heading: ");     Serial.println(gps_heading);
  Serial.print("Satellites: ");  Serial.println(gps_satellites);
  Serial.print("Battery: ");     Serial.print(battery_voltage); Serial.println("V");
  Serial.println("------------------");
}
