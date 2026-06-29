#pragma once

#include <Arduino.h>

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t REMOTE_PACKET_MAGIC = 0x53555031; // "SUP1"

enum PacketType : uint8_t {
  PKT_REMOTE_CONTROL = 1,
  PKT_REMOTE_TELEMETRY = 2,
};

constexpr uint8_t CONTROL_FLAG_IMU_OK = 0x01;
constexpr uint8_t CONTROL_FLAG_ARM_REQUESTED = 0x02;
constexpr uint8_t CONTROL_FLAG_CRUISE_ACTIVE = 0x04;

struct RemoteControlPacket {
  uint32_t magic;
  uint8_t type;
  uint16_t seq;
  uint32_t txMillis;
  uint8_t throttle;       // 0..100
  int8_t steering;        // -100..100
  uint8_t armToggleCount; // incrementat la fiecare apasare ARM
  uint8_t mode;           // 0 manual, 1 sport, rezervat
  uint8_t flags;
} __attribute__((packed));

struct RemoteTelemetryPacket {
  uint32_t magic;
  uint8_t type;
  uint16_t seq;
  uint32_t rxMillis;
  bool armed;
  uint8_t satellites;
  uint8_t mainBatteryPercent;
  uint8_t remoteBatteryPercent;
  uint8_t linkQuality; // 0..100
  int8_t rssi;
  float batteryVoltage;
  float groundSpeed; // m/s
  float heading;     // deg
} __attribute__((packed));
