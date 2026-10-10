#pragma once
#include <Arduino.h>

namespace ctl {
constexpr uint32_t MAGIC = 0x314C5443; // CTL1, little endian
constexpr uint8_t VERSION = 1;
constexpr uint16_t UDP_PORT = 4210;
constexpr uint8_t WIFI_CHANNEL = 6;
constexpr char WIFI_PASSWORD[] = "ESPConTL11"; // Demo only; change on ALL boards.
constexpr uint32_t FAILSAFE_MS = 300;
enum : uint8_t { HELLO = 10, WELCOME = 11 };
enum : uint8_t { CONTROL = 1, PING = 2, PONG = 3, NEUTRAL = 4 };
enum : uint8_t { ROLE_RECEIVER = 1, ROLE_USB = 2 };
enum : uint8_t { FLAG_USB = 1 };
// PONG.flags only: optional USB bridge diagnostics; command layout/version unchanged.
enum : uint8_t {
  PONG_USB_MOUNTED=0x02, PONG_USB_READY=0x04, PONG_USB_CONTROL=0x08,
  PONG_USB_KEY_TX=0x10, PONG_USB_MOUSE_TX=0x20, PONG_USB_SUSPENDED=0x40,
  PONG_USB_STATUS=0x80
};
enum : uint8_t { B_L=1, B_R=2, B_J1=4, B_J2=8 };
#pragma pack(push, 1)
struct Hello {
  uint32_t magic = MAGIC;
  uint8_t version = VERSION, type = HELLO;
  uint16_t bytes = sizeof(Hello);
  uint32_t session = 0;
  uint8_t mac[6] = {};
  uint8_t channel = WIFI_CHANNEL, role = 0;
};
// USB HID usages, NOT ASCII. Mouse deltas are relative, signed.
struct HidReport {
  uint8_t modifiers = 0;
  uint8_t keys[6] = {};
  uint8_t mouseButtons = 0;
  int8_t dx = 0, dy = 0, wheel = 0;
};
struct Packet {
  uint32_t magic = MAGIC;
  uint8_t version = VERSION, type = CONTROL;
  uint16_t bytes = sizeof(Packet);
  uint32_t session = 0, seq = 0, stampUs = 0;
  int16_t axis[4] = {}; // J1X,J1Y,J2X,J2Y: -1000..1000
  uint8_t buttons = 0, flags = 0;
  HidReport hid;
};
#pragma pack(pop)
static_assert(sizeof(Hello)==20, "Hello wire size");
static_assert(sizeof(Packet)==41, "Packet wire size");
inline bool valid(const Packet &p) {
  return p.magic==MAGIC && p.version==VERSION && p.bytes==sizeof(Packet)
      && p.type>=CONTROL && p.type<=NEUTRAL;
}
inline bool valid(const Hello &h) {
  return h.magic==MAGIC && h.version==VERSION && h.bytes==sizeof(Hello);
}
inline bool newer(uint32_t seq, uint32_t previous) {
  return (int32_t)(seq-previous)>0;
}
}
