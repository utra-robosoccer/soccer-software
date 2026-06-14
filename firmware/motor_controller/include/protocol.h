// MiniBot MCU wire protocol — the host/MCU contract (mirrors soccer-firmware).
//
// The Jetson's ros2_control serial hardware (soccer_hardware) and this firmware
// agree byte-for-byte on these frames. CRC16-CCITT (poly 0x1021, init 0xFFFF).
#ifndef MINIBOT_PROTOCOL_H_
#define MINIBOT_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MINIBOT_CRC16_POLY 0x1021u
#define MINIBOT_CRC16_INIT 0xFFFFu

// Host -> MCU: MIT-style impedance command run by the 1 kHz loop ("<fffff" + crc).
typedef struct __attribute__((packed)) {
  float q;       // target position (rad)
  float qd;      // target velocity (rad/s)
  float kp;      // position gain
  float kd;      // velocity gain
  float tau_ff;  // feed-forward torque (N·m)
  uint16_t crc;
} MotorCmd;

// MCU -> host: measured state ("<ffffIH" + crc).
typedef struct __attribute__((packed)) {
  float pos;      // measured position (rad)
  float vel;      // measured velocity (rad/s)
  float eff;      // measured effort/torque from current sense (N·m)
  float temp;     // driver temperature (deg C)
  uint32_t ts_us; // MCU timestamp (microseconds)
  uint16_t crc;
} MotorState;

static inline uint16_t minibot_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = MINIBOT_CRC16_INIT;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ MINIBOT_CRC16_POLY)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

#ifdef __cplusplus
}
#endif

#endif  // MINIBOT_PROTOCOL_H_
