#pragma once
#include <stdint.h>

// Packet format 
// [0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC8]
//
// Pi - Pico  TYPE=0x01  LEN=5
//   [dir] [speed] [servo] [led_mode] [flags]
//
//   flags bitmask:
//     bit 0 (0x01) - reset yaw
//     bit 1 (0x02) - reset encoder
//     bits 2–7     - reserved
//
// Pico → Pi  TYPE=0x02  LEN=14
//   [tof0_H][tof0_L] [tof1_H][tof1_L]
//   [tof2_H][tof2_L] [tof3_H][tof3_L]
//   [batt_H][batt_L]
//   [yaw_H][yaw_L]         degrees * 10
//   [enc_H][enc_L]         int16 ticks

#define UART_START_BYTE   0xAA
#define UART_CMD_TYPE     0x01
#define UART_TELEM_TYPE   0x02
#define UART_BAUD         115200

// Flag bits
#define CMD_FLAG_RESET_YAW      0x01
#define CMD_FLAG_RESET_ENCODER  0x02

void    uart_init();
void    uart_send_telemetry();   // call every 100ms on Core 1
void    uart_receive();          // call every 10ms on Core 1
uint8_t crc8(uint8_t *data, uint8_t len);