#include "uart_comms.h"
#include "shared_state.h"
#include "sensors_imu.h"
#include "encoder.h"
#include "leds.h"
#include <Arduino.h>

uint8_t crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

void uart_init() {
    Serial1.begin(UART_BAUD);
    Serial.println("[OK] UART comms init");
}

void uart_receive() {
    while (Serial1.available() >= 3) {
        if (Serial1.read() != UART_START_BYTE) continue;

        uint8_t msg_type = Serial1.read();
        uint8_t length   = Serial1.read();

        if (length > 32) continue;

        // Wait for payload + CRC
        uint32_t wait = millis();
        while (Serial1.available() < (length + 1)) {
            if (millis() - wait > 50) return;
        }

        uint8_t payload[32];
        for (uint8_t i = 0; i < length; i++) {
            payload[i] = Serial1.read();
        }
        uint8_t received_crc = Serial1.read();

        // Verify CRC over type + len + payload
        uint8_t full[34];
        full[0] = msg_type;
        full[1] = length;
        for (uint8_t i = 0; i < length; i++) full[2 + i] = payload[i];
        uint8_t calc_crc = crc8(full, 2 + length);

        if (calc_crc != received_crc) {
            Serial.println("[WARN] UART CRC mismatch");
            continue;
        }

        // Parse command packet — accept LEN=4 (old) or LEN=5 (with flags)
        if (msg_type == UART_CMD_TYPE && (length == 4 || length == 5)) {
            uint8_t flags = (length == 5) ? payload[4] : 0x00;

            STATE_LOCK();
            g_state.cmd_dir      = (int8_t)payload[0];
            g_state.cmd_speed    = payload[1];
            g_state.cmd_servo    = payload[2];
            g_state.cmd_led_mode = payload[3];
            g_state.cmd_fresh    = true;
            STATE_UNLOCK();

            // Handle flags immediately — these are one-shot actions
            if (flags & CMD_FLAG_RESET_YAW) {
                imu_reset_yaw();
                Serial.println("[CMD] Yaw reset");
            }
            if (flags & CMD_FLAG_RESET_ENCODER) {
                encoder_reset();
                Serial.println("[CMD] Encoder reset");
            }
        }
    }
}

void uart_send_telemetry() {
    STATE_LOCK();
    uint16_t tof[4] = {
        g_state.tof_mm[0], g_state.tof_mm[1],
        g_state.tof_mm[2], g_state.tof_mm[3]
    };
    uint16_t batt_mv = g_state.battery_mv;
    int16_t  yaw10   = g_state.imu_yaw;
    int16_t  enc     = (int16_t)g_state.enc_ticks;
    STATE_UNLOCK();

    uint8_t payload[14] = {
        (uint8_t)(tof[0] >> 8), (uint8_t)(tof[0] & 0xFF),
        (uint8_t)(tof[1] >> 8), (uint8_t)(tof[1] & 0xFF),
        (uint8_t)(tof[2] >> 8), (uint8_t)(tof[2] & 0xFF),
        (uint8_t)(tof[3] >> 8), (uint8_t)(tof[3] & 0xFF),
        (uint8_t)(batt_mv >> 8),(uint8_t)(batt_mv & 0xFF),
        (uint8_t)(yaw10   >> 8),(uint8_t)(yaw10   & 0xFF),
        (uint8_t)(enc     >> 8),(uint8_t)(enc      & 0xFF),
    };

    uint8_t full[16];
    full[0] = UART_TELEM_TYPE;
    full[1] = 14;
    for (uint8_t i = 0; i < 14; i++) full[2 + i] = payload[i];
    uint8_t crc = crc8(full, 16);

    Serial1.write(UART_START_BYTE);
    Serial1.write(UART_TELEM_TYPE);
    Serial1.write((uint8_t)14);
    Serial1.write(payload, 14);
    Serial1.write(crc);
}