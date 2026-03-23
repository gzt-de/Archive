/**
 * @file serial_protocol.c
 * @brief Ninebot Serial Protocol - UART framing and message handling
 *
 * Protocol frame format:
 *   [0x5A] [0xA5] [LEN] [SRC] [DST] [CMD] [SUBCMD] [DATA...] [CKSUM_LO] [CKSUM_HI]
 *
 * Checksum: ones-complement 16-bit sum of bytes from LEN to end of data
 *
 * Reconstructed from:
 * sub_8005514 = serial_checksum
 * sub_800552E = serial_build_frame
 * sub_800557E = serial_enqueue_tx
 * sub_80057D8 = serial_process_tx
 * sub_8005820 = serial_transmit_raw
 * sub_80055E4 = serial_dispatch_rx
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * Globals
 * ============================================================================ */
serial_tx_buf_t g_serial_tx_buf;

extern uint8_t g_serial_rx_active;      /* byte_2000000C */
extern uint8_t g_serial_tx_count;       /* byte_2000000A */
extern uint8_t g_serial_tx_pending;     /* byte_20000009 */

/* ============================================================================
 * Checksum Calculation (sub_8005514)
 *
 * Simple ones-complement 16-bit sum
 * ============================================================================ */
uint16_t serial_checksum(const uint8_t *data, uint32_t length)
{
    int16_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint16_t)(~sum);
}

/* ============================================================================
 * Build Serial Frame (sub_800552E)
 *
 * Constructs a complete Ninebot serial frame in the provided buffer.
 * Returns the total frame length.
 * ============================================================================ */
uint8_t serial_build_frame(uint8_t src_addr, uint8_t dst_addr,
                            uint8_t payload_len, uint8_t cmd,
                            uint8_t subcmd, const void *payload,
                            uint8_t *out_buf)
{
    const uint8_t *payload_bytes = (const uint8_t *)payload;
    uint8_t idx;
    uint16_t cksum;

    /* Frame header */
    out_buf[0] = SERIAL_HEADER_0;   /* 0x5A */
    out_buf[1] = SERIAL_HEADER_1;   /* 0xA5 */

    /* Payload metadata */
    out_buf[2] = payload_len;       /* Length */
    out_buf[3] = src_addr;          /* Source */
    out_buf[4] = dst_addr;          /* Destination */
    out_buf[5] = cmd;               /* Command */
    out_buf[6] = subcmd;            /* Sub-command */

    /* Copy payload data */
    idx = 7;
    for (uint8_t i = 0; i < payload_len; i++) {
        out_buf[idx] = payload_bytes[i];
        idx++;
    }

    /* Calculate checksum over bytes [2..idx-1] */
    cksum = serial_checksum(&out_buf[2], idx - 2);
    out_buf[idx] = (uint8_t)(cksum & 0xFF);
    out_buf[idx + 1] = (uint8_t)(cksum >> 8);

    return idx + 2;
}

/* ============================================================================
 * Enqueue TX Message (sub_800557E)
 *
 * Thread-safe enqueue of a serial message into the ring buffer.
 * Uses a simple lock byte to prevent reentrancy (ISR-safe on Cortex-M3).
 * ============================================================================ */
int serial_enqueue_tx(serial_tx_buf_t *buf, uint8_t src_addr, uint8_t dst_addr,
                       uint8_t payload_len, uint8_t cmd, uint8_t subcmd,
                       const void *payload)
{
    int result = (int)(intptr_t)payload;

    /* Check lock */
    if (buf->busy)
        return result;

    buf->busy = 1;

    if (payload_len <= SERIAL_MAX_PAYLOAD && buf->free_slots) {
        /* Build frame directly into the slot */
        uint8_t slot_idx = buf->write_idx;
        buf->slots[slot_idx].length = serial_build_frame(
            src_addr, dst_addr, payload_len, cmd, subcmd,
            payload, buf->slots[slot_idx].data
        );

        /* Advance write pointer (circular, 8 slots) */
        buf->write_idx = (slot_idx + 1 == SERIAL_TX_BUF_SLOTS) ?
                          0 : slot_idx + 1;
        buf->free_slots--;
        result = buf->free_slots;
    }

    buf->busy = 0;
    return result;
}

/* ============================================================================
 * Process TX Queue (sub_80057D8)
 *
 * Called from the main loop to drain the TX buffer.
 * Sends one message per call to avoid blocking.
 * ============================================================================ */
int serial_process_tx(void)
{
    if (g_serial_rx_active)
        return g_serial_rx_active;

    if (g_serial_tx_pending != g_serial_tx_buf.read_idx) {
        g_serial_rx_active = 1;

        uint8_t slot = g_serial_tx_buf.read_idx;
        int result = uart_serial_transmit(
            g_serial_tx_buf.slots[slot].data,
            (uint16_t)g_serial_tx_buf.slots[slot].length
        );

        if (result) {
            /* Advance read pointer */
            if (++g_serial_tx_buf.read_idx == SERIAL_TX_BUF_SLOTS)
                g_serial_tx_buf.read_idx = 0;
            g_serial_tx_count++;
        }

        g_serial_rx_active = 0;
    }
    return 0;
}

/* ============================================================================
 * Dispatch Received Message (sub_80055E4)
 *
 * Routes incoming serial messages to the appropriate handler based on
 * src/dst address and command byte.
 *
 * Address assignments (Ninebot protocol):
 *   0x00 = Dashboard/Display
 *   0x02 = BMS (Battery Management)
 *   0x04 = Motor Controller (ESC)
 *   0x06/0x07 = External battery / accessories
 *   0x16 (22) = VCU (this unit)
 *   0x23 (35) = Config/OTA handler
 *   0x3D (61) = CAN gateway / bridge
 *   0x3E (62) = BLE module
 *   0x3F (63) = IoT module
 * ============================================================================ */
int serial_dispatch_rx(const uint8_t *msg, int source, int param3, int param4)
{
    uint8_t dst_addr = msg[2];
    uint8_t src_addr = msg[1];
    uint8_t cmd_byte = msg[3];
    uint8_t subcmd   = msg[4];

    /* Messages addressed to VCU (0x16 = 22) */
    if (dst_addr == 22) {
        switch (src_addr) {
            case 35:    /* Configuration / OTA command */
                return config_cmd_handler(msg, source, param3, param4);
            case 2:     /* BMS status response */
                return bms_status_handler(msg, source);
            case 4:     /* ESC status response */
                return esc_status_handler(msg, source);
            case 6:     /* External battery */
            case 7:
                return ext_battery_handler(msg, source);
            default:
                break;
        }
    }
    /* Messages from BLE/IoT (35, 61, 62, 63) - relay or process */
    else if (dst_addr == 35 || dst_addr == 61 || dst_addr == 62 || dst_addr == 63) {
        uint8_t payload_cmd = cmd_byte;

        /* Check if it's a restricted command requiring auth */
        if ((unsigned)(payload_cmd - 7) <= 3) {
            /* Commands 7-10 require authentication */
            if (g_scooter.status_word2 & 1) {  /* word_20003948 bit 0 */
                /* Auth OK - forward */
                return serial_enqueue_tx(&g_serial_tx_buf, 22, src_addr,
                                          1, 11, 9, &msg[4]);
            }
            g_scooter.auth_state = 1;   /* byte_2000390F */
        } else {
            /* Special command handling */
            if (payload_cmd == 7) {
                /* BLE pairing / lock command */
                if (subcmd == 16 && cmd_byte == 43) {
                    /* Handle BLE key exchange */
                    ble_key_exchange_handler(&msg[8]);
                }
                if (cmd_byte == 122) {
                    /* Remote lock/unlock */
                    if (!g_scooter.cruise_mode &&
                        !(g_bms.flags1 & 8) &&
                        (g_scooter.state == SCOOTER_STATE_STARTING ||
                         g_scooter.state == SCOOTER_STATE_READY)) {
                        if (g_bms.current > -70) {
                            g_scooter.auth_state = 1;
                            /* sub_800A044(15, ...) - state transition */
                            led_set_pattern(18);
                        }
                    }
                }
                if (cmd_byte == 2 && subcmd == 130) {
                    if ((g_scooter.status_word2 & 1) && g_scooter.state) {
                        led_set_pattern(18);
                    }
                }
            }
        }

        /* Relay to destination */
        if (source == 1) {
            /* From UART - relay to serial */
            return serial_enqueue_tx(&g_serial_tx_buf, src_addr, dst_addr,
                                      msg[0], cmd_byte, subcmd,
                                      (const void *)&msg[5]);
        } else {
            /* From CAN - relay to CAN */
            return can_send_message(src_addr, dst_addr, msg[0],
                                     cmd_byte, subcmd, (void *)&msg[5]);
        }
    }

    return 0;
}

/* ============================================================================
 * Forward declarations for handlers referenced above
 * ============================================================================ */

/* sub_8007FD0 */
int __attribute__((weak)) config_cmd_handler(const uint8_t *msg, int src, int p3, int p4)
{
    (void)msg; (void)src; (void)p3; (void)p4;
    return 0;
}

/* sub_8007FA8 */
int __attribute__((weak)) bms_status_handler(const uint8_t *msg, int src)
{
    (void)msg; (void)src;
    return 0;
}

/* sub_8007910 */
int __attribute__((weak)) esc_status_handler(const uint8_t *msg, int src)
{
    (void)msg; (void)src;
    return 0;
}

/* sub_8007B68 */
int __attribute__((weak)) ext_battery_handler(const uint8_t *msg, int src)
{
    (void)msg; (void)src;
    return 0;
}

/* sub_800A720 */
void __attribute__((weak)) ble_key_exchange_handler(const uint8_t *data)
{
    (void)data;
}

/* sub_8008B80 */
int __attribute__((weak)) led_set_pattern(int pattern)
{
    (void)pattern;
    return 0;
}
