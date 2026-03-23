/**
 * @file can_protocol.c
 * @brief Ninebot CAN Bus Protocol - ESC/BMS/Accessory Communication
 *
 * The G3 uses CAN bus for communication between:
 *   - VCU (Vehicle Control Unit - this code)
 *   - ESC (Electronic Speed Controller / Motor Controller)
 *   - BMS (Battery Management System)
 *   - Dashboard display
 *   - BLE module
 *   - External battery
 *
 * CAN Message format (Ninebot proprietary):
 *   Word 0 [31:0]: Control bits, addresses, command routing
 *     Bits [31:30]: Reserved flags
 *     Bit  [29]:    Direction/valid flag
 *     Bits [28:26]: Channel/bus ID (0-7)
 *     Bits [23:16]: 0xEF marker (magic)
 *     Bits [15:8]:  Destination address
 *     Bits [7:0]:   Source address / command
 *   Byte 4: Total length (data + 3 header bytes)
 *   Byte 5: Sub-command / sequence
 *   Byte 6: Param 1
 *   Byte 7: Param 2
 *   Bytes 8+: Payload (up to 5 bytes per CAN frame)
 *
 * Reconstructed from:
 * sub_800D908 = can_build_standard_msg
 * sub_800D964 = can_build_nack_msg
 * sub_800D9A4 = can_build_multiframe_msg
 * sub_800CFF4 = can_send_message
 * sub_800CF30 = can_init_filters
 * sub_800D4D0 = can_fw_update_handler (firmware update over CAN)
 * sub_800D784 = can_fw_send_block
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * CAN Message Structure
 * ============================================================================ */
typedef struct __packed {
    uint32_t control;       /* Control word with addresses and flags */
    uint8_t  length;        /* Total frame length */
    uint8_t  subcmd;        /* Sub-command or sequence number */
    uint8_t  param1;        /* Parameter 1 */
    uint8_t  param2;        /* Parameter 2 */
    uint8_t  data[5];       /* Payload data (max 5 bytes) */
} can_msg_t;

/* ============================================================================
 * CAN Peripheral Setup
 * ============================================================================ */

#define CAN_MCR     (*(volatile uint32_t *)(CAN1_BASE + 0x00))
#define CAN_MSR     (*(volatile uint32_t *)(CAN1_BASE + 0x04))
#define CAN_TSR     (*(volatile uint32_t *)(CAN1_BASE + 0x08))
#define CAN_IER     (*(volatile uint32_t *)(CAN1_BASE + 0x14))
#define CAN_BTR     (*(volatile uint32_t *)(CAN1_BASE + 0x1C))
#define CAN_FMR     (*(volatile uint32_t *)(CAN1_BASE + 0x200))

/* ============================================================================
 * CAN Init (sub_8006DB8)
 * ============================================================================ */
int can_init(void)
{
    /* Enter init mode */
    CAN_MCR |= 1;
    while (!(CAN_MSR & 1))
        ;

    /* Configure bit timing for 500kbps at 72MHz APB1=36MHz
     * BRP=3, TS1=12, TS2=5, SJW=1 -> 36MHz/(4*(1+12+5)) = 500kbps */
    CAN_BTR = (0 << 24) |  /* SJW = 1 TQ */
              (4 << 20) |  /* TS2 = 5 TQ */
              (11 << 16) | /* TS1 = 12 TQ */
              3;            /* BRP = 4 */

    /* Enable interrupts: FIFO0 message pending, TX mailbox empty */
    CAN_IER |= (1 << 0) | (1 << 1);

    /* Exit init mode */
    CAN_MCR &= ~1;
    while (CAN_MSR & 1)
        ;

    return 0;
}

/* ============================================================================
 * Build Standard CAN Message (sub_800D908)
 *
 * Constructs a Ninebot-protocol CAN frame.
 * ============================================================================ */
int can_build_standard_msg(uint32_t channel, uint8_t subcmd,
                            int src_addr, int dst_addr,
                            uint8_t param1, uint8_t param2,
                            const void *payload, uint32_t payload_len,
                            can_msg_t *out_msg)
{
    if (!payload || !out_msg || payload_len > 5 || channel > 7)
        return 0;

    /* Build control word */
    out_msg->control = (out_msg->control & 0xA0000000) |
                       ((channel << 26) | (dst_addr << 8) | src_addr) & 0x1F10FFFF |
                       0xEF0000 |       /* Magic marker */
                       0x20000000;      /* Valid flag */

    out_msg->length = payload_len + 3;
    out_msg->subcmd = subcmd;
    out_msg->param1 = param1;
    out_msg->param2 = param2;

    const uint8_t *p = (const uint8_t *)payload;
    for (uint8_t i = 0; i < payload_len; i++)
        out_msg->data[i] = p[i];

    return 1;
}

/* ============================================================================
 * Build NACK/Error Response (sub_800D964)
 * ============================================================================ */
int can_build_nack_msg(int src_addr, int dst_addr, can_msg_t *out_msg)
{
    if (!out_msg)
        return 0;

    out_msg->control = (out_msg->control & 0xA0000000) |
                       (src_addr | (dst_addr << 8)) & 0x713FFFF |
                       0x18EC0000 | 0x20000000;

    out_msg->length = 8;
    out_msg->subcmd = 0xFF;
    out_msg->param1 = 0xFF;
    out_msg->param2 = 0xFF;
    out_msg->data[0] = 0xFF;
    out_msg->data[1] = 0xFF;
    out_msg->data[2] = 0x00;
    out_msg->data[3] = 0xEF;   /* ~0x10 = 0xEF */
    out_msg->data[4] = 0x00;

    return 1;
}

/* ============================================================================
 * Build Multi-Frame Message (sub_800D9A4)
 *
 * For payloads larger than 5 bytes, uses segmented transfer.
 * ============================================================================ */
int can_build_multiframe_msg(int src_addr, int dst_addr,
                              uint32_t total_size, const void *data,
                              uint16_t *frame_counter, can_msg_t *out_msg)
{
    if (!out_msg || (total_size % 7) || !frame_counter)
        return 0;

    uint16_t remaining = *frame_counter;
    if (remaining > 7)
        remaining = 7;

    out_msg->control = (out_msg->control & 0xA0000000) |
                       (src_addr | (dst_addr << 8)) & 0x714FFFF |
                       0x18EB0000 | 0x20000000;

    out_msg->length = 8;
    out_msg->subcmd = (total_size + 6) / 7 + 1;

    /* Copy segment data */
    const uint8_t *p = (const uint8_t *)data;
    uint16_t i;
    for (i = 0; i <= remaining; i++)
        (&out_msg->param1)[i] = p[i];

    /* Pad remaining bytes with 0xFF */
    while (i < 8) {
        (&out_msg->subcmd)[i] = 0xFF;
        i++;
    }

    *frame_counter = remaining;
    return 1;
}

/* ============================================================================
 * Send CAN Message (sub_800CFF4)
 *
 * High-level function to send a message to a specific device.
 * This builds the frame and queues it for transmission.
 * ============================================================================ */
int can_send_message(uint8_t dst, uint8_t src, uint8_t len,
                      uint8_t cmd, uint8_t subcmd, void *data)
{
    can_msg_t msg;
    msg.control = 0;

    if (!can_build_standard_msg(0, subcmd, src, dst, cmd, subcmd,
                                 data, len > 5 ? 5 : len, &msg))
        return 0;

    /* Queue to CAN TX mailbox */
    volatile uint32_t *tx_mailbox = (volatile uint32_t *)(CAN1_BASE + 0x180);

    /* Wait for empty mailbox */
    while (!(CAN_TSR & (1 << 26)))
        ;

    tx_mailbox[0] = msg.control;
    tx_mailbox[1] = msg.length | (msg.subcmd << 8) |
                    (msg.param1 << 16) | (msg.param2 << 24);

    uint32_t data_word = 0;
    for (int i = 0; i < 4 && i < 5; i++)
        data_word |= ((uint32_t)msg.data[i]) << (i * 8);
    tx_mailbox[2] = data_word;

    /* Request transmission */
    tx_mailbox[0] |= (1 << 0);

    return 1;
}

/* ============================================================================
 * CAN Firmware Update Handler (sub_800D4D0)
 *
 * Handles firmware update packets received over CAN bus.
 * Supports segmented transfer of firmware images.
 *
 * Message types:
 *   0xEC (236) = Control messages (start, ack, nack, complete)
 *     subcmd 16 (0x10) = Start transfer
 *     subcmd 17 (0x11) = Ack/continue
 *     subcmd 19 (0x13) = Verify complete
 *     subcmd 255 (0xFF) = Abort
 *   0xEB (235) = Data segments
 * ============================================================================ */
int can_fw_update_handler(fw_update_session_t *session, int can_data, int callbacks)
{
    if (!session || !can_data)
        return 0;

    uint8_t *can_bytes = (uint8_t *)can_data;
    uint8_t msg_type = can_bytes[2];    /* BYTE2 of control word */
    uint8_t src_addr = can_bytes[1];
    uint8_t dst_addr = can_bytes[0];

    if (msg_type == CAN_MSG_TYPE_FW_CTRL) {     /* 0xEC */
        uint8_t subcmd = can_bytes[5];

        /* Check magic marker 0xEF00 */
        uint32_t marker = (can_bytes[10] | (can_bytes[11] << 8)) |
                          (can_bytes[12] << 16);
        if (marker != 0xEF00)
            return subcmd;

        if (subcmd == 16) {
            /* Start transfer command */
            if (session->state != 1) {
                /* Must match existing session addresses */
                if (session->dst_addr != src_addr ||
                    session->src_addr != dst_addr)
                    return session->dst_addr;
            }

            session->dst_addr = src_addr;
            session->src_addr = dst_addr;

            uint16_t data_size = can_bytes[6];
            uint8_t total_chunks = can_bytes[8];
            session->total_size = data_size;
            session->total_chunks = total_chunks;
            session->write_offset = 0;
            session->chunk_offset = 0;
            session->checksum = 0;

            /* Validate: data must fit in buffer, chunk count must match */
            if (session->max_size < data_size ||
                (data_size + 6) / 7 != total_chunks)
            {
                session->state = 1;     /* Reset to idle */
                can_msg_t nack;
                can_build_nack_msg(src_addr, dst_addr, &nack);
                /* Send NACK via callback */
                return 0;
            }

            /* Send ACK: ready to receive */
            can_msg_t ack;
            ack.control = 0;
            /* Build ack with subcmd=17, ready=1 */
            session->state = 10;    /* Active transfer */
            session->retry_counter = 0;
            session->retry_timeout = 1250 / session->chunk_size;

            return 0;
        }
        else if (subcmd == 17) {
            /* Ack/Continue from sender */
            if (session->src_addr != src_addr || session->dst_addr != dst_addr)
                return subcmd;

            if (session->state == 6) {
                /* Waiting for sender ack */
                if (!can_bytes[6]) {
                    session->state = 7;
                    session->retry_counter = 0;
                    session->retry_timeout = 1050 / session->chunk_size;
                }
                return 0;
            }

            if (session->state == 4 || session->state == 7) {
                if (can_bytes[6] && can_bytes[7]) {
                    session->total_chunks = can_bytes[7];
                    uint16_t offset = 7 * (can_bytes[7] - 1);
                    session->write_offset = offset;
                    session->chunk_offset = offset + 7 * can_bytes[6];
                    session->state = 5;
                }
            }
            return 0;
        }
        else if (subcmd == 19) {
            /* Verify complete */
            if (session->state == 9 &&
                session->write_offset == can_bytes[6])
            {
                session->state = 1;     /* Back to idle */
                /* Invoke completion callback */
            }
            return 0;
        }
        else if (subcmd == 255) {
            /* Abort */
            session->state = 1;
            return 0;
        }
    }
    else if (msg_type == CAN_MSG_TYPE_FW_DATA) {    /* 0xEB */
        /* Data segment */
        if (session->src_addr != dst_addr || session->dst_addr != src_addr)
            return 0;

        if (session->state != 10)
            return 0;

        /* Calculate bytes to copy in this segment */
        uint16_t remaining = session->total_size - session->write_offset;
        if (remaining > 7)
            remaining = 7;

        /* Verify sequence number */
        if (session->chunk_offset + 1 != can_bytes[5]) {
            session->state = 1;
            can_msg_t nack;
            can_build_nack_msg(src_addr, dst_addr, &nack);
            return 0;
        }

        /* Copy data bytes */
        for (uint16_t i = 0; i < remaining; i++) {
            uint8_t byte = can_bytes[6 + i];
            ((uint8_t *)session->buffer_ptr)[session->write_offset] = byte;

            /* Running checksum (exclude last 2 bytes which are the checksum) */
            if (session->write_offset < session->total_size - 2)
                session->checksum += byte;

            session->write_offset++;
        }

        session->chunk_offset++;

        /* Check if all data received */
        if (session->write_offset >= session->total_size) {
            /* Verify checksum */
            session->checksum = ~session->checksum;

            uint16_t stored_cksum =
                ((uint8_t *)session->buffer_ptr)[session->total_size - 2] |
                (((uint8_t *)session->buffer_ptr)[session->total_size - 1] << 8);

            session->state = 1;

            if (stored_cksum != session->checksum) {
                /* Checksum mismatch - NACK */
                can_msg_t nack;
                can_build_nack_msg(src_addr, dst_addr, &nack);
                return 0;
            }

            /* Success - send verify response and invoke data callback */
            /* ... */
        } else {
            /* More segments expected - reset timeout */
            session->retry_counter = 0;
            session->retry_timeout = 750 / session->chunk_size;
        }
    }

    return 0;
}

/* ============================================================================
 * CAN Firmware Block Send (sub_800D784)
 *
 * Sends a block of firmware data as segmented CAN frames.
 * ============================================================================ */
int can_fw_send_block(fw_update_session_t *session, uint8_t src, uint8_t dst,
                       uint8_t cmd, uint8_t param1, uint8_t param2,
                       const void *data, uint32_t size, int timeout)
{
    if (!session || !data ||
        session->max_size < size + 5 || size > 0x6F9)
        return 0;

    session->state = 3;     /* Sending */
    session->src_addr = src;
    session->dst_addr = dst;

    uint8_t *buf = (uint8_t *)session->buffer_ptr;
    buf[0] = cmd;
    buf[1] = param1;
    buf[2] = param2;

    /* Calculate running checksum */
    int16_t cksum = param2 + param1 + cmd;
    const uint8_t *src_data = (const uint8_t *)data;

    for (uint32_t i = 0; i < size; i++) {
        cksum += src_data[i];
        buf[i + 3] = src_data[i];
    }

    buf[size + 3] = ~(uint8_t)cksum;
    buf[size + 4] = (uint16_t)(~cksum) >> 8;

    session->total_size = size + 5;
    session->retry_counter = 0;
    session->retry_timeout = (timeout - 1 + session->chunk_size) / session->chunk_size;

    return 1;
}
