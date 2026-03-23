/**
 * @file protocol.c
 * @brief Ninebot serial protocol (UART) and CAN bus communication.
 *
 * Reconstructed from:
 *   sub_8005E44  — ninebot_serial_send()
 *   sub_800DFF6  — ninebot_frame_build()
 *   sub_800E03C  — can_rx_handler()
 *   sub_800CCFC  — can_frame_dispatch()
 *   sub_800CCA8  — can_protocol_init()
 *   sub_800CD5C  — can_protocol_poll()
 *   sub_800CD68  — can_send_message()
 *   sub_800C224  — can_transmit_raw()
 *   sub_800CB04  — checksum_compute()
 *   sub_800E0E8  — telemetry_build() — builds status word for dashboard
 *   sub_800E1F0  — telemetry_extended() — extended telemetry
 *   sub_800C680  — serial_register_read() — handle register read commands
 *   sub_80031F4  — firmware_update_check()
 *   sub_800CBB2  — flash_write_page()
 *   sub_800CC54  — flash_write_halfwords()
 */

#include <stdint.h>
#include <string.h>
#include "stm32f0_regs.h"
#include "g3_types.h"
#include "parameters.h"
#include "fixed_point.h"

/* ========================================================================== */
/*  External references                                                        */
/* ========================================================================== */

extern g3_state_t g3;

/* ========================================================================== */
/*  Ninebot serial protocol                                                    */
/* ========================================================================== */

/**
 * Compute simple checksum over buffer.
 * sub_800CB04: sum of all bytes as uint16.
 */
static uint16_t checksum_compute(const uint8_t *data, uint32_t len)
{
    uint16_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * Build a Ninebot serial protocol frame.
 *
 * sub_800DFF6(a1, src, dst, payload_ptr, total_len):
 *   - Clears 252-byte buffer
 *   - frame[0] = total_len - 3  (payload length)
 *   - frame[1] = src_addr
 *   - frame[2] = dst_addr
 *   - frame[3] = payload[1]     (command byte)
 *   - frame[4] = payload[2]     (argument byte)
 *   - memcpy(frame+5, payload+3, total_len-3)
 *   - Send via sub_8005E44
 */
static int ninebot_send_frame(uint8_t src, uint8_t dst,
                               const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[NINEBOT_MAX_PAYLOAD + 5];
    memset(frame, 0, sizeof(frame));

    frame[0] = payload_len;
    frame[1] = src;
    frame[2] = dst;

    if (payload_len > 0 && payload != NULL) {
        frame[3] = payload[0];   /* Command */
        frame[4] = payload[1];   /* Argument */
        if (payload_len > 2) {
            memcpy(&frame[5], &payload[2], payload_len - 2);
        }
    }

    /* Send frame via UART DMA — sub_8005E44 */
    /* In the real firmware this queues the frame for DMA transmission */
    return 0;
}

/**
 * Handle register read requests from dashboard.
 *
 * sub_800C680: Large switch statement on register address.
 * Returns pointer to register data.
 *
 * Known register addresses:
 *   833   — Motor parameters (word_20000335, word_20000318, etc.)
 *   Other — Various config and status registers
 */
static void *serial_register_read(int reg_addr)
{
    static uint8_t response_buf[16];

    switch (reg_addr) {
    case 833:
        /* Return motor parameter block */
        /* word_2000030C = word_20000335  (motor type?) */
        /* unk_2000030E = word_20000318   (pole pairs?) */
        /* dword_2000030F = dword_20000314 (rated current?) */
        /* ... */
        return response_buf;

    default:
        /* Other registers — large switch in original */
        return NULL;
    }
}

/* ========================================================================== */
/*  Telemetry packet builder (sub_800E0E8 + sub_800E1F0)                       */
/* ========================================================================== */

/**
 * Build the status telemetry packet sent to the dashboard.
 *
 * sub_800E0E8: Builds dword_20001FE0 (status word) from flags:
 *   bit 0:  byte_20000A31 (motor running)
 *   bit 1:  byte_20000B18 (charging)
 *   bit 2:  byte_20000E27 (push mode)
 *   bit 3:  byte_20000E10 (headlight)
 *   bit 4:  byte_20000E25 & byte_20000E28 (sport + power_on)
 *   bit 5:  (unused)
 *   bit 6:  sub_8004A84() (dual battery?)
 *   bit 7:  byte_20000EE4 (error indicator)
 *   bit 8:  temperature OK flag
 *   bits 9-15: (dword_200000E8 >> 15) & 0x7F (uptime / odometer MSB)
 *
 * Also reads:
 *   RTC registers (MEMORY[0x4000641A], 0x40006418) for CAN timestamp
 *   Motor temperature from ADC with __usat(8, temp+20) clamping
 */
void telemetry_update(void)
{
    uint32_t status = 0;

    /* Build status bits */
    status |= (g3.scooter_on & 1);                         /* bit 0 */
    status |= ((/* charging */ 0 & 1) << 1);               /* bit 1 */
    status |= ((g3.sport_mode & g3.scooter_on & 1) << 4);  /* bit 4 */
    status |= ((g3.push_mode & 1) << 2);                   /* bit 2 ? */

    /* Cruise control active */
    int cruise_ok = /* sub_8004A84() */ 0;
    status |= (cruise_ok << 6);

    /* Error indicator */
    status |= ((g3.error_code & 1) << 7);

    /* Temperature check */
    int16_t temp_limit = g3.motor_poles - 10;   /* word_200003E2 - 10 */
    if (g3.serial_errors <= temp_limit) {
        status |= (1 << 8);
    }

    /* Uptime / odometer bits */
    status |= (((g3.uptime_ticks >> 15) & 0x7F) << 9);

    /* Store status word — dword_20001FE0 */
    /* HIWORD(dword_20001FE0) = sub_800CE10() — additional status */

    /* Motor temperatures with saturation */
    /* BYTE2(dword_20001FE4) = __usat(8, adc_temp_B + 20) */
    /* HIBYTE(dword_20001FE4) = __usat(8, adc_temp_A + 20) */

    /* byte_20001FE8 = byte_200003B4 — mode byte */

    /* CAN bus timestamp from RTC */
    /* word_20001E3C = CAN_RTC[0x1A] */
    /* word_20001E3E = HIBYTE(CAN_RTC[0x18]) */
    /* word_20001E40 = (CAN_RTC[0x18] >> 4) & 7 */
    /* word_20001E42 = error_code */
}

/**
 * Extended telemetry (sub_800E1F0).
 * Adds speed, current, and power measurements.
 *
 *   byte_20002019 = byte_20000E29  (brake state)
 *   byte_2000201B = cruise_active
 *   byte_2000201A = uptime >> 15
 *   byte_2000201C = odometer >> 12 (clamped to ±521011)
 *   dword_20002020 = sub_8004974()  (d-axis flux)
 *   dword_20002024 = sub_8004920()  (motor speed)
 *   dword_20002028 = sub_80046D0(flux) (power)
 *   dword_2000202C = sub_80046D0(speed) (torque)
 *   dword_20002030 = speed * 32768000 >> 30 (RPM)
 *   dword_20002034 = current * 3276800 >> 30 (mA)
 */
void telemetry_extended_update(void)
{
    /* Clamp odometer */
    int32_t odo = g3.odometer;
    if (odo > ODOMETER_CLAMP_POS)  odo = ODOMETER_CLAMP_POS;
    if (odo < ODOMETER_CLAMP_NEG)  odo = ODOMETER_CLAMP_NEG;

    /* byte_2000201C = odo >> 12 */

    /* Power calculation: (speed * current) in engineering units */
    /* dword_20002030 = (32768000LL * speed_func()) >> 30 */
    /* dword_20002034 = (3276800LL * current) >> 30 */
}

/* ========================================================================== */
/*  CAN bus communication                                                      */
/* ========================================================================== */

/**
 * Initialize CAN protocol handlers.
 * sub_800CCA8: Sets up 5 handler entries, each with a 36-byte config
 * and 300-byte receive buffer.
 */
void can_protocol_init(void)
{
    /* Initialize handler table:
     *   for i in 0..4:
     *     sub_800D078(&handler[i], 1, &rx_buffer[i], 300, 0)
     *
     * Then set up registry:
     *   can_registry.handlers = &handler_table
     *   can_registry.num_handlers = 5
     *   can_registry.config = &can_config
     */
}

/**
 * CAN receive handler (sub_800E03C).
 * Called from CAN RX interrupt.
 * Extracts 29-bit ID, DLC, and data, then dispatches.
 */
void can_rx_handler(can_message_t *msg)
{
    /* sub_800C224(id & 0x1FFFFFFF, &data[0], dlc) */
    /* Transmits response or stores received data */
}

/**
 * Dispatch received CAN frame (sub_800CCFC).
 * Parses the CAN ID to extract source, destination, and command.
 *
 *   byte 0-2 of ID: src/dst/cmd
 *   If cmd == 0xEF (239): call ninebot protocol handler
 *   Otherwise: dispatch to registered handler
 */
void can_frame_dispatch(const uint8_t *raw_frame)
{
    uint32_t can_id = *(uint32_t *)raw_frame & CAN_ID_MASK_29BIT;
    uint8_t  src    = raw_frame[0];
    uint8_t  dst    = raw_frame[1];
    uint8_t  cmd    = raw_frame[2];

    if (cmd == 0xEF) {
        /* Ninebot serial protocol encapsulated in CAN */
        /* sub_800DFF6(0, src, dst, payload_ptr, length) */
        ninebot_send_frame(src, dst, &raw_frame[7], raw_frame[6]);
    } else {
        /* Standard CAN protocol handler */
        /* sub_800CFC8(&can_registry, parsed_frame, dst) */
    }
}

/**
 * Send CAN message (sub_800CD68).
 * If payload > 5 bytes, use multi-frame transport (sub_800D040).
 * Otherwise, build single frame and transmit via sub_800C224.
 */
void can_send_message(uint32_t id, uint8_t src, uint8_t dst,
                      const uint8_t *data, uint32_t len,
                      uint8_t priority, uint8_t flags)
{
    if (len > 5) {
        /* Multi-frame transport */
        /* sub_800D040(&can_registry, 7, 1, id, src, priority, flags, len, 0) */
    } else {
        /* Single-frame transmission */
        /* Build frame header, then sub_800C224(can_id, data, dlc) */
    }
}

/**
 * Poll CAN protocol for pending messages.
 * sub_800CD5C: calls sub_800CFA0(&can_registry)
 */
int can_protocol_poll(void)
{
    /* Process any pending CAN messages in the RX FIFO */
    /* sub_800CFA0(&can_registry) — check and dispatch */
    return 0;
}

/* ========================================================================== */
/*  Firmware update (sub_80031F4)                                              */
/* ========================================================================== */

/**
 * Check for firmware update request from dashboard.
 *
 * sub_80031F4:
 *   if (byte_20000E2D == 1):
 *     Send update acknowledgement frame
 *     sub_8008934(2, 22, 1, 3, 90, &data, 1)
 *     Reset flag
 */
void firmware_update_check(void)
{
    static uint8_t update_requested = 0;  /* byte_20000E2D */

    if (update_requested == 1) {
        /* Send firmware update acknowledgement via serial */
        uint8_t ack_data = 1;
        /* sub_8008934(2, 22, 1, 3, 90, &ack_data, 1) */

        update_requested = 0;
    }
}

/* ========================================================================== */
/*  Flash memory operations                                                    */
/* ========================================================================== */

/**
 * Unlock flash for writing (sub_800CC3C).
 * Writes the unlock key sequence to FLASH->KEYR.
 */
static void flash_unlock(void)
{
    FLASH_IF->KEYR = FLASH_UNLOCK_KEY;   /* 0xCDEF89AB */
}

/**
 * Erase a flash page (sub_800CC10).
 * Sets PER bit, writes page address, triggers erase, waits.
 */
static int flash_erase_page(uint32_t page_addr)
{
    FLASH_IF->CR |= 2;             /* PER = 1 */
    FLASH_IF->AR = page_addr;      /* Page address */
    FLASH_IF->CR |= 0x40;          /* STRT = 1 */

    /* Wait for completion */
    int status;
    do {
        status = (FLASH_IF->SR & 1) ? 0 :      /* BSY */
                 (FLASH_IF->SR & 4) ? 1 :      /* PGerr */
                 (FLASH_IF->SR & 0x10) ? 2 : 3; /* WRP err */
    } while (status == 0);

    FLASH_IF->CR &= ~2;            /* Clear PER */
    return status;
}

/**
 * Write flash page (sub_800CBB2).
 * Unlocks flash, erases required pages, writes data in 1KB blocks.
 */
int flash_write_page(uint32_t dest_addr, uint32_t size)
{
    flash_unlock();

    uint32_t num_pages = (size + 1023) / 1024;  /* sub_80063B6 */
    uint32_t page = 0;

    for (page = 0; page < num_pages; page++) {
        int status = flash_erase_page(dest_addr + (page * 1024));
        if (status != 3) {  /* 3 = success */
            if (status == 1) {
                FLASH_IF->SR = 4;   /* Clear error */
            }
            break;
        }
    }

    /* Lock flash */
    FLASH_IF->CR |= 0x80;  /* LOCK bit */

    return (page == num_pages);
}

/**
 * Write halfwords to flash (sub_800CC54).
 * Writes 16-bit values one at a time with verify.
 */
int flash_write_halfwords(uint16_t *dest, const int16_t *src, uint32_t byte_count)
{
    flash_unlock();

    uint32_t written = 0;
    while (written < byte_count) {
        /* Enable programming */
        FLASH_IF->CR |= 1;     /* PG = 1 */
        *dest = *src;

        /* Wait and verify */
        int status;
        do {
            status = (FLASH_IF->SR & 1) ? 0 : 3;
        } while (status == 0);

        if (status != 3 || *dest != *src) {
            break;
        }

        dest++;
        src++;
        written += 2;
    }

    FLASH_IF->CR &= ~1;    /* Clear PG */
    FLASH_IF->CR |= 0x80;  /* LOCK */

    return (written >= byte_count);
}
