/**
 * @file config_manager.c
 * @brief Configuration Storage and Device Identity
 *
 * The G3 VCU stores configuration in internal flash using a dual-bank scheme:
 *   Bank A: 0x0800E400 (134344704)
 *   Bank B: 0x0800E800 (134345728)
 *   Version counters at: 0x0800E7FE (Bank A) / 0x0800EBFE (Bank B)
 *
 * On boot, the bank with the higher version counter is loaded.
 * If both are corrupt, a recovery attempt is made from the other bank.
 * A CRC16 at the end of each 512-byte block verifies integrity.
 *
 * Device identity is based on the STM32 Unique ID at 0x1FFFF7E8.
 *
 * Reconstructed from:
 * sub_8006074 = config_load_from_flash
 * sub_80061D4 = config_firmware_info_read
 * sub_8005E04 = config_compute_crc (CRC16 of config data)
 * sub_800A770 = config_init (I2C + config load)
 * sub_8002B98 = uid_verify_integrity
 * sub_80058B0 = flash_sector_prepare
 * sub_8005774 = flash_copy_sector
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * External References
 * ============================================================================ */
extern scooter_runtime_t    g_scooter;
extern int16_t              g_config_buf[256];
extern uint32_t             g_flash_sectors[];

/* ============================================================================
 * Flash Configuration Layout
 * ============================================================================ */
#define CONFIG_SIZE         512
#define CONFIG_BANK_A_ADDR  0x0800E400
#define CONFIG_BANK_B_ADDR  0x0800E800
#define CONFIG_VER_A_ADDR   0x0800E7FE
#define CONFIG_VER_B_ADDR   0x0800EBFE

/* ============================================================================
 * CRC16 Computation (sub_8005E04)
 *
 * Standard CRC16 over the configuration data block.
 * ============================================================================ */
uint16_t config_compute_crc(const int16_t *data, uint32_t word_count)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < word_count; i++) {
        crc ^= (uint16_t)data[i];
        for (int bit = 0; bit < 16; bit++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

/* ============================================================================
 * Load Configuration from Flash (sub_8006074)
 *
 * Implements dual-bank load with integrity checking.
 * Returns the active bank's version counter on success.
 * ============================================================================ */
int config_load_from_flash(void)
{
    uint16_t ver_a, ver_b;
    int16_t config_word_a, config_word_b;
    uint32_t crc;
    int err_a, err_b;

    /* Read version counters from each bank's footer */
    flash_read_to_ram(CONFIG_VER_A_ADDR, &config_word_a, 2);
    ver_a = (uint16_t)config_word_a;

    flash_read_to_ram(CONFIG_VER_B_ADDR, &config_word_b, 2);
    ver_b = (uint16_t)config_word_b;

    /* Determine which bank is newer and try to load it */
    if (ver_a <= ver_b) {
        /* Bank B is newer or equal - try B first */
        flash_read_to_ram(CONFIG_BANK_B_ADDR, g_config_buf, CONFIG_SIZE);

        if (config_compute_crc(g_config_buf, 256) == (uint16_t)config_word_b) {
            /* Bank B valid */
            /* word_20000824 = ver_b */
            return ver_b;
        }

        /* Bank B corrupt - try Bank A */
        err_b = 1;     /* sub_80070C4() - error counter */

        flash_read_to_ram(CONFIG_BANK_A_ADDR, g_config_buf, CONFIG_SIZE);

        if (config_compute_crc(g_config_buf, 256) == (uint16_t)config_word_a) {
            /* Bank A valid - use it */
            return ver_a;
        }

        err_a = 1;

        /* Both banks corrupt */
        if (err_a >= 2) {
            if (err_b) {
                /* Both completely invalid */
                g_config_buf[0] = -1;
                return -1;
            }
            /* Fallback to B */
            flash_read_to_ram(CONFIG_BANK_B_ADDR, g_config_buf, CONFIG_SIZE);
        } else {
            /* Use A */
            flash_read_to_ram(CONFIG_BANK_A_ADDR, g_config_buf, CONFIG_SIZE);
        }
    } else {
        /* Bank A is newer - try A first */
        flash_read_to_ram(CONFIG_BANK_A_ADDR, g_config_buf, CONFIG_SIZE);

        if (config_compute_crc(g_config_buf, 256) == (uint16_t)config_word_a) {
            return ver_a;
        }

        err_a = 1;

        flash_read_to_ram(CONFIG_BANK_B_ADDR, g_config_buf, CONFIG_SIZE);

        if (config_compute_crc(g_config_buf, 256) == (uint16_t)config_word_b) {
            return ver_b;
        }

        err_b = 1;

        /* Both corrupt - same recovery logic */
        if (err_a >= 2) {
            if (err_b) {
                g_config_buf[0] = -1;
                return -1;
            }
        }
    }

    /* Parse loaded config data */
    /* sub_8008FFC - extract structured fields from raw config buffer */
    return 0;
}

/* ============================================================================
 * Save Configuration to Flash (sub_8008FFC partial)
 *
 * Writes the current config to the inactive bank, then updates the version.
 * ============================================================================ */
int config_save_to_flash(void)
{
    /* Determine which bank to write to (the older one) */
    uint16_t ver_a, ver_b;
    flash_read_to_ram(CONFIG_VER_A_ADDR, &ver_a, 2);
    flash_read_to_ram(CONFIG_VER_B_ADDR, &ver_b, 2);

    uint32_t target_bank;
    uint32_t target_ver_addr;
    uint16_t new_version;

    if (ver_a <= ver_b) {
        target_bank = CONFIG_BANK_A_ADDR;
        target_ver_addr = CONFIG_VER_A_ADDR;
        new_version = ver_b + 1;
    } else {
        target_bank = CONFIG_BANK_B_ADDR;
        target_ver_addr = CONFIG_VER_B_ADDR;
        new_version = ver_a + 1;
    }

    /* Erase target bank */
    if (!flash_erase_range(target_bank, CONFIG_SIZE))
        return 0;

    /* Write config data */
    if (!flash_program_verified((uint16_t *)target_bank,
                                 (uint16_t *)g_config_buf, CONFIG_SIZE))
        return 0;

    /* Write version counter */
    uint16_t crc = config_compute_crc(g_config_buf, 256);
    flash_unlock();
    flash_write_halfword((uint16_t *)target_ver_addr, crc);
    flash_lock();

    return 1;
}

/* ============================================================================
 * Read Firmware Info (sub_80061D4)
 *
 * Reads firmware version / metadata from a specific flash sector.
 * ============================================================================ */
void config_firmware_info_read(int sector_index)
{
    uint32_t sector_addr = g_flash_sectors[sector_index];
    uint32_t info[3] = {0, 0, 0};

    flash_read_to_ram(sector_addr, info, 12);

    /* Byte-swap from little-endian flash to internal format */
    uint32_t fw_start = ((info[0] >> 16) & 0xFF) << 16 |
                         ((info[0] >> 24) & 0xFF) << 24 |
                         (info[0] & 0xFF) |
                         ((info[0] >> 8) & 0xFF) << 8;

    uint32_t fw_end   = (info[1] & 0xFF) |
                         ((info[1] >> 8) & 0xFF) << 8 |
                         ((info[1] >> 16) & 0xFF) << 16 |
                         ((info[1] >> 24) & 0xFF) << 24;

    uint32_t sector_size = g_flash_sectors[sector_index + 1] -
                           g_flash_sectors[sector_index];

    /* Validate ranges */
    if (fw_start > sector_size)
        fw_start = 0;
    if (fw_end > sector_size)
        fw_end = sector_size;

    /* Store for firmware update use */
    /* dword_20002F6C = fw_start + sector_addr */
    /* dword_20002F70 = fw_end + sector_addr */
    /* dword_20002F44 = fw_start + sector_addr + 512 */
}

/* ============================================================================
 * UID Integrity Check (sub_8002B98)
 *
 * Verifies the stored UID hasn't been tampered with by checking
 * both a sum and product checksum.
 * ============================================================================ */
bool uid_verify_integrity(const uint32_t *stored_check)
{
    return (stored_check[0] == ~(g_scooter.uid[2] + g_scooter.uid[1] + g_scooter.uid[0])) &&
           (stored_check[1] == ~(g_scooter.uid[2] * g_scooter.uid[1] * g_scooter.uid[0]));
}

/* ============================================================================
 * Device Identity Initialization (sub_800A770 partial)
 *
 * Reads STM32 Unique ID and stores it. Also computes the integrity check.
 * ============================================================================ */
int config_init_device_id(void)
{
    /* Read 96-bit unique device ID from factory-programmed area */
    g_scooter.uid[0] = *(volatile uint32_t *)(UID_BASE + 0);
    g_scooter.uid[1] = *(volatile uint32_t *)(UID_BASE + 4);
    g_scooter.uid[2] = *(volatile uint32_t *)(UID_BASE + 8);

    /* Also store copies for integrity checking */
    /* dword_20003A64..6C = same as uid[0..2] */

    /* Compute combined UID for serial number reporting */
    /* dword_2000082C = uid[0] + uid[1] + uid[2] */

    return 0;
}

/* ============================================================================
 * Flash Sector Copy (sub_8005774)
 *
 * Copies a firmware image from one flash sector to another.
 * Used during OTA updates.
 * ============================================================================ */
int flash_copy_sector(int src_index, int dst_index, uint32_t size)
{
    uint32_t src_addr = g_flash_sectors[src_index];
    uint32_t dst_addr = g_flash_sectors[dst_index];
    uint8_t page_buf[256];

    uint16_t pages = (uint16_t)(size >> 8);

    /* Prepare destination */
    /* sub_80058B0 - erase/prepare destination sector */

    for (uint16_t i = 0; i <= pages; i++) {
        /* Kick watchdog periodically */
        if (!(i & 0x1F))
            iwdg_reload();

        /* Read source page */
        flash_read_to_ram(src_addr + ((uint32_t)i << 8), page_buf, 256);

        /* Write to destination */
        flash_write_from_ram(page_buf, dst_addr + ((uint32_t)i << 8), 256);
    }

    return 0;
}

/* ============================================================================
 * Hex Character Validator (sub_8001E68)
 *
 * Returns whether a character is a valid hex digit (0-9, A-F, a-f).
 * Used for parsing serial number / MAC address strings.
 * ============================================================================ */
bool is_hex_char(unsigned char c)
{
    return !((c - '0' > 9) && (c - 'A' > 5) && (c - 'a' > 5));
}
