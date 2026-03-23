/**
 * @file authentication.c
 * @brief BLE Authentication / Lock Key Verification
 *
 * The G3 uses a challenge-response authentication mechanism:
 * 1. Scooter sends a random challenge (dword_2000397C)
 * 2. App sends back button presses (up/down/power/fn) as key sequence
 * 3. Scooter computes expected response via CRC32/hash (sub_8007834)
 * 4. If match -> unlock; if 3 failures -> lockout with timeout
 *
 * Reconstructed from:
 * sub_800192C = auth_process (main auth handler, called every 100ms)
 * sub_8001BD4 = auth_record_keypress
 * sub_8001C10 = auth_verify_sequence
 * sub_8007834 = auth_compute_hash (CRC32-like hash of key sequence)
 * sub_8007120 = auth_compute_key_length
 * sub_800A044 = state_event_notify (triggers state transitions)
 */

#include "vcu_types.h"
#include "hal_drivers.h"

/* ============================================================================
 * External References
 * ============================================================================ */
extern scooter_runtime_t    g_scooter;
extern serial_tx_buf_t      g_serial_tx_buf;
extern volatile uint8_t     g_btn_up;
extern volatile uint8_t     g_btn_down;
extern volatile uint8_t     g_btn_power;
extern volatile uint8_t     g_btn_fn;

extern int serial_enqueue_tx(serial_tx_buf_t *buf, uint8_t src, uint8_t dst,
                              uint8_t len, uint8_t cmd, uint8_t subcmd,
                              const void *payload);
extern int led_set_pattern(int pattern);
extern void led_clear_all(void);
extern int state_event_notify(int event, ...);

/* ============================================================================
 * Auth State
 * ============================================================================ */
static uint32_t auth_response_buf;      /* dword_200039BE - accumulated key presses */
static uint16_t auth_key_index;         /* word_200039C8 - current position in sequence */
static uint8_t  auth_key_length;        /* byte_200039BC - expected key length */
static uint32_t auth_challenge;         /* dword_2000397C - random challenge value */
static uint32_t auth_expected;          /* dword_200031EE - expected hash result */
static uint32_t auth_calc_buf;          /* dword_20000EFC - running computation */
static uint16_t auth_counter;           /* word_20000EF8 - timeout counter */
static uint16_t auth_delay;             /* word_20000EF4 - delay after failure */
static uint16_t auth_cycle;             /* word_20000EF6 - display cycle counter */
static uint8_t  auth_step;             /* byte_20000EED - sub-state */
static uint8_t  auth_seq;              /* byte_20000EEE - display sequence index */
static uint8_t  auth_retry_count;      /* byte_200039BD - failure count */
static uint8_t  auth_lockout;          /* byte_200039B9 - lockout state */

static uint16_t pw_display_error;      /* word_200033D0 - error display code */
static uint16_t pw_display_ok;         /* word_200033D4 - success display code */

/* Lookup table for retry delay patterns */
static const uint16_t auth_retry_delays[] = { 0, 200, 200 };   /* word_20000F00 */

/* ============================================================================
 * CRC32-like Hash (sub_8007834)
 *
 * Computes a hash of the key sequence buffer.
 * Used to verify the button-press authentication sequence.
 * ============================================================================ */
uint32_t auth_compute_hash(uint32_t *key_buf)
{
    /* This is a simplified reconstruction. The actual algorithm performs
     * a CRC32-like computation over the key buffer bytes. */
    uint32_t hash = 0xFFFFFFFF;
    const uint8_t *bytes = (const uint8_t *)key_buf;

    /* The actual implementation iterates over the key buffer entries */
    for (int i = 0; i < 8; i++) {
        uint8_t b = bytes[i];
        hash ^= b;
        for (int j = 0; j < 8; j++) {
            if (hash & 1)
                hash = (hash >> 1) ^ 0xEDB88320;
            else
                hash >>= 1;
        }
    }

    return hash;
}

/* ============================================================================
 * Compute Expected Key Length (sub_8007120)
 *
 * Derives the expected number of key presses from the challenge value.
 * ============================================================================ */
uint8_t auth_compute_key_length(uint32_t challenge)
{
    /* The challenge is bit-shuffled to determine key length.
     * Typical length is 4-8 presses. */
    uint32_t shuffled = (challenge & 0xFFFF) | ((challenge >> 16) << 16);
    uint8_t len = (uint8_t)((shuffled % 5) + 4);  /* 4 to 8 */
    return len;
}

/* ============================================================================
 * Record Key Press (sub_8001BD4)
 *
 * Adds a button press to the authentication sequence buffer.
 * Button values: 1=up, 2=down, 3=power, 4=fn (ORed with 0x08 flag)
 * ============================================================================ */
void auth_record_keypress(int button_id)
{
    int value = button_id | 0x08;

    /* Clear previous entry's marker if not first press */
    if (auth_key_index) {
        auth_calc_buf &= ~(0x08 << (4 * auth_key_index - 4));
    }

    /* Store this keypress in the running buffer */
    auth_calc_buf |= (uint32_t)value << (4 * auth_key_index);
}

/* ============================================================================
 * Authentication Process (sub_800192C)
 *
 * Main auth handler, called every 100ms from the main task.
 * Manages the complete authentication flow:
 * - Timeout detection
 * - Button press collection
 * - Hash verification
 * - Lockout on repeated failures
 * ============================================================================ */
int auth_process(int a1, int a2, int a3, int a4)
{
    uint8_t tmp_buf[44] = {0};

    /* Auth not active? Reset and return */
    if (!g_scooter.auth_state) {
        auth_step = 0;
        auth_counter = 0;
        g_scooter.status_word2 &= ~0x40;
        return 0;
    }

    /* Timeout counter - max 3000 (0xBB8) cycles = 5 minutes */
    auth_counter++;
    if (auth_counter >= 3000) {
        /* Authentication timed out */
        led_clear_all();
        led_set_pattern(16);    /* Error pattern */
        g_scooter.status_word2 &= ~0x40;
        g_scooter.auth_state = 0;

        /* Notify: auth failed (code 2) */
        tmp_buf[0] = 2;
        serial_enqueue_tx(&g_serial_tx_buf, 22, 62, 1, 3, 115, tmp_buf);
        state_event_notify(EVENT_AUTH_COMPLETE, 0, 0, 0);
        return 0;
    }

    /* If scooter is in READY state and timeout continues, also abort */
    if (g_scooter.state == SCOOTER_STATE_READY && auth_counter >= 3000) {
        return 0;
    }

    /* ===== Phase 1: Challenge generation ===== */
    if (!auth_step && g_scooter.auth_state) {
        if (!auth_delay) {
            state_event_notify(0, 0, 0, 0);
        }

        auth_delay++;
        if (auth_delay > 10) {
            auth_step = 1;
            auth_seq = 0;
            auth_delay = 0;

            /* Compute expected key length from challenge */
            auth_key_length = auth_compute_key_length(
                (g_scooter.auth_challenge & 0xFFFF) |
                ((g_scooter.auth_challenge >> 16) << 16)
            );

            state_event_notify(EVENT_AUTH_COMPLETE,
                               (auth_key_length << 8) | 0x11, 0, 0);
        }
        return 0;
    }

    /* ===== Phase 2: Collecting button presses ===== */

    /* Check each button for press events (state == 4 means "just pressed") */
    int pressed_btn = 0;

    if (g_btn_up == 4) {
        g_btn_up = 0;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 1;
        pressed_btn = 1;
    } else if (g_btn_down == 4) {
        g_btn_down = 0;
        pressed_btn = 2;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 2;
    } else if (g_btn_power == 4) {
        g_btn_power = 0;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 3;
        pressed_btn = 3;
    } else if (g_btn_fn == 4) {
        g_btn_fn = 0;
        pressed_btn = 4;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 4;
    }

    if (pressed_btn) {
        auth_record_keypress(pressed_btn);
        ((uint8_t *)&auth_response_buf)[++auth_key_index] = 8;  /* separator */
    }

    /* Display progress feedback on LEDs */
    if (auth_seq < 15) {
        if (auth_seq == 0) {
            /* Show full challenge pattern */
            auth_compute_hash(&auth_response_buf);
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211,
                               &auth_calc_buf);
        } else if (auth_seq == 5) {
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 100,
                               &pw_display_ok);
        } else if (auth_seq == 10) {
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                               &pw_display_error);
        }

        auth_seq++;
        if (auth_seq >= 15)
            auth_seq = 0;
    }

    /* ===== Phase 3: Verify when all keys collected ===== */
    if (auth_key_index >= auth_key_length) {
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 0;

        if (auth_compute_hash(&auth_response_buf) == auth_expected) {
            /* === SUCCESS === */
            led_set_pattern(18);    /* Success pattern */
            tmp_buf[0] = 1;
            serial_enqueue_tx(&g_serial_tx_buf, 22, 62, 1, 3, 115, tmp_buf);

            /* Store new expected hash */
            auth_expected = auth_challenge;

            /* Mark as authenticated */
            g_scooter.auth_state = 0;
            led_clear_all();
            state_event_notify(EVENT_AUTH_COMPLETE, 0, 0, 0);

            return 1;
        } else {
            /* === FAILURE === */
            led_set_pattern(16);    /* Error pattern */
            tmp_buf[0] = 2;
            serial_enqueue_tx(&g_serial_tx_buf, 22, 62, 1, 3, 115, tmp_buf);
            g_scooter.status_word2 &= ~0x40;

            auth_retry_count++;
            if (auth_retry_count >= 3) {
                /* Lockout! */
                g_scooter.auth_locked = 1;
                pw_display_error = 12;
                auth_delay = 300;   /* 30 second lockout */
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                                   &pw_display_error);
            } else {
                /* Retry with visual feedback */
                pw_display_error = auth_retry_delays[auth_retry_count] | 4;
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                                   &pw_display_error);
                auth_compute_hash(&auth_response_buf);
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211,
                                   &auth_calc_buf);

                if (auth_lockout == 1) {
                    auth_delay = 20;
                } else {
                    auth_delay = 200;
                }
            }

            led_clear_all();
            state_event_notify(EVENT_AUTH_COMPLETE, 0, 0, 0);
            g_scooter.auth_state = 0;
        }
    }

    return 0;
}

/* ============================================================================
 * Auth Verify Sequence (sub_8001C10)
 *
 * Called during ride authentication to re-verify the stored key sequence.
 * Used after power cycle to check if the saved key still matches.
 * ============================================================================ */
int auth_verify_sequence(void)
{
    led_set_pattern(0);     /* Clear LED pattern */

    /* Same button collection logic as auth_process Phase 2 */
    int pressed_btn = 0;

    if (g_btn_up == 4) {
        g_btn_up = 0;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 1;
        pressed_btn = 1;
    } else if (g_btn_down == 4) {
        g_btn_down = 0;
        pressed_btn = 2;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 2;
    } else if (g_btn_power == 4) {
        g_btn_power = 0;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 3;
        pressed_btn = 3;
    } else if (g_btn_fn == 4) {
        g_btn_fn = 0;
        pressed_btn = 4;
        ((uint8_t *)&auth_response_buf)[auth_key_index] = 4;
    }

    if (pressed_btn) {
        auth_record_keypress(pressed_btn);
        ((uint8_t *)&auth_response_buf)[++auth_key_index] = 8;
    }

    if (auth_key_index < auth_key_length) {
        /* Still collecting - handle delay/retry display */
        if (auth_delay) {
            auth_delay--;
            if (auth_retry_count >= 3) {
                if (!auth_delay)
                    g_scooter.lock_flag = 1;    /* byte_20003987 */
                return 0;
            }
            if (!auth_delay) {
                if (auth_lockout == 1) {
                    auth_lockout = 2;
                }
                led_clear_all();
            }
        }

        /* Display cycling feedback */
        if (auth_cycle >= 15 || auth_delay) {
            auth_cycle = 0;
        } else {
            if (auth_cycle == 0) {
                auth_compute_hash(&auth_response_buf);
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211,
                                   &auth_calc_buf);
            } else if (auth_cycle == 5) {
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 100,
                                   &pw_display_ok);
            } else if (auth_cycle == 10) {
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                                   &pw_display_error);
            }
            auth_cycle++;
        }
        return 0;
    }

    /* All keys entered - check against stored expected value */
    if (auth_delay)
        goto handle_delay;

    ((uint8_t *)&auth_response_buf)[auth_key_index] = 0;

    if (auth_compute_hash(&auth_response_buf) != auth_expected) {
        /* Mismatch */
        led_set_pattern(16);
        auth_key_index = 0;

        auth_retry_count++;
        if (auth_retry_count >= 3) {
            auth_lockout = 1;
            pw_display_error = 12;
            auth_delay = 300;
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                               &pw_display_error);
        } else {
            pw_display_error = auth_retry_delays[auth_retry_count] | 4;
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                               &pw_display_error);
            auth_compute_hash(&auth_response_buf);
            serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211,
                               &auth_calc_buf);

            if (auth_lockout == 1) {
                auth_delay = 20;
                state_event_notify(0, 20, 0, 0);
            } else {
                auth_delay = 200;
            }
        }

handle_delay:
        auth_delay--;
        if (auth_retry_count >= 3) {
            if (!auth_delay)
                g_scooter.lock_flag = 1;
            return 0;
        }

        if (!auth_delay) {
            if (auth_lockout == 1) {
                auth_lockout = 2;
            }
            led_clear_all();
        }

        /* Display cycling */
        if (auth_cycle >= 15 || auth_delay) {
            auth_cycle = 0;
        } else {
            if (auth_cycle == 0) {
                auth_compute_hash(&auth_response_buf);
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211,
                                   &auth_calc_buf);
            } else if (auth_cycle == 5) {
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 100,
                                   &pw_display_ok);
            } else if (auth_cycle == 10) {
                serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 2, 3, 98,
                                   &pw_display_error);
            }
            auth_cycle++;
        }
        return 0;
    }

    /* Success! */
    serial_enqueue_tx(&g_serial_tx_buf, 22, 35, 4, 3, 211, &auth_calc_buf);

    if (g_scooter.status_word3 & 0x4000)
        led_set_pattern(73);    /* Special success pattern */
    else
        led_set_pattern(31);    /* Standard success */

    pw_display_error = 8;
    return 1;
}
