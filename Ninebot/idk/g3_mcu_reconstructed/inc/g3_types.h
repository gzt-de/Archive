/**
 * @file g3_types.h
 * @brief Type definitions, global state structures, and enumerations
 *        for the Ninebot Max G3 MCU firmware.
 */

#ifndef G3_TYPES_H
#define G3_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "fixed_point.h"

/* ========================================================================== */
/*  Ride mode enumeration                                                      */
/* ========================================================================== */

typedef enum {
    RIDE_MODE_IDLE      = 0,    /* Motor off, standby */
    RIDE_MODE_ECO       = 1,    /* Low power / eco mode */
    RIDE_MODE_NORMAL    = 2,    /* Standard ride mode */
    RIDE_MODE_SPORT     = 3,    /* High performance */
    RIDE_MODE_LOCK      = 4,    /* Locked / immobilized */
    RIDE_MODE_PUSH      = 5,    /* Push assist / walk mode */
} ride_mode_t;

/* byte_20002018 — active ride substate for the control loop */
typedef enum {
    CTRL_STATE_IDLE     = 0,
    CTRL_STATE_ECO      = 1,
    CTRL_STATE_RIDE     = 2,
    CTRL_STATE_SPORT    = 3,
    CTRL_STATE_LOCKED   = 4,
    CTRL_STATE_PUSH     = 5,
} ctrl_state_t;

/* ========================================================================== */
/*  Hall sensor sector mapping                                                 */
/* ========================================================================== */

typedef enum {
    HALL_SECTOR_AC = 17,    /* 0x11 — Hall state: A+C */
    HALL_SECTOR_BC = 34,    /* 0x22 — Hall state: B+C */
    HALL_SECTOR_AB = 51,    /* 0x33 — Hall state: A+B */
} hall_sector_t;

/* ========================================================================== */
/*  PID / 3rd-order controller (maps to sub_8001DFC / sub_8001E3E)            */
/* ========================================================================== */

typedef struct {
    q15_t   Kp;             /* Proportional gain (offset 0)  */
    q15_t   Ki;             /* Integral gain     (offset 4)  */
    q15_t   Kd;             /* Derivative gain   (offset 8)  */
    q15_t   Ka;             /* Anti-windup gain  (offset 12) */
    q15_t   input;          /* Current input     (offset 16) */
    q15_t   output;         /* Filtered output   (offset 20) */
    q15_t   state_p;        /* P state           (offset 24) */
    q15_t   state_i;        /* I accumulator     (offset 28) */
    q15_t   state_d;        /* D state           (offset 32) */
    q15_t   state_a;        /* Anti-windup state (offset 36) */
    q15_t   prev_err;       /* Previous error    (offset 40) */
    uint8_t enabled;        /* Controller active (offset 44) */
    uint8_t pad[3];
} pid3_controller_t;

/* ========================================================================== */
/*  2nd-order low-pass filter (maps to sub_800723E / sub_800727A)             */
/* ========================================================================== */

typedef struct {
    q15_t coeff_a;          /* Filter coefficient a */
    q15_t coeff_b;          /* Filter coefficient b */
    q15_t *state_ptr;       /* Pointer to state variable */
    q15_t state[2];         /* Internal state */
    uint8_t enabled;
    uint8_t pad[3];
} lowpass2_filter_t;

/* ========================================================================== */
/*  Rate limiter / ramp (maps to sub_8002966)                                  */
/* ========================================================================== */

static inline q15_t rate_limit(q15_t target, q15_t *current, q15_t max_step)
{
    if (max_step < 0) max_step = -max_step;
    q15_t diff = *current - target;
    if (diff > max_step) {
        *current -= max_step;
    } else if (-diff > max_step) {
        *current += max_step;
    } else {
        *current = target;
    }
    return *current;
}

/* ========================================================================== */
/*  Debounce / edge-detect timer (maps to sub_80099D0 etc.)                    */
/* ========================================================================== */

typedef struct {
    uint16_t count;         /* Current count */
    uint16_t threshold;     /* Trigger threshold */
    uint8_t  input;         /* Current input state */
    uint8_t  output;        /* Debounced output */
    uint8_t  count_mode;    /* 1 = count up, 0 = count down */
    uint8_t  reset_mode;    /* Reset behavior */
    uint16_t prescaler;     /* Tick prescaler */
    uint16_t reserved;
} debounce_timer_t;

/* ========================================================================== */
/*  Motor FOC state (large struct at dword_200000F8 pointer)                   */
/* ========================================================================== */

typedef struct {
    /* ---- Configuration (offsets 0..39) ---- */
    q15_t   speed_ref;              /* +0:   Speed reference */
    q15_t   speed_limit_fwd;        /* +4:   Forward speed limit */
    q15_t   speed_limit_rev;        /* +8:   Reverse speed limit */
    q15_t   current_limit;          /* +12:  Current limit */
    q15_t   pid_params[4];          /* +16:  PID tuning block */
    q15_t   ramp_rate;              /* +32:  Speed ramp rate */
    uint8_t control_flags[4];       /* +36:  Control flag bytes */
    q15_t   foc_ref_id;             /* +40:  FOC d-axis reference */

    uint8_t pad1[220];              /* ... gap ... */

    /* ---- Measured / computed (offsets ~260+) ---- */
    q15_t   motor_temperature;      /* +260: Motor temp (float repr) */
    q15_t   flux_d;                 /* +264: d-axis flux estimate */
    q15_t   flux_q;                 /* +268: q-axis flux estimate */

    uint8_t pad2[220];

    /* ---- FOC PI outputs (offsets ~484+) ---- */
    lowpass2_filter_t current_filter; /* +484: Current loop filter */
    q15_t   v_alpha;                /* +532: Alpha voltage */
    q15_t   v_beta;                 /* +536: Beta voltage */
    q15_t   id_error;               /* +540: d-axis error */
    q15_t   iq_error;               /* +544: q-axis error */
    q15_t   torque_ref;             /* +548: Torque reference */

    uint8_t pad3[16];

    q15_t   iq_ref_limited;         /* +564: Limited q-current ref */
    uint8_t mode_select;            /* +569: Drive mode selector */

    uint8_t pad4[94];

    uint8_t brake_enable;           /* +668: Brake active flag */
    uint8_t field_weakening;        /* +669: Field weakening active */
    uint8_t speed_mode;             /* +670: Speed/torque mode flag */

    uint8_t pad5[17];

    q15_t   i_alpha;                /* +688: Alpha current */
    q15_t   i_beta;                 /* +692: Beta current */
    q15_t   id_measured;            /* +696: Measured d-current */
    q15_t   iq_measured;            /* +700: Measured q-current */

    uint8_t pad6[8];

    q15_t   vd_output;             /* +712: d-axis voltage output */
    q15_t   vq_output;             /* +716: q-axis voltage output */

    uint8_t pad7[20];

    uint8_t regeneration_on;        /* +740: Regen braking active */
} foc_state_t;

/* ========================================================================== */
/*  Motor control parameter block                                              */
/* ========================================================================== */

typedef struct {
    q15_t   max_speed_rpm;          /* Max motor speed (Q15 RPM) */
    q15_t   max_speed_fwd;          /* Forward speed limit */
    q15_t   max_speed_rev;          /* Reverse speed limit */
    q15_t   brake_current;          /* Regen braking current */
    q15_t   max_accel;              /* Max acceleration */
    q15_t   max_decel;              /* Max deceleration */
    q15_t   current_limit_q;        /* q-axis current limit */
    q15_t   ramp_rate;              /* Speed ramp rate */
    q15_t   field_weaken_start;     /* FW start speed */
    uint8_t mode_id;                /* Mode identifier */
    uint8_t regen_enable;           /* Regen enabled */
    uint8_t cruise_enable;          /* Cruise ctrl enabled */
    uint8_t kers_enable;            /* KERS braking enabled */
    q15_t   iq_ref_limit;           /* q-current saturation */
    q15_t   temp_limit_hi;          /* High temperature (float) */
    q15_t   temp_limit_lo;          /* Low temperature (float) */
    uint8_t push_assist;            /* Push assist active */
} motor_params_t;

/* ========================================================================== */
/*  Ninebot serial protocol frame                                              */
/* ========================================================================== */

#define NINEBOT_MAX_PAYLOAD  252

typedef struct {
    uint8_t  length;        /* Payload length (including src/dst/cmd) */
    uint8_t  src_addr;      /* Source address */
    uint8_t  dst_addr;      /* Destination address */
    uint8_t  cmd;           /* Command byte */
    uint8_t  arg;           /* Argument byte */
    uint8_t  payload[NINEBOT_MAX_PAYLOAD];
    uint16_t checksum;      /* XOR/sum checksum */
} ninebot_frame_t;

/* ========================================================================== */
/*  CAN bus message structure                                                  */
/* ========================================================================== */

typedef struct {
    uint32_t id;            /* 29-bit extended ID (or 11-bit standard) */
    uint8_t  dlc;           /* Data length code (0-8) */
    uint8_t  data[8];       /* Payload */
} can_message_t;

/* ========================================================================== */
/*  CAN protocol handler registry                                              */
/* ========================================================================== */

#define CAN_HANDLER_MAX     5

typedef struct {
    void    *handlers;          /* Pointer to handler table */
    uint16_t num_handlers;
    uint16_t reserved;
    void    *config;            /* Pointer to CAN config */
} can_registry_t;

/* ========================================================================== */
/*  ADC sample buffer (maps to unk_20000FA8 area)                              */
/* ========================================================================== */

#define ADC_CHANNELS        5
#define ADC_HISTORY_DEPTH   39

typedef struct {
    uint16_t samples[ADC_HISTORY_DEPTH][ADC_CHANNELS];
    uint16_t filtered[ADC_CHANNELS];    /* Median-filtered results */
} adc_buffer_t;

/* ========================================================================== */
/*  Throttle state                                                             */
/* ========================================================================== */

typedef struct {
    q15_t   raw_voltage;        /* Raw ADC reading (Q15) */
    q15_t   filtered;           /* Filtered value */
    q15_t   output;             /* Mapped throttle output 0..Q15_ONE */
    q15_t   deadband_low;       /* Low deadband threshold */
    q15_t   deadband_high;      /* High deadband threshold */
    q15_t   max_output;         /* Max allowed output */
    uint8_t error_state;        /* Throttle error (0=OK, 0xFD/-3, 0xFE/-2) */
    uint8_t enabled;            /* Throttle active */
} throttle_state_t;

/* ========================================================================== */
/*  Speed observer / estimator                                                 */
/* ========================================================================== */

typedef struct {
    q15_t   bandwidth;              /* Observer bandwidth */
    q15_t   gain;                   /* Observer gain */
    q15_t   speed_estimate;         /* Estimated speed (Q15 RPM) */
    q15_t   angle_estimate;         /* Estimated elec. angle */
    q15_t   flux_estimate;          /* Flux linkage estimate */
    q15_t   max_speed;              /* Maximum observable speed */
    q15_t   min_speed;              /* Minimum speed threshold */
    q15_t   integrator;             /* Internal integrator */
    q15_t   max_ramp;               /* Max ramp (slew rate) */
    uint8_t converged;              /* Observer has converged */
    uint8_t phase_valid;            /* Phase angle is valid */
    uint8_t pad[2];
} speed_observer_t;

/* ========================================================================== */
/*  Global firmware state (all globals collected)                               */
/* ========================================================================== */

typedef struct {
    /* ---- Ride state ---- */
    uint8_t         ride_mode;          /* byte_2000064E */
    uint8_t         ctrl_state;         /* byte_20002018 */
    uint8_t         scooter_locked;     /* byte_20000E11 */
    uint8_t         scooter_on;         /* byte_20000E28 */
    uint8_t         eco_mode;           /* byte_20000E26 */
    uint8_t         sport_mode;         /* byte_20000E25 */
    uint8_t         push_mode;          /* byte_20000E27 */
    uint8_t         cruise_active;      /* byte_20000E0F */
    uint8_t         cruise_ready;       /* byte_20000E13 */
    uint8_t         brake_lever;        /* byte_20000E28 */
    uint8_t         tail_light_on;      /* byte_20000E2B */
    uint8_t         headlight_on;       /* byte_20000E10 */
    uint8_t         kers_active;        /* byte_20000E2A */
    uint8_t         ebs_active;         /* byte_20000108 */

    /* ---- Motor ---- */
    q15_t           motor_speed;        /* dword_20000F24 — current speed (Q15) */
    q15_t           target_speed;       /* dword_20000F2C — target speed */
    q15_t           motor_current;      /* dword_20000774 — phase current */
    q15_t           bus_voltage;        /* dword_20000764 — DC bus voltage */
    q15_t           motor_angle;        /* dword_200003C4 — electrical angle */
    q15_t           motor_temp;         /* dword_20000B08 — motor temperature */
    q15_t           battery_temp;       /* dword_20000A38 — battery temperature */

    /* ---- Hall sensor ---- */
    uint8_t         hall_sector;        /* byte_20000071 — current Hall sector */
    uint8_t         hall_direction;     /* byte_200003CF — rotation direction */
    uint8_t         hall_error_count;   /* byte_20000070 — hall error counter */
    uint8_t         hall_valid;         /* byte_20000072 — which hall signal */

    /* ---- Telemetry / protocol ---- */
    q15_t           odometer;           /* dword_200000EC — distance */
    uint32_t        uptime_ticks;       /* dword_200000E8 — system ticks */
    uint16_t        serial_errors;      /* word_20000024 / word_20000026 */
    uint16_t        battery_percent;    /* computed from voltage curve */

    /* ---- Speed limits from dashboard/BMS ---- */
    uint16_t        speed_limit_kmh;    /* word_200003D4 */
    uint16_t        speed_limit_min;    /* word_200003D6 */
    uint16_t        wheel_diameter;     /* word_200003E8 */
    uint16_t        motor_poles;        /* word_200003EA */

    /* ---- Error state ---- */
    uint32_t        error_flags;        /* dword_20000B10 — bitfield */
    uint8_t         error_code;         /* byte_20000660 */

    /* ---- Calibration ---- */
    uint32_t        adc_offset_a;       /* dword_20001138 */
    uint32_t        adc_offset_b;       /* dword_2000113C */
    uint32_t        adc_offset_c;       /* dword_20001140 */

    /* ---- Flash config read at boot ---- */
    uint16_t        cfg_kp;             /* word_20001F24 */
    uint16_t        cfg_ki;             /* word_20001F26 */
    uint16_t        cfg_kd;             /* word_20001F28 */
    uint16_t        cfg_accel;          /* word_20001F2A */
    uint16_t        cfg_decel;          /* word_20001F2C */
    uint16_t        cfg_fw_start;       /* word_20001F2E */
    uint16_t        cfg_regen;          /* word_20001F30 */
    uint16_t        cfg_brake;          /* word_20001F32 */
    uint16_t        cfg_valid_marker;   /* word_20001F34 (0 or 0xFF = defaults) */

    /* ---- Dashboard commands (received via serial) ---- */
    uint16_t        dash_command;       /* word_20001FC8 */
    uint8_t         dash_mode;          /* byte_20001FC1 */
    uint8_t         dash_lock;          /* byte_20001FC2 */
    uint16_t        dash_speed_limit;   /* word_20001FCA */
    uint16_t        dash_throttle;      /* word_20001FD0 */
    uint16_t        dash_brake;         /* word_20001FD2 */
    uint16_t        dash_flags;         /* word_20001FD6 */
} g3_state_t;

extern g3_state_t g3;

/* ========================================================================== */
/*  Telemetry output buffer (sent to dashboard)                                */
/* ========================================================================== */

typedef struct {
    uint32_t status_word;       /* dword_20001FE0 — packed status bits */
    uint8_t  motor_temp_a;      /* BYTE2(dword_20001FE4) */
    uint8_t  motor_temp_b;      /* HIBYTE(dword_20001FE4) */
    uint8_t  mode_byte;         /* byte_20001FE8 */
    uint8_t  pad;
    int16_t  batt_voltage;      /* word_20001E3C */
    int16_t  batt_current;      /* word_20001E3E */
    int16_t  batt_percent;      /* word_20001E40 */
    int16_t  error_code;        /* word_20001E42 */
} telemetry_packet_t;

/* Speed config table entry (maps to dword_20000268 array) */
typedef struct {
    q15_t   max_speed;
    q15_t   max_accel;
    q15_t   max_brake;
    q15_t   field_weaken;
    q15_t   current_limit;
    q15_t   ramp_up;
    q15_t   ramp_down;
} speed_config_entry_t;

/* ========================================================================== */
/*  Lookup tables (placeholders — actual data was embedded in flash)           */
/* ========================================================================== */

/* Sine lookup table — 512 entries, Q15 (located at 0x800EAD0) */
extern const int32_t sin_table_512[512];

/* Inverse sqrt lookup — (located at 0x800DAD0) */
extern const uint32_t isqrt_table[1024];

/* ADC-to-temperature lookup — 236 entries (located at 0x800F30C) */
extern const int16_t adc_temp_table[236];

/* AHB prescaler table (located at 0x800F2EC area) */
extern const uint8_t ahb_prescaler_table[16];

/* APB prescaler table */
extern const uint8_t apb_prescaler_table[8];

#endif /* G3_TYPES_H */
