/**
 * @file ride_ctrl.c
 * @brief Ride control state machine for Ninebot Max G3.
 *
 * This is the core application logic that determines what the motor
 * should do based on throttle input, brake input, speed, and mode.
 *
 * Reconstructed from:
 *   sub_80032DC  — ride_ctrl_init()
 *   sub_8004DE0  — ride_ctrl_process() — the main 1kHz tick
 *   sub_8003C2C  — mode_idle()
 *   sub_8003C90  — mode_eco()
 *   sub_8003E90  — mode_ride() — the largest and most complex function
 *   sub_80035C4  — mode_sport()
 *   sub_800396C  — mode_locked()
 *   sub_8003568  — mode_off() / motor disable
 *   sub_8003BDC  — throttle_process()
 *   sub_8003B9C  — brake_process()
 *   sub_8003940  — push_detect() — detect walking speed
 *   sub_8003A18  — cruise_control()
 *   sub_80058E0  — mode_transition() — handle mode switching
 *   sub_800564C  — power_on_check() — check conditions for motor enable
 *   sub_800562C  — eco_mode_check()
 *   sub_80055F8  — direction_check()
 *   sub_80056D4  — speed_limit_calc()
 */

#include <stdint.h>
#include "g3_types.h"
#include "parameters.h"
#include "fixed_point.h"

/* ========================================================================== */
/*  External references                                                        */
/* ========================================================================== */

extern g3_state_t       g3;
extern foc_state_t     *foc_ptr;
extern int              foc_apply_params(motor_params_t *params);
extern q15_t            foc_measure_speed(void);
extern int              foc_speed_controller_tick(q15_t target, q15_t measured);
extern void             foc_current_limiter(foc_state_t *foc, q15_t speed,
                                            q15_t *ref, int brake);

/* Forward declarations */
static void mode_idle(motor_params_t *p);
static void mode_eco(motor_params_t *p);
static void mode_ride(motor_params_t *p);
static void mode_sport(motor_params_t *p);
static void mode_locked(motor_params_t *p);
static void mode_off(motor_params_t *p);

/* ========================================================================== */
/*  State variables                                                            */
/* ========================================================================== */

static motor_params_t   active_params;          /* dword_20000B4C area */
static q15_t            throttle_output;        /* dword_20000F24 — processed throttle */
static q15_t            target_speed;           /* dword_20000F2C */
static q15_t            target_speed_filtered;  /* dword_20000F30 */
static q15_t            speed_decel_rate;       /* dword_20000F34 */
static uint8_t          system_initialized;     /* byte_20000F48 */
static uint8_t          prev_mode;              /* byte_20000F38 */
static uint8_t          prev_locked;            /* byte_20000111 */
static uint8_t          regen_steady_count;     /* word_20000116 */
static uint8_t          brake_active_internal;  /* byte_20000110 */
static q15_t            cruise_setpoint;        /* dword_20000130 */

/* Speed configuration table — one entry per ride mode */
/* dword_20000268 is an array of pointers to these configs */
static speed_config_entry_t speed_configs[4];

/* ========================================================================== */
/*  Debounce timers (sub_80099D0 init, sub_80098E4 tick)                       */
/* ========================================================================== */

static debounce_timer_t tmr_power_on;          /* unk_20000DBC */
static debounce_timer_t tmr_direction;          /* unk_20000CE0 */
static debounce_timer_t tmr_regen;             /* unk_20000D6C */

/* ========================================================================== */
/*  Initialization (sub_80032DC)                                               */
/* ========================================================================== */

/**
 * Initialize the ride control system.
 *
 * sub_80032DC (~150 lines):
 *   1. Initializes ~20 debounce timers with various thresholds
 *   2. Reads PID gains from flash or uses defaults
 *   3. Configures speed observer (sub_800723E, sub_80072BC)
 *   4. Initializes the FOC PID controller (sub_8002990)
 *   5. Sets initial ride state to IDLE
 */
void ride_ctrl_init(void)
{
    /* Initialize debounce timers (sub_80099D0 calls) */
    /* Each timer has: threshold, prescaler, count_mode, reset_mode */

    /* Timer configs from the decompiled init sequence: */
    /* tmr @ unk_20000CB8: threshold=200, prescaler=1, mode=1 */
    /* tmr @ unk_20000D30: threshold=400, prescaler=1, mode=1 */
    /* tmr @ unk_20000CF4: threshold=1,   prescaler=1, mode=3 */
    /* tmr @ unk_20000D1C: threshold=400, prescaler=1, mode=1 */
    /* tmr @ unk_20000CE0: threshold=2,   prescaler=1, mode=1 */
    /* tmr @ unk_20000D80: threshold=60,  prescaler=1, mode=1 */
    /* tmr @ unk_20000C7C: threshold=60,  prescaler=1, mode=1 */
    /* tmr @ unk_20000C90: threshold=600, prescaler=1, mode=2 */
    /* tmr @ unk_20000D44: threshold=20,  prescaler=1, mode=1 */
    /* tmr @ unk_20000D58: threshold=30,  prescaler=4, mode=1 */
    /* tmr @ unk_20000CA4: threshold=20,  prescaler=1, mode=1 */
    /* tmr @ unk_20000D6C: threshold=100, prescaler=1, mode=1 */
    /* tmr @ unk_20000D94: threshold=60,  prescaler=1, mode=1 */
    /* tmr @ unk_20000DA8: threshold=2,   prescaler=1, mode=1 */
    /* tmr @ unk_20000DBC: threshold=2,   prescaler=1, mode=1 */
    /* tmr @ unk_20000DD0: threshold=20,  prescaler=1, mode=1 */
    /* tmr @ unk_20000DE4: threshold=20,  prescaler=1, mode=1 */
    /* tmr @ unk_20000DF8: threshold=60,  prescaler=1, mode=1 */

    target_speed = 0;
    g3.sport_mode = 0;
    g3.eco_mode   = 0;

    /* Read PID config from flash or use defaults */
    q15_t kp, ki, kd, accel, decel, fw_start, regen_gain, brake_gain;

    if (!g3.cfg_valid_marker || g3.cfg_valid_marker == 255) {
        /* Flash config invalid — use defaults */
        kp        = DEFAULT_KP;
        ki        = DEFAULT_KI;
        kd        = DEFAULT_KD;
        accel     = DEFAULT_ACCEL;
        decel     = DEFAULT_DECEL;
        fw_start  = DEFAULT_FW_START;
        regen_gain= DEFAULT_REGEN;
        brake_gain= DEFAULT_BRAKE;
    } else {
        /* Load from flash */
        kp        = g3.cfg_kp << 7;
        ki        = g3.cfg_ki << 7;
        kd        = g3.cfg_kd << 7;
        accel     = g3.cfg_accel << 7;
        decel     = g3.cfg_decel << 7;
        fw_start  = g3.cfg_fw_start << 7;
        regen_gain= g3.cfg_regen << 7;
        brake_gain= g3.cfg_brake << 7;
    }

    /* Initialize FOC PID (sub_8002990) */
    /* sub_8001DFC: set PID gains: Kp=1000, Ki=0x11859820, Kd=19865110 */
    /* sub_800723E: configure observer filter */
    /* sub_80072BC: set observer gains */

    g3.ctrl_state = CTRL_STATE_IDLE;
    system_initialized = 1;
}

/* ========================================================================== */
/*  Push detection (sub_8003940)                                               */
/* ========================================================================== */

/**
 * Detect if the scooter is being pushed (walking speed).
 * Returns true if the throttle sensor detects sub_800A4F8 < 1638
 * (approximately 50mA / ~0.05 in Q15 — very low throttle).
 */
static int push_detected(void)
{
    return system_initialized && (/* throttle_adc < 1638 */ 0);
}

/* ========================================================================== */
/*  Mode transition logic (sub_80058E0)                                        */
/* ========================================================================== */

/**
 * Handle ride mode transitions with safety checks.
 *
 * Rules:
 *   - If mode changes, reset tail_light_on flag
 *   - Sport→any: keep tail light
 *   - Ride→Eco or Eco→Ride: keep tail light
 *   - If error flags set (sub_8004DB0), force tail light off
 *   - If push mode and not in ride: force off
 */
static void mode_transition(uint8_t new_mode)
{
    if (prev_mode != new_mode)
        g3.tail_light_on = 0;

    if (prev_mode == 3)  /* SPORT */
        g3.tail_light_on = 1;
    if (prev_mode == 2 && new_mode == 1)  /* RIDE→ECO */
        g3.tail_light_on = 1;
    if (prev_mode == 1 && new_mode == 2)  /* ECO→RIDE */
        g3.tail_light_on = 1;

    /* Error check */
    if (g3.error_flags & ERR_FLAG_MASK)
        g3.tail_light_on = 0;

    prev_mode = new_mode;
}

/* ========================================================================== */
/*  Idle mode (sub_8003C2C)                                                    */
/* ========================================================================== */

/**
 * Motor is off / coasting.
 * No torque applied, monitor for re-enable conditions.
 */
static void mode_idle(motor_params_t *p)
{
    p->brake_current    = 0;
    p->max_decel        = g3.battery_temp;      /* dword_20000B44 */
    p->max_accel        = 0;                     /* dword_20000B40 */

    q15_t max_fwd = 0; /* dword_20000B3C */
    q15_t max_rev = 0; /* dword_20000B34 */
    p->max_speed_fwd    = (max_fwd > max_rev) ? max_fwd : max_rev;

    p->max_speed_rpm    = 0; /* dword_20000B30 */
    p->mode_id          = 1; /* Idle mode = 1 */
    p->push_assist      = 0;
    p->regen_enable     = 0;
    p->cruise_enable    = 0;
    p->kers_enable      = 0;

    p->temp_limit_hi    = 1101004800;  /* ~30.0 float */
    p->temp_limit_lo    = 1101004800;
}

/* ========================================================================== */
/*  Eco mode (sub_8003C90)                                                     */
/* ========================================================================== */

/**
 * Eco / low-power mode.
 * Reduced speed and current limits.
 * Gradual speed ramp-up based on throttle.
 */
static void mode_eco(motor_params_t *p)
{
    cruise_setpoint = Q15_HALF;

    p->brake_current = 0;
    p->max_decel     = 0; /* dword_20000228 */

    /* Throttle-dependent speed limit */
    q15_t throttle_pos = g3.motor_speed;  /* dword_20000F24 */

    if (/* throttle sensor value */ 0 <= 0) {
        /* No throttle: limit based on current speed */
        if (throttle_pos >= 32769 && throttle_pos < 114688) {
            /* Moderate speed: ramp up slowly */
            if (throttle_pos >= 104857 && !(g3.dash_flags & 2)) {
                /* Fast enough and no flag: increase limit */
                if (regen_steady_count >= 130) {
                    if (regen_steady_count < 250)
                        regen_steady_count++;
                    p->max_accel = throttle_pos + 2621;
                    if (p->max_accel > 131072)
                        p->max_accel = 131072;
                } else {
                    regen_steady_count++;
                }
            }
        } else {
            /* Slow or very fast: use conservative limit */
            if (regen_steady_count > 2)
                regen_steady_count -= 2;
            q15_t lim = throttle_pos + 2621;
            if (lim >= SPEED_LIMIT_ECO_Q15)
                lim = SPEED_LIMIT_ECO_Q15;
            p->max_accel = lim;
        }
    } else {
        /* Throttle active: scale by throttle position */
        p->max_accel = 0; /* Computed from throttle * max */
    }

    p->max_speed_fwd = 0; /* dword_20000220 */
    p->mode_id       = 2; /* Eco */
    p->cruise_enable = 0;
    p->kers_enable   = 0;
    p->push_assist   = 0;

    p->temp_limit_hi = 1097859072;   /* ~25.0 float */
    p->temp_limit_lo = 1097859072;
}

/* ========================================================================== */
/*  Normal ride mode (sub_8003E90) — THE BIG ONE                               */
/* ========================================================================== */

/**
 * Normal ride mode — the most complex function in the firmware (~500 lines).
 *
 * This handles:
 *   1. Throttle processing with push-mode detection
 *   2. Speed target computation from throttle + speed limits
 *   3. Cruise control engagement/disengagement
 *   4. Regenerative braking calculation
 *   5. Electronic Braking System (EBS) integration
 *   6. Field weakening speed reference
 *   7. Temperature derating
 *   8. Battery SOC limiting
 *   9. Over-speed protection
 *
 * Key state variables updated:
 *   dword_20000120  — throttle_level (from observer or push detect)
 *   dword_20000128  — torque_command
 *   dword_2000013C  — speed_reference
 *   dword_20000180  — raw_speed_setpoint
 *   dword_20000184  — filtered_speed_setpoint
 *   dword_2000018C  — final_speed_limit
 */
static void mode_ride(motor_params_t *p)
{
    q15_t throttle_level;
    q15_t speed_setpoint;
    q15_t speed_limit;

    /* ---- 1. Throttle input ---- */
    if (g3.push_mode) {
        /* Push assist: use fixed low throttle from observer */
        throttle_level = 0;  /* dword_20000BD4 */
    } else {
        /* Normal: read from throttle processing module */
        throttle_level = 0;  /* sub_800A514(&unk_20000B84) */
    }

    /* ---- 2. Speed setpoint computation ---- */
    /* Get max speed for current temperature */
    q15_t max_speed_temp = 0;  /* sub_80048AC(motor_temp) */
    q15_t raw_max = max_speed_temp;
    if (max_speed_temp > raw_max)
        raw_max = max_speed_temp;

    speed_setpoint = raw_max;

    /* ---- 3. Torque command ---- */
    if (g3.push_mode) {
        /* Direct speed command in push mode */
        p->iq_ref_limit = speed_setpoint;
    } else {
        /* Throttle-modulated torque */
        q15_t torque = q15_mul(throttle_level, speed_setpoint);
        p->iq_ref_limit = torque;
    }

    /* ---- 4. Cruise control ---- */
    if (!g3.cruise_active && g3.tail_light_on && (g3.dash_command & 4)) {
        /* Check cruise engagement conditions */
        if (/* dashboard cruise enabled */ 0) {
            q15_t cruise_spd = 0;  /* dword_20000B34 */
            if (cruise_spd < 0) cruise_spd = -cruise_spd;
            q15_t brake_limit = cruise_spd / 8;
            q15_t cmd = p->iq_ref_limit;

            if (p->max_speed_rpm >= cmd) {
                if (cmd < -brake_limit)
                    cmd = -brake_limit;
            } else {
                cmd = p->max_speed_rpm;
            }
            p->iq_ref_limit = cmd;
        }
    }

    /* ---- 5. Regenerative braking ---- */
    q15_t regen_level = g3.motor_speed;  /* dword_20000F24 */

    /* Over-speed regen: if speed > target, apply braking */
    if (g3.bus_voltage <= 3178496 || g3.motor_current >= -98304) {
        /* Battery/current OK: check speed */
        if (regen_level < 0 /* half max decel speed */)
            brake_active_internal = 0;
    } else {
        brake_active_internal = 1;
    }

    /* ---- 6. Electronic braking system (EBS) ---- */
    if ((g3.dash_command & 0x4000) || brake_active_internal) {
        /* Apply regenerative braking force */
        q15_t brake_force = 0;  /* sub_8005798 */
        rate_limit(brake_force, &p->brake_current, 13107);
    } else {
        q15_t brake_force = 0;
        rate_limit(brake_force, &p->brake_current, 3276);
    }

    /* ---- 7. Speed limit from throttle position ---- */
    if (g3.push_mode) {
        /* sub_80056D4: push mode speed limit */
    } else {
        /* Normal speed limit calculation */
        q15_t speed_diff = target_speed - regen_level;
        if (speed_diff > 163840) speed_diff = 163840;
        if (speed_diff < 0)      speed_diff = 0;

        speed_limit = speed_diff / 5 * g3.speed_limit_min;

        /* Clamp speed */
        if (regen_level > 3276800)
            regen_level = 3276800;
        if (regen_level < Q15_HALF)
            regen_level = Q15_HALF;

        if (p->max_speed_rpm >= speed_limit) {
            if (speed_limit < regen_level)
                speed_limit = regen_level;
        } else {
            speed_limit = p->max_speed_rpm;
        }
    }

    p->max_speed_rpm = speed_limit;
    p->mode_id       = 2;   /* Normal ride */
    p->cruise_enable = 1;
    p->kers_enable   = 1;

    /* ---- 8. Determine motor enable conditions ---- */
    p->regen_enable = ((uint32_t)(regen_level + 65536) > 131072)
                      && !push_detected()
                      || (g3.ebs_active && push_detected() && !g3.cruise_active);
    p->push_assist = 1;

    /* ---- 9. Over-speed protection ---- */
    p->max_accel = target_speed + 131072;  /* +4 in Q15 */
    p->max_decel = speed_decel_rate;
}

/* ========================================================================== */
/*  Sport mode (sub_80035C4)                                                   */
/* ========================================================================== */

/**
 * Sport mode — similar to ride mode but with higher limits.
 * Higher current limits, faster acceleration, higher speed cap.
 */
static void mode_sport(motor_params_t *p)
{
    /* Most logic is shared with mode_ride() */
    mode_ride(p);

    /* Override with sport-specific limits */
    p->mode_id = 3;  /* Sport */
}

/* ========================================================================== */
/*  Locked mode (sub_800396C)                                                  */
/* ========================================================================== */

/**
 * Anti-theft lock mode.
 * Motor resists movement with a braking torque proportional to speed.
 *
 * sub_800396C:
 *   - Computes brake force from |motor_current|
 *   - If |current| >= 1310720 (40A): max brake = 327680
 *   - Otherwise: brake = |current| / 4
 *   - Sets special PID gains for maximum holding torque
 */
static void mode_locked(motor_params_t *p)
{
    q15_t abs_current = g3.motor_current;
    if (abs_current < 0) abs_current = -abs_current;

    q15_t brake_force;
    if (abs_current >= 1310720) {
        brake_force = 327680;   /* Max brake force (~10 in Q15) */
    } else {
        brake_force = abs_current / 4;
    }

    p->brake_current   = brake_force;
    p->max_decel       = 0; /* dword_20000260 */
    p->max_accel       = 0; /* dword_2000025C */
    p->max_speed_fwd   = 0; /* dword_20000258 */
    p->max_speed_rpm   = 0; /* dword_2000024C */
    p->mode_id         = 5; /* Lock mode */
    p->ramp_rate        = 392123;  /* High gain for stiff resistance */
    p->regen_enable    = 0;
    p->cruise_enable   = 0;
    p->kers_enable     = 0;
    p->push_assist     = 0;

    p->temp_limit_hi   = 1106247680;  /* ~40.0 float */
    p->temp_limit_lo   = 1106247680;
}

/* ========================================================================== */
/*  Main ride control tick (sub_8004DE0)                                       */
/* ========================================================================== */

/**
 * Called every ~1ms from the main loop.
 * This is the master state machine that orchestrates everything.
 *
 * sub_8004DE0 flow:
 *   1. Process throttle (sub_8003BDC)
 *   2. Process speed PI control (sub_8005124, sub_80054B8, etc.)
 *   3. Read dashboard commands
 *   4. Select mode and call appropriate handler
 *   5. Apply overcurrent protection
 *   6. Run FOC speed controller (sub_8002A4E → sub_8002B54)
 *   7. Compute error-dependent derating
 *   8. Apply output to FOC (sub_8004ACC)
 */
int ride_ctrl_process(void)
{
    if (!system_initialized)
        return 0;

    /* ---- 1. Throttle processing ---- */
    /* sub_8003BDC:
     *   dword_20000100 = HIBYTE(word_20001FD0) << 15  — raw throttle
     *   dword_20000104 = dash_speed_limit or 6553 if braking
     *   sub_8004CA4(dword_20000104) — configure speed limit
     *   sub_8007D24(control_struct, dword_20000A44) — apply throttle
     */

    /* ---- 2. Speed processing chain ---- */
    /* Multiple filter/processing steps in series */

    /* ---- 3. Dashboard command processing ---- */
    /* Read mode from dashboard serial: ride/eco/sport/lock */

    /* ---- 4. Measure current speed ---- */
    g3.motor_speed = foc_measure_speed();
    /* Low-pass filter the speed measurement */
    /* sub_800727A(&speed, &filter_state, 5368709) */

    /* ---- 5. Temperature derating ---- */
    /* sub_8004640() — apply temperature curves */

    /* ---- 6. Speed observer update ---- */
    /* sub_8002A4E() → sub_8002B54() */
    foc_speed_controller_tick(target_speed, g3.motor_speed);

    /* ---- 7. Mode selection and parameter fill ---- */
    motor_params_t *p = &active_params;

    if (g3.scooter_locked) {
        /* LOCKED */
        mode_transition(4);
        mode_locked(p);
        g3.ctrl_state = CTRL_STATE_LOCKED;
    }
    else if (g3.scooter_on && !g3.scooter_locked) {
        /* Motor is enabled */
        if (g3.scooter_on && g3.sport_mode) {
            mode_transition(3);
            mode_sport(p);
            g3.ctrl_state = CTRL_STATE_SPORT;
        }
        else if (g3.scooter_on && g3.eco_mode) {
            mode_transition(1);
            mode_eco(p);
            g3.ctrl_state = CTRL_STATE_ECO;
        }
        else if (g3.scooter_on) {
            mode_transition(2);
            mode_ride(p);
            g3.ctrl_state = g3.push_mode ? CTRL_STATE_PUSH : CTRL_STATE_RIDE;
        }
        else {
            mode_transition(0);
            mode_idle(p);
            g3.ctrl_state = CTRL_STATE_IDLE;
        }
    }
    else {
        mode_transition(0);
        mode_idle(p);
        g3.ctrl_state = CTRL_STATE_IDLE;
    }

    /* ---- 8. Throttle sensor watchdog ---- */
    /* If locked and motor not spinning, reset energy recovery */
    if (!g3.scooter_locked) {
        /* sub_80053E8() — additional safety checks */
    }

    /* Locked mode: if motor is not moving, freeze odometer */
    if (!prev_locked && g3.scooter_locked) {
        /* dword_20000E20 = sub_8004CFC() — freeze position */
    }
    prev_locked = g3.scooter_locked;

    /* ---- 9. Throttle interlock ---- */
    if (g3.motor_speed < 262144 /* ~8 in Q15 */ || g3.push_mode) {
        /* sub_800A63C(&throttle_observer, 0) — disable throttle */
    } else {
        /* sub_800A63C(&throttle_observer, 1) — enable throttle */
    }

    /* ---- 10. Apply parameters to FOC ---- */
    foc_apply_params(p);

    return 1;
}
