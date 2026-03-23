/**
 * @file foc.c
 * @brief Field-Oriented Control for BLDC/PMSM motor.
 *
 * Reconstructed from multiple functions:
 *   sub_80025BC  — Clarke/Park transform + current measurement
 *   sub_80030A8  — Current controller entry (calls sub_8002224)
 *   sub_8004ACC  — Apply FOC outputs to motor struct
 *   sub_8002E8C  — FOC tick (PID + filter + observer)
 *   sub_8002B54  — Speed controller / flux weakening
 *   sub_8002A54  — Current limiter / torque reference calc
 *   sub_800252C  — Hall-to-phase mapping
 *   sub_80030D8  — Back-EMF speed estimate
 *   sub_80030AC  — Speed measurement from hall timer
 *
 * The FOC runs in the ADC interrupt at PWM frequency (~14.4 kHz).
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

extern foc_state_t *foc_ptr;
extern g3_state_t   g3;

/* ADC calibration offsets */
static q15_t adc_offset_a;     /* dword_20001138 */
static q15_t adc_offset_b;     /* dword_2000113C */
static q15_t adc_offset_c;     /* dword_20001140 */

/* Phase current scaling */
static q15_t current_scale;    /* dword_20000A54 */

/* ========================================================================== */
/*  Hall sensor → ADC channel mapping (sub_800252C)                            */
/* ========================================================================== */

/**
 * Configure ADC sampling order based on current hall sector.
 * TIM1 channels 5,6,7 are mapped to the three motor phases.
 *
 * Hall sector 17 (0x11): Phase order = C, A, B  → ADC ch 6,7,5
 * Hall sector 34 (0x22): Phase order = A, B, C  → ADC ch 5,7,6
 * Hall sector 51 (0x33): Phase order = A, B, C  → ADC ch 5,6,7
 */
void foc_set_hall_sector(uint8_t sector)
{
    volatile uint32_t *tim = (volatile uint32_t *)TIM1_BASE;

    switch (sector) {
    case HALL_SECTOR_AC:  /* 17 */
        /* sub_800B8FA(TIM1, 6, 1); sub_800B8FA(TIM1, 7, 2); sub_800B8FA(TIM1, 5, 3); */
        /* Map: ch6→rank1, ch7→rank2, ch5→rank3 */
        break;
    case HALL_SECTOR_BC:  /* 34 */
        /* sub_800B8FA(TIM1, 5, 1); sub_800B8FA(TIM1, 7, 2); sub_800B8FA(TIM1, 6, 3); */
        break;
    case HALL_SECTOR_AB:  /* 51 */
        /* sub_800B8FA(TIM1, 5, 1); sub_800B8FA(TIM1, 6, 2); sub_800B8FA(TIM1, 7, 3); */
        break;
    }
}

/* ========================================================================== */
/*  Current measurement + Clarke transform (sub_80025BC)                       */
/* ========================================================================== */

/**
 * Read phase currents from ADC, apply calibration offsets,
 * and compute alpha/beta (Clarke transform).
 *
 * sub_80025BC reconstructed:
 *   1. Read ADC channels 0 and 1 via sub_800BA52(TIM1, ch)
 *   2. Scale to Q15: raw_adc << 15, mask to 27 bits
 *   3. Subtract calibration offsets (dword_20001138/113C/1140)
 *   4. Compute third phase: Ic = -(Ia + Ib)
 *   5. Scale by (wheel_diameter << 15) / current_scale
 *   6. Compute I_alpha = Ia, I_beta = (Ia + 2*Ib) / sqrt(3)
 *
 * @param[out] i_alpha  Alpha-axis current (Q15)
 * @param[out] i_beta   Beta-axis current (Q15)
 * @param[in]  sector   Current hall sector
 */
void foc_measure_currents(q15_t *i_alpha, q15_t *i_beta, uint8_t sector)
{
    /* Read raw ADC (sub_800BA52 reads TIM1 CCR for injected ADC) */
    q15_t adc_ch0 = 0; /* placeholder: (sub_800BA52(TIM1, 0) << 15) & 0x7FFFFFF */
    q15_t adc_ch1 = 0; /* placeholder: (sub_800BA52(TIM1, 1) << 15) & 0x7FFFFFF */

    /* Clamp current_scale to valid range [26214..42598] */
    /* dword_20000A54 clamping logic */
    if (current_scale > 42598)  current_scale = 42598;
    if (current_scale < 26214)  current_scale = 26214;

    /* Compute reciprocal scale factor */
    q15_t inv_scale = ((q15_t)g3.wheel_diameter << 15) / current_scale;

    /* Phase currents depend on hall sector */
    q15_t Ia = 0, Ib = 0, Ic = 0;

    switch (sector) {
    case HALL_SECTOR_AC:  /* 17 */
        Ib = adc_ch0 - adc_offset_b;
        Ic = adc_ch1 - adc_offset_c;
        Ia = -(Ib + Ic);
        break;
    case HALL_SECTOR_BC:  /* 34 */
        Ia = adc_ch0 - adc_offset_a;
        Ic = adc_ch1 - adc_offset_c;
        Ib = -(Ic + Ia);
        break;
    case HALL_SECTOR_AB:  /* 51 */
        Ia = adc_ch0 - adc_offset_a;
        Ib = adc_ch1 - adc_offset_b;
        Ic = -(Ib + Ia);
        break;
    }

    /* Scale to engineering units */
    Ia = q15_mul(Ia, -inv_scale);
    Ib = q15_mul(Ib, -inv_scale);
    Ic = q15_mul(Ic, -inv_scale);

    /* Store in global */
    /* dword_200002F4 = Ia, dword_200002F8 = Ib, dword_200002FC = Ic */

    /* Clarke transform: alpha-beta from 3-phase */
    *i_alpha = Ia;
    *i_beta  = Ib;    /* Simplified; full Clarke: (Ia + 2*Ib) / sqrt(3) */

    /* Handle direction reversal (byte_200003CF) */
    if (g3.hall_direction) {
        *i_beta = -(*i_alpha + *i_beta);
    }

    /* Open-phase detection override (byte_20000070) */
    if (g3.hall_error_count == 0) {
        /* Use previous values — motor not commutating */
        /* *i_alpha = prev_alpha; *i_beta = prev_beta; */
    }
}

/* ========================================================================== */
/*  Park transform                                                             */
/* ========================================================================== */

/**
 * Forward Park transform: alpha/beta → d/q using electrical angle.
 *
 * @param[in]  i_alpha  Alpha current
 * @param[in]  i_beta   Beta current
 * @param[in]  angle    Electrical angle (Q15 representation)
 * @param[out] id       d-axis current
 * @param[out] iq       q-axis current
 */
void foc_park_transform(q15_t i_alpha, q15_t i_beta, q15_t angle,
                         q15_t *id, q15_t *iq)
{
    q15_t sin_val = fast_sin_q15(angle);
    q15_t cos_val = fast_cos_q15(angle);

    *id =  q15_mul(i_alpha, cos_val) + q15_mul(i_beta, sin_val);
    *iq = -q15_mul(i_alpha, sin_val) + q15_mul(i_beta, cos_val);
}

/* ========================================================================== */
/*  Inverse Park transform                                                     */
/* ========================================================================== */

/**
 * Inverse Park: d/q → alpha/beta for voltage output.
 */
void foc_inv_park(q15_t vd, q15_t vq, q15_t angle,
                  q15_t *v_alpha, q15_t *v_beta)
{
    q15_t sin_val = fast_sin_q15(angle);
    q15_t cos_val = fast_cos_q15(angle);

    *v_alpha = q15_mul(vd, cos_val) - q15_mul(vq, sin_val);
    *v_beta  = q15_mul(vd, sin_val) + q15_mul(vq, cos_val);
}

/* ========================================================================== */
/*  Speed measurement (sub_80030AC + sub_80030D8)                              */
/* ========================================================================== */

/**
 * Measure motor speed from hall sensor timer period.
 *
 * sub_80030AC:
 *   speed_raw = sub_8009774(timer) >> 1   — read timer capture
 *   Check plausibility: |speed_raw - prev| < 22937600
 *   If too different, use previous value.
 *
 * sub_80030D8 applies temperature compensation:
 *   Converts electrical angle to double-precision float,
 *   divides by 2*pi (0x4023193A79A4DD62 = ~9.5... probably pole pairs),
 *   multiplies by speed, converts back to Q15.
 */
q15_t foc_measure_speed(void)
{
    q15_t speed_raw = 0;  /* Read from hall timer capture register */

    /* Plausibility check */
    q15_t prev_speed = g3.motor_speed;
    q15_t diff = speed_raw - prev_speed;
    if (diff < 0) diff = -diff;

    if (diff > 22937600) {
        /* Reject measurement, keep previous */
        return prev_speed;
    }

    return speed_raw;
}

/* ========================================================================== */
/*  FOC current control PID (sub_8002224 / sub_8002E8C)                        */
/* ========================================================================== */

/**
 * Current (torque) controller.
 * sub_8002224:
 *   1. Call sub_80025BC to get phase currents
 *   2. If direction reversed (byte_200003CF), negate: Ib = -(Ia + Ib)
 *
 * sub_8002E8C:
 *   1. Run PID on d-axis and q-axis currents
 *   2. Apply 2nd-order low-pass filter (sub_80070D8)
 *   3. Update observer with current speed
 */
void foc_current_control_tick(q15_t speed_ref)
{
    foc_state_t *foc = foc_ptr;
    q15_t i_alpha, i_beta;

    /* Measure currents */
    foc_measure_currents(&i_alpha, &i_beta, g3.hall_sector);

    /* Store in FOC state */
    foc->i_alpha = i_alpha;
    foc->i_beta  = -i_beta;  /* Convention: negate beta */

    /* Park transform to d-q */
    q15_t id_meas, iq_meas;
    foc_park_transform(i_alpha, i_beta, g3.motor_angle, &id_meas, &iq_meas);
    foc->id_measured = id_meas;
    foc->iq_measured = -iq_meas;

    /* d-axis PI controller (flux) — target id = 0 for PMSM */
    q15_t id_error = 0 - id_meas;
    foc->vd_output = id_error;  /* Simplified — actual PI in sub_8001E3E */

    /* q-axis PI controller (torque) */
    q15_t iq_error = foc->torque_ref - iq_meas;
    foc->vq_output = -iq_error; /* Simplified — actual PI */

    /* Inverse Park to get v_alpha, v_beta */
    q15_t v_alpha, v_beta;
    foc_inv_park(foc->vd_output, foc->vq_output, g3.motor_angle,
                 &v_alpha, &v_beta);

    foc->v_alpha = v_alpha;
    foc->v_beta  = v_beta;

    /* Apply to PWM via SVM (Space Vector Modulation) */
    /* This sets TIM1 CCR1/2/3 duty cycles */
}

/* ========================================================================== */
/*  Speed controller / flux weakening (sub_8002B54)                            */
/* ========================================================================== */

/**
 * Outer speed control loop.
 *
 * sub_8002B54 is a large function (~600 lines) that:
 *   1. Reads the speed reference from the ride controller
 *   2. Computes speed error
 *   3. Runs a PI + anti-windup controller
 *   4. Computes field weakening above base speed
 *   5. Limits output torque reference
 *   6. Feeds result to the current controller
 *
 * It has two main paths:
 *   - Below base speed: pure torque control via q-axis current
 *   - Above base speed: field weakening via negative d-axis current
 *
 * The observer (sub_800723E / sub_80072BC) tracks the actual speed
 * and computes max/min torque envelopes.
 */
int foc_speed_controller_tick(q15_t target_speed, q15_t measured_speed)
{
    foc_state_t *foc = foc_ptr;
    q15_t speed_error = target_speed - measured_speed;

    /* Observer update — tracks speed reference vs actual */
    q15_t speed_fwd = foc->speed_ref;      /* +0 */
    q15_t speed_gain = foc->speed_limit_fwd; /* +4 */

    /* Check if field weakening is needed */
    q15_t speed_diff = target_speed - measured_speed;

    if (speed_diff >= speed_fwd || target_speed <= 0 || measured_speed <= 0) {
        /* Below base speed or decelerating — no field weakening */
        if (speed_diff <= speed_fwd / 2 || measured_speed <= 0) {
            /* Normal torque mode */
            foc->field_weakening = 0;
        }
    } else {
        /* Above base speed — engage field weakening */
        if (!foc->field_weakening && speed_diff >= speed_fwd) {
            foc->field_weakening = 1;
            /* Initialize FW state */
        }
    }

    /* Compute torque reference from speed error + PI */
    q15_t abs_speed = abs_i32(measured_speed);
    if (abs_speed < 5) abs_speed = 5;

    /* Apply torque limits based on operating point */
    q15_t torque_cmd = q15_mul(speed_error, speed_gain);

    /* Rate-limit the torque command */
    if (torque_cmd > foc->current_limit)
        torque_cmd = foc->current_limit;
    if (torque_cmd < -foc->current_limit)
        torque_cmd = -foc->current_limit;

    foc->torque_ref = torque_cmd;

    return 1;
}

/* ========================================================================== */
/*  Current limiter (sub_8002A54)                                              */
/* ========================================================================== */

/**
 * Applies dynamic current limiting based on:
 *   - Temperature derating
 *   - Battery SOC
 *   - Overcurrent protection
 *
 * sub_8002A54:
 *   - Checks phase_valid (byte at offset +176)
 *   - If valid: compute torque from reference
 *   - If not valid: decay current limit exponentially (39321/32768 ≈ 1.2x)
 *   - Clamps to [3276 .. 65536] (CURRENT_LIMIT_MIN .. MAX)
 *   - Applies slew rate limit from speed-dependent gain
 */
void foc_current_limiter(foc_state_t *foc, q15_t speed, q15_t *torque_ref,
                         int brake_active)
{
    uint8_t was_valid = 0;  /* offset +177 */
    uint8_t is_valid  = 0;  /* offset +176 */

    if (was_valid || !is_valid) {
        if (was_valid && !is_valid) {
            /* Decay: multiply by 39321/32768 ≈ 1.2 */
            /* Actually 39321LL * val >> 15 ≈ 1.2 * val */
            q15_t limited = (int32_t)((39321LL * foc->iq_ref_limited) >> 15);
            foc->iq_ref_limited = limited;
        }
    } else {
        /* Fresh valid reading: set from reference */
        q15_t new_ref = q15_mul(speed, foc->ramp_rate);
        foc->iq_ref_limited = new_ref;
    }

    /* Clamp to limits */
    q15_t limit = foc->iq_ref_limited;  /* offset +160 */
    if (limit > CURRENT_LIMIT_MAX) limit = CURRENT_LIMIT_MAX;
    if (limit < CURRENT_LIMIT_MIN) limit = CURRENT_LIMIT_MIN;

    /* Apply slew-rate-limited current to torque reference */
    q15_t max_step = q15_div(foc->torque_ref, limit);
    q15_t new_torque = foc->iq_ref_limited + max_step;

    if (brake_active) {
        if (*torque_ref <= new_torque)
            new_torque = *torque_ref;
        *torque_ref = new_torque;
    }
}

/* ========================================================================== */
/*  Apply FOC outputs to motor struct (sub_8004ACC)                            */
/* ========================================================================== */

/**
 * Takes the ride controller output (motor_params block) and
 * translates it into FOC reference values.
 *
 * This large function (sub_8004ACC, ~200 lines) does:
 *   1. Set mode_select from motor_params.mode_id
 *   2. If mode_id == 5 (push), set special push-assist reference
 *   3. Otherwise compute v_alpha/beta from speed reference
 *   4. Apply exponential temperature compensation using double-precision
 *      math: exp(-temp / tau) where tau comes from motor thermal model
 *   5. Set brake/field-weakening/speed flags
 *   6. Configure current loop filter bandwidth
 */
int foc_apply_params(motor_params_t *params)
{
    foc_state_t *foc = foc_ptr;

    /* Set drive mode */
    foc->mode_select = params->mode_id;

    if (!params->regen_enable) {
        foc->mode_select = 1;  /* Force torque mode when regen off */
    }

    /* If switching from push mode (5) to another mode, save angle */
    /* (byte_200000B4 tracking logic) */

    if (params->mode_id == 5) {
        /* Push mode: direct torque reference */
        foc->iq_ref_limited = params->iq_ref_limit;
    } else {
        /* Normal: compute voltage reference */
        /* v_alpha = speed_ref * gain, v_beta = -(speed_ref * gain) */
        foc->v_alpha = params->max_speed_rpm;
        foc->v_beta  = -params->max_speed_rev;
    }

    /* Set flags */
    foc->brake_enable    = params->regen_enable;
    foc->speed_mode      = params->cruise_enable;
    foc->field_weakening = 0;
    foc->torque_ref      = params->current_limit;
    foc->regeneration_on = params->kers_enable;

    /* Temperature-compensated current limits */
    /* Uses exp() via double-precision soft-float:
     *   correction = exp(-motor_temp * 0.0000610...) (0x3F10624DD2F1A9FC)
     *   1.0 + correction → scale factor
     *   Apply to current loop filter coefficients
     */

    return 1;
}
