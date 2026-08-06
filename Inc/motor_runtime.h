/*
 * motor_runtime.h - shared motor-control runtime state
 *
 * Definitions live in motor_runtime.c. Modules that need these symbols
 * include this header (or common.h / signal.h for the older shared set).
 */
#ifndef MOTOR_RUNTIME_H_
#define MOTOR_RUNTIME_H_

#include <stdint.h>
#include "common.h"

/* --- commutation / BEMF --- */
extern char step;
extern volatile char rising;
/* forward is in common.h */
extern char desync_check;
extern volatile uint8_t bemfcounter;
extern volatile uint8_t zcfound;
extern volatile uint32_t commutation_interval;
extern volatile uint16_t commutation_intervals[6];
extern volatile uint32_t average_interval;
extern uint32_t last_average_interval;
extern volatile uint16_t lastzctime;
extern volatile uint16_t thiszctime;
extern volatile uint16_t waitTime;
extern uint16_t advance;
extern uint8_t temp_advance;
extern uint8_t auto_advance_level;
/*
 * Advance-schedule normalization: kerpm of electrical frequency per centivolt
 * of pack, Q12. Derived from motor kV and pole count once per settings load
 * (settings.c) so the hot path needs only a multiply and a shift:
 *
 *   max_kerpm = (advance_erpm_scale_q12 * V_ref) >> 12
 *
 * V_ref is a peak-hold of battery_voltage while throttle is applied (see
 * runtime_loop.c): instantaneous pack sag under load must not shrink the
 * ceiling or it fights the eRPM load correction. At idle V_ref tracks live
 * voltage so SoC is re-learned between loads.
 *
 * The ceiling is 15/16 of the ideal free-run electrical frequency
 * (kv * V * poles/2): real free-run sits a few percent below ideal (IR drop,
 * iron loss), and without the scale-down full-throttle free-run often stops
 * one advance notch short of the duty-curve top (23). Zero scale means the
 * schedule cannot be normalized (implausible kV or pole count) and the
 * caller must fall back to the duty-cycle proxy.
 */
extern uint16_t advance_erpm_scale_q12;
extern volatile char old_routine;
extern volatile uint32_t zero_crosses;
extern volatile uint32_t polling_mode_changeover;
/* Missed-ZC blind-step fallback (bemf_zc.c) */
extern volatile uint8_t zc_deadline_armed;
extern volatile uint8_t zc_blind_steps;
extern volatile uint8_t zc_pre_seen;
extern volatile uint8_t zc_demag_run;
extern volatile uint32_t zc_demag_accepts;
/*
 * INTERVAL_TIMER ticks of dead reckoning since the rotor last reported its
 * position. Blind steps add their extrapolated interval, an accepted crossing
 * clears it (a demag-late one charges an interval instead - see bemf_zc.c).
 * Drives both the restart decision and how much duty authority the loop is
 * allowed, so degradation is continuous instead of a set of trip points.
 */
extern volatile uint32_t zc_blind_ticks;
/*
 * Time without rotor position evidence before the loop is restarted through
 * the startup path. This is upstream AM32's stall criterion and the only
 * restart threshold in the ZC path; faults.c owns the rail itself.
 */
#define BEMF_STALL_TICKS 45000u
extern uint8_t filter_level;
extern uint8_t bad_count;
extern uint8_t bad_count_threshold;
extern uint8_t min_bemf_counts_up;
extern uint8_t min_bemf_counts_down;
extern char prop_brake_active;
extern uint8_t changeover_step;
extern uint8_t stuckcounter;
/* uint32_t always: DroneCAN telemetry and SITL stats both use 32-bit. */
extern uint32_t desync_happened;

/* --- duty / throttle --- */
extern volatile uint16_t duty_cycle;
extern uint16_t duty_cycle_setpoint;
extern uint16_t last_duty_cycle;
extern volatile uint16_t duty_cycle_maximum;
extern uint16_t minimum_duty_cycle;
extern uint16_t min_startup_duty;
extern uint16_t startup_max_duty_cycle;
extern uint16_t stall_protect_minimum_duty;
extern uint16_t adjusted_duty_cycle;
extern volatile uint16_t adjusted_input;
extern volatile uint16_t input;
extern volatile uint16_t newinput;
extern volatile char armed;
extern volatile uint8_t running;
extern volatile char stepper_sine;
extern char maximum_throttle_change_ramp;
extern uint8_t max_duty_cycle_change;
extern volatile uint8_t max_ramp_startup;
extern volatile uint8_t max_ramp_low_rpm;
extern volatile uint8_t max_ramp_high_rpm;
/* Voltage-compensated working copies + BEMF-headroom governor ceiling
 * (computed in runtime_loop.c at 1 kHz; see runtimeTransientGovernorTick) */
extern volatile uint8_t max_ramp_startup_vcomp;
extern volatile uint8_t max_ramp_low_rpm_vcomp;
extern volatile uint8_t max_ramp_high_rpm_vcomp;
extern volatile uint16_t gov_duty_ceiling;
extern volatile uint8_t ramp_divider;
extern uint16_t ramp_count;
extern volatile uint32_t pwm_to_arr_scale_q16;
extern volatile uint32_t throttle_duty_slope_q16;
extern volatile uint32_t sine_throttle_duty_slope_q16;
extern volatile uint16_t tim1_arr;
extern uint16_t prop_brake_duty_cycle;

/* --- control / PID --- */
extern fastPID currentPid;
extern fastPID speedPid;
extern fastPID stallPid;
extern char use_speed_control_loop;
extern char use_current_limit;
extern int16_t use_current_limit_adjust;
extern int32_t input_override;
extern int32_t stall_protection_adjust;
extern uint16_t stall_protect_target_interval;
extern uint16_t target_e_com_time;
extern uint8_t drive_by_rpm;
extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
extern uint32_t MINIMUM_RPM_SPEED_CONTROL;
extern char brushed_direction_set;
extern char reversing_dead_band;
extern uint16_t reverse_speed_threshold;
extern uint16_t enter_sine_angle;
extern char return_to_center;
extern char do_once_sinemode;
extern uint16_t current_angle;
extern uint16_t desired_angle;
extern int16_t phase_A_position;
extern int16_t phase_B_position;
extern int16_t phase_C_position;
/* Half-period sine table + reconstructing lookup. The original 360-entry
 * table satisfies pwmSin[i] + pwmSin[i+180] == 360 exactly, so the lookup
 * returns bit-identical values for the full [0,360) range while the flash
 * table halves to 360 bytes. */
extern const int16_t pwmSinHalf[180];
static inline int16_t pwmSinLookup(int16_t a)
{
	return (a < 180) ? pwmSinHalf[a] : (int16_t)(360 - pwmSinHalf[a - 180]);
}
extern uint16_t step_delay;
extern uint16_t gate_drive_offset;
extern uint16_t motor_kv;
extern uint8_t dead_time_override;
extern uint32_t MCU_Id;
extern uint32_t REV_Id;

/* --- sensing / telemetry tick --- */
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_input;
#ifdef NXP
extern uint16_t ADC_raw_temp[];
#else
extern uint16_t ADC_raw_temp;
#endif
extern uint16_t ADC_raw_ntc;
extern uint16_t ADC_smoothed_input;
extern int16_t converted_degrees;
extern uint8_t temperature_offset;
/* Boxcar average of the raw current ADC (getSmoothedCurrent(), control_loop.c).
 * The window is a macro, not a `const uint8_t`: Cortex-M0 has no hardware
 * divide, and a runtime divisor pulls in the __aeabi_uidiv helper on every ADC
 * conversion. As a compile-time constant the divide folds to a multiply-shift. */
#define NUM_CURRENT_READINGS 50
extern uint8_t current_read_index;
extern uint32_t current_total;
extern uint16_t current_readings[NUM_CURRENT_READINGS];
extern char cell_count;
extern uint16_t e_rpm;
extern uint16_t k_erpm;
extern volatile int e_com_time;
extern char bemf_timeout;
extern uint8_t bemf_timeout_happened;
extern uint16_t armed_timeout_count;
extern uint16_t one_khz_loop_counter;
extern uint16_t ledcounter;
extern volatile uint16_t tenkhzcounter;
extern uint16_t telem_ms_count;
extern uint8_t telemetry_interval_ms;
extern volatile uint16_t signaltimeout;
extern char dshot;
extern uint8_t dshotcommand;
extern uint8_t last_dshot_command;
extern char play_tone_flag;
extern char low_rpm_throttle_limit;
extern uint16_t low_rpm_level;
extern uint16_t high_rpm_level;
extern uint16_t throttle_max_at_low_rpm;
extern uint16_t throttle_max_at_high_rpm;
extern char fast_accel;
extern char fast_deccel;
extern uint16_t servo_low_threshold;
extern uint16_t servo_high_threshold;
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;
extern volatile uint8_t PROCESS_ADC_FLAG;
extern int32_t consumed_current;
extern char send_esc_info_flag;
extern uint16_t VOLTAGE_DIVIDER;
extern char LOW_VOLTAGE_CUTOFF;
extern uint16_t low_cell_volt_cutoff;
extern uint16_t low_voltage_count;
/* degrees_celsius, battery_voltage, actual_current: common.h / signal.h */

#endif /* MOTOR_RUNTIME_H_ */
