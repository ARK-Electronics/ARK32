/*
 * control_loop.c - extracted from main.c (behavior-neutral split)
 */

#include "control_loop.h"
#include "motor_runtime.h"
#include "faults.h"
#include "esc_state.h"
#include "commutation.h"
#include "bemf_zc.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "gate_driver.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "dshot.h"
#include "signal.h"
#include "sounds.h"
#include "hwci_perf.h"
#include "ADC.h"
#include "kiss_telemetry.h"
#ifdef USE_LED_STRIP
#	ifndef NXP
#		include "WS2812.h"
#	endif
#endif
#ifdef USE_CRSF_INPUT
#	include "crsf.h"
#endif
#if DRONECAN_SUPPORT
#	include "DroneCAN/DroneCAN.h"
#endif

int32_t doPidCalculations(struct fastPID *pidnow, int actual, int target)
{
	pidnow->error = actual - target;
	pidnow->integral = pidnow->integral + pidnow->error * pidnow->Ki;
	if (pidnow->integral > pidnow->integral_limit) {
		pidnow->integral = pidnow->integral_limit;
	}
	if (pidnow->integral < -pidnow->integral_limit) {
		pidnow->integral = -pidnow->integral_limit;
	}

	pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
	pidnow->last_error = pidnow->error;

	pidnow->pid_output = pidnow->error * pidnow->Kp + pidnow->integral + pidnow->derivative;

	if (pidnow->pid_output > pidnow->output_limit) {
		pidnow->pid_output = pidnow->output_limit;
	}
	if (pidnow->pid_output < -pidnow->output_limit) {
		pidnow->pid_output = -pidnow->output_limit;
	}
	return pidnow->pid_output;
}

uint16_t getSmoothedCurrent()
{
	// Sample once: ADC_raw_current is written by ADC_DMA_Callback(), which also
	// runs from the DMA ISR. The sum is incremental, so the value stored in the
	// ring and the value added here must be the same one or the running total
	// drifts permanently when the two reads straddle an update.
	const uint16_t sample = ADC_raw_current;

	current_total -= current_readings[current_read_index];
	current_readings[current_read_index] = sample;
	current_total += sample;

	current_read_index++;
	if (current_read_index >= NUM_CURRENT_READINGS) {
		current_read_index = 0;
	}

	return current_total / NUM_CURRENT_READINGS;
}

void setInput()
{
	if (eepromBuffer.bi_direction) {
		if (dshot == 0) {
			if (eepromBuffer.rc_car_reverse) {
				if (newinput > (1000 + (servo_dead_band << 1))) {
					if (forward == eepromBuffer.dir_reversed) {
						adjusted_input = 0;
						//               if (running) {
						prop_brake_active = 1;
						if (return_to_center) {
							forward = 1 - eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, DSHOT_CMD_MAX,
								     DSHOT_MAX_THROTTLE);
					}
				}
				if (newinput < (1000 - (servo_dead_band << 1))) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input =
							map(newinput, 0, 1000 - (servo_dead_band << 1), DSHOT_MAX_THROTTLE, DSHOT_CMD_MAX);
					}
				}
				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
					adjusted_input = 0;
					if (prop_brake_active) {
						prop_brake_active = 0;
						return_to_center = 1;
					}
				}
			} else {
				if (newinput > (1000 + (servo_dead_band << 1))) {
					if (forward == eepromBuffer.dir_reversed) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							forward = 1 - eepromBuffer.dir_reversed;
							zero_crosses = 0;
							bemfZcResetTrend();
							old_routine = 1;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 1000;
						}
					}
					adjusted_input =
						map(newinput, 1000 + (servo_dead_band << 1), 2000, DSHOT_CMD_MAX, DSHOT_MAX_THROTTLE);
				}
				if (newinput < (1000 - (servo_dead_band << 1))) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							zero_crosses = 0;
							bemfZcResetTrend();
							old_routine = 1;
							forward = eepromBuffer.dir_reversed;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 1000;
						}
					}
					adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), DSHOT_MAX_THROTTLE, DSHOT_CMD_MAX);
				}

				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
					adjusted_input = 0;
					brushed_direction_set = 0;
				}
			}
		}
		if (dshot) {
			if (eepromBuffer.rc_car_reverse) {
				if (newinput > DSHOT_3D_REVERSE_MAX) {
					if (forward == eepromBuffer.dir_reversed) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = 1 - eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = ((newinput - DSHOT_3D_FORWARD_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) -
								 reversing_dead_band;
					}
				}
				if (newinput <= DSHOT_3D_REVERSE_MAX && newinput > DSHOT_CMD_MAX) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input =
							((newinput - DSHOT_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) - reversing_dead_band;
					}
				}
				if (newinput < DSHOT_MIN_THROTTLE) {
					adjusted_input = 0;
					if (prop_brake_active) {
						prop_brake_active = 0;
						return_to_center = 1;
					}
				}
			} else {
				if (newinput > DSHOT_3D_REVERSE_MAX) {
					if (forward == eepromBuffer.dir_reversed) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							forward = 1 - eepromBuffer.dir_reversed;
							zero_crosses = 0;
							bemfZcResetTrend();
							old_routine = 1;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 0;
						}
					}
					adjusted_input =
						((newinput - DSHOT_3D_FORWARD_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) - reversing_dead_band;
				}
				if (newinput <= DSHOT_3D_REVERSE_MAX && newinput > DSHOT_CMD_MAX) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							zero_crosses = 0;
							bemfZcResetTrend();
							old_routine = 1;
							forward = eepromBuffer.dir_reversed;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 0;
						}
					}
					adjusted_input = ((newinput - DSHOT_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) - reversing_dead_band;
				}
				if (newinput < DSHOT_MIN_THROTTLE) {
					adjusted_input = 0;
					brushed_direction_set = 0;
				}
			}
		}
	} else {
		adjusted_input = newinput;
	}
#ifndef BRUSHED_MODE
	if (faultHandleStuckRotorIfNeeded()) {
		/* drive cut and latched; skip normal throttle map */
	} else {
#	ifdef FIXED_DUTY_MODE
		input = FIXED_DUTY_MODE_POWER * 20 + DSHOT_CMD_MAX;
#	else
		if (eepromBuffer.use_sine_start) {
			if (adjusted_input < 30) { // dead band ?
				input = 0;
			}
			if (adjusted_input > 30 && adjusted_input < (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
				input = map(adjusted_input, 30, (eepromBuffer.sine_mode_changeover_thottle_level * 20), DSHOT_CMD_MAX, 160);
			}
			if (adjusted_input >= (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
				input = map(adjusted_input, (eepromBuffer.sine_mode_changeover_thottle_level * 20), DSHOT_MAX_THROTTLE, 160,
					    DSHOT_MAX_THROTTLE);
			}
		} else {
			if (use_speed_control_loop) {
				if (drive_by_rpm) {
					target_e_com_time = 60000000 /
							    map(adjusted_input, DSHOT_CMD_MAX, DSHOT_MAX_THROTTLE,
								MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) /
							    (eepromBuffer.motor_poles / 2);
					if (adjusted_input < DSHOT_CMD_MAX) { // dead band ?
						input = 0;
						speedPid.error = 0;
						input_override = 0;
					} else {
						input = (uint16_t)(input_override / 10000); // speed control pid override
						if (input > DSHOT_MAX_THROTTLE) {
							input = DSHOT_MAX_THROTTLE;
						}
						if (input < DSHOT_MIN_THROTTLE) {
							input = DSHOT_MIN_THROTTLE;
						}
					}
				} else {
					input = (uint16_t)(input_override / 10000); // speed control pid override
					if (input > DSHOT_MAX_THROTTLE) {
						input = DSHOT_MAX_THROTTLE;
					}
					if (input < DSHOT_MIN_THROTTLE) {
						input = DSHOT_MIN_THROTTLE;
					}
				}
			} else {
				input = adjusted_input;
			}
		}
#	endif
	}
#endif
#ifndef BRUSHED_MODE
	if (escMaySixStepThrottle()) {
		if (input >= DSHOT_CMD_MAX + (80 * eepromBuffer.use_sine_start)) {
			// Re-entry into six-step happens HERE, at input-frame rate -
			// not in runtimeMotorModeTick. The episode-rail coast (holdoff
			// or latched fault) must gate this branch or the mode tick's
			// running=0 and this restart chatter against each other at
			// kHz rate, pulsing startMotor() commutations through the
			// whole "coast". escIsFault() also covers the latch when
			// stuck_rotor_protection is disabled in EEPROM (the
			// faultHandleStuckRotorIfNeeded gate above honors that flag;
			// the episode rail is independent of it).
			if (!escIsDriving() && !escIsFault() && !faultDesyncRestartHoldoffActive()) {
				/* Wake DRV with IRQs enabled (ENABLE settle is multi-ms on
				 * DRV8350H). comStep gateDriverEnsure() is then a no-op. */
				allOff();
				gateDriverWakeBlocking();
				if (!old_routine) {
					startMotor();
				}
				escEnterRunningOpenLoop();
				last_duty_cycle = min_startup_duty;
			}

			// straight line from (in_min, out_min) to (DSHOT_MAX_THROTTLE, 2000) using a
			// startup computed Q16 slope, avoids calling map() at input rate
			if (eepromBuffer.use_sine_start) {
				duty_cycle_setpoint =
					input >= DSHOT_MAX_THROTTLE ? 2000
					: input <= 137		    ? minimum_duty_cycle + 40
								    : minimum_duty_cycle + 40 +
									      (uint16_t)(((uint32_t)(input - 137) * sine_throttle_duty_slope_q16) >> 16);
			} else {
				duty_cycle_setpoint =
					input >= DSHOT_MAX_THROTTLE ? 2000
					: input <= DSHOT_CMD_MAX
						? minimum_duty_cycle
						: minimum_duty_cycle +
							  (uint16_t)(((uint32_t)(input - DSHOT_CMD_MAX) * throttle_duty_slope_q16) >> 16);
			}

			if (!eepromBuffer.rc_car_reverse) {
				prop_brake_active = 0;
			}
		}

		if (input < DSHOT_CMD_MAX + (80 * eepromBuffer.use_sine_start)) {
			if (play_tone_flag != 0) {
				switch (play_tone_flag) {
					case 1:
						playDefaultTone();
						break;
					case 2:
						playChangedTone();
						break;
					case 3:
						playBeaconTune3();
						break;
					case 4:
						playInputTune2();
						break;
					case 5:
						playDefaultTone();
						break;
				}
				play_tone_flag = 0;
			}

			if (!eepromBuffer.comp_pwm) {
				duty_cycle_setpoint = 0;
				if (!escIsDriving()) {
					old_routine = 1;
					zero_crosses = 0;
					bemfZcResetTrend();
					if (eepromBuffer.brake_on_stop) {
						fullBrake();
					} else {
						if (!prop_brake_active) {
							allOff();
						}
					}
				}
				if (eepromBuffer.rc_car_reverse && prop_brake_active) {
#	ifndef PWM_ENABLE_BRIDGE

					if (dshot == 0)
						prop_brake_duty_cycle = (getAbsDif(1000, newinput) + 1000);
					if (dshot) {
						if (newinput <= DSHOT_3D_REVERSE_MAX && newinput > DSHOT_CMD_MAX)
							prop_brake_duty_cycle =
								((newinput - DSHOT_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) - reversing_dead_band;
						if (newinput > DSHOT_3D_REVERSE_MAX)
							prop_brake_duty_cycle =
								((newinput - DSHOT_3D_FORWARD_MIN_THROTTLE) * 2 + DSHOT_CMD_MAX) -
								reversing_dead_band;
					}
					if (prop_brake_duty_cycle >= (1999)) {
						fullBrake();
					} else {
						proportionalBrake();
					}
#	endif
				}
			} else {
				if (!escIsDriving()) {
					old_routine = 1;
					zero_crosses = 0;
					bemfZcResetTrend();
					bad_count = 0;
					if (eepromBuffer.brake_on_stop > 0) {
						if (!eepromBuffer.use_sine_start) {
#	ifndef PWM_ENABLE_BRIDGE
							if (eepromBuffer.brake_on_stop == 1) {
								prop_brake_duty_cycle = eepromBuffer.drag_brake_strength * 200;
								if (prop_brake_duty_cycle >= (1999)) {
									fullBrake();
								} else {
									/* Flag first so gateDriverPoll will not sleep mid-brake. */
									prop_brake_active = 1;
									proportionalBrake(); /* gateDriverEnsure inside */
								}
							}
#	else
							// todo add proportional braking for pwm/enable style bridge.
#	endif
						}
					} else {
						allOff();
					}
					duty_cycle_setpoint = 0;
				}

				phase_A_position = ((step - 1) * 60) + enter_sine_angle;
				if (phase_A_position > 359) {
					phase_A_position -= 360;
				}
				phase_B_position = phase_A_position + 119;
				if (phase_B_position > 359) {
					phase_B_position -= 360;
				}
				phase_C_position = phase_A_position + 239;
				if (phase_C_position > 359) {
					phase_C_position -= 360;
				}

				if (eepromBuffer.use_sine_start == 1) {
					escToSineStart();
				}
				duty_cycle_setpoint = 0;
			}
		}
		if (!prop_brake_active) {
			if (input >= DSHOT_CMD_MAX && (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection))) {
				if (duty_cycle_setpoint < min_startup_duty) {
					duty_cycle_setpoint = min_startup_duty;
				}
				if (duty_cycle_setpoint > startup_max_duty_cycle) {
					duty_cycle_setpoint = startup_max_duty_cycle;
				}
			}

			if (duty_cycle_setpoint > duty_cycle_maximum) {
				duty_cycle_setpoint = duty_cycle_maximum;
			}

			if (stall_protection_adjust > 0 && input > DSHOT_CMD_MAX) {
				duty_cycle_setpoint = duty_cycle_setpoint + (uint16_t)(stall_protection_adjust / 10000);
			}

			/*
			 * Protection ceilings, composed and applied last.
			 *
			 * min(), not a product: each ceiling is an independent "do
			 * not exceed", so the binding one wins. Scaling them
			 * together would double-derate - thermal at 50% and current
			 * at 50% would give 25%, when 50% is what either one asked
			 * for. min() also makes them safe to run simultaneously
			 * without any cross-coupling: whichever is slack sits at
			 * 2000 and contributes nothing, and the current loop's
			 * integrator cannot wind up behind a binding thermal
			 * ceiling because it is clamped to 2000 either way.
			 *
			 * Neither can step thrust. The thermal ceiling moves with a
			 * filtered die temperature (64 ms IIR over a thermal mass
			 * measured in seconds), the current ceiling is itself an
			 * integrator moving at most 10 duty units per 1 kHz tick,
			 * and the 20 kHz ramp limiter slews applied duty toward
			 * whatever they allow at max_duty_cycle_change in BOTH
			 * directions (control_loop.c ~line 750).
			 *
			 * After the stall-protection boost, not before: a boost that
			 * can push duty back through a protection ceiling is not a
			 * ceiling. This is a deliberate change from upstream, where
			 * the current clamp ran first and the crawler boost could
			 * overshoot it.
			 */
			uint16_t protect_ceiling = thermal_duty_ceiling;
			if (use_current_limit && use_current_limit_adjust < (int16_t)protect_ceiling) {
				protect_ceiling = (uint16_t)use_current_limit_adjust;
			}
			duty_limit_ceiling = protect_ceiling;
			if (duty_cycle_setpoint > protect_ceiling) {
				duty_cycle_setpoint = protect_ceiling;
			}
		}
	}
	// Missed-ZC power cut (BLHeli-style): while commutating blind the
	// rotor position is unknown - bound the energy driven into a possibly
	// wrong phase. On DRV8328 (F051 4IN1) VDS is latched until nSLEEP
	// reset; on DRV8350H (ARK_G431_CAN) the board has a resistor-set VDS
	// limit that auto-retries in 8 ms. FAULT_N is polled in
	// faultPollGateDriver (OTW / short VDS pulses do not cut PWM).
	// Applied here in setInput
	// (on F051 that is the DShot EXTI IRQ, not the main loop) rather than
	// inside the O3 RAM_FUNC 20 kHz body (bench bisect showed adding code
	// there disturbs F051 startup). Pulling last_duty_cycle down as well
	// makes the cut immediate: the 20 kHz slew limiter ramps from
	// last_duty_cycle, so capping only the setpoint would let up to
	// 37.5 ms of ramp-down stand between a blind step and the cap
	// (max_ramp_startup while zero_crosses < 150 - live here, since blind
	// stepping arms at 100). The 20 kHz tick may overwrite last_duty_cycle
	// once before it sees the cap; the next DShot frame re-applies it.
	// zc_demag_run feeds the same cut: consecutive demag-late accepts mean
	// the loop is commutating late off the demag duration (bemf_zc.c) - the
	// rotor position is known but the drive phase is not trusted, and the
	// current reduction is itself the recovery mechanism (shorter demag
	// re-exposes the pre-crossing dwell).
	// BEMF-headroom governor (runtime_loop.c): duty may not exceed the
	// equilibrium duty implied by the live eRPM plus a fixed voltage
	// headroom - bounds slip current / demag duration on heavy props
	// without a per-motor shunt. 2000 while disarmed (estimator not yet
	// confident), so this is a no-op until ~0.3 s of steady running.
	if (!old_routine && running && zero_crosses > 150 && duty_cycle_setpoint > gov_duty_ceiling) {
		duty_cycle_setpoint = gov_duty_ceiling;
	}
	/*
	 * Position-confidence duty authority.
	 *
	 * Replaces a two-step cliff (cap 500 at two untrusted steps, 250 at four
	 * or during a grind hold) with a continuous fade over the same
	 * dead-reckoning budget the restart uses. Authority is full while the
	 * rotor is reporting its position and falls linearly to zero as the
	 * budget is spent, reaching zero exactly where bemf_zc.c hands off to the
	 * stall rail - so there is no step to fall off and no separate threshold
	 * to tune. A handful of blind steps now costs a few percent of throttle
	 * instead of three quarters of it, which is the difference between an
	 * article that flies through a rough patch and one that cannot take off.
	 *
	 * The protective intent is unchanged where it matters: sustained dead
	 * reckoning still walks authority down to nothing, and it does so faster
	 * at low rpm (each blind step is a longer slice of the budget) than at
	 * high rpm, which is the right way round.
	 */
	if (!old_routine && running && (zc_blind_ticks || zc_demag_run)) {
		/* Scale the commanded duty by remaining confidence rather than
		 * clamping to a fixed ceiling: no full-scale constant, and the
		 * reduction is proportional to what was actually asked for.
		 * setInput recomputes the setpoint from the input every call, so
		 * this does not compound. */
		uint32_t stale = zc_blind_ticks;
		/*
		 * Demag-late crossings are arriving, just late, so the loop is
		 * mistimed rather than lost. Same fade, charged per consecutive
		 * late step, but floored at half the budget: unlike dead
		 * reckoning this condition has no restart behind it (bemf_zc.c
		 * deliberately does not escalate it, because a loaded spool-up is
		 * legitimately demag-late for long stretches), so authority has
		 * to bottom out somewhere the motor still turns. The current
		 * reduction is itself the recovery - less current shortens demag
		 * and the pre-crossing dwell reappears - so the fade only has to
		 * be deep enough to break the spiral, not to stop the motor.
		 */
		uint32_t demag = (uint32_t)zc_demag_run * commutation_interval;
		if (demag > BEMF_STALL_TICKS / 2u) {
			demag = BEMF_STALL_TICKS / 2u;
		}
		stale += demag;
		const uint32_t confidence = (stale < BEMF_STALL_TICKS) ? (BEMF_STALL_TICKS - stale) : 0u;
		const uint32_t allowed = ((uint32_t)duty_cycle_setpoint * confidence) / BEMF_STALL_TICKS;
		if (duty_cycle_setpoint > allowed) {
			duty_cycle_setpoint = (uint16_t)allowed;
		}
		/* Pull last_duty_cycle down too: the 20 kHz slew limiter ramps
		 * from it, so capping only the setpoint would leave ramp-down
		 * time standing between the loss of confidence and the cap. */
		if (last_duty_cycle > allowed) {
			last_duty_cycle = (uint16_t)allowed;
		}
	}
#endif
}

RAM_FUNC void tenKhzRoutine()
{ // 20khz as of 2.00 to be renamed
	HWCI_PERF_CTRL_ENTER();
	/*
     * Do not call escReconcileFromFlags() here: it lives in flash and forces
     * a long-call veneer into every 20 kHz tick. Policy predicates below are
     * flag-backed (see esc_state.h); full enum reconcile runs once per main
     * loop in runtimeMotorModeTick().
     */
	duty_cycle = duty_cycle_setpoint;
	tenkhzcounter++;
	ledcounter++;
	ramp_count++;
	one_khz_loop_counter++;
#if defined(USE_DRV_NFAULT)
	{
		/* nFAULT classify timebase must run during sine start too. The
		 * PID 1 kHz block is inside !escInSineStart() and would freeze gd_ms. */
		static uint16_t gd_khz_div;
		if (++gd_khz_div > PID_LOOP_DIVIDER) {
			gd_khz_div = 0;
			faultGateDriverTick1kHz();
		}
	}
#endif
	if (!escIsArmed()) {
		if (cell_count == 0) {
			if (inputSet) {
				if (adjusted_input == 0) {
					armed_timeout_count++;
					if (armed_timeout_count > LOOP_FREQUENCY_HZ) { // one second
						/*
						 * DShot/PWM: zero_input_count confirms many true-zero
						 * frames (noisy edges). DroneCAN: ESCRaw may be only
						 * 10 Hz; holding adjusted_input==0 for this full
						 * second is already the safety gate — do not also
						 * require >30 transfercomplete samples (that needs
						 * ~30 Hz or the 1 kHz reinject path).
						 */
						const uint8_t zero_ok = (zero_input_count > 30)
#if DRONECAN_SUPPORT
									/* Pure CAN mode, or AUTO with a live RawCommand stream. */
									|| (eepromBuffer.input_type == DRONECAN_IN) || DroneCAN_active()
#endif
							;
						if (zero_ok) {
							escToArmedIdle();
#ifdef USE_LED_STRIP
							//	send_LED_RGB(0,0,0);
							delayMicros(1000);
							send_LED_RGB(0, 255, 0);
#endif
#ifdef USE_RGB_LED
							setIndividualRGBLed(0, 1, 0);
#endif
							if ((cell_count == 0) && eepromBuffer.low_voltage_cut_off == 1) {
								cell_count = battery_voltage / 370;
								for (int i = 0; i < cell_count; i++) {
									playInputTune();
									delayMillis(100);
									RELOAD_WATCHDOG_COUNTER();
								}
							} else {
#ifdef MCU_AT415
								play_tone_flag = 4;
#else
								playInputTune();
#endif
							}
							if (!servoPwm && !dshot) {
								eepromBuffer.rc_car_reverse = 0;
							}
						} else {
							inputSet = 0;
							armed_timeout_count = 0;
							escToDisarmed();
						}
					}
				} else {
					armed_timeout_count = 0;
				}
			}
		}
	}

	if (eepromBuffer.telemetry_on_interval) {
		telem_ms_count++;
		if (telem_ms_count > ((telemetry_interval_ms - 1 + eepromBuffer.telemetry_on_interval) * 20)) {
			// telemetry_on_interval = 1 is a boolean, but it can also be 2 or more to indicate an identifier
			// by making the interval just slightly different with an unique identifier, we can guarantee that many ESCs can communicate on just one signal
			// there will be some collisions but not as many as if two ESCs always tried to talk at once.
			send_telemetry = 1;
			telem_ms_count = 0;
		}
	}

#ifndef BRUSHED_MODE

	if (!escInSineStart()) {
#	ifndef CUSTOM_RAMP
		if (escInPollZcDrive()) {
			//				send_LED_RGB(255, 0, 0);
			maskPhaseInterrupts();
			getBemfState();
			if (!zcfound) {
				if (rising) {
					if (bemfcounter > min_bemf_counts_up) {
						zcfound = 1;
						zcfoundroutine();
					}
				} else {
					if (bemfcounter > min_bemf_counts_down) {
						zcfound = 1;
						zcfoundroutine();
					}
				}
			}
		}
#	endif
		if (one_khz_loop_counter > PID_LOOP_DIVIDER) { // 1khz PID loop
			PROCESS_ADC_FLAG = 1;		       // set flag to do new adc read at lower priority
			one_khz_loop_counter = 0;
			faultDesyncEpisodeTick1kHz();
			if (use_current_limit && escIsDriving()) {
				/*
				 * The /10000 is a DEAD ZONE, not just a scale: it is an
				 * integer divide, so the ceiling does not move at all
				 * until the PID output exceeds 10000 - i.e. until the
				 * overshoot exceeds 5000/Kp centiamps. At the shipped
				 * Kp 200 (current_P 100) that is ~0.5 A of resolution;
				 * at Kp 10 it would be 10 A, and the limiter would sit
				 * inert while current ran to twice the setpoint.
				 * Measured, SITL heavy_13inch at an 8 A limit: P=100
				 * holds 7.7 A, P=50 7.1 A, P=25 6.5 A, P=5 14.7 A.
				 * Lower these gains only with a bench number in hand.
				 */
				use_current_limit_adjust -=
					(int16_t)(doPidCalculations(&currentPid, actual_current, eepromBuffer.limits.current * 2 * 100) /
						  10000);
				if (use_current_limit_adjust < minimum_duty_cycle) {
					use_current_limit_adjust = minimum_duty_cycle;
				}
				if (use_current_limit_adjust > 2000) {
					use_current_limit_adjust = 2000;
				}
			} else {
				/*
				 * Release the ceiling whenever the loop that owns it is
				 * not running. It is an integrator with no other reset:
				 * a run that ended while current-limited used to leave
				 * it low, and the NEXT start then found its setpoint
				 * capped below min_startup_duty (the clamp is applied
				 * after the startup floor) until the PID walked it back
				 * up - hundreds of ms of weak spool-up inherited from
				 * the previous run.
				 *
				 * This does release across a desync coast too, so a
				 * restart begins with full authority and the loop needs
				 * ~100 ms to re-bind if the load is still over the
				 * limit. That is the right trade here: this limiter is
				 * a 50 ms-averaged backstop against SUSTAINED current
				 * and never caught that transient anyway, the ramp
				 * limiter still bounds how fast duty can return, and on
				 * ARK_G431_CAN the DRV8350 VDS trip is what covers the
				 * instantaneous case.
				 */
				use_current_limit_adjust = 2000;
			}
			if (eepromBuffer.stall_protection && escIsDriving()) { // this boosts throttle as the rpm gets lower, for crawlers
				// and rc cars only, do not use for multirotors.
				stall_protection_adjust +=
					(doPidCalculations(&stallPid, commutation_interval, stall_protect_target_interval));
				if (stall_protection_adjust > 150 * 10000) {
					stall_protection_adjust = 150 * 10000;
				}
				if (stall_protection_adjust <= 0) {
					stall_protection_adjust = 0;
				}
			}
			if (use_speed_control_loop && escIsDriving()) {
				input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time);
				if (input_override > DSHOT_MAX_THROTTLE * 10000) {
					input_override = DSHOT_MAX_THROTTLE * 10000;
				}
				if (input_override < 0) {
					input_override = 0;
				}
				if (zero_crosses < 100) {
					speedPid.integral = 0;
				}
			}
		}
		if (ramp_count > ramp_divider) {
			ramp_count = 0;
			// Single ramp path for all targets: the legacy compile-time
			// VOLTAGE_BASED_RAMP variant (no target defined it) is
			// subsumed by the runtime voltage compensation - _vcomp are
			// the working copies scaled by measured Vbat at 1 kHz
			// (runtimeTransientGovernorTick): same VOLTS/ms on any pack.
			if (zero_crosses < 150 || last_duty_cycle < 150) {
				max_duty_cycle_change = max_ramp_startup_vcomp;
			} else {
				if (average_interval > 500) {
					max_duty_cycle_change = max_ramp_low_rpm_vcomp;
				} else {
					max_duty_cycle_change = max_ramp_high_rpm_vcomp;
				}
			}
#	ifdef CUSTOM_RAMP
			//         max_duty_cycle_change = eepromBuffer[30];
#	endif
			// NOTE: reactive slew pacing was tried here and does NOT work
			// (bench slewhold-snap-40 / slewhold2-snap-40): the first
			// ~15 ms of a too-fast transient are electrically silent -
			// no rejects, no demag-late, no misses - and then the ZC
			// window closes within ~3 commutations, so every observable
			// distress signal arrives after lock is unrecoverable.
			//
			// There is therefore NO in-flight mechanism that saves a
			// too-fast ramp, and deliberately so: a learned back-off here
			// (each desync episode halving the configured ramp for the
			// rest of the power cycle) was tried, shipped, and reverted -
			// per-ESC learned state is invisible to the FC and makes a
			// multirotor asymmetric. See the note in
			// faultDesyncEpisodeCharge. These limits are what the eeprom
			// says and stay that way; a ramp that outruns the motor is a
			// tuning defect, and repeated desyncs are bounded by the
			// episode latch, not by quietly slowing this ESC down.
			if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
				duty_cycle = last_duty_cycle + max_duty_cycle_change;
			}
			if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
				duty_cycle = last_duty_cycle - max_duty_cycle_change;
			}
		} else {
			duty_cycle = last_duty_cycle;
		}

		/* Inside !escInSineStart(): escIsDriving() ≡ running. */
		if (escIsDriving() && input > DSHOT_CMD_MAX) {
			if (eepromBuffer.variable_pwm) {}
			adjusted_duty_cycle = (((uint32_t)duty_cycle * pwm_to_arr_scale_q16) >> 16) + 1;

		} else {
			if (escInBrake() || prop_brake_active) {
				adjusted_duty_cycle = tim1_arr - (((uint32_t)prop_brake_duty_cycle * pwm_to_arr_scale_q16) >> 16);
			} else {
				if ((eepromBuffer.brake_on_stop == 2) && escIsArmed()) { // require arming for active brake
					comStep(2);
					adjusted_duty_cycle =
						DEAD_TIME + (((uint32_t)eepromBuffer.active_brake_power * pwm_to_arr_scale_q16) >> 16) * 10;
				} else {
					adjusted_duty_cycle = (((uint32_t)duty_cycle * pwm_to_arr_scale_q16) >> 16);
				}
			}
		}
		last_duty_cycle = duty_cycle;
		SET_AUTO_RELOAD_PWM(tim1_arr);
		SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
	}
#endif // ndef brushed_mode
	faultSignalTimeoutTick();
	HWCI_PERF_CTRL_EXIT();
}

void processDshot()
{
	if (compute_dshot_flag == 1) {
		computeDshotDMA();
		compute_dshot_flag = 0;
	}
	if (compute_dshot_flag == 2) {
		if (e_com_time > 65535) { // beyond dshot range
			make_dshot_package(65535);
		} else {
			make_dshot_package(e_com_time);
		}
		compute_dshot_flag = 0;
		return;
	}
	setInput();
}
