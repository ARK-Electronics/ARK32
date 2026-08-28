/*
 * faults.h - ESC fault and signal-loss handling
 *
 * Behavior-neutral extract of stuck-rotor, BEMF stall, and signal-timeout
 * policies previously inlined in setInput / tenKhzRoutine / main().
 */
#ifndef FAULTS_H_
#define FAULTS_H_

#include <stdint.h>

/* Optional fault IDs for telemetry / logging. */
typedef enum {
	FAULT_NONE = 0,
	FAULT_STUCK_ROTOR,
	FAULT_SIGNAL_TIMEOUT,
	FAULT_BEMF_STALL,
	/*
	 * Gate-driver nFAULT classes. Not DRV status bits (H-interface has no
	 * SPI). On DRV8350H they come from pin duration + bridge conduction;
	 * on DRV8328 every nFAULT is a latched cut (UVLO vs other is a V hint).
	 */
	FAULT_GD_UVLO,	  /* bus below useful pack voltage */
	FAULT_GD_OCP,	  /* VDS retry pulse, or retry budget exceeded */
	FAULT_GD_OTW,	  /* nFAULT held, bridge still driving (DRV OTW) */
	FAULT_GD_OTSD,	  /* held + dead, MCU die in thermal band (log label) */
	FAULT_GD_UNKNOWN, /* held + dead, no UVLO/thermal signature (GDF etc.) */
} fault_id_t;

/*
 * Classified cause while nFAULT warning/Hi-Z/latch is active.
 * FAULT_NONE when healthy. Best-effort only — not a DRV status register.
 */
fault_id_t faultGateDriverCause(void);

/* Short name for logs ("UVLO", "OCP", "OTW", "OTSD", "nFAULT", or ""). */
const char *faultGateDriverCauseName(fault_id_t cause);

/* Consume a one-shot gate-driver log queued by faultPollGateDriver.
 * Returns 0 if none, FAULT_GD_LOG_WARNING, or FAULT_GD_LOG_ERROR.
 * *cause is FAULT_NONE when there is nothing pending. */
#define FAULT_GD_LOG_NONE 0
#define FAULT_GD_LOG_WARNING 1
#define FAULT_GD_LOG_ERROR 2
uint8_t faultGateDriverConsumeLog(fault_id_t *cause);

/*
 * Stuck-rotor protection (was the top of setInput after throttle map).
 * If bemf_timeout_happened has exceeded the threshold, cut drive and latch.
 * Returns 1 when the fault is active (caller must skip normal input mapping).
 */
uint8_t faultHandleStuckRotorIfNeeded(void);

/*
 * 20 kHz signal-watchdog tick (end of tenKhzRoutine).
 * Implemented as RAM_FUNC so F051 RAM-resident tenKhzRoutine does not
 * pay a flash long-call veneer every tick.
 */
void faultSignalTimeoutTick(void);

/*
 * Main-loop poll: if signaltimeout is large enough, disarm and NVIC_SystemReset.
 * Armed: half second; disarmed: two seconds.
 */
void faultPollSignalTimeout(void);

/*
 * Main-loop BEMF timeout bookkeeping: clear counters under low throttle /
 * early run, and set bemf_timeout threshold from load.
 */
void faultUpdateBemfTimeoutPolicy(void);

/*
 * Main-loop stall: INTERVAL_TIMER_COUNT has run past the fixed stall window
 * while running. Increments bemf_timeout_happened and restarts ZC search.
 */
void faultHandleBemfIntervalStall(void);

/*
 * Episode-level desync rail (leaky bucket across restart cycles).
 *
 * Single desync episodes are already bounded (blind-step cap, miss bucket,
 * demag-late power cut). A bad EEPROM tune can still loop forever:
 * restart → spool → desync spike → restart. Each episode charges this
 * bucket; healthy closed-loop time drains it; zero throttle (pilot
 * intervention) clears it. At the limit, latch ESC_FAULT_STUCK so drive
 * stays off until the fault path clears.
 *
 * charge kinds: jump-check desync, stall-rail trip of an established run
 * (zero_crosses > 100; includes the blind/miss limit handoff, which always
 * follows an established closed loop), and future demag-late saturation.
 */
typedef enum {
	DESYNC_EPISODE_JUMP = 0,
	DESYNC_EPISODE_STALL_RAIL,
	DESYNC_EPISODE_ACQ_FAIL,
} desync_episode_kind_t;

void faultDesyncEpisodeCharge(desync_episode_kind_t kind);

/*
 * Acquisition-phase desync rail.
 *
 * Every other rail in the ZC path is gated on an ESTABLISHED loop: the
 * blind-step deadline needs zero_crosses >= 100 (bemf_zc.c), the BEMF
 * headroom governor needs > 150 (runtime_loop.c), the grind detector and
 * miss bucket both need blind steps to exist at all, and the episode charge
 * itself needs zc_at_desync > 100 (runtimeProcessDesyncCheck). A loop that
 * desyncs BEFORE acquisition completes therefore accumulates no state
 * anywhere - it restarts, reaches zc 20..50, desyncs, and repeats
 * indefinitely with every counter reading zero.
 *
 * Measured on the SITL racer_5inch model at HEAD (dshot 700): ~20 desyncs
 * per second sustained, zero_crosses never past 48, desync_episode_bucket
 * flat at 0 after 38 desyncs, so neither the restart backoff nor the latch
 * ever engages and the rotor limit-cycles at ~2.9k rpm indefinitely.
 *
 * The established-run gate is still correct for the JUMP charge - a few
 * interval jumps while acquiring are normal startup roughness on light
 * motors, and charging them stacks holdoff onto honest starts (the reason
 * for that gate, see faultDesyncEpisodeCharge). What was missing is a rail
 * scoped to the acquisition regime itself. Count early desyncs; a handful
 * is roughness and is forgiven the moment the loop genuinely acquires,
 * while a sustained inability to get past acquisition charges the SAME
 * episode bucket and so inherits the existing backoff and latch.
 */
void faultNoteEarlyDesync(void);
/* 1 kHz: drain bucket when closed-loop is healthy; tick restart holdoff. */
void faultDesyncEpisodeTick1kHz(void);
/* True while a post-desync coast is mandatory (caller must not re-start). */
uint8_t faultDesyncRestartHoldoffActive(void);

/*
 * Stall-rail trips on an ESTABLISHED run this arm cycle - the same
 * zero_crosses > 100 gate the episode charge uses, so the many legitimate
 * kicks a heavy prop needs at low throttle are not counted as errors.
 * Cleared with desync_happened on the armed 0->1 edge (see
 * faultErrorCountReset()).
 *
 * volatile: read from the main loop (telemetry) while the grind rail that
 * forces this trip runs in tenKhzRoutine (ISR context).
 */
extern volatile uint32_t fault_stall_trips;

/*
 * Set once the loop has genuinely established (zero_crosses > 100) while
 * the motor is driving; cleared when running stops and with the other
 * counters in faultErrorCountReset(). Not the ESC `armed` flag — that
 * stays 1 across PX4 arm/disarm.
 *
 * The error-count rails cannot gate on the instantaneous zero_crosses:
 * that counter is reset in ten places, including by the desync and stall
 * handling being measured. An established run that desyncs therefore
 * re-enters the jump check with a count rebuilt from zero (observed: 75 on
 * a steady 900-throttle spool), and a > 100 test would misfile it as an
 * acquisition kick and drop it from esc.Status.error_count / NodeStatus.
 * A latch has the arm-cycle lifetime the DSDL error_count wants, so the
 * gate keeps its intent - a start that NEVER got going never sets it -
 * without depending on a counter the fault path clears.
 *
 * volatile: written from tenKhzRoutine (ISR context), read from the main
 * loop (telemetry).
 */
extern volatile uint8_t fault_run_established;

/*
 * Acquisition-rail episode charges this power cycle ("start resisted":
 * batches of early desyncs that never reached acquisition - see
 * faultNoteEarlyDesync).
 *
 * This counter exists because it is the ONLY part of the episode machinery
 * that does not self-heal: bucket, holdoff and latch all converge back to
 * the configured settings within seconds of an obstruction clearing, so a
 * ground spool that was fought by grass or debris leaves no trace by the
 * time the vehicle is armed. A monotonic count lets an FC (or a human
 * reading telemetry) refuse takeoff on a start that was resisted.
 *
 * Deliberately does NOT influence control: no episode kind changes the
 * configured ramp or any other tuning parameter. See the note in
 * faultDesyncEpisodeCharge for why persistent learned state was removed.
 *
 * Monotonic per power cycle (not cleared on arm) so the pre-takeoff history
 * survives to be read. volatile: written in ISR context (main-loop charge),
 * read from telemetry.
 */
extern volatile uint8_t fault_acq_resist_events;

/*
 * Total hard-error events this arm cycle, for uavcan.equipment.esc.Status
 * error_count (DSDL: "Resets when the motor restarts").
 *
 * Sums the two PRIMITIVE run-killers, which are mutually exclusive:
 *   - desync_happened   : the jump-desync check in runtimeProcessDesyncCheck
 *   - fault_stall_trips : the INTERVAL_TIMER stall rail
 *
 * Both are zeroed on ESC armed 0->1 and on FC ArmingStatus 0->1 via
 * faultErrorCountReset() so a clean re-arm reports error_count 0. Do not
 * clear only one addend - lifetimes must match.
 *
 * Deliberately NOT additional addends, because each already funnels into the
 * stall rail and would double-count one physical failure:
 *   - blind-grind rail (faultDesyncEpisodeTick1kHz) kicks INTERVAL_TIMER to
 *     46000 so the stall rail is guaranteed to trip on the next main pass
 *   - blind-step / miss-bucket limit (bemf_zc.c) hands off the same way
 *   - the episode-rail latch is a CONSEQUENCE of accumulating the above, and
 *     is a state (ESC_FAULT_STUCK), not an independent event
 * Attribution between those sub-causes belongs in the AM32-reserved FlexDebug
 * payload, not in this scalar.
 */
uint32_t faultErrorCount(void);

/* Zero both error_count addends and the established-run latch. Call on
 * ESC armed 0->1 and on FC ArmingStatus 0->1. */
void faultErrorCountReset(void);

/*
 * Gate-driver nFAULT poll. Two chip policies (see faults.c):
 *
 * DRV8350H (ARK 12S CAN): pin duration + bridge conducting *while driven*.
 *   pulse ~8 ms     : VDS auto-retry — keep PWM, count pulses if seen
 *   held + live     : OTW warning — keep PWM (do not Hi-Z on throttle idle)
 *   held + dead     : Hi-Z until pin releases; MCU temp is a log label only
 *                     (not HIZ vs latch). Persistent pin-low at zero throttle
 *                     ENABLE tRST, then latch. Sleep-on-idle is the LATCH tRST.
 *
 * DRV8328 (ARK 4IN1): every nFAULT already Hi-Z's the FETs (no OTW-only,
 * no 8 ms retry). Cut PWM, latch stuck until zero throttle; nSLEEP sleep
 * clears the DRV latch.
 *
 * Call from the main loop (not the 20 kHz path). No-op without the pin.
 */
void faultPollGateDriver(void);

/* 1 kHz timebase for nFAULT duration (called from tenKhzRoutine's 1 kHz
 * branch). No-op without the pin. */
void faultGateDriverTick1kHz(void);

/* 1 while PWM must stay off (DRV Hi-Z wait or latched trip). Not set for
 * OTW warning or the classify window — those keep driving. Sleep
 * (ENABLE/nSLEEP low) does not count: the pin is asserted by VCP UVLO then. */
uint8_t faultGateDriverFaultActive(void);

/* 1 while nFAULT is held and the bridge is still driving (OTW class). */
uint8_t faultGateDriverWarningActive(void);

/* 1 while a Hi-Z wait needs ENABLE high so the pin can auto-clear and so
 * a zero-throttle ENABLE tRST can unlatch GDF. LATCH does not set this:
 * sleep-on-idle is that state's reset. */
uint8_t faultGateDriverKeepAwake(void);

#endif /* FAULTS_H_ */
