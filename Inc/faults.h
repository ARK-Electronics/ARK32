/*
 * faults.h - ESC fault and signal-loss handling
 *
 * Behavior-neutral extract of stuck-rotor, BEMF stall, and signal-timeout
 * policies previously inlined in setInput / tenKhzRoutine / main().
 */
#ifndef FAULTS_H_
#define FAULTS_H_

#include <stdint.h>

/* Optional fault IDs for future telemetry / logging (not yet exposed). */
typedef enum {
	FAULT_NONE = 0,
	FAULT_STUCK_ROTOR,
	FAULT_SIGNAL_TIMEOUT,
	FAULT_BEMF_STALL,
} fault_id_t;

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
 * Single desync episodes are already bounded (the dead-reckoning budget in
 * bemf_zc.c and the demag-late authority fade). A bad EEPROM tune can still loop forever:
 * restart → spool → desync spike → restart. Each episode charges this
 * bucket; healthy closed-loop time drains it; zero throttle (pilot
 * intervention) clears it. At the limit, latch ESC_FAULT_STUCK so drive
 * stays off until the fault path clears.
 *
 * charge kinds: jump-check desync, stall-rail trip of an established run
 * (zero_crosses > 100; includes the dead-reckoning budget handoff in
 * bemf_zc.c, which always follows an established closed loop), and future
 * demag-late saturation.
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
 * headroom governor needs > 150 (runtime_loop.c), the dead-reckoning budget
 * needs blind steps to exist at all, and the episode charge
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
 * volatile: read from the main loop (telemetry) while the ZC path that
 * forces this trip runs in ISR context.
 */
extern volatile uint32_t fault_stall_trips;

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
 * Both are zeroed on arm (0->1) via faultErrorCountReset() so a clean re-arm
 * reports error_count 0; the flight controller then sees only faults that
 * occurred after this arm. Do not clear only one addend - lifetimes must match.
 *
 * Deliberately NOT additional addends, because each already funnels into the
 * stall rail and would double-count one physical failure:
 *   - the dead-reckoning budget (bemf_zc.c) kicks INTERVAL_TIMER past
 *     BEMF_STALL_TICKS so the stall rail trips on the next main pass
 *   - the episode-rail latch is a CONSEQUENCE of accumulating the above, and
 *     is a state (ESC_FAULT_STUCK), not an independent event
 * Attribution between those sub-causes belongs in the AM32-reserved FlexDebug
 * payload, not in this scalar.
 */
uint32_t faultErrorCount(void);

/* Zero both error_count addends. Call only on the armed 0->1 edge. */
void faultErrorCountReset(void);

#endif /* FAULTS_H_ */
