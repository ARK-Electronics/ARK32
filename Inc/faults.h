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
 * while running - no zero-cross evidence for BEMF_STALL_TICKS, including the
 * dead-reckoning budget handoff from bemf_zc.c, which kicks INTERVAL_TIMER
 * here when blind stepping has run out of position.
 *
 * This mirrors upstream AM32 exactly (Src/main.c, "INTERVAL_TIMER_COUNT >
 * 45000"): increment bemf_timeout_happened, hand back to the poll-ZC path and
 * re-acquire with the bridge still driving. It does NOT cut power and does not
 * stop the motor at flight throttle - losing sync is not the same event as
 * losing the rotor, and a coast on a quad is a dropped corner.
 *
 * The stop lives in faultHandleStuckRotorIfNeeded: repeated stalls with no
 * healthy run in between mean poll mode is not finding crossings either, i.e.
 * the rotor genuinely is not turning, and driving it is what cooks FETs.
 * bemf_timeout_happened is the counter shared by both, throttle-scaled and
 * cleared by faultUpdateBemfTimeoutPolicy - again upstream's policy verbatim.
 */
void faultHandleBemfIntervalStall(void);

/*
 * Acquisition-phase desync counter ("start resisted") - OBSERVABILITY ONLY.
 *
 * A loop that desyncs BEFORE acquisition completes accumulates no state
 * anywhere: every rail in the ZC path is gated on an established loop (the
 * blind-step deadline needs zero_crosses >= 100 in bemf_zc.c, the BEMF
 * headroom governor needs > 150 in runtime_loop.c, the dead-reckoning budget
 * needs blind steps to exist at all). It restarts, reaches zc 20..50,
 * desyncs, and repeats with every counter reading zero.
 *
 * Measured on the SITL racer_5inch model (dshot 700): ~20 desyncs per second
 * sustained, zero_crosses never past 48.
 *
 * Batches of early desyncs bump fault_acq_resist_events so an FC can refuse
 * takeoff on a start that was fought (grass, debris, a wrong prop match). A
 * handful is normal startup roughness and is forgiven the moment the loop
 * genuinely acquires. It deliberately changes NOTHING about control - see the
 * note on fault_acq_resist_events.
 */
void faultNoteEarlyDesync(void);
/* 1 kHz: forgive the early-desync batch once the loop is solidly established. */
void faultAcqResistTick1kHz(void);

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
 * This counter exists because nothing else survives the event: the loop
 * self-heals the moment an obstruction clears, so a ground spool that was
 * fought by grass or debris leaves no trace by the time the vehicle is armed.
 * A monotonic count lets an FC (or a human reading telemetry) refuse takeoff
 * on a start that was resisted.
 *
 * Deliberately does NOT influence control - it is a counter and nothing else.
 * Neither this nor any other fault path changes the configured ramp or any
 * other tuning parameter; the ESC either behaves as configured or stops.
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
 *   - the stuck-rotor latch is a CONSEQUENCE of accumulating the above, and
 *     is a state (ESC_FAULT_STUCK), not an independent event
 * Attribution between those sub-causes belongs in the AM32-reserved FlexDebug
 * payload, not in this scalar.
 */
uint32_t faultErrorCount(void);

/* Zero both error_count addends. Call only on the armed 0->1 edge. */
void faultErrorCountReset(void);

#endif /* FAULTS_H_ */
