/*
 * DroneCAN.c - support for DroneCAN protocol for ESC control and telemetry
 */

#include "targets.h"

// #pragma GCC optimize("O0")

#if DRONECAN_SUPPORT

#	include "peripherals.h"
#	include "serial_telemetry.h"
#	include "common.h"
#	include "signal.h"
#	include "version.h"
#	include "eeprom.h"
#	include "faults.h"
#	include <stdarg.h>
#	include <stdio.h>
#	include <string.h>
#	include "sys_can.h"
#	include <canard.h>
#	include "phaseouts.h"
#	include "functions.h"
#	include "filter.h"
#	include "debug_uart.h"
#	include "settings.h"
#	include "motor_runtime.h"
#	include "esc_state.h"

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#	include "dsdl_generated/dronecan_msgs.h"

#	if CANARD_ENABLE_CANFD
static bool dronecan_tx_canfd;

static void dc_note_rx_frame(const CanardCANFrame *f)
{
	/* Latch CAN FD TX on the first FD frame. Classic DNA/NodeStatus on
	 * an FD bus must not flip Status and telemetry back to classic. */
	if (f->canfd) {
		dronecan_tx_canfd = true;
	}
}

#		define DC_BROADCAST(...) canardBroadcast(__VA_ARGS__, dronecan_tx_canfd)
#		define DC_RESPOND(...) canardRequestOrRespond(__VA_ARGS__, dronecan_tx_canfd)
#	else
static void dc_note_rx_frame(const CanardCANFrame *f)
{
	(void)f;
}
#		define DC_BROADCAST canardBroadcast
#		define DC_RESPOND canardRequestOrRespond
#	endif

#	if CANARD_ENABLE_TAO_OPTION
#		define DC_ENCODE(fn, pkt, buf) fn((pkt), (buf), !dronecan_tx_canfd)
#	else
#		define DC_ENCODE(fn, pkt, buf) fn((pkt), (buf))
#	endif

#	ifndef PREFERRED_NODE_ID
#		define PREFERRED_NODE_ID 0
#	endif

#	ifndef CANARD_POOL_SIZE
#		define CANARD_POOL_SIZE 4096
#	endif

// use set input at 1kHz
#	define TARGET_PERIOD_US 1000U

static CanardInstance canard;
static uint8_t canard_memory_pool[CANARD_POOL_SIZE];

struct CANStats canstats;

static bool dronecan_armed;
static bool done_startup;

#	define APP_SIGNATURE_MAGIC1 0x68f058e6
#	define APP_SIGNATURE_MAGIC2 0xafcee5a0

#	ifndef DRONECAN_HW_VERSION_MAJOR
#		define DRONECAN_HW_VERSION_MAJOR 0
#		define DRONECAN_HW_VERSION_MINOR 71
#	endif

/*
  application signature, filled in by set_app_signature.py

  used: nothing in C ever reads this, so LTO drops it as dead exactly the way it
  drops .file_name, leaving the section empty. No LTO'd CAN target exists today
  (SITL is native and opts out), so this is here to stop adding one from
  silently shipping an image with no signature.
 */
const struct {
	uint32_t magic1;
	uint32_t magic2;
	uint32_t fwlen; // total fw length in bytes
	uint32_t crc1;	// crc32 up to start of app_signature
	uint32_t crc2;	// crc32 from end of app_signature to end of fw
	char mcu[16];
	uint32_t unused[2];
} app_signature __attribute__((used)) AM32_FLASH_SECTION(".app_signature") = {
	.magic1 = APP_SIGNATURE_MAGIC1,
	.magic2 = APP_SIGNATURE_MAGIC2,
	.fwlen = 0,
	.crc1 = 0,
	.crc2 = 0,
	.mcu = AM32_MCU,
};

/*
  PX4 APDescriptor (APDesc00). PX4 copies any SD-card-root .bin whose first
  1 KiB contains this block to /ufw/<board_id>.bin and flashes nodes whose
  GetNodeInfo hardware_version matches board_id = (hw_major << 8) | hw_minor.

  CRCs / image_size / git_hash are filled by scripts/px4_uavcan_image.py
  *before* set_app_signature.py so the AM32 bootloader CRC stays valid.
  GetNodeInfo reads image_crc back so PX4 does not re-flash every boot.
  volatile: those fields are 0 at compile time; without it LTO folds
  GetNodeInfo into literal zeros and PX4 sees image_crc == 0 forever.
 */
#	define PX4_APDESC_SIGNATURE_0 0x40
#	define PX4_APDESC_SIGNATURE_1 0xa2
#	define PX4_APDESC_SIGNATURE_2 0xe4
#	define PX4_APDESC_SIGNATURE_3 0xf1
#	define PX4_APDESC_SIGNATURE_4 0x64
#	define PX4_APDESC_SIGNATURE_5 0x68
#	define PX4_APDESC_SIGNATURE_6 0x91
#	define PX4_APDESC_SIGNATURE_7 0x06

struct px4_app_descriptor {
	uint8_t signature[8];
	union {
		uint64_t image_crc;
		struct {
			uint32_t crc32_block1;
			uint32_t crc32_block2;
		};
	};
	uint32_t image_size;
	uint32_t git_hash;
	uint8_t major_version;
	uint8_t minor_version;
	uint16_t board_id;
	uint8_t reserved[8];
} __attribute__((packed));

_Static_assert(sizeof(struct px4_app_descriptor) == 36, "PX4 APDescriptor must be 36 bytes");

const volatile struct px4_app_descriptor px4_app_descriptor __attribute__((used, aligned(8))) AM32_FLASH_SECTION(".px4_app_descriptor") = {
	.signature = {PX4_APDESC_SIGNATURE_0, PX4_APDESC_SIGNATURE_1, PX4_APDESC_SIGNATURE_2, PX4_APDESC_SIGNATURE_3,
		      PX4_APDESC_SIGNATURE_4, PX4_APDESC_SIGNATURE_5, PX4_APDESC_SIGNATURE_6, PX4_APDESC_SIGNATURE_7},
	.image_crc = 0,
	.image_size = 0,
	.git_hash = 0,
	.major_version = VERSION_MAJOR,
	.minor_version = VERSION_MINOR,
	.board_id = (uint16_t)((DRONECAN_HW_VERSION_MAJOR << 8) | DRONECAN_HW_VERSION_MINOR),
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

enum VarType {
	T_BOOL = 0,
	T_UINT8,
	T_UINT16,
	T_STRING,
};

#	define PACKED __attribute__((__packed__))

/*
  structure sent with FlexDebug
 */
static struct PACKED {
	uint8_t version;
	uint32_t commutation_interval;
	uint16_t num_commands;
	uint16_t num_input;
	uint16_t rx_errors;
	uint16_t rxframe_error;
	int32_t rx_ecode;
	uint8_t auto_advance_level;
	// version 2 fields (upstream am32 / PR#394)
	uint16_t duty_cycle;	     // demanded duty, 0..2000
	uint16_t duty_cycle_maximum; // low-rpm/temperature duty clamp
	uint16_t adjusted_input;     // input after mode mapping, 0..2047
	uint16_t adc_raw_current;    // current sense ADC counts
	uint16_t adc_raw_volts;	     // voltage sense ADC counts
	uint8_t flags;		     // bit0 armed, bit1 running, bit2 stepper_sine
	/* ARK append-only after upstream v2: "start resisted" count
	 * (faults.h). FC-side visibility of a resisted ground spool BEFORE
	 * takeoff — every other counter in the episode machinery self-heals. */
	uint8_t acq_resist_events;
} debug1;

static void can_printf(const char *fmt, ...);
static uint32_t millis32(void);

// some convenience macros
#	define MIN(a, b) ((a) < (b) ? (a) : (b))
#	define C_TO_KELVIN(temp) (temp + 273.15f)
#	define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/*
  access to settings from main.c
 */
extern uint16_t motor_kv;
extern volatile char armed;
extern volatile uint32_t commutation_interval;
extern uint8_t auto_advance_level;
extern uint16_t low_cell_volt_cutoff;
extern volatile uint16_t duty_cycle;
extern volatile uint16_t duty_cycle_maximum;
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_volts;
extern volatile char stepper_sine;

static uint16_t last_can_input;
static uint64_t last_heartbeat_us;
static struct {
	uint32_t sum;
	uint32_t count;
} current;

extern void saveEEpromSettings(void);
extern void loadEEpromSettings(void);
static void set_input(uint16_t input);

/*
  the set of parameters to present to the user over DroneCAN
*/
struct parameter {
	char *name;
	enum VarType vtype;
	uint16_t min_value;
	uint16_t max_value;
	uint16_t default_value;
	void *ptr;
};

#	include "eeprom_params.c"

/*
  get settings from eeprom
*/
static void load_settings(void)
{
	/*
      run through parameters checking for those in the eepromBuffer
      structure. For those parameters reset to default if out of
      range. This allows the use of a defaults array that does not
      include the CAN parameters
     */
	for (uint8_t i = 0; i < ARRAY_SIZE(parameters); i++) {
		const struct parameter *p = &parameters[i];
		/*
          only accept settings in range
         */
		switch (p->vtype) {
			case T_BOOL:
			case T_UINT8: {
				uint8_t *pvalue = (uint8_t *)p->ptr;
				/* uint16: CURRENT_P/D advertise 510 (Kp = byte * 2). */
				uint16_t max_value = p->max_value;
				if (pvalue == &eepromBuffer.current_limit) {
					max_value = max_value / 2;
				}
				if (pvalue == &eepromBuffer.timing_advance) {
					max_value = max_value + 10;
				}
				if (*pvalue < p->min_value || *pvalue > max_value) {
					*pvalue = p->default_value;
				}
				break;
			}
			case T_UINT16:
			case T_STRING:
				break;
		}
	}
}

/*
  save settings to flash
 */
static void save_settings(void)
{
	saveEEpromSettings();
	can_printf("saved settings");
	debugUartPrint("param: saved settings\r\n");
}

/*
  pending deferred save state. Param sets only mark settings dirty;
  the main DroneCAN_update loop coalesces bursts into a single flash
  write once the bus has been quiet for SETTINGS_SAVE_QUIET_MS.
  (from am32-firmware/AM32#359)
 */
#	define SETTINGS_SAVE_QUIET_MS 500
static struct {
	bool dirty;
	uint32_t last_change_ms;
} pending_save;

static void mark_settings_dirty(void)
{
	pending_save.dirty = true;
	pending_save.last_change_ms = millis32();
}

/*
  hold our node status as a static variable. It will be updated on any errors
*/
static struct uavcan_protocol_NodeStatus node_status;

static bool safe_to_write_settings(void)
{
	return !running || newinput == 0;
}

/*
  simple 16 bit random number generator
*/
static uint16_t get_random16(void)
{
	static uint32_t m_z = 1234;
	static uint32_t m_w = 76542;
	m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
	m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
	return ((m_z << 16) + m_w) & 0xFFFF;
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific

  NOTE: this should be in functions.c
*/
static uint64_t micros64(void)
{
	static uint64_t base_us;
	static uint16_t last_cnt;
	// the static state must be updated atomically, this is called from
	// both interrupt handlers and the main loop. Save and restore
	// PRIMASK so a caller's critical section is not ended early
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
#	ifdef ARTERY
	uint16_t cnt = UTILITY_TIMER->cval;
#	else
	uint16_t cnt = UTILITY_TIMER->CNT;
#	endif
	if (cnt < last_cnt) {
		base_us += 0x10000;
	}
	last_cnt = cnt;
	const uint64_t ret = base_us + cnt;
	if (!primask) {
		__enable_irq();
	}
	return ret;
}

/*
  get monotonic time in milliseconds since startup
*/
static uint32_t millis32(void)
{
	return micros64() / 1000ULL;
}

/*
  default settings, based on public/assets/eeprom_default.bin in AM32 configurator
  update to 2.19 default
 */
/* The erase skeleton is generated from schema/eeprom.json: the 48 byte AM32
 * configurator image, byte-for-byte, so schema/eeprom-defaults.hex and the
 * factory pages stay what the configurator expects.
 *
 * It is NOT the protection envelope this product ships. Upstream carries
 * 0x8d (141) and 0x66 (102) at bytes 43/44, both just OUTSIDE the ranges
 * settings.c arms (70..140 C, 1..100 raw = 2..200 A), so the stock image
 * silently disables both limiters - and a DroneCAN param ERASE memcpy's it
 * back over the page. Leaving it at that would mean "restore defaults"
 * quietly turns the protections off on a shipped ESC: the one config change
 * nobody would think to re-check. Byte 46 (input_type) is upstream's DShot,
 * which on a CAN-only board would take the ESC off the bus.
 *
 * apply_post_skeleton_defaults() therefore re-applies this target's shipped
 * values from the TARGET_DEFAULT_* macros in targets.h after the memcpy. It
 * is per-target on purpose: SITL keeps the generic defaults, ARK_G431_CAN
 * gets 105 C over a 15 C band and 100 raw = 200 A.
 *
 * 105 C: foldback onset, in line with professional 12S practice (APD/T-Motor
 * derate around 110) and inside the G4 die sensor's factory calibration span
 * (30..110 C). CAVEAT worth carrying: those vendors read an NTC on the power
 * stage, this reads the MCU die, and the die-to-FET delta on this board has
 * not been benched - if it turns out large, the onset belongs lower.
 * 100 raw = 200 A: the highest the enable gate accepts, ~80% of the ARK 12S
 * shunt rating, and above anything a healthy craft draws - a backstop, not a
 * flight limiter.
 *
 * Agreement between the macros and each product JSON is gated by
 * scripts/check-erase-defaults.py in CI.
 *
 * NOTE the skeleton is still upstream's everywhere else, so an erase also
 * reverts e.g. byte 5 max_ramp to 0xa0 (16 %/ms) rather than the ARK 2 %/ms.
 * That is a pre-existing hole in the erase path and wants its own fix.
 */
#	include "eeprom_defaults.h"

/*
  Bytes an erase has to restore that the 48 byte configurator skeleton either
  gets wrong for this product (43/44/46) or does not reach at all (184).
  Keep the skeleton itself untouched so it stays configurator-compatible.
 */
static void apply_post_skeleton_defaults(void)
{
	eepromBuffer.temperature_limit = TARGET_DEFAULT_TEMPERATURE_LIMIT;
	eepromBuffer.current_limit = TARGET_DEFAULT_CURRENT_LIMIT;
	eepromBuffer.can_temp_derate_band = TARGET_DEFAULT_TEMP_DERATE_BAND;
	/* AUTO: first available of DShot/PWM, with DroneCAN prioritised while the
	 * RawCommand stream is live (see DroneCAN_active). Never leave a CAN-only
	 * board on upstream's DShot default. */
	eepromBuffer.input_type = 0;
}

#	ifdef MCU_SITL
/* Seed a missing SITL eeprom file the way a factory-flashed ESC comes up:
 * the configurator skeleton with this target's protection envelope applied,
 * i.e. exactly what a DroneCAN param erase leaves behind. Seeding the bare
 * skeleton instead would boot SITL with the limiters disabled. */
const uint8_t *DroneCAN_default_settings(unsigned *len);
const uint8_t *DroneCAN_default_settings(unsigned *len)
{
	static uint8_t seeded[EEPROM_SIZE];
	static uint8_t built;
	if (!built) {
		EEprom_t saved = eepromBuffer;
		memset(eepromBuffer.buffer, 0xff, sizeof(eepromBuffer.buffer));
		memcpy(eepromBuffer.buffer, default_settings, sizeof(default_settings));
		apply_post_skeleton_defaults();
		memcpy(seeded, eepromBuffer.buffer, sizeof(seeded));
		eepromBuffer = saved;
		built = 1;
	}
	*len = sizeof(seeded);
	return seeded;
}
#	endif

static const uint8_t advance_level_v3_remap[] = {
	0x00, 0x08, 0x10, 0x16 // old values 0-3 map to new values 0,8,16,22
};

/*
 * Broadcast uavcan.protocol.debug.LogMessage.
 * level: UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_*
 */
static void can_log(uint8_t level, const char *fmt, ...)
{
	struct uavcan_protocol_debug_LogMessage pkt;
	memset(&pkt, 0, sizeof(pkt));

	pkt.level.value = level;
	/* Short fixed source so GUI tools group ESC logs. */
	static const char src[] = "AM32";
	pkt.source.len = (uint8_t)(sizeof(src) - 1u);
	memcpy(pkt.source.data, src, pkt.source.len);

	uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
	va_list ap;
	va_start(ap, fmt);
	uint32_t n = vsnprintf((char *)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
	va_end(ap);
	pkt.text.len = (uint8_t)MIN(n, sizeof(pkt.text.data));

	uint32_t len = DC_ENCODE(uavcan_protocol_debug_LogMessage_encode, &pkt, buffer);
	static uint8_t logmsg_transfer_id;

	DC_BROADCAST(&canard, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE, UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID, &logmsg_transfer_id,
		     CANARD_TRANSFER_PRIORITY_LOW, buffer, len);
}

/* printf-style LogMessage at INFO (param/save notices, etc.). */
static void can_printf(const char *fmt, ...)
{
	/* Wrap through can_log so source/level stay consistent. */
	char buf[90];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "%s", buf);
}

/*
 * One-shot fault LogMessages: gate-driver consume queue, or stuck-rotor
 * latch. Prefer a gate-driver error when both rise together because its
 * latch also forces ESC_FAULT_STUCK. Warnings do not cause a stuck latch.
 */
static void DroneCAN_pollFaultLogMessages(void)
{
	static uint8_t prev_stuck;
	fault_id_t cause = FAULT_NONE;
	const uint8_t gd_level = faultGateDriverConsumeLog(&cause);
	const uint8_t stuck = (uint8_t)(escGetState() == ESC_FAULT_STUCK);

	if (gd_level == FAULT_GD_LOG_WARNING) {
		if (cause == FAULT_GD_OCP) {
			can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, "nFAULT retry");
		} else if (cause == FAULT_GD_OTW) {
			can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, "nFAULT OTW");
		} else {
			can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, "nFAULT");
		}
	} else if (gd_level == FAULT_GD_LOG_ERROR) {
		switch (cause) {
			case FAULT_GD_UVLO:
				can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "nFAULT UVLO");
				break;
			case FAULT_GD_OCP:
				can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "nFAULT OCP");
				break;
			case FAULT_GD_OTSD:
				can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "nFAULT OTSD");
				break;
			default:
				can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "nFAULT");
				break;
		}
	}
	if (gd_level != FAULT_GD_LOG_ERROR && stuck && !prev_stuck) {
		can_log(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "stuck");
	}

	prev_stuck = stuck;
}

/* Map ESC / gate state into NodeStatus.health for 1 Hz NodeStatus. */
static uint8_t DroneCAN_nodeHealth(void)
{
	const esc_state_t st = escGetState();

	/* Cannot drive: stuck latch, gate-driver latch, or LVC. */
	if (st == ESC_FAULT_STUCK || faultGateDriverFaultActive() || st == ESC_FAULT_LVC) {
		return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL;
	}
	/* Signal loss is usually followed by reset — still an error while held. */
	if (st == ESC_FAULT_SIGNAL) {
		return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
	}
	/*
	 * WARNING: held nFAULT, established hard trips
	 * (faultErrorCount: stall + established jump desync), or post-desync
	 * holdoff. Acquisition roughness and commanded-stop coast stay OK.
	 */
	if (faultGateDriverWarningActive() || faultErrorCount() > 0 || faultDesyncRestartHoldoffActive()) {
		return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING;
	}
	return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
}

/*
  handle parameter GetSet request
*/
static void handle_param_GetSet(CanardInstance *ins, CanardRxTransfer *transfer)
{
	struct uavcan_protocol_param_GetSetRequest req;
	if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
		return;
	}

	volatile const struct parameter *p = NULL;
	if (req.name.len != 0) {
		for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++) {
			if (req.name.len == strlen(parameters[i].name) &&
			    strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
				p = &parameters[i];
				break;
			}
		}
	} else if (req.index < ARRAY_SIZE(parameters)) {
		p = &parameters[req.index];
	}
	if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
		const char last_dir_reversed = eepromBuffer.direction_reversed;
		const char last_bi_direction = eepromBuffer.bidirectional_mode;
		int32_t set_log_val = 0;
		uint8_t set_log_is_str = 0;

		/*
	  a parameter set command
	*/
		switch (p->vtype) {
			case T_UINT8: {
				uint8_t *ptr8 = (uint8_t *)p->ptr;
				if (ptr8 == &eepromBuffer.current_limit || ptr8 == &eepromBuffer.current_pid_p ||
				    ptr8 == &eepromBuffer.current_pid_d) {
					*ptr8 = req.value.integer_value / 2;
					set_log_val = (int32_t)req.value.integer_value; /* user-facing amps */
				} else {
					*ptr8 = req.value.integer_value;
					set_log_val = (int32_t)req.value.integer_value;
				}
				if (ptr8 == &eepromBuffer.timing_advance) {
					*ptr8 = req.value.integer_value + 10; // adjust for advance level offset for eeprom v3
					set_log_val = (int32_t)req.value.integer_value;
				}
				if (ptr8 == &eepromBuffer.motor_poles) {
					applyMotorIdentitySettings();
				}
				break;
			}
			case T_UINT16: {
				uint16_t *ptr16 = (uint16_t *)p->ptr;
				*ptr16 = req.value.integer_value;
				set_log_val = (int32_t)req.value.integer_value;
				if (ptr16 == &motor_kv) {
					eepromBuffer.motor_kv = (uint8_t)((*(uint16_t *)p->ptr - 20) / 40);
					/* Advance / RPM envelopes were computed at boot from
					 * old kV — refresh them now (AM32 identity tables). */
					applyMotorIdentitySettings();
				} else if (ptr16 == &low_cell_volt_cutoff) {
					eepromBuffer.low_voltage_threshold = (uint8_t)(*ptr16 - 250);
				}
				break;
			}
			case T_BOOL:
				*(uint8_t *)p->ptr = req.value.boolean_value ? 1 : 0;
				set_log_val = req.value.boolean_value ? 1 : 0;
				break;
			case T_STRING:
				if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE) {
					if (p->ptr == (void *)eepromBuffer.startup_melody) {
						for (size_t i = 0; i < sizeof(eepromBuffer.startup_melody); i++) {
							if (i < req.value.string_value.len) {
								eepromBuffer.startup_melody[i] = req.value.string_value.data[i];
							} else {
								eepromBuffer.startup_melody[i] = 0xFF;
							}
						}
						set_log_is_str = 1;
					}
				}
				break;
			default:
				return;
		}

#	ifdef USE_DEBUG_UART
		if (set_log_is_str) {
			debugUartPrintf("param: %s=<string len=%u>\r\n", p->name, (unsigned)req.value.string_value.len);
		} else {
			debugUartPrintf("param: %s=%ld\r\n", p->name, (long)set_log_val);
		}
		if (p->ptr == (void *)&motor_kv || p->ptr == (void *)&eepromBuffer.motor_poles) {
			debugUartPrintf("param: applied kv=%u poles=%u erpm_lo=%u erpm_hi=%u adv_q12=%u\r\n", (unsigned)motor_kv,
					(unsigned)eepromBuffer.motor_poles, (unsigned)low_rpm_level, (unsigned)high_rpm_level,
					(unsigned)advance_erpm_scale_q12);
		}
#	else
		(void)set_log_val;
		(void)set_log_is_str;
#	endif

		if (last_dir_reversed != eepromBuffer.direction_reversed || last_bi_direction != eepromBuffer.bidirectional_mode) {
			// make dir_reversed and bi_direction change work without
			// reboot
			forward = 1 - eepromBuffer.direction_reversed;
			running = 0;
			armed = 0;
			set_input(0);
		}

		/* Coalesce flash writes: deferred save in DroneCAN_update (AM32#359). */
		mark_settings_dirty();
	}

	/*
      for both set and get we reply with the current value
    */
	struct uavcan_protocol_param_GetSetResponse pkt;
	memset(&pkt, 0, sizeof(pkt));

	if (p != NULL) {
		const uint32_t eindex = (uint32_t)(((const uint8_t *)p->ptr) - &eepromBuffer.buffer[0]);
		switch (p->vtype) {
			case T_UINT8:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.value.integer_value = *(uint8_t *)p->ptr;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				if (eindex < sizeof(default_settings)) {
					pkt.default_value.integer_value = default_settings[eindex];
				} else {
					pkt.default_value.integer_value = p->default_value;
				}
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
				pkt.max_value.integer_value = p->max_value;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
				pkt.min_value.integer_value = p->min_value;

				// special case scaling
				if ((uint8_t *)p->ptr == &eepromBuffer.current_limit || (uint8_t *)p->ptr == &eepromBuffer.current_pid_p ||
				    (uint8_t *)p->ptr == &eepromBuffer.current_pid_d) {
					pkt.default_value.integer_value *= 2;
					pkt.value.integer_value *= 2;
				}
				if ((uint8_t *)p->ptr == &eepromBuffer.timing_advance) {
					// automatically remap old values
					if ((uint64_t)pkt.value.integer_value < sizeof(advance_level_v3_remap)) {
						pkt.value.integer_value = advance_level_v3_remap[pkt.value.integer_value];
					}
					// adjust for advance level offset for eeprom v3
					if (pkt.value.integer_value >= 10) {
						pkt.value.integer_value -= 10;
					}
				}
				break;
			case T_UINT16:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.value.integer_value = *(uint16_t *)p->ptr;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.default_value.integer_value = p->default_value;
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
				pkt.max_value.integer_value = p->max_value;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
				pkt.min_value.integer_value = p->min_value;
				break;
			case T_STRING:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
				if (p->ptr == (void *)eepromBuffer.startup_melody) {
					pkt.value.string_value.len = sizeof(eepromBuffer.startup_melody);
					if (pkt.value.string_value.len > sizeof(pkt.value.string_value.data)) {
						pkt.value.string_value.len = sizeof(pkt.value.string_value.data);
					}
					for (size_t i = 0; i < pkt.value.string_value.len; i++) {
						pkt.value.string_value.data[i] = eepromBuffer.startup_melody[i];
					}
				}
				break;
			case T_BOOL:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
				pkt.value.boolean_value = (*(uint8_t *)p->ptr) ? true : false;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
				if (eindex < sizeof(default_settings)) {
					pkt.default_value.boolean_value = !!default_settings[eindex];
				} else {
					pkt.default_value.boolean_value = !!p->default_value;
				}
				break;
			default:
				return;
		}
		pkt.name.len = strlen(p->name);
		strcpy((char *)pkt.name.data, p->name);
	}

	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
	uint16_t total_size = DC_ENCODE(uavcan_protocol_param_GetSetResponse_encode, &pkt, buffer);

	DC_RESPOND(ins, transfer->source_node_id, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_ID,
		   &transfer->transfer_id, transfer->priority, CanardResponse, &buffer[0], total_size);
}

/*
  handle parameter executeopcode request
*/
static void handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer)
{
	struct uavcan_protocol_param_ExecuteOpcodeRequest req;
	if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
		return;
	}
	struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
	memset(&pkt, 0, sizeof(pkt));

	pkt.ok = false;

	if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
		if (!safe_to_write_settings()) {
			can_printf("No erase while running");
		} else {
			can_printf("resetting to defaults");
			memset(eepromBuffer.buffer, 0xff, sizeof(eepromBuffer.buffer));
			memcpy(eepromBuffer.buffer, default_settings, sizeof(default_settings));
			apply_post_skeleton_defaults();
			save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
			loadEEpromSettings();
			load_settings();
			pending_save.dirty = false;
			pkt.ok = true;
		}
	}
	if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
		if (!safe_to_write_settings()) {
			can_printf("No save while running");
		} else {
			pending_save.dirty = false;
			save_settings();
			pkt.ok = true;
		}
	}

	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
	uint16_t total_size = DC_ENCODE(uavcan_protocol_param_ExecuteOpcodeResponse_encode, &pkt, buffer);

	DC_RESPOND(ins, transfer->source_node_id, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
		   &transfer->transfer_id, transfer->priority, CanardResponse, &buffer[0], total_size);
}

/*
  handle RestartNode request
*/
static void handle_RestartNode(CanardInstance *ins, CanardRxTransfer *transfer)
{
	// reboot the ESC
	allOff();
	set_rtc_backup_register(0, 0);
	NVIC_SystemReset();
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
	struct uavcan_protocol_GetNodeInfoResponse pkt;

	memset(&pkt, 0, sizeof(pkt));

	node_status.uptime_sec = micros64() / 1000000ULL;
	node_status.health = DroneCAN_nodeHealth();
	pkt.status = node_status;

	// fill in your major and minor firmware version
	pkt.software_version.major = VERSION_MAJOR;
	pkt.software_version.minor = VERSION_MINOR;
	pkt.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC |
						    UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	pkt.software_version.vcs_commit = px4_app_descriptor.git_hash;
	pkt.software_version.image_crc = px4_app_descriptor.image_crc;

	pkt.hardware_version.major = DRONECAN_HW_VERSION_MAJOR;
	pkt.hardware_version.minor = DRONECAN_HW_VERSION_MINOR;

	sys_can_getUniqueID(pkt.hardware_version.unique_id);

#	ifdef DRONECAN_NODE_NAME
	snprintf((char *)pkt.name.data, sizeof(pkt.name.data), "%s#M%u", DRONECAN_NODE_NAME, eepromBuffer.can_esc_index + 1);
#	else
	strncpy((char *)pkt.name.data, FIRMWARE_NAME, sizeof(pkt.name.data));
#	endif
	pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

	uint16_t total_size = DC_ENCODE(uavcan_protocol_GetNodeInfoResponse_encode, &pkt, buffer);

	DC_RESPOND(ins, transfer->source_node_id, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_ID,
		   &transfer->transfer_id, transfer->priority, CanardResponse, &buffer[0], total_size);
}

extern void transfercomplete();
extern void setInput();

/*
  process throttle input from DroneCAN
 */
static void set_input(uint16_t input)
{
	if (!armed && input != 0 && eepromBuffer.can_require_arming && dronecan_armed && !eepromBuffer.can_require_zero_throttle) {
		// allow restart if unexpected ESC reboot in flight
		faultErrorCountReset(); // armed 0->1: per-arm error_count (DSDL)
		armed = 1;
	}

	const uint16_t unfiltered_input = (dronecan_armed || !eepromBuffer.can_require_arming) ? input : 0;
	const uint16_t filtered_input = Filter2P_apply(unfiltered_input, eepromBuffer.can_filter_hz, 1000);

	newinput = filtered_input;
	last_can_input = unfiltered_input;
	inputSet = 1;

	/*
	 * RawCommand is already mapped onto the AM32 11-bit range.
	 * Bidirectional CAN needs the DShot mapper in setInput(). Never
	 * clear a detected wire protocol: AUTO keeps computeDshotDMA()
	 * running so DShot can resume after the 250 ms RawCommand failsafe.
	 * `dshot = bi_direction` used to force dshot=0 whenever reverse
	 * was off and permanently stole the wire path.
	 */
	if (eepromBuffer.bidirectional_mode) {
		dshot = 1;
	}

	transfercomplete();
	setInput();

	canstats.num_input++;
}

/*
  handle a ESC RawCommand request
*/
static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
	struct uavcan_equipment_esc_RawCommand cmd;
	if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
		return;
	}
	// see if it is for us
	if (cmd.cmd.len <= eepromBuffer.can_esc_index) {
		return;
	}

	// throttle demand is a value from -8191 to 8191. Negative values
	// are for reverse throttle
	const int16_t input_can = cmd.cmd.data[(unsigned)eepromBuffer.can_esc_index];

	/*
      we need to map onto the AM32 expected range, which is a 11 bit number, where:
      0: off
      1-46: special codes
      48-2047: throttle
    */
	uint16_t this_input = 0;
	if (input_can == 0) {
		this_input = 0;
	} else if (eepromBuffer.bidirectional_mode) {
		const float scaled_value = input_can * (1000.0 / 8192);
		if (scaled_value >= 0) {
			this_input = (uint16_t)(1047 + scaled_value);
		} else {
			this_input = (uint16_t)(47 + scaled_value * -1);
		}
	} else if (input_can > 0) {
		const float scaled_value = input_can * (2000.0 / 8192);
		this_input = (uint16_t)(47 + scaled_value);
	}

	const uint64_t ts = micros64();
	canstats.num_commands++;
	canstats.total_commands++;
	canstats.last_raw_command_us = ts;

	set_input(this_input);
}

/*
  handle ArmingStatus messages
*/
static void handle_ArmingStatus(CanardInstance *ins, CanardRxTransfer *transfer)
{
	struct uavcan_equipment_safety_ArmingStatus cmd;
	if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &cmd)) {
		return;
	}

	const uint8_t was_armed = dronecan_armed;
	dronecan_armed = (cmd.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
	/* ESC `armed` stays 1 after the first zero-throttle arm, so the
	 * escToArmedIdle 0->1 reset never runs again. Each FC arm is a new
	 * drive session — clear error_count so NodeStatus is OK until a real
	 * in-session stall/desync. */
	if (dronecan_armed && !was_armed) {
		faultErrorCountReset();
	}
	if (!dronecan_armed && eepromBuffer.can_require_arming && canstats.last_raw_command_us != 0) {
		set_input(0);
	}
}

/*
  handle a BeginFirmwareUpdate request from a management tool like
  DroneCAN GUI tool or MissionPlanner
 */
static void handle_begin_firmware_update(CanardInstance *ins, CanardRxTransfer *transfer)
{
	if (!safe_to_write_settings()) {
		can_printf("No update while running");
		return;
	}

	struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
	if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
		return;
	}

	sys_can_disable_IRQ();

	uint32_t reg[2] = {0, 0};
	if (req.image_file_remote_path.path.len <= 8) {
		// path is normally hashed and fits in 8 bytes, put in rtc backup registers 1 and 2
		memcpy((uint8_t *)reg, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);

		uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
		struct uavcan_protocol_file_BeginFirmwareUpdateResponse reply;
		memset(&reply, 0, sizeof(reply));
		reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

		uint32_t total_size = DC_ENCODE(uavcan_protocol_file_BeginFirmwareUpdateResponse_encode, &reply, buffer);

		DC_RESPOND(ins, transfer->source_node_id, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
			   UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID, &transfer->transfer_id, transfer->priority, CanardResponse,
			   &buffer[0], total_size);

		while (canardPeekTxQueue(&canard) != NULL) {
			DroneCAN_processTxQueue();
		}

		// time to transmit
		delayMillis(2);
	}

	set_rtc_backup_register(1, reg[0]);
	set_rtc_backup_register(2, reg[1]);

	// tell the bootloader we are doing fw update
	set_rtc_backup_register(0, (canardGetLocalNodeID(&canard) << 24) | (transfer->source_node_id << 16) | RTC_BKUP0_FWUPDATE);

	// reboot and let bootloader handle the request, this means the
	// first request doesn't get a reply, and the client re-sends. We
	// need this to get the path to the client. We could instead
	// define a memory block which is not reset on boot and put the
	// path there, but this is simpler
	NVIC_SystemReset();
}

/*
  data for dynamic node allocation process
*/
static struct {
	uint32_t send_next_node_id_allocation_request_at_ms;
	uint32_t node_id_allocation_unique_id_offset;
} DNA;

/*
  handle a DNA allocation packet
*/
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
	if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
		// already allocated
		return;
	}

	// Rule C - updating the randomized time interval
	DNA.send_next_node_id_allocation_request_at_ms =
		millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
		(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

	if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
		DNA.node_id_allocation_unique_id_offset = 0;
		return;
	}

	// Copying the unique ID from the message
	struct uavcan_protocol_dynamic_node_id_Allocation msg;

	if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
		/* bad packet */
		return;
	}

	// Obtaining the local unique ID
	uint8_t my_unique_id[sizeof(msg.unique_id.data)];
	sys_can_getUniqueID(my_unique_id);

	// Matching the received UID against the local one
	if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
		DNA.node_id_allocation_unique_id_offset = 0;
		// No match, return
		return;
	}

	if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
		// The allocator has confirmed part of unique ID, switching to
		// the next stage and updating the timeout.
		DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
		DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

	} else {
		// Allocation complete - copying the allocated node ID from the message
		canardSetLocalNodeID(ins, msg.node_id);
	}
}

/*
  ask for a dynamic node allocation
*/
static void request_DNA()
{
	const uint32_t now = millis32();
	static uint8_t node_id_allocation_transfer_id = 0;

	DNA.send_next_node_id_allocation_request_at_ms =
		now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
		(get_random16() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

	// Structure of the request is documented in the DSDL definition
	// See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
	uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
	allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

	if (DNA.node_id_allocation_unique_id_offset == 0) {
		allocation_request[0] |= 1; // First part of unique ID
	}

	uint8_t my_unique_id[16];
	sys_can_getUniqueID(my_unique_id);

	static const uint8_t MaxLenOfUniqueIDInRequest = 6;
	uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);

	if (uid_size > MaxLenOfUniqueIDInRequest) {
		uid_size = MaxLenOfUniqueIDInRequest;
	}

	memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

	/* DNA is a classic 8-byte anonymous transfer. If we echo CAN FD
	 * from RawCommand here, the allocator never completes. */
#	if CANARD_ENABLE_CANFD
	canardBroadcast(&canard, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
			&node_id_allocation_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, &allocation_request[0], (uint16_t)(uid_size + 1),
			false);
#	else
	canardBroadcast(&canard, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
			&node_id_allocation_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, &allocation_request[0], (uint16_t)(uid_size + 1));
#	endif

	// Preparing for timeout; if response is received, this value will be updated from the callback.
	DNA.node_id_allocation_unique_id_offset = 0;
}

/*
  This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
	// tell main loop we have had signal so we don't reset
	signaltimeout = 0;

	canstats.on_receive++;
	// switch on data type ID to pass to the right handler function
	if (transfer->transfer_type == CanardTransferTypeRequest) {
		// check if we want to handle a specific service request
		switch (transfer->data_type_id) {
			case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
				handle_GetNodeInfo(ins, transfer);
				break;
			}
			case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
				handle_param_GetSet(ins, transfer);
				break;
			}
			case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
				handle_param_ExecuteOpcode(ins, transfer);
				break;
			}
			case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
				handle_RestartNode(ins, transfer);
				break;
			}
			case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
				handle_begin_firmware_update(ins, transfer);
				break;
			}
		}
	}
	if (transfer->transfer_type == CanardTransferTypeBroadcast) {
		// check if we want to handle a specific broadcast message
		switch (transfer->data_type_id) {
			case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
				handle_RawCommand(ins, transfer);
				break;
			}
			case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
				handle_DNA_Allocation(ins, transfer);
				break;
			}
			case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
				handle_ArmingStatus(ins, transfer);
				break;
			}
		}
	}
}

/*
  This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
  by the local node.
  If the callback returns true, the library will receive the transfer.
  If the callback returns false, the library will ignore the transfer.
  All transfers that are addressed to other nodes are always ignored.

  This function must fill in the out_data_type_signature to be the signature of the message.
*/
static bool shouldAcceptTransfer(const CanardInstance *ins, uint64_t *out_data_type_signature, uint16_t data_type_id,
				 CanardTransferType transfer_type, uint8_t source_node_id)
{
	canstats.should_accept++;
	if (transfer_type == CanardTransferTypeRequest) {
		// check if we want to handle a specific service request
		switch (data_type_id) {
			case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
				return true;
			}
			case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
				return true;
			}
			case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
				return true;
			}
			case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
				return true;
			}
			case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
				return true;
			}
		}
	}
	if (transfer_type == CanardTransferTypeBroadcast) {
		// see if we want to handle a specific broadcast packet
		switch (data_type_id) {
			case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
				*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
				return true;
			}
			case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
				*out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
				return true;
			}
			case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
				*out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
				return true;
			}
		}
	}
	// we don't want any other messages
	return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs.
  health reflects latched faults / recent hard errors (see DroneCAN_nodeHealth).
*/
static void send_NodeStatus(void)
{
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

	node_status.uptime_sec = micros64() / 1000000ULL;
	node_status.health = DroneCAN_nodeHealth();
	node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	node_status.sub_mode = 0;

	// put number of commands we have received in vendor status since the last NodeStatus
	// this means vendor status gives us approximate command rate in commands/second
	node_status.vendor_specific_status_code = canstats.num_commands;
	canstats.num_commands = 0;

	uint32_t len = DC_ENCODE(uavcan_protocol_NodeStatus_encode, &node_status, buffer);

	// we need a static variable for the transfer ID. This is
	// incremeneted on each transfer, allowing for detection of packet
	// loss
	static uint8_t transfer_id;

	DC_BROADCAST(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID, &transfer_id,
		     CANARD_TRANSFER_PRIORITY_LOW, buffer, len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
	/*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
	canardCleanupStaleTransfers(&canard, timestamp_usec);

	/*
      Transmit the node status message
    */
	send_NodeStatus();

#	ifdef CAN_TERM_PIN
	setup_portpin(CAN_TERM_PIN, eepromBuffer.can_term_enable ? CAN_TERM_POLARITY : !CAN_TERM_POLARITY);
#	endif
}

/*
  send ESC status at TELEM_RATE Hz
*/
static void send_ESCStatus(void)
{
	struct uavcan_equipment_esc_Status pkt;
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];

	/* Hard-error events this arm cycle (DSDL: resets when the motor
	 * restarts; both addends zeroed on armed 0->1). Was desync_happened
	 * alone, which missed every run killed by the stall rail (and so also
	 * the blind-grind and blind/miss-limit paths that funnel into it) - a
	 * grinding motor reported error_count 0. See faultErrorCount(). */
	pkt.error_count = faultErrorCount();
	pkt.voltage = battery_voltage * 0.01;

	pkt.current = (current.sum / (float)current.count) * 0.01;
	current.sum = 0;
	current.count = 0;

	pkt.temperature = C_TO_KELVIN(degrees_celsius);
	pkt.rpm = (e_rpm * 200) / eepromBuffer.motor_poles;
	/* Instant demand factor: applied duty 0..2000 → 0..100% of full scale. */
	{
		uint16_t pct = duty_cycle / 20u;
		if (pct > 100u) {
			pct = 100u;
		}
		pkt.power_rating_pct = (uint8_t)pct;
	}
	pkt.esc_index = eepromBuffer.can_esc_index;

	uint32_t len = DC_ENCODE(uavcan_equipment_esc_Status_encode, &pkt, buffer);

	// we need a static variable for the transfer ID. This is
	// incremeneted on each transfer, allowing for detection of packet
	// loss
	static uint8_t transfer_id;

	DC_BROADCAST(&canard, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE, UAVCAN_EQUIPMENT_ESC_STATUS_ID, &transfer_id,
		     CANARD_TRANSFER_PRIORITY_LOW, buffer, len);
}

/*
  send FlexDebug at DEBUG_RATE Hz
*/
static void send_FlexDebug(void)
{
	static struct {
		uint32_t total_commands;
		uint32_t num_input;
	} last;
	/*
      popupate debug1
     */
	debug1.version = 2;
	debug1.commutation_interval = commutation_interval;
	debug1.auto_advance_level = auto_advance_level;
	debug1.acq_resist_events = fault_acq_resist_events;
	debug1.num_commands = canstats.total_commands - last.total_commands;
	debug1.num_input = canstats.num_input - last.num_input;
	debug1.rx_errors = canstats.rx_errors;
	debug1.rxframe_error = canstats.rxframe_error;
	debug1.rx_ecode = canstats.rx_ecode;
	debug1.duty_cycle = duty_cycle;
	debug1.duty_cycle_maximum = duty_cycle_maximum;
	debug1.adjusted_input = adjusted_input;
	debug1.adc_raw_current = ADC_raw_current;
	debug1.adc_raw_volts = ADC_raw_volts;
	debug1.flags = (armed ? 1 : 0) | (running ? 2 : 0) | (stepper_sine ? 4 : 0);

	last.num_input = canstats.num_input;
	last.total_commands = canstats.total_commands;

	struct dronecan_protocol_FlexDebug pkt;
	uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];

	pkt.id = DRONECAN_PROTOCOL_FLEXDEBUG_AM32_RESERVE_START + 0;
	pkt.u8.len = sizeof(debug1);
	memcpy(pkt.u8.data, (const uint8_t *)&debug1, sizeof(debug1));
	uint32_t len = DC_ENCODE(dronecan_protocol_FlexDebug_encode, &pkt, buffer);

	static uint8_t transfer_id;

	DC_BROADCAST(&canard, DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE, DRONECAN_PROTOCOL_FLEXDEBUG_ID, &transfer_id,
		     CANARD_TRANSFER_PRIORITY_LOW, buffer, len);
}

/*
  receive one frame, only called from interrupt context
*/
void DroneCAN_receiveFrame(void)
{
	CanardCANFrame rx_frame = {0};
	while (sys_can_receive(&rx_frame) > 0) {
		canstats.num_receive++;
		dc_note_rx_frame(&rx_frame);
		int ecode = canardHandleRxFrame(&canard, &rx_frame, micros64());
		if (ecode != CANARD_OK && ecode != -CANARD_ERROR_RX_NOT_WANTED) {
			canstats.rx_ecode = ecode;
			canstats.rxframe_error++;
		}
	}
}

void DroneCAN_handleFrame(const CanardCANFrame *rx_frame)
{
	canstats.num_receive++;
	dc_note_rx_frame(rx_frame);
	int ecode = canardHandleRxFrame(&canard, rx_frame, micros64());
	if (ecode != CANARD_OK && ecode != -CANARD_ERROR_RX_NOT_WANTED) {
		canstats.rx_ecode = ecode;
		canstats.rxframe_error++;
	}
}

/*
  Transmits all frames from the TX queue
*/
void DroneCAN_processTxQueue(void)
{
	for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
		const int16_t tx_res = sys_can_transmit(txf);
		if (tx_res == 0) {
			// no space, stop trying
			break;
		}
		// success or error, remove frame
		canardPopTxQueue(&canard);
	}
}

static void DroneCAN_Startup(void)
{
	load_settings();

	canardInit(&canard,
		   canard_memory_pool, // Raw memory chunk used for dynamic allocation
		   sizeof(canard_memory_pool),
		   onTransferReceived,	 // Callback, see CanardOnTransferReception
		   shouldAcceptTransfer, // Callback, see CanardShouldAcceptTransfer
		   NULL);

	if (eepromBuffer.can_node != 0) {
		canardSetLocalNodeID(&canard, eepromBuffer.can_node);
	}

	// initialise low level CAN peripheral hardware
	sys_can_init();
#	if CANARD_ENABLE_CANFD
	dronecan_tx_canfd = sys_can_prefer_canfd_tx();
#	endif

	/*
	 * DRONECAN_IN (5) is exclusive: disable DShot/PWM IRQs so noise on the
	 * signal pin cannot fight CAN. AUTO (0) and the fixed wire types keep
	 * capture live; when both CAN and wire are present, DroneCAN_active()
	 * makes RawCommand win until the stream times out (~250 ms).
	 */
	if (eepromBuffer.input_type == DRONECAN_IN) {
#	ifdef MCU_L431
		NVIC_DisableIRQ(DMA1_Channel5_IRQn);
		NVIC_DisableIRQ(EXTI15_10_IRQn);
		EXTI->IMR1 &= ~(1U << 15);
#	elif defined(MCU_G431)
		NVIC_DisableIRQ(EXTI15_10_IRQn);
		EXTI->IMR1 &= ~(1U << 15);
#	elif defined(MCU_AT415)
		NVIC_DisableIRQ(DMA1_Channel6_IRQn);
		NVIC_DisableIRQ(EXINT15_10_IRQn);
		EXINT->inten &= ~EXINT_LINE_15;
#	elif defined(MCU_SITL)
		NVIC_DisableIRQ(SITL_IRQ_DMA);
		NVIC_DisableIRQ(SITL_IRQ_EXTI15);
#	else
#		error "unsupported MCU"
#	endif
	}
}

void DroneCAN_update()
{
	sys_can_disable_IRQ();
	sys_can_service();

	static uint64_t next_1hz_service_at;
	static uint64_t next_telem_service_at;
	static uint64_t next_flexdebug_at;
	if (!done_startup) {
		DroneCAN_Startup();
		done_startup = true;
		set_rtc_backup_register(0, RTC_BKUP0_BOOTED);
	}

	if (canstats.on_receive == 5) {
		// indicate to bootloader that we were fully operational
		set_rtc_backup_register(0, RTC_BKUP0_SIGNAL);
	}

	DroneCAN_processTxQueue();

	// see if we are still doing DNA
	if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
		// we're still waiting for a DNA allocation of our node ID
		if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
			request_DNA();
		}
		sys_can_enable_IRQ();
		return;
	}

	const uint64_t ts = micros64();

	/* Rising-edge stuck / nFAULT → one LogMessage (not rate-limited spam). */
	DroneCAN_pollFaultLogMessages();

	if (ts >= next_1hz_service_at) {
		next_1hz_service_at += 1000000ULL;
		process1HzTasks(ts);
	}
	if (eepromBuffer.can_telem_rate > 0 && ts >= next_telem_service_at) {
		next_telem_service_at += 1000000ULL / eepromBuffer.can_telem_rate;
		send_ESCStatus();
	}
	if (eepromBuffer.can_debug_rate > 0 && ts >= next_flexdebug_at) {
		next_flexdebug_at += 1000000ULL / eepromBuffer.can_debug_rate;
		send_FlexDebug();
	}

	DroneCAN_processTxQueue();

	/* Deferred settings save: quiet window + safe to write flash (AM32#359). */
	if (pending_save.dirty && (millis32() - pending_save.last_change_ms) >= SETTINGS_SAVE_QUIET_MS && safe_to_write_settings()) {
		pending_save.dirty = false;
		save_settings();
	}

	if (canstats.last_raw_command_us != 0 && ts - canstats.last_raw_command_us > 250000ULL) {
		/*
          we have stopped getting CAN RawCommand, zero input.

          The input filter has to be forced to zero as well, not just
          fed a zero sample: this is the only zero the failsafe gets
          (the 1kHz re-injection below is disabled once
          last_raw_command_us is cleared), and one sample through a
          slow filter leaves nearly all of the previous throttle
          applied - with INPUT_FILTER_HZ=10 over 99% of it. The motor
          would keep running at the last commanded throttle for as
          long as any other accepted DroneCAN transfer kept the main
          loop's signal watchdog fed.
         */
		canstats.last_raw_command_us = 0;
		Filter2P_reset(0);
		set_input(0);
	}
	if (canstats.last_raw_command_us != 0 && ts - last_heartbeat_us > TARGET_PERIOD_US) {
		/*
          ensure at least 1kHz signal is seen by main code, but only
          once we have received a RawCommand
         */
		set_input(last_can_input);
		last_heartbeat_us = ts;
	}

	sys_can_enable_IRQ();

	// keep summed current for averaging
	current.sum += actual_current;
	current.count++;
}

bool DroneCAN_active(void)
{
	/*
	 * True while a RawCommand stream is live (refreshed by handle_RawCommand
	 * and cleared by the 250 ms failsafe above). Sticky total_commands is
	 * NOT used: after CAN drops out, DShot/PWM under AUTO must be able to
	 * take over without a reboot.
	 */
	return canstats.last_raw_command_us != 0;
}

#endif // DRONECAN_SUPPORT
