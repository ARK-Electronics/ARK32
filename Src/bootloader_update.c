/*
 * App-side bootloader update (ArduPilot-style).
 *
 * When EMBED_BOOTLOADER is set, compare flash @ MCU base to the embedded
 * image; if different, program the BL region with IRQs off, verify, reset
 * only on full success.
 *
 * The image itself is a committed .bin pulled in by Src/bl_image.S; this file
 * only sees its bounds. HWCI_PERF builds omit EMBED_BOOTLOADER so neither the
 * ~4 KiB image nor this logic is linked.
 */

#include "bootloader_update.h"

#include "eeprom.h"
#include "main.h"
#include "peripherals.h"
#include "targets.h"

#include <stdint.h>
#include <string.h>

#if defined(EMBED_BOOTLOADER) && defined(MCU_F051)

/*
 * Bounds of the image assembled by Src/bl_image.S, defined by the .bl_image
 * output section in the linker script. Linker symbols carry no value of their
 * own - only their address matters - so they are declared as arrays and the
 * length is the difference between them. The linker script asserts the size
 * matches the bootloader region and that the image sits outside it.
 */
extern const uint8_t _bl_image_start[];
extern const uint8_t _bl_image_end[];

#	ifndef MCU_FLASH_START
#		define MCU_FLASH_START 0x08000000u
#	endif

/* F051 page = 1 KiB; erase runs when address is page-aligned in save_flash_nolib. */
#	define BL_PAGE_BYTES 1024u
#	define BL_CHUNK_BYTES 256u
/* Per-page program/verify attempts before aborting the whole update. */
#	define BL_MAX_PAGE_ATTEMPTS 4u

void maybe_update_bootloader(void)
{
	const uint32_t len = (uint32_t)(_bl_image_end - _bl_image_start);
	const uint8_t *want = _bl_image_start;
	const uint8_t *have = (const uint8_t *)(uintptr_t)MCU_FLASH_START;

	if (len == 0u || (len & 1u) != 0u) {
		return;
	}

	if (memcmp(have, want, len) == 0) {
		return;
	}

	/*
	 * Arm IWDG before the highest-risk flash sequence. Once started it
	 * cannot be stopped; failure paths return to main which reloads and
	 * reconfigures at the normal MX_IWDG_Init site. Success path resets.
	 */
	MX_IWDG_Init();
	RELOAD_WATCHDOG_COUNTER();

	/*
	 * Same flash bank as this code on F051, but we only erase/program the
	 * bootloader pages below the app. Disable IRQs so no vector fetch races
	 * a half-written state mid-chunk (mirrors AM32-bootloader bl_update).
	 */
	__disable_irq();

	uint32_t off = 0;
	uint32_t page_base = UINT32_MAX;
	uint32_t page_attempts = 0;

	while (off < len) {
		const uint32_t this_page = off & ~(BL_PAGE_BYTES - 1u);
		if (this_page != page_base) {
			page_base = this_page;
			page_attempts = 0;
		}

		uint32_t chunk = BL_CHUNK_BYTES;
		if (chunk > (len - off)) {
			chunk = len - off;
		}

		/* Non-const pointer: save_flash_nolib only reads the buffer. */
		uint8_t *src = (uint8_t *)(uintptr_t)&want[off];
		const uint32_t addr = MCU_FLASH_START + off;

		RELOAD_WATCHDOG_COUNTER();
		save_flash_nolib(src, (int)chunk, addr);

		if (memcmp((const void *)(uintptr_t)addr, &want[off], chunk) != 0) {
			/*
			 * Flash can only program 1→0 without erase. A mid-page
			 * failure must re-erase from the page base, not retry
			 * the chunk alone. Bound attempts so a sticky fault
			 * cannot hang forever with IRQs off.
			 */
			page_attempts++;
			if (page_attempts >= BL_MAX_PAGE_ATTEMPTS) {
				__enable_irq();
				RELOAD_WATCHDOG_COUNTER();
				/* Leave the already-running app image usable. */
				return;
			}
			off = page_base;
			continue;
		}
		off += chunk;
	}

	/* Reset only when the full BL region matches the embedded image. */
	if (memcmp(have, want, len) == 0) {
		NVIC_SystemReset();
	}

	/* Partial/failed write: keep running so the app can still be reached. */
	__enable_irq();
	RELOAD_WATCHDOG_COUNTER();
}

#else /* !EMBED_BOOTLOADER || !MCU_F051 */

void maybe_update_bootloader(void)
{
	/* Not embedded (HWCI) or MCU not supported in this prototype. */
}

#endif
