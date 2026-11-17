/*
 * App-side bootloader update (ArduPilot-style).
 *
 * When EMBED_BOOTLOADER is set, compare flash @ MCU base to the embedded
 * image; if different, program the BL region with IRQs off, verify, reset
 * only on full success.
 *
 * The image itself is a committed .bin pulled in by Src/bl_image.S; this file
 * only sees its bounds. F051 embeds on release builds; G431 CAN always embeds
 * so a manual or PX4 app flash still refreshes the on-chip BL. Use
 * EMBED_BOOTLOADER=0 or NO_EMBED_BL=1 to omit the image and this logic.
 */

#include "bootloader_update.h"

#include "eeprom.h"
#include "main.h"
#include "peripherals.h"
#include "sounds.h"
#include "targets.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(EMBED_BOOTLOADER) && (defined(MCU_F051) || defined(MCU_G431))

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

/*
 * Erase runs when address is page-aligned in save_flash_nolib.
 * F051: 1 KiB pages. G431: 2 KiB pages (single-bank 128 KiB).
 */
#	if defined(MCU_G431)
#		define BL_PAGE_BYTES 2048u
#	else
#		define BL_PAGE_BYTES 1024u
#	endif
#	define BL_CHUNK_BYTES 256u
/* Per-page program/verify attempts before aborting the whole update. */
#	define BL_MAX_PAGE_ATTEMPTS 4u

/*
 * G431 I/D flash caches are on at reset (FLASH_ACR ICEN|DCEN). A verify
 * memcmp after program can hit a stale D-cache line; the retry path then
 * re-reads the same addresses and turns one transient fail into a 4-strike
 * abort with a half-written BL page. F051 has no D-cache.
 */
#	if defined(MCU_G431)
static void flash_flush_caches(void)
{
	if ((FLASH->ACR & FLASH_ACR_DCEN) != 0u) {
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;
	}
	if ((FLASH->ACR & FLASH_ACR_ICEN) != 0u) {
		FLASH->ACR &= ~FLASH_ACR_ICEN;
		FLASH->ACR |= FLASH_ACR_ICRST;
		FLASH->ACR &= ~FLASH_ACR_ICRST;
		FLASH->ACR |= FLASH_ACR_ICEN;
	}
}
#	else
static void flash_flush_caches(void) {}
#	endif

#	if defined(MCU_G431)
/*
 * G431 (ARK 12S CAN) rewrite policy, V20. The F051 path below is unchanged.
 *
 * - Never downgrade. The bootloader stamps its version into default_settings[]
 *   (bootloader/bootloader/DroneCAN/DroneCAN.c): 0x01 0x02 <VER> 0x01 0x23
 *   "NEOESC f051", once in each of V18, V19 and V20. An older app on a newer
 *   chip used to put V18 back on any byte difference.
 * - A same-version mismatch is a damaged bootloader (an interrupted rewrite)
 *   and is repaired; an unidentifiable one (or our trampoline, below) too.
 * - When the on-chip bootloader is identifiable, rewrite only after a software
 *   reset (sticky RCC_CSR_SFTRSTF, nothing here clears it): the PX4 update path
 *   and the signal-loss reset both get there with the supply long settled,
 *   never on the bounce of a battery plug-in.
 * - Trampoline: page 0 is first rewritten with just the app's stack pointer and
 *   reset vector, then pages 1..N-1, then page 0 in full. Power loss during
 *   the long middle phase boots the app directly (initAfterJump() sets its own
 *   VTOR), which sees a damaged bootloader and repairs it. Only the two page-0
 *   erase+write windows (about 45 ms total) can still brick; before V20 the
 *   whole ~350 ms rewrite could.
 */
static int bl_image_version(const uint8_t *p, uint32_t len)
{
	static const uint8_t tail[] = {0x01u, 0x23u, 'N', 'E', 'O', 'E', 'S', 'C', ' ', 'f', '0', '5', '1'};
	for (uint32_t i = 0; i + 3u + sizeof(tail) <= len; i++) {
		if (p[i] == 0x01u && p[i + 1u] == 0x02u && memcmp(&p[i + 3u], tail, sizeof(tail)) == 0) {
			return p[i + 2u];
		}
	}
	return -1;
}

/* Program [start, end) of the bootloader region from the embedded image with
 * per-page verify and bounded retries (the loop the F051 path uses). */
static bool bl_program_range(const uint8_t *want, uint32_t start, uint32_t end)
{
	uint32_t off = start;
	uint32_t page_base = UINT32_MAX;
	uint32_t page_attempts = 0;

	while (off < end) {
		const uint32_t this_page = off & ~(BL_PAGE_BYTES - 1u);
		if (this_page != page_base) {
			page_base = this_page;
			page_attempts = 0;
		}

		uint32_t chunk = BL_CHUNK_BYTES;
		if (chunk > (end - off)) {
			chunk = end - off;
		}

		/* Non-const pointer: save_flash_nolib only reads the buffer. */
		uint8_t *src = (uint8_t *)(uintptr_t)&want[off];
		const uint32_t addr = MCU_FLASH_START + off;

		RELOAD_WATCHDOG_COUNTER();
		save_flash_nolib(src, (int)chunk, addr);
		flash_flush_caches();

		if (memcmp((const void *)(uintptr_t)addr, &want[off], chunk) != 0) {
			/* Re-erase from the page base; bounded so a sticky fault cannot
			 * hang with IRQs off. */
			page_attempts++;
			if (page_attempts >= BL_MAX_PAGE_ATTEMPTS) {
				return false;
			}
			off = page_base;
			continue;
		}
		off += chunk;
	}
	return true;
}

void maybe_update_bootloader(void)
{
	const uint32_t len = (uint32_t)(_bl_image_end - _bl_image_start);
	const uint8_t *want = _bl_image_start;
	const uint8_t *have = (const uint8_t *)(uintptr_t)MCU_FLASH_START;
	/* The app's vector table sits right after the bootloader region. */
	const uint32_t *app_vec = (const uint32_t *)(uintptr_t)(MCU_FLASH_START + len);

	if (len == 0u || (len % BL_PAGE_BYTES) != 0u) {
		return;
	}

	/*
	 * Recognise our own trampoline first: an interrupted rewrite can leave
	 * torn doublewords in the other pages, so don't scan them in that case.
	 * The trampoline already makes the unit boot, so its repair waits for a
	 * software reset like any other rewrite (repairing on a bouncing plug-in
	 * would reopen the page-0 window this sequence exists to avoid).
	 */
	const bool have_tramp = memcmp(have, app_vec, 8u) == 0;
	if (!have_tramp && memcmp(have, want, len) == 0) {
		return;
	}

	const int want_v = bl_image_version(want, len);
	const int have_v = have_tramp ? -1 : bl_image_version(have, len);
	if (want_v < 0 || have_v > want_v) {
		return; /* never downgrade, never write an unidentified image */
	}
	if ((have_v >= 0 || have_tramp) && (RCC->CSR & RCC_CSR_SFTRSTF) == 0u) {
		return; /* wait for a software reset */
	}

	/* The trampoline needs a plausible app vector table where this code runs. */
	const bool trampoline = SCB->VTOR == (uint32_t)(uintptr_t)app_vec && (app_vec[0] & 0xFFF00000u) == 0x20000000u &&
				(app_vec[1] & 1u) != 0u && app_vec[1] > (uint32_t)(uintptr_t)app_vec;

	/*
	 * Arm IWDG before the highest-risk flash sequence. Once started it
	 * cannot be stopped; failure paths return to main which reloads and
	 * reconfigures at the normal MX_IWDG_Init site. Success path resets.
	 */
	MX_IWDG_Init();
	RELOAD_WATCHDOG_COUNTER();
	__disable_irq();

	bool ok = false;
	if (trampoline) {
		uint8_t tramp[8];
		memcpy(tramp, app_vec, sizeof(tramp));
		bool tramp_ok = have_tramp;
		if (!tramp_ok) {
			RELOAD_WATCHDOG_COUNTER();
			save_flash_nolib(tramp, (int)sizeof(tramp), MCU_FLASH_START);
			flash_flush_caches();
			tramp_ok = memcmp(have, tramp, sizeof(tramp)) == 0;
		}
		if (tramp_ok) {
			ok = bl_program_range(want, BL_PAGE_BYTES, len) && bl_program_range(want, 0u, BL_PAGE_BYTES);
			if (!ok && memcmp(have, want, BL_PAGE_BYTES) != 0) {
				/* page 0 failed: never leave it undefined, put the trampoline back */
				RELOAD_WATCHDOG_COUNTER();
				save_flash_nolib(tramp, (int)sizeof(tramp), MCU_FLASH_START);
				flash_flush_caches();
			}
		} else {
			ok = bl_program_range(want, 0u, len); /* trampoline failed: plain order */
		}
	} else {
		ok = bl_program_range(want, 0u, len);
	}

	/* Reset only when the full BL region matches the embedded image. */
	flash_flush_caches();
	if (ok && memcmp(have, want, len) == 0) {
		/* Next boot plays a short success chirp before the normal tune. */
		bootSoundMarkBootloaderUpdated();
		NVIC_SystemReset();
	}

	/* Partial/failed write: keep running so the app can still be reached
	 * (page 0 is either the old loader, the trampoline, or the new image). */
	__enable_irq();
	RELOAD_WATCHDOG_COUNTER();
}

#	else /* F051 */

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
	 * Same flash bank as this code (F051 32 KiB; G431 128 KiB single-bank),
	 * but we only erase/program the bootloader pages below the app.
	 * Disable IRQs so no vector fetch races a half-written state mid-chunk
	 * (mirrors AM32-bootloader bl_update).
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
		flash_flush_caches();

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
	flash_flush_caches();
	if (memcmp(have, want, len) == 0) {
		/* Next boot plays a short success chirp before the normal tune. */
		bootSoundMarkBootloaderUpdated();
		NVIC_SystemReset();
	}

	/* Partial/failed write: keep running so the app can still be reached. */
	__enable_irq();
	RELOAD_WATCHDOG_COUNTER();
}

#	endif /* MCU_G431 */

#else /* !EMBED_BOOTLOADER || !(MCU_F051 || MCU_G431) */

void maybe_update_bootloader(void)
{
	/* Not embedded (kill switch / F051 HWCI) or MCU not supported. */
}

#endif
