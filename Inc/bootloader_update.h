#pragma once

/*
 * Optionally rewrite the AM32 bootloader from an image embedded in the app.
 * No-op when EMBED_BOOTLOADER is not defined (e.g. HWCI_PERF builds).
 *
 * Call once early after clocks/flash are usable and before motors run.
 * Arms IWDG only when an update is about to run. Calls NVIC_SystemReset()
 * only after the full BL region verifies (after marking a next-boot success
 * chirp via bootSoundMarkBootloaderUpdated); on failure returns so the app
 * image keeps running (bounded retries, no infinite hang with IRQs off).
 */
void maybe_update_bootloader(void);
