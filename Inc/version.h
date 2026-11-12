/*
 * Ship version. EEPROM stores only these two bytes (AM32 layout, bytes 3-4),
 * so this pair is the full version GCS and the configurator see.
 *
 * Major 32 marks ARK32 vs upstream AM32. Minor is the ship increment.
 * Artifact names are ARK32_<FILE_NAME>_MAJOR.MINOR (e.g. 32.0).
 *
 * An optional VERSION_TAG may still be defined for nightlies/RCs (appended
 * as -TAG); ship releases omit it.
 * Update this file for new releases.
 */
#define VERSION_MAJOR 32
#define VERSION_MINOR 1
#define VERSION_TAG "beta1"

/* EEPROM layout revision. 4: max_ramp became an exact duty rate (no
 * pack-voltage scaling). The byte layout is unchanged from 3 and the
 * firmware does not rewrite a stored ramp; the ARK32 configurator migrates
 * a v3 ramp. Boot stamps this version over an older page (main.c), so the
 * configurator must read a v3 page before flashing this firmware. The image
 * carries this number in its .file_name block ("e4", motor_runtime.c) so the
 * configurator can tell a layout-4 hex from a layout-3 one.
 *
 * Upstream AM32 2.21/2.22 also uses layout 4, adding byte 13
 * brake_on_zero_throttle (0-9) and byte 14 demag_comp (0-4). ARK32 names
 * those bytes but does not use them, and defaults them to 0 like upstream's
 * default page; every other byte has the same offset and meaning, so either
 * firmware and either configurator can read the other's page. */
#define EEPROM_VERSION 4
