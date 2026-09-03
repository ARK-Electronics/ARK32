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
#define VERSION_MINOR 0

#define EEPROM_VERSION 3
