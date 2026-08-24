/*
 * Firmware version for releases.
 *
 * MAJOR.MINOR track the upstream AM32 base this tree is based on (EEPROM still
 * stores only these two numeric fields for protocol compatibility).
 * VERSION_PATCH is the ARK ship-increment. It is included in artifact names
 * (e.g. 3.0.3) and embedded as a second C-string in the 32-byte `.file_name`
 * flash region (`FILE_NAME\0MAJOR.MINOR.PATCH\0`) so the ARK32 configurator
 * can show the full ship version after connect.
 * Fork identity is the ARK32_ artifact prefix, not a -ark version suffix.
 * An optional VERSION_TAG may still be defined for nightlies/RCs (appended
 * as -TAG); ship releases omit it.
 * Update this file for new releases.
 */
#define VERSION_MAJOR 3
#define VERSION_MINOR 0
#define VERSION_PATCH 3

#define EEPROM_VERSION 3
