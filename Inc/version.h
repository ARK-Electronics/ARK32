/*
 * Firmware version for releases.
 *
 * MAJOR.MINOR track the upstream AM32 base this tree is based on (EEPROM still
 * stores only these two numeric fields for protocol compatibility).
 * VERSION_PATCH is the ARK ship-increment. It is included in artifact names
 * (e.g. 3.0.3-ark) and embedded as a second C-string in the 32-byte
 * `.file_name` flash region (`FILE_NAME\0MAJOR.MINOR.PATCH[-TAG]\0`) so the
 * ARK32 configurator can show the full ship version after connect.
 * VERSION_TAG marks ARK-fork builds vs upstream AM32.
 * Update this file for new releases.
 */
#define VERSION_MAJOR 3
#define VERSION_MINOR 0
#define VERSION_PATCH 3
#define VERSION_TAG "ark"

#define EEPROM_VERSION 3
