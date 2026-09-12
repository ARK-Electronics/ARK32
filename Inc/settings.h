/*
 * settings.h - EEPROM load/apply and bootloader device-info
 *
 * Behavior-neutral extract of loadEEpromSettings / saveEEpromSettings /
 * checkDeviceInfo previously in main.c.
 */
#ifndef SETTINGS_H_
#define SETTINGS_H_

void loadEEpromSettings(void);
void saveEEpromSettings(void);
void checkDeviceInfo(void);
/* Recompute kV/poles-derived runtime tables after a live param change. */
void applyMotorIdentitySettings(void);

#endif /* SETTINGS_H_ */
