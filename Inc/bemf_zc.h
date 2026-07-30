/*
 * bemf_zc.h - zero-cross detection and commutation scheduling
 */
#ifndef BEMF_ZC_H_
#define BEMF_ZC_H_

void PeriodElapsedCallback(void);
void interruptRoutine(void);
void startMotor(void);
/* Reset turn-on-grid compensation state (duty-slew history) on reverse/stop. */
void bemfZcResetTrend(void);

#endif /* BEMF_ZC_H_ */
