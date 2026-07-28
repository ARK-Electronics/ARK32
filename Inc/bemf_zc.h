/*
 * bemf_zc.h - zero-cross detection and commutation scheduling
 */
#ifndef BEMF_ZC_H_
#define BEMF_ZC_H_

void PeriodElapsedCallback(void);
void interruptRoutine(void);
void startMotor(void);
/* Clear the acceleration trend used for the commutation point (bemf_zc.c). */
void bemfZcResetTrend(void);

#endif /* BEMF_ZC_H_ */
