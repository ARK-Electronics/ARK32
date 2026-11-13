#ifndef ZC_FRONTEND_H_
#define ZC_FRONTEND_H_

/*
 * Zero-crossing front end selection.
 *
 * The commutation scheduler, state machine, ramp, faults and settings are
 * shared by every MCU. The front end - comparator gate and blanking, confirm
 * filter, edge delivery and the missed-crossing (deadline / blind-step)
 * policy - is per MCU, because detection quality is a property of the board:
 * a policy validated on the F051 4IN1 (#137) broke the G431 12S CAN ESC,
 * whose comparators read low-speed crossings with an off-window offset.
 *
 * Each front end is a source fragment textually included by Src/bemf_zc.c
 * (ZC_FRONTEND_IMPL), so the firmware's single -flto link keeps the same
 * translation units and order and the F051 image stays byte-identical.
 * ZC_FRONTEND_G431 selects the G431 front end on a non-G431 build (the
 * ARK_G4_12S_SITL_CAN SITL target).
 */

#include "targets.h"

#if defined(MCU_F051)
#	define ZC_FRONTEND_F051 1
#	define ZC_FRONTEND_IMPL "../Mcu/f051/Inc/zc_frontend_f051.h"
#elif defined(MCU_G431) || defined(ZC_FRONTEND_G431)
#	ifndef ZC_FRONTEND_G431
#		define ZC_FRONTEND_G431 1
#	endif
#	define ZC_FRONTEND_IMPL "../Mcu/g431/Inc/zc_frontend_g431.h"
#else
#	define ZC_FRONTEND_STOCK 1
#	define ZC_FRONTEND_IMPL "zc_frontend_stock.h"
#endif

#endif /* ZC_FRONTEND_H_ */
