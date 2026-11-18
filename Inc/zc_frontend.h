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
 *
 * Front-end behaviour outside the fragment, in shared files, is keyed on
 * ZC_FRONTEND_G431 / ZC_FRONTEND_F051 (never on the MCU): the confirm
 * filter tiers (runtime_loop.c), poll re-entry and the poll-to-interrupt
 * handoff (commutation.c), masking comparator IRQs on re-entering poll
 * (esc_state.c) and the acquisition grace and soft stall budget (faults.c).
 */

#include "targets.h" /* selects ZC_FRONTEND_F051 / _G431 / _STOCK (end of targets.h) */

#if defined(ZC_FRONTEND_F051)
#	define ZC_FRONTEND_IMPL "../Mcu/f051/Inc/zc_frontend_f051.h"
#elif defined(ZC_FRONTEND_G431)
#	define ZC_FRONTEND_IMPL "../Mcu/g431/Inc/zc_frontend_g431.h"
#else
#	define ZC_FRONTEND_IMPL "zc_frontend_stock.h"
#endif

#endif /* ZC_FRONTEND_H_ */
