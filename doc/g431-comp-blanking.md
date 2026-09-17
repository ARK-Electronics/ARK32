# G431 comparator output blanking

Design notes for **STM32G431/G491** (`ARK_G431_CAN`) hardware COMP blanking
used to reject PWM turn-on switching noise on the BEMF zero-cross path.

| | |
|--|--|
| Targets | `ARK_G431_CAN` (and any G4 target that defines `COMP_BLANK_TICKS`) |
| Knobs | `COMP_BLANK_TICKS` (TIM1 OC5 window), `ZC_SEARCH_BLANK_64THS` (software post-commutation blank) in [`Inc/targets.h`](../Inc/targets.h) |
| Code | [`Mcu/g431/Src/comparator.c`](../Mcu/g431/Src/comparator.c) (`changeCompInput`), [`Mcu/g431/Src/peripherals.c`](../Mcu/g431/Src/peripherals.c) (TIM1 CH5), EXTI path in [`Mcu/g431/Src/stm32g4xx_it.c`](../Mcu/g431/Src/stm32g4xx_it.c) |

## Ideal blanking vs ST blanking

Two different meanings of “blank the comparator during the PWM edge” matter
for motor control:

| Model | Behavior while blank is active | Effect on EXTI (edge-triggered) |
|--------|--------------------------------|----------------------------------|
| **Hold-last (ideal for BEMF ZC)** | Output stays at its **previous** level until the blank ends | No manufactured edge; a real crossing is either delayed cleanly to unblank or never seen until the line actually changes after blank |
| **Force-low (STM32G4 COMP)** | Output is forced **low** for the blank window | Only transparent if the line was already low; if it was high, **enabling blank creates a falling edge** |

Alka (AM32) summarized the motor-control preference clearly: blanking that
**stops the output from changing** for the blank period (hold last state) is
what you want for ZC. ST Micro implemented the other thing — force the gated
output low while the blanking timer source is active.

On G4 this is not a configurable mode. The COMP block’s blanking source only
implements force-low. There is no hardware “sample-and-hold the last COMP
output until the PWM blank ends” path.

Also: `COMPx_CSR.VALUE` / `LL_COMP_ReadOutputLevel()` / `getCompOutputLevel()`
are taken **before** polarity and blanking are applied (see ST LL headers).
Firmware confirm loops that read CSR.VALUE therefore see the **true analog
decision**, while EXTI is wired to the **gated** output. False edges from
force-low can still be rejected in software; real crossings destroyed by
force-low never appear as edges at all (they go blind instead).

## Why force-low is polarity-asymmetric for AM32 ZC

AM32 arms EXTI for the **next** expected BEMF polarity each step
(`changeCompInput()` / `rising`):

| Step polarity | Pre-crossing COMP level | Awaited EXTI edge | Force-low blanking |
|---------------|-------------------------|-------------------|--------------------|
| Falling BEMF (`rising == 0`) | Low | Rising | **Safe.** Force-low is a no-op before the cross. Noise cannot invent a rising edge during the window. A true cross inside the window becomes a **late** rising edge when blank ends (at most `COMP_BLANK_TICKS` late). |
| Rising BEMF (`rising == 1`) | High | Falling | **Hostile.** Force-low **manufactures** a falling edge at every blank open (once per PWM period for the whole ZC search). A real cross **inside** the window can be **destroyed**: output is already low; when blank ends the raw level is also low → no edge → blind step. |

So ST blanking only works cleanly when you are trying to blank a **rising**
event (pre-cross low). That matches Alka’s statement: *“it only works if you
are trying to blank a rising event.”*

Manufactured falling edges on rising-BEMF steps tend to show up as a **PWM-rate
ISR + confirm-reject storm** (confirm still reads ungated CSR.VALUE and rejects
the false edge). Destroyed real crossings are silent and show up as blind
steps / open-loop residency, not as rejects.

## What ARK32 does

1. **TIM1 CH5 (pinless) → OC5REF** as the COMP blanking source. PWM1 with
   `CCR5 = COMP_BLANK_TICKS` holds OC5REF high for the first N ticks of every
   PWM period (see peripherals init).
2. **Window sizing:** default `COMP_BLANK_TICKS = 160` (1.0 µs @ 160 MHz TIM1).
   Complementary PWM turns the high side on one dead-time after `CNT == 0`, so
   the hard switching transient is near **dead time**, not at zero. The window
   is sized past that edge and still clears the turn-on pile-up peak used by
   bemf_zc compensation (see comments on `COMP_BLANK_TICKS` in `targets.h`).
3. **Arm blanking only on falling-BEMF steps** in `changeCompInput()`:
   - `rising == 0` → `LL_COMP_BLANKINGSRC_TIM1_OC5_COMPx`
   - `rising == 1` → `LL_COMP_BLANKINGSRC_NONE`
4. **Software post-commutation blank** (`ZC_SEARCH_BLANK_64THS`, default 32 =
   half interval) still discards freewheel demag edges in the IRQ regardless
   of hardware blanking. That is an ignore-window, not hold-last, but it is the
   main remaining time-domain noise axis on G4 (no COMP power/speed mode).

Do **not** latch blanking on for all six steps at init. That was tried and is
wrong on rising-BEMF for the reasons above.

## What we are *not* doing (yet)

| Idea | Status |
|------|--------|
| Hardware hold-last blanking | **Unavailable** on G4 COMP |
| Invert POLARITY on rising-BEMF so pre-cross is low, then blank all six steps | Speculative: only correct if polarity is applied **before** the blank gate. RM0440 §24.3 does not state the order; needs a scope on a redirected COMP output before trusting it |
| Software EXTI mask / “ignore until CNT > N” as synthetic hold | Possible refinement; partially covered by `ZC_SEARCH_BLANK_*` and confirm |
| Longer software blank at low RPM | Sweepable via `ZC_SEARCH_BLANK_64THS`; hard ceiling near ~40/64 before eating the expected ZC (see `targets.h`) |

## Bench checks

Useful SWD peeks while free-running (app at `0x08004000`):

| Register | Expectation |
|----------|-------------|
| `TIM1_CCR5` (`0x40012C48`) | `COMP_BLANK_TICKS` (160) |
| `TIM1_CCMR3` OC5M | PWM1 mode |
| `COMPx_CSR` BLANKSEL `[21:19]` while spinning | Toggles between **0** (NONE) and **1** (TIM1 OC5) on successive steps |
| BLANKSEL at idle / after stop | Dominantly **0** |

Functional free-run checks (no prop): low-throttle startup matrix should show
short open-loop residency and no stuck latch once blanking is correctly
polarity-armed; full free-run smoke to high duty should stay in closed loop
without bemf stuck latch.

**Flight Stand note:** high free-run RPM (e.g. 1980 kV @ ~6S) can exceed a
stand optical **max RPM user cutoff** (historically 45k on the ARK bench) and
latch the stand even when the ESC is healthy. Raise that cutoff for no-prop
characterization, or gate optical cutoffs when using ESC eRPM over SWD.

## References

- ST: RM0440 (STM32G4) comparator chapter — blanking source, output path;
  LL `stm32g4xx_ll_comp.h` (`LL_COMP_ReadOutputLevel` notes VALUE before
  polarity/blanking).
- AM32 / Alka: design preference for hold-last blanking; observation that ST
  force-low only blanks rising events cleanly when EXTI is edge-triggered.
- ARK commits on this tree: TIM1 OC5 blank enable; per-step arming + 1.0 µs
  window; sweepable `ZC_SEARCH_BLANK_64THS`.
