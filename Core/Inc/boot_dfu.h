/**
  ******************************************************************************
  * @file    boot_dfu.h
  * @brief   Software entry into the ST system-memory bootloader (USB DFU).
  *
  * USER-OWNED FILE. CubeMX has never generated this and must never own it.
  * Placement rule 1: survives regeneration without USER CODE fences.
  *
  * Why software entry: BOOT0 is not accessible on this board, so the
  * BOOT0-pin pattern is unavailable.
  *
  * Mechanism (reset first, jump second):
  *   1. BootDFU_RequestAndReset() writes BOOT_DFU_MAGIC to RTC_BKP0R and
  *      issues a system reset. RTC_BKPxR is "System reset: not affected"
  *      (RM0434 Rev 16 §34.7.20).
  *   2. The system reset clears PWR_CR4.C2BOOT (reset value 0, RM0434 §5.6.4),
  *      so CPU2 and the BLE stack stay parked (RM0434 §2.4). This matters: the
  *      bootloader clocks USB from HSI48 + CRS (AN2606 Rev 70 Table 183), which
  *      is exactly the oscillator CPU2 powers down once BLE runs.
  *   3. BootDFU_CheckAndJump(), first statement of main(), sees the magic,
  *      clears it, remaps system flash to 0x0000 0000 and jumps to it with the
  *      chip still in post-reset state (no PLL, no SysTick, no NVIC enables,
  *      PRIMASK clear) - which satisfies AN2606 §4.1 "jump from user code".
  *
  * Exit from DFU: power-cycle (the flag is cleared before the jump).
  ******************************************************************************
  */

#ifndef BOOT_DFU_H
#define BOOT_DFU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Arbitrary signature. Only an exact match triggers the jump, so a stale or
 * random backup-register value boots the application normally. */
#define BOOT_DFU_MAGIC   (0xB007DF00UL)

/* Call as the FIRST statement of main(), before HAL_Init(). Returns normally
 * unless a DFU request is pending, in which case it never returns. */
void BootDFU_CheckAndJump(void);

/* Set the request flag and reset the device. Never returns. The caller is
 * responsible for closing files and detaching USB first. */
void BootDFU_RequestAndReset(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* BOOT_DFU_H */
