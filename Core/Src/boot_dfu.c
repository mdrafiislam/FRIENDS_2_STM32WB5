/**
  ******************************************************************************
  * @file    boot_dfu.c
  * @brief   Software entry into the ST system-memory bootloader (USB DFU).
  *
  * USER-OWNED FILE. See boot_dfu.h for the mechanism and its references.
  *
  * Register-level facts used here, and where they come from:
  *
  *   System memory     0x1FFF0000, 28 KB, holds the bootloader
  *                     (RM0434 Rev 16 §2.2 Table 1; AN2606 Rev 70 Table 183;
  *                     SYSTEM_MEMORY_BASE / SYSTEM_MEMORY_END_ADDR in
  *                     stm32wb5mxx.h). CPU fetches top-of-stack from +0 and
  *                     the reset vector from +4 (RM0434 §2.3).
  *
  *   SYSCFG_MEMRMP     offset 0x000, MEM_MODE[2:0] bits 2:0 (RM0434 §11.2.1):
  *                       000 = main flash at 0x0000 0000
  *                       001 = system flash at 0x0000 0000
  *                     Written via __HAL_SYSCFG_REMAPMEMORY_FLASH() /
  *                     __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH()
  *                     (stm32wbxx_hal.h -> LL_SYSCFG_SetRemapMemory, which is
  *                     MODIFY_REG(MEMRMP, MEM_MODE, 0 or MEM_MODE_0)).
  *                     SYSCFG has no clock-enable bit on this part: neither
  *                     RCC_APB2ENR in stm32wb5mxx.h nor stm32wbxx_hal_rcc.h
  *                     defines one.
  *
  *   RTC_BKP0R         RTC offset 0x50, not affected by system reset
  *                     (RM0434 §34.7.20). Exempt from the RTC_WPR key
  *                     sequence; write access needs only PWR_CR1.DBP = 1
  *                     (RM0434 §34.4.6). Readable with RCC_APB1ENR1.RTCAPBEN,
  *                     whose reset value is 1 (APB1ENR1 reset 0x0000 0400,
  *                     bit 10, RM0434 §6.4.20).
  *
  *   PWR_CR1.DBP       set via HAL_PWR_EnableBkUpAccess() (SET_BIT(PWR->CR1,
  *                     PWR_CR1_DBP), stm32wbxx_hal_pwr.c). Written twice to
  *                     flush the APB-AHB bridge, as app_entry.c
  *                     Reset_BackupDomain() does. PWR has no clock-enable bit
  *                     on this part (RM0434 §6.4.20 lists none).
  *
  * ASSUMPTION: RTC_BKP0R reads correctly at the top of main(), where
  * SystemInit() has already cleared LSI1ON/LSI2ON so RTCCLK is stopped.
  * RM0434 §34 states the register survives system reset but does not state
  * whether an APB read needs RTCCLK running. Observable: if this assumption is
  * wrong, 'b' simply reboots into the application instead of DFU.
  *
  * ASSUMPTION: the STM32WB5MMG module die runs the same bootloader as the
  * STM32WB55xx entry in AN2606 §81 (ID 0xD5 at 0x1FFF6FFE). Confirm once by
  * reading that byte over ST-LINK.
  ******************************************************************************
  */

#include "boot_dfu.h"
#include "main.h"      /* stm32wbxx_hal.h: CMSIS device header, SCB, HAL_PWR */

/* Stack pointer sanity window: the bootloader uses the first 20 KB of SRAM1
 * (AN2606 Table 183), so its initial MSP must lie inside SRAM1. */
#define BOOT_DFU_SRAM_LO   (SRAM_BASE)
#define BOOT_DFU_SRAM_HI   (SRAM_BASE + SRAM1_SIZE)

static void BootDFU_EnableBackupWrite(void)
{
  HAL_PWR_EnableBkUpAccess();
  HAL_PWR_EnableBkUpAccess();   /* second write flushes the APB-AHB bridge */
}

void BootDFU_CheckAndJump(void)
{
  if (RTC->BKP0R != BOOT_DFU_MAGIC)
  {
    /* Normal boot. Restore the canonical map explicitly so the application
     * vectors from its own table even if it was entered through the
     * bootloader's Go command ("Run after programming"), which AN2606 §4.1
     * says does not reset the peripherals the bootloader used.
     *   MEM_MODE[2:0] = 000 -> main flash at 0x0000 0000 (RM0434 §11.2.1)
     *   VTOR = FLASH_BASE (0x08000000), where the linker script places
     *   .isr_vector first in FLASH. On a clean reset both writes are no-ops
     *   in effect: MEM_MODE is already 000 and 0x0 already aliases flash. */
    __HAL_SYSCFG_REMAPMEMORY_FLASH();
    SCB->VTOR = FLASH_BASE;
    __DSB();
    __ISB();
    return;
  }

  /* Consume the request first, so a power cycle or reset out of the
   * bootloader returns to the application rather than looping into DFU. */
  BootDFU_EnableBackupWrite();
  RTC->BKP0R = 0U;
  (void)RTC->BKP0R;             /* read back: write has landed before we go on */
  HAL_PWR_DisableBkUpAccess();  /* DBP back to its post-reset value */

  const uint32_t boot_sp = *(volatile const uint32_t *)(SYSTEM_MEMORY_BASE);
  const uint32_t boot_pc = *(volatile const uint32_t *)(SYSTEM_MEMORY_BASE + 4U);

  /* If system memory does not look like a vector table (e.g. RDP level 2
   * blocks it), boot the application instead of faulting. */
  if ((boot_sp <= BOOT_DFU_SRAM_LO) || (boot_sp > BOOT_DFU_SRAM_HI) ||
      (boot_pc < SYSTEM_MEMORY_BASE) || (boot_pc > SYSTEM_MEMORY_END_ADDR))
  {
    return;
  }

  /* MEM_MODE[2:0] = 001 -> system flash aliased at 0x0000 0000
   * (RM0434 §11.2.1), the same map a BOOT0-selected system-memory boot
   * produces (RM0434 §2.3). VTOR = 0 then points at the bootloader's table
   * through that alias. */
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  SCB->VTOR = 0U;
  __DSB();
  __ISB();

  /* AN2606 §4.1 pre-jump conditions, all already true at this point because
   * nothing has run since reset except SystemInit():
   *   - peripheral clocks off, PLL off   (HAL_Init/SystemClock_Config not run)
   *   - no interrupts enabled or pending (NVIC untouched, SysTick not started)
   *   - interrupts globally enabled      (PRIMASK not set by anyone)
   * AN2606 requires interrupts to be ENABLED for the USB DFU bootloader, so
   * there is deliberately no __disable_irq() here.
   *
   * MSP and the branch are done in one asm block so no compiler-generated
   * stack access can occur between switching stacks and leaving. */
  __asm volatile (
    "msr msp, %0 \n\t"
    "bx  %1      \n\t"
    :
    : "r" (boot_sp), "r" (boot_pc)
    : "memory");

  for (;;) { }                  /* not reached */
}

void BootDFU_RequestAndReset(void)
{
  BootDFU_EnableBackupWrite();
  RTC->BKP0R = BOOT_DFU_MAGIC;
  (void)RTC->BKP0R;             /* read back before resetting */

  /* SYSRESETREQ -> system reset of the whole device (RM0434 §6.1.2).
   * __NVIC_SystemReset() issues DSB before and after the AIRCR write
   * (core_cm4.h). */
  NVIC_SystemReset();
}
