/*************************************************************************************************/
/*!
 * \file
 *
 * \brief  System hooks.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/
#include <string.h>

#include "pal_sys.h"
#include "pal_led.h"
#include "pal_rtc.h"
#include "pal_timer.h"
#include "pal_crypto.h"
#include "pal_uart.h"
#include "pal_bb.h"
#include "mxc_device.h"
#include "board.h"
#include "mcr_regs.h"
#include "gcr_regs.h"
#include "lp.h"
#include "wut.h"
#include "uart.h"
#include "sema.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef PAL_SYS_ENABLE_STANDBY
#define PAL_SYS_ENABLE_STANDBY            0
#endif

#define PAL_SYS_MIN_STANDBY_USEC          3000

/* Empirically determined wakeup times */
#ifdef __riscv
#define PAL_SYS_STANDBY_WAKEUP_USEC_120   1850  /* 120 MHz clock */
#define PAL_SYS_STANDBY_WAKEUP_USEC_60    2200  /*  60 MHz clock */
#else
#define PAL_SYS_STANDBY_WAKEUP_USEC_120   950  /* 120 MHz clock */
#define PAL_SYS_STANDBY_WAKEUP_USEC_60    1100 /*  60 MHz clock */
#endif

#ifndef PAL_SYS_RISCV_LOAD
#define PAL_SYS_RISCV_LOAD                0
#endif

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      Number of assertions. */
static uint32_t palSysAssertCount;

/*! \brief      Trap enabled flag. */
static volatile bool_t PalSysAssertTrapEnable;

/*! \brief      Busy client count. */
static uint32_t palSysBusyCount;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Enter a critical section.
 */
/*************************************************************************************************/
void PalEnterCs(void)
{
  __disable_irq();
}

/*************************************************************************************************/
/*!
 *  \brief  Exit a critical section.
 */
/*************************************************************************************************/
void PalExitCs(void)
{
  __enable_irq();
}

/*************************************************************************************************/
/*!
 *  \brief      Common platform initialization.
 */
/*************************************************************************************************/
void PalSysInit(void)
{
  /* Delay to prevent lockup when debugging */
#ifdef DEBUG
  volatile int i;
  for(i = 0; i < 0x3FFFF; i++) {}
#endif

  palSysAssertCount = 0;
  PalSysAssertTrapEnable = TRUE;
  palSysBusyCount = 0;

  /* Enable wakeup sources */
  MXC_PWRSEQ->lppwen |= (MXC_F_PWRSEQ_LPPWEN_CPU1 | MXC_F_PWRSEQ_LPPWEN_UART0 | MXC_F_PWRSEQ_LPPWEN_UART1 |
                         MXC_F_PWRSEQ_LPPWEN_UART2 | MXC_F_PWRSEQ_LPPWEN_UART3 | MXC_F_PWRSEQ_LPPWEN_TMR0 | MXC_F_PWRSEQ_LPPWEN_TMR1);

  PalLedInit();
  PalLedOff(PAL_LED_ID_ERROR);
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);
  PalCryptoInit();
  PalRtcInit();

#ifndef __riscv
#if PAL_SYS_RISCV_LOAD

  /* Halt the RISCV */
  MXC_SYS_RISCVShutdown();

#ifdef DEBUG
  /* Enable RISCV debugger GPIO */
  MXC_GPIO_Config(&gpio_cfg_rv_jtag);
#endif

  /* Initialize the Semaphore peripheral */
  MXC_SEMA_Init();
  MXC_SEMA_InitBoxes();

  /* Enable semaphore interrupt and clear state */
  NVIC_ClearPendingIRQ(RISCV_IRQn);
  NVIC_EnableIRQ(RISCV_IRQn);

  /* Start the RISCV core */
  MXC_SYS_RISCVRun();

  /* Give the RISCV time to startup */
  volatile int j;
  for(j = 0; j < 0xFFFFFF; j++) {}

#endif
#endif

#ifdef __riscv
  /* Initialize the Semaphore peripheral */
  MXC_SEMA_Init();

  /* Enable ARM incoming interrupts */
  NVIC_ClearPendingIRQ(PF_IRQn);
  NVIC_EnableIRQ(PF_IRQn);

#endif
}

/*************************************************************************************************/
/*!
 *  \brief      System fault trap.
 */
/*************************************************************************************************/
void PalSysAssertTrap(void)
{

  PalEnterCs();
  PalLedOn(PAL_LED_ID_ERROR);
  palSysAssertCount++;
  while (PalSysAssertTrapEnable);
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Set system trap.
 *
 *  \param      enable    Enable assert trap or not.
 */
/*************************************************************************************************/
void PalSysSetTrap(bool_t enable)
{
  PalSysAssertTrapEnable = enable;
}

/*************************************************************************************************/
/*!
 *  \brief      Get assert count.
 */
/*************************************************************************************************/
uint32_t PalSysGetAssertCount(void)
{
  return palSysAssertCount;
}

/*************************************************************************************************/
/*!
 *  \brief      Count stack usage.
 *
 *  \return     Number of bytes used by the stack.
 */
/*************************************************************************************************/
uint32_t PalSysGetStackUsage(void)
{
  /* Not available; stub routine. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Set WUT and enter standby.
 */
/*************************************************************************************************/
static void palSysEnterStandby(void)
{
  uint32_t rtcCount, schUsec, targetTick, rtcElapsed, schUsecElapsed;
  uint64_t wakeupUs;

  /* Get the time until the next event */
  MXC_WUT_Store();
  rtcCount = MXC_WUT->cnt;
  schUsec = PalTimerGetExpTime();

  /* TODO: Figure out if RTC is active */

  /* Regular sleep if we don't have time for deep sleep */
  if (schUsec < PAL_SYS_MIN_STANDBY_USEC) {
    MXC_LP_EnterSleepMode();
    return;
  }

  /* Determine the time needed for wakeup restore based on the system clock */
  if(SystemCoreClock == 120000000) {
    wakeupUs = PAL_SYS_STANDBY_WAKEUP_USEC_120;
  } else {
    wakeupUs = PAL_SYS_STANDBY_WAKEUP_USEC_60;
  }
  /* Arm WUT for wakeup from scheduler timer */
  targetTick = rtcCount;
  targetTick += (uint64_t)(schUsec - wakeupUs) *
    (uint64_t)PAL_RTC_TICKS_PER_SEC / (uint64_t)1000000;

  MXC_WUT->cmp = targetTick;

  /* Enable wakeup from WUT */
  NVIC_EnableIRQ(WUT0_IRQn);
  MXC_LP_EnableWUTAlarmWakeup();

  /* Stop the scheduler timer */
  PalTimerStop();

  /* Shutdown BB */
  PalBbDisable();

  /* Re-enable interrupts for wakeup */
  PalExitCs();

#ifndef __riscv
  MXC_LP_EnterStandbyMode();
#else
  /* Wake the ARM to enter standby */
  MXC_SEMA->irq1 |= MXC_F_SEMA_REVA_IRQ1_EN;
  MXC_LP_EnterSleepMode();
#endif

  /* Disable interrupts until we complete the recovery */
  PalEnterCs();

  /* Restore the BB hardware */
  PalBbEnable();
  PalBbRestore();

  /* Restore the BB counter */
  MXC_WUT_RestoreBBClock(BB_CLK_RATE_HZ);

  /* Update the scheduler timer */
  rtcElapsed = MXC_WUT->cnt - rtcCount;
  schUsecElapsed = (uint64_t)rtcElapsed * (uint64_t)1000000 / (uint64_t)PAL_RTC_TICKS_PER_SEC;
  PalTimerRestore(schUsec - schUsecElapsed);

  /* Reset RTC compare value to prevent unintended rollover */
  MXC_WUT->cmp = PAL_MAX_RTC_COUNTER_VAL;
}

/*************************************************************************************************/
/*!
 *  \brief      System sleep.
 *
 *  \param      nextWakeMs  Next CPU wakeup time.
 *
 *  \note       Caller of this routine must ensure IRQ are disabled before entering this call.
 */
/*************************************************************************************************/
void PalSysSleep(void)
{
  if (palSysBusyCount) {
    /* Work pending; do not sleep yet. */
    return;
  }

  #ifdef DEBUG
  #if PAL_SYS_ENABLE_STANDBY == 0
  /* Stay active to prevent debugger dropout */
  return;
  #endif
  #endif

  /* Can not disable BLE DBB and 32 MHz clock while trim procedure is ongoing */
  if(MXC_WUT_TrimPending() != E_NO_ERROR) {
    MXC_LP_EnterSleepMode();
    return;
  }

  /* Figure out if the UART is active */
  if(PalUartGetState(PAL_UART_ID_TERMINAL) == PAL_UART_STATE_BUSY) {
    MXC_LP_EnterSleepMode();
    return;
  }

  /* Figure out if the scheduler timer is active */
  if(PalTimerGetState() != PAL_TIMER_STATE_BUSY) {
    MXC_LP_EnterSleepMode();
    return;
  }

  /* Prevent characters from being corrupted if still transmitting,
    UART will shutdown in deep sleep  */
  if(MXC_UART_GetActive(MXC_UART_GET_UART(TERMINAL_UART)) != E_NO_ERROR) {
    /* We will not get another UART interrupt, sleep for a short amount of time
      before determining if we can enter standby */
    PalTimerSleep(25);
    return;
  }

  if(PAL_SYS_RISCV_LOAD == 1) {
    uint32_t wutDiff = MXC_WUT_GetCompare() - MXC_WUT_GetCount();

    /* Enter DS if RISCV primed WUT for wakeup */
    if(PAL_SYS_ENABLE_STANDBY && ((wutDiff > 10) && (wutDiff < 0x100000))) {
      MXC_LP_EnterStandbyMode();
    } else {
      MXC_LP_EnterLowPowerMode();
    }
    return;
  }

  if(PAL_SYS_ENABLE_STANDBY) {
    palSysEnterStandby();
  } else {
    MXC_LP_EnterSleepMode();
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Set system busy.
 */
/*************************************************************************************************/
void PalSysSetBusy(void)
{
  PalEnterCs();
  palSysBusyCount++;
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Set system idle.
 */
/*************************************************************************************************/
void PalSysSetIdle(void)
{
  PalEnterCs();
  if (palSysBusyCount) {
    palSysBusyCount--;
  }
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Check if system is busy.
 *
 *  \return     TRUE if system is busy.
 */
/*************************************************************************************************/
bool_t PalSysIsBusy(void)
{
  bool_t sysIsBusy = FALSE;
  PalEnterCs();
  sysIsBusy = ((palSysBusyCount == 0) ? FALSE : TRUE);
  PalExitCs();
  return sysIsBusy;
}
