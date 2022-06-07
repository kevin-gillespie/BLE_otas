/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

/* MXC */
#include "mxc_device.h"
#include "board.h"
#include "mxc_assert.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* Maxim CMSIS */
#include "lp.h"
#include "pwrseq_regs.h"
#include "wut.h"
#include "mcr_regs.h"
#include "icc.h"
#include "pb.h"
#include "led.h"
#include "uart.h"

/* Bluetooth Cordio library */
#include "pal_timer.h"
#include "pal_uart.h"
#include "pal_bb.h"

#define WUT_RATIO           (configRTC_TICK_RATE_HZ / configTICK_RATE_HZ)
#define MAX_WUT_SNOOZE      (5*configRTC_TICK_RATE_HZ)
#define MIN_SYSTICK         2
#define MIN_WUT_TICKS       150
#define WAKEUP_US           1000

/*
 * Sleep-check function
 *
 * Your code should over-ride this weak function and return E_NO_ERROR if
 * tickless sleep is permissible (ie. no UART/SPI/I2C activity). Any other
 * return code will prevent FreeRTOS from entering tickless idle.
 */
int freertos_permit_tickless(void)
{
    /* Can not disable BLE DBB and 32 MHz clock while trim procedure is ongoing */
    if(MXC_WUT_TrimPending() != E_NO_ERROR) {
        return E_BUSY;
    }

    /* Figure out if the UART is active */
    if(PalUartGetState(PAL_UART_ID_TERMINAL) == PAL_UART_STATE_BUSY) {
        return E_BUSY;
    }

    /* Figure out if the scheduler timer is active */
    if(PalTimerGetState() != PAL_TIMER_STATE_BUSY) {
        return E_BUSY;
    }

    /* Prevent characters from being corrupted if still transmitting,
      UART will shutdown in deep sleep  */
    if(MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {
        /* We will not get another UART interrupt, sleep for a short amount of time
          before determining if we can enter standby */
        return E_BUSY;
    }

    return E_NO_ERROR;
}

/*
 * This function overrides vPortSuppressTicksAndSleep in portable/.../ARM_CM4F/port.c
 *
 * DEEPSLEEP mode will stop SysTick from counting, so that can't be
 * used to wake up. Instead, calculate a wake-up period for the WUT to
 * interrupt the WFI and continue execution.
 *
 */
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    uint32_t idleTicks, actualTicks, preCapture, postCapture;
    uint32_t schUsec, schUsecElapsed, bleSleepTicks;

    /* We do not currently handle to case where the WUT is slower than the RTOS tick */
    MXC_ASSERT(configRTC_TICK_RATE_HZ >= configTICK_RATE_HZ);

    if (SysTick->VAL < MIN_SYSTICK) {
        /* Avoid sleeping too close to a systick interrupt */
        return;
    }

    /* Calculate the number of WUT ticks, but we need one to synchronize */
    idleTicks = (xExpectedIdleTime - 1) * WUT_RATIO;

    if(idleTicks > MAX_WUT_SNOOZE) {
        idleTicks = MAX_WUT_SNOOZE;
    }

    /* Check to see if we meet the minimum requirements for deep sleep */
    if (idleTicks < MIN_WUT_TICKS) {
        return;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
       method as that will mask interrupts that should exit sleep mode. */
    __asm volatile( "cpsid i" );

    /* If a context switch is pending or a task is waiting for the scheduler
       to be unsuspended then abandon the low power entry. */
    /* Also check the MXC drivers for any in-progress activity */
    if ((eTaskConfirmSleepModeStatus() == eAbortSleep) ||
            (freertos_permit_tickless() != E_NO_ERROR)) {
        /* Re-enable interrupts - see comments above the cpsid instruction()
           above. */
        __asm volatile( "cpsie i" );
        return;
    }

    /* Disable SysTick */
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);

    /* Snapshot the current WUT value */
    MXC_WUT_Store();
    preCapture = MXC_WUT_GetCount();
    schUsec = PalTimerGetExpTime();

    /* Regular sleep if we don't have time for deep sleep */
    if (schUsec < (MIN_WUT_TICKS * 1000000 / configRTC_TICK_RATE_HZ)) {
        MXC_LP_EnterSleepMode();
    } else {

        /* Adjust idleTicks for the time it takes to restart the BLE hardware */
        idleTicks -= (uint64_t)(WAKEUP_US) *
                     (uint64_t)configRTC_TICK_RATE_HZ / (uint64_t)1000000;

        /* Calculate the time to the next BLE scheduler event */
        bleSleepTicks = (uint64_t)(schUsec - WAKEUP_US) *
                        (uint64_t)configRTC_TICK_RATE_HZ / (uint64_t)1000000;

        if(bleSleepTicks < idleTicks) {
            MXC_WUT->cmp = preCapture + bleSleepTicks;
        } else {
            MXC_WUT->cmp = preCapture + idleTicks;
        }

        /* Stop the scheduler timer */
        PalTimerStop();

        /* Shutdown BB */
        PalBbDisable();

        LED_Off(1);

        MXC_LP_EnterSleepMode();

        LED_On(1);

        /* Restore the BB hardware */
        PalBbEnable();
        PalBbRestore();

        /* Restore the BB counter */
        MXC_WUT_RestoreBBClock(BB_CLK_RATE_HZ);

        /* Update the scheduler timer */
        actualTicks = MXC_WUT->cnt - preCapture;
        schUsecElapsed = (uint64_t)actualTicks * (uint64_t)1000000 / (uint64_t)configRTC_TICK_RATE_HZ;
        PalTimerRestore(schUsec - schUsecElapsed);
    }

    /* Recalculate actualTicks for the FreeRTOS tick counter update */
    postCapture = MXC_WUT_GetCount();
    actualTicks = postCapture - preCapture;

    /* Re-enable interrupts - see comments above the cpsid instruction()
       above. */
    __asm volatile( "cpsie i" );

    /*
     * Advance ticks by # actually elapsed
     */
    portENTER_CRITICAL();
    vTaskStepTick( (actualTicks / WUT_RATIO) );
    portEXIT_CRITICAL();

    /* Re-enable SysTick */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

}
