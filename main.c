/*************************************************************************************************/
/*!
*  \file   main.c
*
*  \brief  Main file for dats application.
*
*  Copyright (c) 2013-2019 Arm Ltd. All Rights Reserved.
*
*  Copyright (c) 2019 Packetcraft, Inc.
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*/
/*************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mxc_device.h"
#include "wut.h"
#include "lp.h"
#include "led.h"
#include "board.h"

/*! \brief  Stack initialization for app. */
extern void StackInitDats(void);
extern void bleStartup(void);

/* =| vApplicationIdleHook |==============================
 *
 *  Call the user defined function from within the idle task.  This
 *  allows the application designer to add background functionality
 *  without the overhead of a separate task.
 *  NOTE: vApplicationIdleHook() MUST NOT, UNDER ANY CIRCUMSTANCES,
 *  CALL A FUNCTION THAT MIGHT BLOCK.
 *
 * =======================================================
 */
void vApplicationIdleHook(void)
{
    /* Sleep while idle */
    LED_Off(1);

    MXC_LP_EnterSleepMode();

    LED_On(1);
}

void vTask1(void *pvParameters)
{
  TickType_t xLastWakeTime;

  /* Get task start time */
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
	printf("self defined task is running \n");
	/* Wait 1 second until next run */
	vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ);
  }
}

/*************************************************************************************************/
/*!
*  \fn     main
*
*  \brief  Entry point for demo software.
*
*  \param  None.
*
*  \return None.
*/
/*************************************************************************************************/
int main(void)
{
    /* Delay to prevent bricks */
    volatile int i;
    for(i = 0; i < 0x3FFFFF; i++) {}

	bleStartup();

    //WsfOsEnterMainLoop();

	xTaskCreate(vTask1, (const char *)"Task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);

    /* Start scheduler */
    vTaskStartScheduler();

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");
    while (1) {
        __NOP();
    }

    /* Quiet GCC warnings */
    return -1;
}


typedef struct __attribute__((packed)) ContextStateFrame {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t return_address;
    uint32_t xpsr;
} sContextStateFrame;

/*****************************************************************/
void HardFault_Handler(void)
{
    __asm(
        " TST LR, #4\n"
        " ITE EQ \n"
        " MRSEQ R0, MSP \n"
        " MRSNE R0, PSP \n"
        " B HardFault_Decoder \n");
}

/*****************************************************************/
/* Disable optimizations for this function so "frame" argument */
/* does not get optimized away */
__attribute__((optimize("O0")))
void HardFault_Decoder(sContextStateFrame *frame)
{
    /* Hang here */
    while(1) {}
}
