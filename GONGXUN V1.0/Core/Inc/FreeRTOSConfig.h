/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
/* USER CODE BEGIN 0 */
  extern void configureTimerForRunTimeStats(void);
  extern unsigned long getRunTimeCounterValue(void);
/* USER CODE END 0 */
#endif
#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 56 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)6400)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configGENERATE_RUN_TIME_STATS            1
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xQueueGetMutexHolder        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_eTaskGetState               1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

#define xPortSysTickHandler SysTick_Handler

/* USER CODE BEGIN 2 */
/* Definitions needed when configGENERATE_RUN_TIME_STATS is on */
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS configureTimerForRunTimeStats
#define portGET_RUN_TIME_COUNTER_VALUE getRunTimeCounterValue
/* USER CODE END 2 */

/* USER CODE BEGIN Defines */
#define configureTimerForRunTimeStats() (CPU_RunTime = 0UL) 
#define getRunTimeCounterValue() CPU_RunTime



/*
        positionEventHandle
*/

#define qRPosition (uint32_t)((1 << 0) & (0x00FFFFFF))
#define oBJPosition (uint32_t)((1 << 1) & (0x00FFFFFF))
#define hardPosition (uint32_t)((1 << 2) & (0x00FFFFFF))
#define softPosition (uint32_t)((1 << 3) & (0x00FFFFFF))
#define DoftPosition (uint32_t)((1 << 4) & (0x00FFFFFF))








/*
        carEventHandle
*/

#define positionGot (uint32_t)((1 << 1) & (0x00FFFFFF))
#define positionChange (uint32_t)((1 << 22) & (0x00FFFFFF))

#define carReady (uint32_t)((1 << 0) & (0x00FFFFFF))

#define calibGo (uint32_t)((1 << 2) & (0x00FFFFFF))
#define caliBrated (uint32_t)((1 << 3) & (0x00FFFFFF))
#define caliInited (uint32_t)((1 << 23) & (0x00FFFFFF))

#define keepStraight (uint32_t)((1 << 4) & (0x00FFFFFF))

#define calibYawAfter (uint32_t)((1 << 5) & (0x00FFFFFF))

#define rotateGO (uint32_t)((1 << 6) & (0x00FFFFFF))
#define rotateAfter (uint32_t)((1 << 7) & (0x00FFFFFF))
#define rotateInited (uint32_t)((1 << 17) & (0x00FFFFFF))

#define taskCodeGot (uint32_t)((1 << 8) & (0x00FFFFFF))
#define carGo (uint32_t)((1 << 9) & (0x00FFFFFF))

#define redDetected (uint32_t)((1 << 10) & (0x00FFFFFF))
#define greenDetected (uint32_t)((1 << 11) & (0x00FFFFFF))
#define blueDetected (uint32_t)((1 << 12) & (0x00FFFFFF))

#define armFinish (uint32_t)((1 << 13) & (0x00FFFFFF))
#define imuRawDataReceived (uint32_t)((1 << 14) & (0x00FFFFFF))
#define yawDataGot (uint32_t)((1 << 15) & (0x00FFFFFF))
#define xyGot (uint32_t)((1 << 16) & (0x00FFFFFF))

#define oBJGo (uint32_t)((1 << 18) & (0x00FFFFFF))
#define oBJReady (uint32_t)((1 << 19) & (0x00FFFFFF))

#define aRMGo (uint32_t)((1 << 20) & (0x00FFFFFF))
#define aRMReady (uint32_t)((1 << 21) & (0x00FFFFFF))


  /*
          armEventHandle
  */

#define armStandby (uint32_t)((1 << 0) & (0x00FFFFFF))
#define armQRStart (uint32_t)((1 << 2) & (0x00FFFFFF))
#define turntableStandby (uint32_t)((1 << 3) & (0x00FFFFFF))
#define turntableTake (uint32_t)((1 << 4) & (0x00FFFFFF))
#define armClosePut (uint32_t)((1 << 5) & (0x00FFFFFF))
#define armFarPut (uint32_t)((1 << 6) & (0x00FFFFFF))
#define armCloseTake (uint32_t)((1 << 7) & (0x00FFFFFF))
#define armFarTake (uint32_t)((1 << 8) & (0x00FFFFFF))
#define armAdjust (uint32_t)((1 << 9) & (0x00FFFFFF))

      extern volatile unsigned long CPU_RunTime;
  /* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
  /* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
