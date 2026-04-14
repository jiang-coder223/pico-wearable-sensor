#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* 基本設定 */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0

#define configCPU_CLOCK_HZ        125000000
#define configTICK_RATE_HZ        1000

#define configMAX_PRIORITIES      5
#define configMINIMAL_STACK_SIZE  128
#define configTOTAL_HEAP_SIZE     (50 * 1024)

#define configMAX_TASK_NAME_LEN   16

/* 必填（你現在缺的） */
#define configUSE_IDLE_HOOK       0
#define configUSE_TICK_HOOK       0
#define configENABLE_MPU          0

/* Tick type（新版一定要） */
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

/* 同步 */
#define configUSE_MUTEXES         1
#define configUSE_COUNTING_SEMAPHORES 1

/* API */
#define INCLUDE_vTaskDelay        1
#define INCLUDE_vTaskDelayUntil   1
#define INCLUDE_vTaskDelete       1

#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY 1
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 1024
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTimerPendFunctionCall 1


#define configASSERT(x) if((x) == 0) { portDISABLE_INTERRUPTS(); for(;;); }

#endif