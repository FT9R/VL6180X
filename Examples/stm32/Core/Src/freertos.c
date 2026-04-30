/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "trace.h"
#include "usart.h"
#include "vl6180x_ifc.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define VL6180X_SINGLE_SHOT
// #define VL6180X_CONTINUOUS
#define VL6180X_INTERLEAVED
// #define VL6180X_ASYNC
// #define VL6180X_USE_INTERRUPT // Can be used with `VL6180X_ASYNC`
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
    .name = "ledTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tofTask */
osThreadId_t tofTaskHandle;
const osThreadAttr_t tofTask_attributes = {
    .name = "tofTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for proximityQueue */
osMessageQueueId_t proximityQueueHandle;
const osMessageQueueAttr_t proximityQueue_attributes = {.name = "proximityQueue"};
/* Definitions for ambientQueue */
osMessageQueueId_t ambientQueueHandle;
const osMessageQueueAttr_t ambientQueue_attributes = {.name = "ambientQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedTask(void *argument);
void TofTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
    task. It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()). If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */

#ifndef NDEBUG
    HAL_GPIO_TogglePin(HOOK_IDLE_GPIO_Port, HOOK_IDLE_Pin);
#endif
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */

#ifndef NDEBUG
    HAL_GPIO_TogglePin(HOOK_TICK_GPIO_Port, HOOK_TICK_Pin);
#endif
}

/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */

#ifndef NDEBUG
    printf("Stack overflow in %s\n", pcTaskName);
    printf("Remaining stack: %i bytes\n", osThreadGetStackSpace(xTask));
    __BKPT(2);
#endif
}

/* USER CODE END 4 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of proximityQueue */
    proximityQueueHandle = osMessageQueueNew(16, sizeof(float), &proximityQueue_attributes);

    /* creation of ambientQueue */
    ambientQueueHandle = osMessageQueueNew(16, sizeof(float), &ambientQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of ledTask */
    ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

    /* creation of tofTask */
    tofTaskHandle = osThreadNew(TofTask, NULL, &tofTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* Task switch hooks */
    vTaskSetApplicationTaskTag((TaskHandle_t) ledTaskHandle, (TaskHookFunction_t) TASK_TAG_LED);
    vTaskSetApplicationTaskTag((TaskHandle_t) tofTaskHandle, (TaskHookFunction_t) TASK_TAG_TOF);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_LedTask */
/**
 * @brief  Function implementing the ledTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
    /* USER CODE BEGIN LedTask */
    float proximity;
    /* Infinite loop */
    for (;;) {
        if (osMessageQueueGet(proximityQueueHandle, &proximity, NULL, osWaitForever) == osOK) {
            if (proximity < 11.0f)
                LEDG_ON, LEDY_OFF, LEDR_OFF;
            else if (proximity < 30.0f)
                LEDG_OFF, LEDY_ON, LEDR_OFF;
            else if (proximity < 40.0f)
                LEDG_OFF, LEDY_OFF, LEDR_ON;
            else
                LEDG_OFF, LEDY_OFF, LEDR_OFF;
        }
    }
    /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_TofTask */
/**
 * @brief Function implementing the tofTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TofTask */
void TofTask(void *argument)
{
    /* USER CODE BEGIN TofTask */
    static uint32_t stackFree;
    static vl6180x_t tof;
    static vl6180x_status_t statusRange, statusAmbient;
    const uint32_t MEASUREMENT_PERIOD = 200; // [ms]
    char plotString[50] = {0};

    static struct {
        float filtered;
        const float alpha;
        uint16_t raw;
    } ambient = {.alpha = 0.15};

    static struct {
        float filtered;
        const float alpha;
        uint16_t raw;
        uint16_t limit;
    } range = {.alpha = 0.2, .limit = 120};

    vl6180_SetUp(&tof);
#ifdef VL6180X_SINGLE_SHOT
    /* Nothing to start */
#elif defined(VL6180X_CONTINUOUS) || defined(VL6180X_ASYNC)
    /* Choose only one: ambient or range */
    vl6180x_StartContinuous(&tof, VL6180X_MODE_RANGE, MEASUREMENT_PERIOD);
#elif defined VL6180X_INTERLEAVED
    vl6180x_StartContinuous(&tof, VL6180X_MODE_INTERLEAVED, MEASUREMENT_PERIOD);
#else
#error "No VL6180X mode defined"
#endif
    /* Infinite loop */
    for (;;) {
#if defined(VL6180X_SINGLE_SHOT) || defined(VL6180X_INTERLEAVED)
        statusAmbient = vl6180x_Read(&tof, &ambient.raw, VL6180X_MODE_ALS, MEASUREMENT_PERIOD);
#endif
#if defined(VL6180X_ASYNC) && defined(VL6180X_USE_INTERRUPT)
        if (VL6180X_GET_INT == GPIO_PIN_RESET)
#endif
#ifndef VL6180X_ASYNC
            statusRange = vl6180x_Read(&tof, &range.raw, VL6180X_MODE_RANGE, MEASUREMENT_PERIOD);
#else
        statusRange = vl6180x_Read(&tof, &range.raw, VL6180X_MODE_RANGE, 0);
#endif

        if (statusAmbient != VL6180X_STAT_OK)
            printf("Incomplete ALS measurement, status: %u\n", statusAmbient);
        if (statusRange != VL6180X_STAT_OK)
            printf("Incomplete range measurement, status: %u\n", statusRange);
        if (statusAmbient == VL6180X_STAT_OK && statusRange == VL6180X_STAT_OK) {
            /* Limiter */
            range.raw = range.raw > range.limit ? range.limit : range.raw;

            /* EMA filter */
            ambient.filtered = ambient.alpha * ambient.raw + (1.0f - ambient.alpha) * ambient.filtered;
            range.filtered = range.alpha * range.raw + (1.0f - range.alpha) * range.filtered;

            /* Logic */
            osMessageQueuePut(proximityQueueHandle, &range.filtered, 0, 10);
            printf("Ambient: %.2f count, range: %.2f mm\n", (double) ambient.filtered, (double) range.filtered);
            snprintf(plotString, sizeof(plotString), "%.2f\r\n", (double) range.filtered);
            HAL_UART_Transmit(&huart4, (const uint8_t *) plotString, strnlen(plotString, sizeof(plotString)), 1000);
        }

        stackFree = osThreadGetStackSpace(osThreadGetId());
        osDelay(MEASUREMENT_PERIOD);
    }
    /* USER CODE END TofTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
