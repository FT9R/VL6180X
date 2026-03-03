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
// #define VL6180X_INTERLEAVED
#define VL6180X_ASYNC
#define VL6180X_USE_INTERRUPT // Can be used with `VL6180X_ASYNC`
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
    .stack_size = 128 * 4,
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
    /* add threads, ... */
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
    for (;;)
    {
        if (osMessageQueueGet(proximityQueueHandle, &proximity, NULL, osWaitForever) == osOK)
        {
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
    static vl6180x_t tof;
    static float rangeFiltered = 120;
    static float ambientFiltered = 0;
    const float alphaRange = 0.3f;
    const float alphaAmbient = 0.2f;
    const uint16_t APPLICATION_RANGE_LIMIT = 120;

    vl6180x_SetUp(&tof);
#ifdef VL6180X_SINGLE_SHOT
    /* Nothing to start */
    asm("nop");
#elif defined(VL6180X_CONTINUOUS)
    /* Choose only one: ambient or range */
    vl6180x_StartRangeContinuous(&tof, 100);
#elif defined(VL6180X_INTERLEAVED)
    vl6180x_StartInterleavedContinuous(&tof, 200);
#elif defined(VL6180X_ASYNC)
    /* Choose only one: ambient or range */
    vl6180x_StartRangeContinuous(&tof, 500);
#endif
    /* Infinite loop */
    for (;;)
    {
#ifdef VL6180X_SINGLE_SHOT
        ambientFiltered = alphaAmbient * vl6180x_ReadAmbientSingle(&tof) + (1 - alphaAmbient) * ambientFiltered;
        rangeFiltered = alphaRange * vl6180x_ReadRangeSingle(&tof) + (1 - alphaRange) * rangeFiltered;
#elif defined(VL6180X_CONTINUOUS)
        rangeFiltered = alphaRange * vl6180x_ReadRangeContinuous(&tof) + (1 - alphaRange) * rangeFiltered;
#elif defined(VL6180X_INTERLEAVED)
        ambientFiltered = alphaAmbient * vl6180x_ReadAmbientContinuous(&tof) + (1 - alphaAmbient) * ambientFiltered;
        rangeFiltered = alphaRange * vl6180x_ReadRangeContinuous(&tof) + (1 - alphaRange) * rangeFiltered;
#elif defined(VL6180X_ASYNC)
#ifdef VL6180X_USE_INTERRUPT
        if (VL6180X_GET_INT == GPIO_PIN_RESET) // Active low
#endif
        {
            uint16_t range = vl6180x_ReadRangeAsync(&tof); // Returns `VL6180X_NO_READINGS` if result is not ready
            if (range == VL6180X_NO_READINGS)
                range = APPLICATION_RANGE_LIMIT;
            rangeFiltered = alphaRange * range + (1 - alphaRange) * rangeFiltered;
        }
#endif

        osMessageQueuePut(proximityQueueHandle, &rangeFiltered, 0, 10);
        printf("Range: %.2f mm, ambient: %.2f\n", (double) rangeFiltered, (double) ambientFiltered);
        osDelay(10);
    }

    /* USER CODE END TofTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
