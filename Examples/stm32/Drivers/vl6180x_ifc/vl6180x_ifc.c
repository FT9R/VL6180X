#include "vl6180x_ifc.h"
#include "cmsis_os.h"
#include "i2c.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool vl6180_Read(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (handle == NULL || data == NULL || size == 0)
        return false;

    return HAL_I2C_Mem_Read(handle, address << 1, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout) == HAL_OK;
}

static bool vl6180_Write(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (handle == NULL || data == NULL || size == 0)
        return false;

    return HAL_I2C_Mem_Write(handle, address << 1, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout) == HAL_OK;
}

static void vl6180_CE(uint8_t level)
{
    switch (level) {
    case 0:
        HAL_GPIO_WritePin(VL6180X_CE_GPIO_Port, VL6180X_CE_Pin, GPIO_PIN_RESET);
        break;

    case 1:
        HAL_GPIO_WritePin(VL6180X_CE_GPIO_Port, VL6180X_CE_Pin, GPIO_PIN_SET);
        break;

    default:
        break;
    }
}

static void vl6180_Delay(uint32_t ms)
{
    osDelay(ms);
}

static void vl6180_Print(const char *const fmt, ...)
{
#ifndef NDEBUG
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
}

void vl6180_SetUp(vl6180x_t *dev)
{
    osDelay(1000);
    memset(dev, 0, sizeof(*dev));
    dev->interface.handle = &hi2c1;
    dev->interface.read = vl6180_Read;
    dev->interface.write = vl6180_Write;
    dev->interface.ce = vl6180_CE;
    dev->interface.delay = vl6180_Delay;
#ifndef NDEBUG
    dev->interface.print = vl6180_Print;
#endif

    if (vl6180x_Init(dev, 0x29, true) != VL6180X_STAT_OK)
        Error_Handler();

    vl6180x_ConfigureDefault(dev);
}