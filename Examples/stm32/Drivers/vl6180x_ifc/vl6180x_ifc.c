#include "vl6180x_ifc.h"
#include "cmsis_os.h"
#include "i2c.h"
#include <string.h>

static uint8_t vl6180_Read(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (handle == NULL)
        return 0;
    if (data == NULL)
        return 0;
    if (size == 0)
        return 0;

    return HAL_I2C_Mem_Read(handle, address << 1, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout) == HAL_OK;
}

static uint8_t vl6180_Write(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size,
                            uint32_t timeout)
{
    if (handle == NULL)
        return 0;
    if (data == NULL)
        return 0;
    if (size == 0)
        return 0;

    return HAL_I2C_Mem_Write(handle, address << 1, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout) == HAL_OK;
}

static void vl6180_CE(uint8_t level)
{
    switch (level)
    {
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

static void vl6180x_Delay(uint32_t ms)
{
    osDelay(ms);
}

void vl6180x_SetUp(vl6180x_t *dev)
{
    memset(dev, 0, sizeof(vl6180x_t));
    dev->interface.handle = &hi2c1;
    dev->interface.read = vl6180_Read;
    dev->interface.write = vl6180_Write;
    dev->interface.ce = vl6180_CE;
    dev->interface.delay = vl6180x_Delay;
    dev->sampleReadyTimeout = 1000;
    // dev->address = 0b0101001;

    vl6180x_Init(dev, true);
    vl6180x_ConfigureDefault(dev);

    if (dev->error != VL6180X_ERR_NONE)
        Error_Handler();
}