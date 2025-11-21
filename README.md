# VL6180X Driver

Platform-independent, dependency-free C driver for the **ST VL6180X Time-of-Flight distance and ambient light sensor**.  
Provides initialization, configuration and measurement functions for range (mm) and ALS (ambient light) using an abstracted I²C interface.  
This driver based on [`Pololu's VL6180X library for Arduino`](https://github.com/pololu/vl6180x-arduino).


## Features

- Platform-independent
- A few devices on the same and different I2C bus support with separated handles
- Polling, Interrupt and Asynchronous modes support
- Hardware reset support  
- Device ID validation  
- Timeouts for blocking operations  
- Range scaling (1x–3x)  
- No external library dependencies, except for standard C

## Usage
- Assign your platform-specific functions:

```C
uint8_t (*read)(void *handle, uint16_t addr, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
uint8_t (*write)(void *handle, uint16_t addr, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
void (*ce)(uint8_t level); // 0 - pin reset
void (*delay)(uint32_t ms);
void *handle; // Optional, used by platform read/write functions
```

- Common device setup
```C
void vl6180x_SetUp(vl6180x_t *dev)
{
    i2c1_Init(&i2c1Handle, 100000);
    gpio_Init(); // For chip enable (gpio0) and interrupt pin (gpio1)

    memset(dev, 0, sizeof(vl6180x_t));
    tof.interface.handle = &i2c1Handle; // Optional, used by ctx_i2c_read and ctx_i2c_write
    tof.interface.read  = ctx_i2c_read;
    tof.interface.write = ctx_i2c_write;
    tof.interface.ce    = ctx_gpio_ce;
    tof.interface.delay = ctx_delay;
    tof.sampleReadyTimeout = 1000;
    tof.address = 0b1010101; // If you are sure that device has non-default address 

    vl6180x_Init(&tof, true);
    vl6180x_ConfigureDefault(&tof);
}
```

1. Single‑Shot (Range + Ambient)
```C
void Task_SingleShot(void)
{
    vl6180x_t tof;
    vl6180x_SetUp(&tof);

    while (1)
    {
        uint16_t range = vl6180x_ReadRangeSingle(&tof);
        uint16_t ambient = vl6180x_ReadAmbientSingle(&tof);
    }
}
```

2. Continuous Range Measurement
```C
void Task_ContinuousRange(void)
{
    vl6180x_t tof;
    vl6180x_SetUp(&tof);
    vl6180x_StartRangeContinuous(&tof, 100);

    while (1)
    {
        uint16_t range = vl6180x_ReadRangeContinuous(&tof);
    }
}
```

3. Interleaved Continuous (Ambient + Range)
```C
void Task_Interleaved(void)
{
    vl6180x_t tof;
    vl6180x_SetUp(&tof);
    vl6180x_StartInterleavedContinuous(&tof, 100);

    while (1)
    {
        uint16_t ambient = vl6180x_ReadAmbientContinuous(&tof);
        uint16_t range = vl6180x_ReadRangeContinuous(&tof);
    }
}
```
4. Asynchronous Range (Polling Interrupt Register - no interrupt pin used)  
```C
void Task_AsyncPoll(void)
{
    vl6180x_t tof;
    vl6180x_SetUp(&tof);
    vl6180x_StartRangeContinuous(&tof, 500);

    while (1)
    {
        uint16_t range = vl6180x_ReadRangeAsync(&tof); // UINT16_MAX if not ready

        if (range != UINT16_MAX)
        {
            // Use range result
        }
    }
}
```

5. Asynchronous with External GPIO (Active‑Low GPIO1)
```C
void Task_AsyncInt(void)
{
    vl6180x_t tof;
    vl6180x_SetUp(&tof);
    vl6180x_StartRangeContinuous(&tof, 500);

    while (1)
    {
        if (IsIntPinLow())
            uint16_t range = vl6180x_ReadRangeAsync(&tof);
    }
}
```
## Examples
* [`STM32 + FreeRTOS`](/Examples/stm32/Core/Src/freertos.c)
