# VL6180X Driver

Platform-independent, dependency-free C driver for the **ST VL6180X Time-of-Flight distance and ambient light sensor**.  
Provides initialization, configuration and measurement functions for range (mm) and ALS (ambient light) using an abstracted I²C interface.  
This driver based on [`Pololu's VL6180X library for Arduino`](https://github.com/pololu/vl6180x-arduino).


## Features

- Platform-independent
- No external library dependencies, except for standard C
- Device ID validation  
- Hardware reset support  
- A few devices on the same and different I2C bus support with separated handles
- Polling, Interrupt and Asynchronous modes support
- Timeouts for blocking operations  
- Ability to change range scaling (1x–2x-3x) and part to part offset

## Usage
- Assign platform-specific functions within handle prior `vl6180x_Init()`:

```C
void *handle; // Optional handle for I2C interface
void (*print)(const char *const fmt, ...); // Optional debug print
void (*ce)(uint8_t level); // Optional chip enable (gpio0), open drain - no pull - active high
bool (*read)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
bool (*write)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
void (*delay)(uint32_t ms);
```

- Minimal device setup
```C
void vl6180_SetUp(vl6180x_t *dev)
{
    i2c1_Init(&i2c1Handle, 400000);

    memset(dev, 0, sizeof(*dev));
    dev->interface.read = vl6180_Read;
    dev->interface.write = vl6180_Write;
    dev->interface.delay = vl6180_Delay;
    vl6180x_Init(dev, 0x29, false);
}
```

1. Single‑Shot (Ambient + Range)
```C
void Task_SingleShot(void)
{
    vl6180x_t tof;
    uint16_t ambient, range;
    vl6180_SetUp(&tof);

    while (1) {
        vl6180x_Read(&tof, &ambient, VL6180X_MODE_ALS, 200);
        vl6180x_Read(&tof, &range, VL6180X_MODE_RANGE, 200);
    }
}
```

2. Continuous Range
```C
void Task_ContinuousRange(void)
{
    vl6180x_t tof;
    uint16_t range;
    vl6180_SetUp(&tof);
    vl6180x_StartContinuous(&tof, VL6180X_MODE_RANGE, 100);

    while (1)
        vl6180x_Read(&tof, &range, VL6180X_MODE_RANGE, 200);
}
```

3. Interleaved Continuous (Ambient + Range)
```C
void Task_ContinuousInterleaved(void)
{
    vl6180x_t tof;
    uint16_t ambient, range;
    vl6180_SetUp(&tof);
    vl6180x_StartContinuous(&tof, VL6180X_MODE_INTERLEAVED, 200);

    while (1) {
        vl6180x_Read(&tof, &ambient, VL6180X_MODE_ALS, 200);
        vl6180x_Read(&tof, &range, VL6180X_MODE_RANGE, 200);
    }
}
```
4. Asynchronous Range (no interrupt pin used)  
```C
void Task_Async(void)
{
    vl6180x_t tof;
    vl6180x_status_t ret;
    uint16_t range;
    vl6180_SetUp(&tof);
    vl6180x_StartContinuous(&tof, VL6180X_MODE_RANGE, 200);

    while (1)
        /* Range won't be updated if conversion still in-progress */
        vl6180x_Read(&tof, &range, VL6180X_MODE_RANGE, 0);
}
```

5. Asynchronous Range (Active‑Low GPIO1) 
```C
void Task_AsyncInterrupt(void)
{
    vl6180x_t tof;
    vl6180x_status_t ret;
    uint16_t range;
    vl6180_SetUp(&tof);
    vl6180x_StartContinuous(&tof, VL6180X_MODE_RANGE, 200);

    while (1)
        if (IsInterruptPinLow())
            vl6180x_Read(&tof, &range, VL6180X_MODE_RANGE, 0);
}
```
## Examples
* [`STM32 + FreeRTOS`](/Examples/stm32/Core/Src/freertos.c)
