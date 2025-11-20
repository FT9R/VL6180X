# VL6180X Driver

A lightweight, platform-independent, dependency-free C driver for the **ST VL6180X Time-of-Flight distance and ambient light sensor**.  
Provides initialization, configuration, and measurement functions for range (mm) and ALS (ambient light) using an abstracted I²C interface.

## Overview
This driver based on [`Pololu's VL6180X library for Arduino`](https://github.com/pololu/vl6180x-arduino) and provides:
- A complete C driver for the VL6180X  
- Any microcontroller support by using abstracted I²C interface
- Support for single-shot and continuous measurement modes  
- Range scaling (1×, 2×, 3×)  
- Error/status reporting  
- Default configuration values based on AN4545    


## Features

- Platform-independent
- A few devices support with separated handles
- Hardware reset support  
- Device ID validation  
- Single-shot & continuous range/ALS measurement  
- Interleaved range + ALS mode  
- Timeouts for blocking operations  
- Range scaling (1x–3x)  
- No external library dependencies, except for standard C

## Usage
Assign your platform-specific functions:

```C
uint8_t (*read)(void *handle, uint16_t addr, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
uint8_t (*write)(void *handle, uint16_t addr, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
void (*ce)(uint8_t level); // 0 - pin reset
void (*delay)(uint32_t ms);
void *handle; // Optional, used by platform read/write functions
```

### Basic Example
```C
#include "vl6180x.h"

void tof_Test()
{
    vl6180x_t tof = {0};

    i2c_Init(100000);
    gpio_Init(); // For chip enable (gpio0)

    tof.interface.read  = my_i2c_read;
    tof.interface.write = my_i2c_write;
    tof.interface.ce    = my_gpio_ce;
    tof.interface.delay = my_delay;
    tof.sampleReadyTimeout = 1000;
    tof.address = 0b1010101; // If you are sure that device has non-default address 

    vl6180x_Init(&tof, true);
    vl6180x_ConfigureDefault(&tof);

    while (1)
    {
        uint16_t range = vl6180x_ReadRangeSingle(&tof);
        uint16_t als   = vl6180x_ReadAmbientSingle(&tof);
    }
}
```
## Examples
* [`STM32 + FreeRTOS`](/Examples/stm32/Core/Src/freertos.c)
