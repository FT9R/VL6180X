#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    VL6180X_STATE_IDLE,
    // FIXME: provide more states
} vl6180x_state_t;

typedef enum {
    VL6180X_ERR_NONE,
    VL6180X_ERR_ARG,
    VL6180X_ERR_ID,
    VL6180X_ERR_STATE,
    VL6180X_ERR_I2C_READ,
    VL6180X_ERR_I2C_WRITE
} vl6180x_error_t;

typedef enum {
    VL6180X_RANGE_ERROR_NONE, // No error; Valid measurement
    VL6180X_RANGE_ERROR_SYSERR_1, // System error; VCSEL Continuity Test; No measurement possible
    VL6180X_RANGE_ERROR_SYSERR_2, // System error; VCSEL Watchdog Test; No measurement possible
    VL6180X_RANGE_ERROR_SYSERR_3, // System error; VCSEL Watchdog; No measurement possible
    VL6180X_RANGE_ERROR_SYSERR_4, // System error; PLL1 Lock; No measurement possible
    VL6180X_RANGE_ERROR_SYSERR_5, // System error; PLL2 Lock; No measurement possible
    VL6180X_RANGE_ERROR_ECEFAIL, // Early Convergence Estimate; Check fail
    VL6180X_RANGE_ERROR_NOCONVERGE, // Max convergence; System didn't converge before the specified time limit
    VL6180X_RANGE_ERROR_RANGEIGNORE, // Range ignore; No Target Ignore; Ignore threshold check failed
    VL6180X_RANGE_ERROR_SNR = 11, // Max Signal To Noise Ratio; Ambient conditions too high
    VL6180X_RANGE_ERROR_RAWUFLOW, // Raw Range underflow; Target too close
    VL6180X_RANGE_ERROR_RAWOFLOW, // Raw Range overflow; Target too far
    VL6180X_RANGE_ERROR_RANGEUFLOW, // Range underflow; Target too close
    VL6180X_RANGE_ERROR_RANGEOFLOW, // Range overflow; Target too far
    VL6180X_RANGE_ERROR_DRIVER, // Custom driver error during request; See `dev.error`
} vl6180x_range_error_t;

typedef struct vl6180x_s {
    struct {
        uint8_t (*read)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
        uint8_t (*write)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
        void (*ce)(uint8_t level); // Chip enable (gpio0), open drain - no pull - active high
        void (*delay)(uint32_t ms);
        void *handle; // Optional user-defined handle for I2C interface
    } interface;

    uint8_t address; // Device address. Provide before initialization if different from default
    uint8_t scaling; // Range scaling factor (1x, 2x, or 3x)
    int8_t ptp_offset; // Part to part range offset
    uint32_t sampleReadyTimeout; // Timeout for continuous range/ambient readings
    vl6180x_state_t state; // Driver operation state
    vl6180x_error_t error; // Driver error code
} vl6180x_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize sensor with settings from ST application note AN4545,
 * section "SR03 settings" - "Mandatory : private registers"
 * @param dev device handle
 * @param reset whether to perform a hardware reset by toggling the chip enable pin
 */
void vl6180x_Init(vl6180x_t *dev, bool reset);

/**
 * @brief Set the I2C slave address of the sensor
 * @param dev device handle
 * @param new_addr new I2C address
 */
void vl6180x_SetAddress(vl6180x_t *dev, uint8_t new_addr);

/**
 * @brief Configure some settings for the sensor's default behavior from AN4545
 *  - "Recommended : Public registers" and "Optional: Public registers"
 * @param dev device handle
 * @note This function does not set up GPIO1 as an interrupt output as suggested
 */
void vl6180x_ConfigureDefault(vl6180x_t *dev);

/**
 * @brief Set range scaling factor. The sensor uses 1x scaling by default, giving range measurements in units of mm.
Increasing the scaling to 2x or 3x makes it give raw values in units of 2 mm or 3 mm instead. In other words, a bigger
scaling factor increases the sensor's potential maximum range but reduces its resolution
 * @param dev device handle
 * @param new_scaling new scaling factor (1, 2, or 3)
 */
void vl6180x_SetScaling(vl6180x_t *dev, uint8_t new_scaling);

/**
 * @brief Start continuous ranging measurements with the given period
 * @param dev device handle
 * @param period Time delay between measurements in Ranging continuous mode. Step size = 10ms
 * @note The period must be greater than the time it takes to perform a measurement.
 * See section "Continuous mode limits" in the datasheet for details
 */
void vl6180x_StartRangeContinuous(vl6180x_t *dev, uint16_t period);

/**
 * @brief Start continuous ambient light measurements with the given period
 * @param dev device handle
 * @param period Time delay between measurements in ALS continuous mode. Step size = 10ms
 * @note The period must be greater than the time it takes to perform a measurement.
 * See section "Continuous mode limits" in the datasheet for details
 */
void vl6180x_StartAmbientContinuous(vl6180x_t *dev, uint16_t period);

/**
 * @brief Start continuous interleaved measurements with the given period
 * @param dev device handle
 * @param period Time delay between measurements in interleaved continuous mode. Step size = 10ms
 * @note In this mode, each ambient light measurement is immediately followed by a range measurement.
 * @note The datasheet recommends using this mode instead of running "range and ALS continuous modes simultaneously.
 * @note The period must be greater than the time it takes to perform both measurements.
 * See section "Continuous mode limits" in the datasheet for details.
 */
void vl6180x_StartInterleavedContinuous(vl6180x_t *dev, uint16_t period);

/**
 * @brief Stop continuous measurements
 * @param dev device handle.
 * @note This will actually start a single measurement of range and/or ambient light if continuous mode is not active,
 * so it's a good idea to wait a few hundred ms after calling this function to let that complete before starting
 * continuous mode again or taking a reading
 */
void vl6180x_StopContinuous(vl6180x_t *dev);

/**
 * @brief Performs a single-shot ranging measurement
 * @param dev device handle
 * @return measured range in millimeters or `UINT16_MAX` in case of error/object absence
 * @note This function automatically scales the result according to the current scaling factor
 */
uint16_t vl6180x_ReadRangeSingle(vl6180x_t *dev);

/**
 * @brief Returns a range reading when continuous mode is activated
 * @param dev device handle
 * @return measured range in millimeters or `UINT16_MAX` in case of error/object absence
 * @note This function automatically scales the result according to the current scaling factor
 */
uint16_t vl6180x_ReadRangeContinuous(vl6180x_t *dev);

/**
 * @brief Immediately reads range result if available, otherwise returns `UINT16_MAX`
 * @param dev device handle
 * @return measured range in millimeters or `UINT16_MAX` if result is not ready or in case of error/object absence
 * @note This function automatically scales the result according to the current scaling factor.
 * @note Interrupt pin (gpio1) can be checked before calling this function to avoid unnecessary I2C traffic
 */
uint16_t vl6180x_ReadRangeAsync(vl6180x_t *dev);

/**
 * @brief Performs a single-shot ambient light measurement
 * @param dev device handle
 * @return measured ambient light level or `0` in case of error
 */
uint16_t vl6180x_ReadAmbientSingle(vl6180x_t *dev);

/**
 * @brief Returns an ambient light reading when continuous mode is activated
 * @param dev device handle
 * @return measured ambient light level or `0` in case of error
 */
uint16_t vl6180x_ReadAmbientContinuous(vl6180x_t *dev);

/**
 * @brief Immediately reads ambient light result if available, otherwise returns `0`
 * @param dev device handle
 * @return measured ambient light level or `0` if result is not ready or in case of error
 * @note Interrupt pin (gpio1) can be checked before calling this function to avoid unnecessary I2C traffic
 */
uint16_t vl6180x_ReadAmbientAsync(vl6180x_t *dev);

/**
 * @brief Get ranging success/error status code (Use it before using a measurement)
 * @param dev device handle
 * @return error code - one of possible `VL6180X_RANGE_ERROR_*` values
 * or `VL6180X_RANGE_ERROR_DRIVER` in case of driver error
 */
vl6180x_range_error_t vl6180x_ReadRangeStatus(vl6180x_t *dev);

#ifdef __cplusplus
}
#endif