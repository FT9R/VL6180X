#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Function return codes */
typedef enum {
    VL6180X_STAT_OK,
    VL6180X_STAT_HANDLE_NF, // Device handle not found or identity check failed
    VL6180X_STAT_PLATFORM_NF, // One or more of basic platform functions not found
    VL6180X_STAT_ARG_INVALID, // Invalid argument provided
    VL6180X_STAT_I2C_FAIL, // I2C bus failure detected
    VL6180X_STAT_ID_NS, // Could not check device ID or its not supported
    VL6180X_STAT_TIMEOUT, // Timeout expired for blocking operation
    VL6180X_STAT_BUSY, // Device is still busy during asynchronous operation
    VL6180X_STAT_SATURATED // `RESULT__RANGE_VAL` or `RESULT__ALS_VAL` saturation detected
} vl6180x_status_t;

/* `vl6180x_ReadRangeStatus` specific error codes */
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
} vl6180x_range_error_t;

/* Measurement mode */
typedef enum { VL6180X_MODE_RANGE, VL6180X_MODE_ALS, VL6180X_MODE_INTERLEAVED } vl6180x_mode_t;

/* Device handle structure */
typedef struct vl6180x_s {
    struct {
        void *handle; // Optional handle for I2C interface
        void (*print)(const char *const fmt, ...); // Optional debug print
        void (*ce)(uint8_t level); // Optional chip enable (gpio0), open drain - no pull - active high
        bool (*read)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
        bool (*write)(void *handle, uint16_t address, uint16_t reg, uint8_t *data, uint16_t size, uint32_t timeout);
        void (*delay)(uint32_t ms);
    } interface;

    /* Per-device driver internals - do not modify */
    struct {
        uint32_t identity; // Device handle identity to check if `vl6180x_t` structure can be safely used
        vl6180x_status_t error; // Last detected driver error
        uint8_t address; // Device I2C slave address
        uint8_t scaling; // Range scaling factor (1x, 2x, or 3x)
        int8_t ptpOffset; // Part to part range offset
        bool rangeContinuous; // Ranging mode: `true` - continuous, `false` - single
        bool ambientContinuous; // ALS mode: `true` - continuous, `false` - single
    } cache;
} vl6180x_t;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Initialize sensor with settings from ST application note AN4545,
 * section `SR03 settings` - `Mandatory: private registers`
 * @param dev device handle
 * @param address 7bit device I2C slave address from datasheet
 * @param reset whether to perform a hardware reset by toggling the chip enable pin
 * @return Operation exit status
 */
vl6180x_status_t vl6180x_Init(vl6180x_t *dev, uint8_t address, bool reset);

/**
 * @brief Set the I2C slave address of the sensor
 * @param dev device handle
 * @param newAddr new I2C address
 * @return Operation exit status
 */
vl6180x_status_t vl6180x_SetAddress(vl6180x_t *dev, uint8_t newAddr);

/**
 * @brief Configure some settings for the sensor's default behavior from AN4545
 *  - `Recommended: Public registers` and `Optional: Public registers`
 * @param dev device handle
 * @return Operation exit status
 * @note This function DOES set up GPIO1 as an interrupt output as suggested
 */
vl6180x_status_t vl6180x_ConfigureDefault(vl6180x_t *dev);

/**
 * @brief Set new range scaling factor and offset
 * @param dev device handle
 * @param newScaling new scaling factor (1, 2, or 3)
 * @param newOffset new part-to-part range offset or `dev.cache.ptpOffset` to keep existing offset, [mm]
 * @return Operation exit status
 * @note The sensor uses 1x scaling by default, giving range measurements in units of mm.
Increasing the scaling to 2x or 3x makes it give raw values in units of 2 mm or 3 mm instead. In other words, a bigger
scaling factor increases the sensor's potential maximum range but reduces its resolution
 */
vl6180x_status_t vl6180x_SetScalingAndOffset(vl6180x_t *dev, uint8_t newScaling, int8_t newOffset);

/**
 * @brief Start continuous `range` or `ALS count` measurements with the given period
 * @param dev device handle
 * @param mode which measurements to start: range, ALS or both
 * @param period time delay between measurements in continuous mode. Step size = 10ms
 * @return Operation exit status
 * @note In `VL6180X_MODE_INTERLEAVED` each ambient light measurement is immediately followed by a range measurement.
 * @note The period must be greater than the time it takes to perform a measurement.
 * See section `Continuous mode limits` in the datasheet for details
 */
vl6180x_status_t vl6180x_StartContinuous(vl6180x_t *dev, vl6180x_mode_t mode, uint16_t period);

/**
 * @brief Stop continuous `range` and `ALS count` measurements
 * @param dev device handle
 * @return Operation exit status
 * @note This will actually start a single measurement of range and/or ambient light if continuous mode is not active,
 * so it's a good idea to wait a few hundred ms after calling this function to let that complete before starting
 * continuous mode again or taking a reading
 */
vl6180x_status_t vl6180x_StopContinuous(vl6180x_t *dev);

/**
 * @brief Get `range` or `ALS count` result
 * @param dev device handle
 * @param result pointer to variable to keep the result
 * @param mode what to read: `VL6180X_MODE_RANGE` or `VL6180X_MODE_ALS`
 * @param timeout how long wait for result, `0` - async, [ms]
 * @return Operation exit status
 * @note `result` will be updated only in case of successful and complete read.
 * @note This function automatically scales range result according to the scaling factor.
 * @note If `vl6180x_StartContinuous()` has not been called prior read, single-shot measurement will be issued
 */
vl6180x_status_t vl6180x_Read(vl6180x_t *dev, uint16_t *result, vl6180x_mode_t mode, uint32_t timeout);

/**
 * @brief Get ranging success/error status code (Use it before using a measurement)
 * @param dev device handle
 * @param rangeStatus pointer to variable to keep the ranging status code
 * @return Operation exit status
 */
vl6180x_status_t vl6180x_RangeStatus(vl6180x_t *dev, vl6180x_range_error_t *rangeStatus);
#ifdef __cplusplus
}
#endif