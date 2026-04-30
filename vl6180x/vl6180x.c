#include "vl6180x.h"
#include <string.h>

/* Constants */
#define HANDLE_IDENTITY   (uint32_t) 0x22A51B3B
#define I2C_READ_TIMEOUT  (uint32_t) 1000
#define I2C_WRITE_TIMEOUT (uint32_t) 1000
#define DEVICE_ID         (uint8_t) 0xB4
static const uint16_t SCALER_LUT[] = {0, 253, 127, 84};
static const uint8_t POLL_PERIOD = 1;

/* Macro */
#define HANDLE_CHECK                                               \
    do {                                                           \
        if (dev == NULL || dev->cache.identity != HANDLE_IDENTITY) \
            ERROR_SET(VL6180X_STAT_HANDLE_NF);                     \
    }                                                              \
    while (0)

#define READ_REG(REG, DATA, SIZE)                  \
    do {                                           \
        if (!read_reg(dev, (REG), (DATA), (SIZE))) \
            ERROR_SET(VL6180X_STAT_I2C_FAIL);      \
    }                                              \
    while (0)

#define WRITE_REG(REG, DATA, SIZE)                  \
    do {                                            \
        if (!write_reg(dev, (REG), (DATA), (SIZE))) \
            ERROR_SET(VL6180X_STAT_I2C_FAIL);       \
    }                                               \
    while (0)

#define PRINT(...)                             \
    do {                                       \
        if (dev->interface.print)              \
            dev->interface.print(__VA_ARGS__); \
    }                                          \
    while (0)

#define ERROR_SET(ERR)                                                    \
    do {                                                                  \
        dev->cache.error = (ERR);                                         \
        PRINT("Error %u occurred in %s()\n", dev->cache.error, __func__); \
        return dev->cache.error;                                          \
    }                                                                     \
    while (0)

/* Custom types */
typedef enum {
    IDENTIFICATION__MODEL_ID = 0x000,
    IDENTIFICATION__MODEL_REV_MAJOR = 0x001,
    IDENTIFICATION__MODEL_REV_MINOR = 0x002,
    IDENTIFICATION__MODULE_REV_MAJOR = 0x003,
    IDENTIFICATION__MODULE_REV_MINOR = 0x004,
    IDENTIFICATION__DATE_HI = 0x006,
    IDENTIFICATION__DATE_LO = 0x007,
    IDENTIFICATION__TIME = 0x008, // 16-bit

    SYSTEM__MODE_GPIO0 = 0x010,
    SYSTEM__MODE_GPIO1 = 0x011,
    SYSTEM__HISTORY_CTRL = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014,
    SYSTEM__INTERRUPT_CLEAR = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD = 0x017,

    SYSRANGE__START = 0x018,
    SYSRANGE__THRESH_HIGH = 0x019,
    SYSRANGE__THRESH_LOW = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022, // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD = 0x026, // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES = 0x02D,
    SYSRANGE__VHV_RECALIBRATE = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE = 0x031,

    SYSALS__START = 0x038,
    SYSALS__THRESH_HIGH = 0x03A,
    SYSALS__THRESH_LOW = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD = 0x03E,
    SYSALS__ANALOGUE_GAIN = 0x03F,
    SYSALS__INTEGRATION_PERIOD = 0x040,

    RESULT__RANGE_STATUS = 0x04D,
    RESULT__ALS_STATUS = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO = 0x04F,
    RESULT__ALS_VAL = 0x050, // 16-bit
    RESULT__HISTORY_BUFFER_0 = 0x052, // 16-bit
    RESULT__HISTORY_BUFFER_1 = 0x054, // 16-bit
    RESULT__HISTORY_BUFFER_2 = 0x056, // 16-bit
    RESULT__HISTORY_BUFFER_3 = 0x058, // 16-bit
    RESULT__HISTORY_BUFFER_4 = 0x05A, // 16-bit
    RESULT__HISTORY_BUFFER_5 = 0x05C, // 16-bit
    RESULT__HISTORY_BUFFER_6 = 0x05E, // 16-bit
    RESULT__HISTORY_BUFFER_7 = 0x060, // 16-bit
    RESULT__RANGE_VAL = 0x062,
    RESULT__RANGE_RAW = 0x064,
    RESULT__RANGE_RETURN_RATE = 0x066, // 16-bit
    RESULT__RANGE_REFERENCE_RATE = 0x068, // 16-bit
    RESULT__RANGE_RETURN_SIGNAL_COUNT = 0x06C, // 32-bit
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070, // 32-bit
    RESULT__RANGE_RETURN_AMB_COUNT = 0x074, // 32-bit
    RESULT__RANGE_REFERENCE_AMB_COUNT = 0x078, // 32-bit
    RESULT__RANGE_RETURN_CONV_TIME = 0x07C, // 32-bit
    RESULT__RANGE_REFERENCE_CONV_TIME = 0x080, // 32-bit

    RANGE_SCALER = 0x096, // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h
    READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A,
    FIRMWARE__BOOTUP = 0x119,
    FIRMWARE__RESULT_SCALER = 0x120,
    I2C_SLAVE__DEVICE_ADDRESS = 0x212,
    INTERLEAVED_MODE__ENABLE = 0x2A3,
} regAddr_t;

static inline int32_t constrain(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline bool read_reg(vl6180x_t *dev, uint16_t reg, void *data, uint8_t size)
{
    uint8_t buf[4] = {0};
    uint32_t result = 0;
    if (size == 0 || size > 4)
        return false;

    if (!dev->interface.read(dev->interface.handle, dev->cache.address, reg, buf, size, I2C_READ_TIMEOUT))
        return false;
    for (uint8_t i = 0; i < size; i++)
        result = (result << 8) | buf[i];
    switch (size) {
    case 1:
        *(uint8_t *) data = (uint8_t) result;
        break;
    case 2:
        *(uint16_t *) data = (uint16_t) result;
        break;
    case 4:
        *(uint32_t *) data = (uint32_t) result;
        break;
    }
    return true;
}

static inline bool write_reg(vl6180x_t *dev, uint16_t reg, uint32_t data, uint8_t size)
{
    uint8_t buf[4];
    if (size == 0 || size > 4)
        return false;

    buf[0] = (uint8_t) (data >> 24);
    buf[1] = (uint8_t) (data >> 16);
    buf[2] = (uint8_t) (data >> 8);
    buf[3] = (uint8_t) (data >> 0);
    return dev->interface.write(dev->interface.handle, dev->cache.address, reg, &buf[4u - size], size,
                                I2C_WRITE_TIMEOUT);
}

vl6180x_status_t vl6180x_Init(vl6180x_t *dev, uint8_t address, bool reset)
{
    uint16_t s;
    uint8_t id;
    uint8_t freshOut;
    vl6180x_status_t status;
    uint8_t responseAttempt = 0;
    if (dev == NULL)
        return VL6180X_STAT_HANDLE_NF;

    /* Check basic platform */
    if (dev->interface.read == NULL || dev->interface.write == NULL || dev->interface.delay == NULL)
        ERROR_SET(VL6180X_STAT_PLATFORM_NF);

    /* Reset internal registers to defaults if requested */
    if (reset && dev->interface.ce != NULL) {
        dev->interface.ce(0), dev->interface.delay(100);
        dev->interface.ce(1), dev->interface.delay(100);
    }

    /* Check device ID */
    dev->cache.address = address;
    while (true) {
        dev->interface.delay(100);
        READ_REG(IDENTIFICATION__MODEL_ID, &id, 1);
        if (id == DEVICE_ID)
            break;
        if (++responseAttempt >= 5)
            ERROR_SET(VL6180X_STAT_ID_NS);
    }

    // Store part-to-part range offset so it can be adjusted if scaling is changed
    READ_REG(SYSRANGE__PART_TO_PART_RANGE_OFFSET, &dev->cache.ptpOffset, 1);
    READ_REG(SYSTEM__FRESH_OUT_OF_RESET, &freshOut, 1);
    if (freshOut == 1) {
        dev->cache.scaling = 1;
        WRITE_REG(0x207, 0x01, 1);
        WRITE_REG(0x208, 0x01, 1);
        WRITE_REG(0x096, 0x00, 1);
        WRITE_REG(0x097, 0xFD, 1); // RANGE_SCALER = 253
        WRITE_REG(0x0E3, 0x01, 1);
        WRITE_REG(0x0E4, 0x03, 1);
        WRITE_REG(0x0E5, 0x02, 1);
        WRITE_REG(0x0E6, 0x01, 1);
        WRITE_REG(0x0E7, 0x03, 1);
        WRITE_REG(0x0F5, 0x02, 1);
        WRITE_REG(0x0D9, 0x05, 1);
        WRITE_REG(0x0DB, 0xCE, 1);
        WRITE_REG(0x0DC, 0x03, 1);
        WRITE_REG(0x0DD, 0xF8, 1);
        WRITE_REG(0x09F, 0x00, 1);
        WRITE_REG(0x0A3, 0x3C, 1);
        WRITE_REG(0x0B7, 0x00, 1);
        WRITE_REG(0x0BB, 0x3C, 1);
        WRITE_REG(0x0B2, 0x09, 1);
        WRITE_REG(0x0CA, 0x09, 1);
        WRITE_REG(0x198, 0x01, 1);
        WRITE_REG(0x1B0, 0x17, 1);
        WRITE_REG(0x1AD, 0x00, 1);
        WRITE_REG(0x0FF, 0x05, 1);
        WRITE_REG(0x100, 0x05, 1);
        WRITE_REG(0x199, 0x05, 1);
        WRITE_REG(0x1A6, 0x1B, 1);
        WRITE_REG(0x1AC, 0x3E, 1);
        WRITE_REG(0x1A7, 0x1F, 1);
        WRITE_REG(0x030, 0x00, 1);
        WRITE_REG(SYSTEM__FRESH_OUT_OF_RESET, 0x00, 1);
    }
    else {
        // Sensor has already been initialized, so try to get scaling settings by reading registers
        READ_REG(RANGE_SCALER, &s, 2);
        dev->cache.scaling = (s == SCALER_LUT[3]) ? 3 : (s == SCALER_LUT[2]) ? 2 : 1;

        // Adjust the part-to-part range offset value read earlier to account for
        // existing scaling. If the sensor was already in 2x or 3x scaling mode,
        // precision will be lost calculating the original (1x) offset, but this can
        // be resolved by resetting the sensor and Arduino again.
        dev->cache.ptpOffset *= dev->cache.scaling;
    }
    dev->cache.identity = HANDLE_IDENTITY;

    /* Stop any possible ongoing measurements */
    status = vl6180x_StopContinuous(dev);
    dev->interface.delay(500);

    PRINT("VL6180 initial scaling: %u\n", dev->cache.scaling);
    PRINT("VL6180 initial offset: %i\n", dev->cache.ptpOffset);
    PRINT("VL6180 fresh out of reset: %u\n", freshOut);
    return status;
}

vl6180x_status_t vl6180x_SetAddress(vl6180x_t *dev, uint8_t newAddr)
{
    HANDLE_CHECK;

    WRITE_REG(I2C_SLAVE__DEVICE_ADDRESS, newAddr & 0x7F, 1);
    dev->cache.address = newAddr;
    return VL6180X_STAT_OK;
}

vl6180x_status_t vl6180x_ConfigureDefault(vl6180x_t *dev)
{
    HANDLE_CHECK;

    /* Recommended: Public registers */
    // readout__averaging_sample_period = 48
    WRITE_REG(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30, 1);

    // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to table "Actual gain values" in
    // datasheet)
    WRITE_REG(SYSALS__ANALOGUE_GAIN, 0x46, 1);

    // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range
    // measurements)
    WRITE_REG(SYSRANGE__VHV_REPEAT_RATE, 0xFF, 1);

    // sysals__integration_period = 99 (100 ms)
    WRITE_REG(SYSALS__INTEGRATION_PERIOD, 0x0063, 2);

    // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
    WRITE_REG(SYSRANGE__VHV_RECALIBRATE, 0x01, 1);

    /* Optional: Public registers */
    // sysrange__intermeasurement_period = 9 (100 ms)
    WRITE_REG(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09, 1);

    // sysals__intermeasurement_period = 49 (500 ms)
    WRITE_REG(SYSALS__INTERMEASUREMENT_PERIOD, 0x31, 1);

    // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
    WRITE_REG(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24, 1);

    /* Reset other settings to power-on defaults */
    // sysrange__max_convergence_time = 49 (49 ms)
    WRITE_REG(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31, 1);

    // disable interleaved mode
    WRITE_REG(INTERLEAVED_MODE__ENABLE, 0x00, 1);

    // configure interrupt pin gpio1: out, active-low
    WRITE_REG(SYSTEM__MODE_GPIO1, 0x10, 1);

    // reset range scaling factor to 1x
    return vl6180x_SetScalingAndOffset(dev, 1, dev->cache.ptpOffset);
}

vl6180x_status_t vl6180x_SetScalingAndOffset(vl6180x_t *dev, uint8_t newScaling, int8_t newOffset)
{
    uint8_t const DEFAULT_CROSSTALK_VALID_HEIGHT = 20; // Default value
    uint8_t rce;
    HANDLE_CHECK;
    if (newScaling < 1 || newScaling > 3)
        ERROR_SET(VL6180X_STAT_ARG_INVALID);

    WRITE_REG(RANGE_SCALER, SCALER_LUT[newScaling], 2);
    dev->cache.scaling = newScaling;

    // apply scaling on part-to-part offset
    WRITE_REG(SYSRANGE__PART_TO_PART_RANGE_OFFSET, newOffset / dev->cache.scaling, 1);
    dev->cache.ptpOffset = newOffset;

    // apply scaling on CrossTalkValidHeight
    WRITE_REG(SYSRANGE__CROSSTALK_VALID_HEIGHT, DEFAULT_CROSSTALK_VALID_HEIGHT / dev->cache.scaling, 1);

    // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

    // enable early convergence estimate only at 1x scaling
    READ_REG(SYSRANGE__RANGE_CHECK_ENABLES, &rce, 1);
    WRITE_REG(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (dev->cache.scaling == 1), 1);
    return VL6180X_STAT_OK;
}

vl6180x_status_t vl6180x_StartContinuous(vl6180x_t *dev, vl6180x_mode_t mode, uint16_t period)
{
    int32_t periodReg = (int32_t) (period / 10) - 1;
    periodReg = constrain(periodReg, 0, 254);
    HANDLE_CHECK;

    switch (mode) {
    case VL6180X_MODE_RANGE:
        WRITE_REG(SYSRANGE__INTERMEASUREMENT_PERIOD, periodReg, 1);
        WRITE_REG(SYSRANGE__START, 0x03, 1);
        dev->cache.rangeContinuous = true;
        break;
    case VL6180X_MODE_INTERLEAVED:
        WRITE_REG(INTERLEAVED_MODE__ENABLE, 0x01, 1);
    case VL6180X_MODE_ALS:
        WRITE_REG(SYSALS__INTERMEASUREMENT_PERIOD, periodReg, 1);
        WRITE_REG(SYSALS__START, 0x03, 1);
        dev->cache.ambientContinuous = true;
        if (mode == VL6180X_MODE_INTERLEAVED)
            dev->cache.rangeContinuous = true;
        break;
    default:
        ERROR_SET(VL6180X_STAT_ARG_INVALID);
    }
    return VL6180X_STAT_OK;
}

vl6180x_status_t vl6180x_StopContinuous(vl6180x_t *dev)
{
    HANDLE_CHECK;

    WRITE_REG(SYSRANGE__START, 0x01, 1);
    WRITE_REG(SYSALS__START, 0x01, 1);
    WRITE_REG(INTERLEAVED_MODE__ENABLE, 0x00, 1);
    dev->cache.rangeContinuous = false;
    dev->cache.ambientContinuous = false;
    return VL6180X_STAT_OK;
}

vl6180x_status_t vl6180x_Read(vl6180x_t *dev, uint16_t *result, vl6180x_mode_t mode, uint32_t timeout)
{
    uint8_t interruptStatus;
    uint8_t rangeRaw;
    uint16_t ambientRaw;
    HANDLE_CHECK;
    if (result == NULL)
        ERROR_SET(VL6180X_STAT_ARG_INVALID);

    switch (mode) {
    case VL6180X_MODE_RANGE:
        if (!dev->cache.rangeContinuous)
            WRITE_REG(SYSRANGE__START, 0x01, 1);

        /* Poll for New Sample Ready threshold event */
        while (true) {
            READ_REG(RESULT__INTERRUPT_STATUS_GPIO, &interruptStatus, 1);
            if ((interruptStatus & 0x07) == 0x04)
                break;
            if (timeout == 0)
                return VL6180X_STAT_BUSY;
            if (timeout < POLL_PERIOD)
                ERROR_SET(VL6180X_STAT_TIMEOUT);
            dev->interface.delay(POLL_PERIOD);
            timeout -= POLL_PERIOD;
        }

        /* Get result */
        READ_REG(RESULT__RANGE_VAL, &rangeRaw, 1);
        WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x01, 1);
        if (rangeRaw == UINT8_MAX)
            return VL6180X_STAT_SATURATED;
        *result = rangeRaw * dev->cache.scaling;
        break;
    case VL6180X_MODE_ALS:
        if (!dev->cache.ambientContinuous)
            WRITE_REG(SYSALS__START, 0x01, 1);

        /* Poll for New Sample Ready threshold event */
        while (true) {
            READ_REG(RESULT__INTERRUPT_STATUS_GPIO, &interruptStatus, 1);
            if ((interruptStatus & 0x38) == 0x20)
                break;
            if (timeout == 0)
                return VL6180X_STAT_BUSY;
            if (timeout < POLL_PERIOD)
                ERROR_SET(VL6180X_STAT_TIMEOUT);
            dev->interface.delay(POLL_PERIOD);
            timeout -= POLL_PERIOD;
        }

        /* Get result */
        READ_REG(RESULT__ALS_VAL, &ambientRaw, 2);
        WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x02, 1);
        if (ambientRaw == UINT16_MAX)
            return VL6180X_STAT_SATURATED;
        *result = ambientRaw;
        break;
    default:
        ERROR_SET(VL6180X_STAT_ARG_INVALID);
    }
    return VL6180X_STAT_OK;
}

vl6180x_status_t vl6180x_RangeStatus(vl6180x_t *dev, vl6180x_range_error_t *rangeStatus)
{
    uint8_t raw;
    HANDLE_CHECK;
    if (rangeStatus == NULL)
        ERROR_SET(VL6180X_STAT_ARG_INVALID);

    READ_REG(RESULT__RANGE_STATUS, &raw, 1);
    *rangeStatus = (vl6180x_range_error_t) (raw >> 4);
    return VL6180X_STAT_OK;
}