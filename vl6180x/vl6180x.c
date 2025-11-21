#include "vl6180x.h"

#define DEVICE_ADDRESS    0b0101001
#define DEVICE_ID         0xB4
#define I2C_READ_TIMEOUT  1000
#define I2C_WRITE_TIMEOUT 1000

/* Macro */
#define READ_REG(REG, DATA, SIZE)                                                                         \
    do                                                                                                    \
    {                                                                                                     \
        if (!dev->interface.read(dev->interface.handle, dev->address, (REG), (uint8_t *) &(DATA), (SIZE), \
                                 I2C_READ_TIMEOUT))                                                       \
        {                                                                                                 \
            dev->error = VL6180X_ERR_I2C_READ;                                                            \
            goto error;                                                                                   \
        }                                                                                                 \
    }                                                                                                     \
    while (0)

#define WRITE_REG(REG, DATA, SIZE)                                                                          \
    do                                                                                                      \
    {                                                                                                       \
        if (!dev->interface.write(dev->interface.handle, dev->address, (REG), (uint8_t[]) {(DATA)}, (SIZE), \
                                  I2C_WRITE_TIMEOUT))                                                       \
        {                                                                                                   \
            dev->error = VL6180X_ERR_I2C_WRITE;                                                             \
            goto error;                                                                                     \
        }                                                                                                   \
    }                                                                                                       \
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

static uint16_t const ScalerValues[] = {0, 253, 127, 84};
static bool internalFunctionUsage = false;

static inline int32_t constrain(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void vl6180x_Init(vl6180x_t *dev, bool reset)
{
    uint8_t id = 0;
    uint8_t responseAttmept = 0;
    uint8_t freshOut = 1;
    uint16_t s = 0;

    if (dev == NULL)
        goto error;

    dev->address = (dev->address == 0) ? DEVICE_ADDRESS : dev->address;
    dev->error = VL6180X_ERR_NONE;
    dev->state = VL6180X_STATE_IDLE; // FIXME: state check/set at every operation (not internal)

    if (reset)
    {
        dev->interface.ce(0);
        dev->interface.delay(100);
        dev->interface.ce(1);
    }

    /* Check for device id */
    while (true)
    {
        dev->interface.delay(100);
        READ_REG(IDENTIFICATION__MODEL_ID, id, 1);
        if (id == DEVICE_ID)
            break;

        if (++responseAttmept >= 5)
        {
            dev->error = VL6180X_ERR_ID;
            goto error;
        }
    }

    // Store part-to-part range offset so it can be adjusted if scaling is changed
    READ_REG(SYSRANGE__PART_TO_PART_RANGE_OFFSET, dev->ptp_offset, 1);

    READ_REG(SYSTEM__FRESH_OUT_OF_RESET, freshOut, 1);
    if (freshOut == 1)
    {
        dev->scaling = 1;

        WRITE_REG(0x207, 0x01, 1);
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

        WRITE_REG(SYSTEM__FRESH_OUT_OF_RESET, 0, 1);
    }
    else
    {
        // Sensor has already been initialized, so try to get scaling settings by
        // reading registers.
        READ_REG(RANGE_SCALER, s, 2);

        if (s == ScalerValues[3])
            dev->scaling = 3;
        else if (s == ScalerValues[2])
            dev->scaling = 2;
        else
            dev->scaling = 1;

        // Adjust the part-to-part range offset value read earlier to account for
        // existing scaling. If the sensor was already in 2x or 3x scaling mode,
        // precision will be lost calculating the original (1x) offset, but this can
        // be resolved by resetting the sensor and Arduino again.
        dev->ptp_offset *= dev->scaling;
    }

error:
    return;
}

void vl6180x_SetAddress(vl6180x_t *dev, uint8_t newAddr)
{
    if (dev == NULL)
        goto error;

    WRITE_REG(I2C_SLAVE__DEVICE_ADDRESS, newAddr & 0x7F, 1);
    dev->address = newAddr;

error:
    return;
}

void vl6180x_ConfigureDefault(vl6180x_t *dev)
{
    if (dev == NULL)
        goto error;

    // "Recommended : Public registers"

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

    // "Optional: Public registers"

    // sysrange__intermeasurement_period = 9 (100 ms)
    WRITE_REG(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09, 1);

    // sysals__intermeasurement_period = 49 (500 ms)
    WRITE_REG(SYSALS__INTERMEASUREMENT_PERIOD, 0x31, 1);

    // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
    WRITE_REG(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24, 1);

    // Reset other settings to power-on defaults

    // sysrange__max_convergence_time = 49 (49 ms)
    WRITE_REG(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31, 1);

    // disable interleaved mode
    WRITE_REG(INTERLEAVED_MODE__ENABLE, 0, 1);

    // configure interrupt pin gpio1: out, active-low
    WRITE_REG(SYSTEM__MODE_GPIO1, 0x10, 1);

    // reset range scaling factor to 1x
    internalFunctionUsage = true;
    vl6180x_SetScalingAndOffset(dev, 1, dev->ptp_offset);
    internalFunctionUsage = false;

error:
    return;
}

void vl6180x_SetScalingAndOffset(vl6180x_t *dev, uint8_t newScaling, int8_t newOffset)
{
    uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT
    uint8_t rce = 0;

    if (dev == NULL)
        goto error;

    if ((newScaling < 1) || (newScaling > 3))
    {
        dev->error = VL6180X_ERR_ARG;
        goto error;
    }

    dev->scaling = newScaling;
    WRITE_REG(RANGE_SCALER, ScalerValues[dev->scaling], 2);

    // apply scaling on part-to-part offset
    dev->ptp_offset = newOffset;
    WRITE_REG(SYSRANGE__PART_TO_PART_RANGE_OFFSET, dev->ptp_offset / dev->scaling, 1);

    // apply scaling on CrossTalkValidHeight
    WRITE_REG(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / dev->scaling, 1);

    // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

    // enable early convergence estimate only at 1x scaling
    READ_REG(SYSRANGE__RANGE_CHECK_ENABLES, rce, 1);
    WRITE_REG(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (dev->scaling == 1), 1);

error:
    return;
}

void vl6180x_StartRangeContinuous(vl6180x_t *dev, uint16_t period)
{
    int32_t period_reg;

    if (dev == NULL)
        goto error;

    period_reg = (int32_t) (period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    WRITE_REG(SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg, 1);
    WRITE_REG(SYSRANGE__START, 0x03, 1);

error:
    return;
}

void vl6180x_StartAmbientContinuous(vl6180x_t *dev, uint16_t period)
{
    int32_t period_reg;

    if (dev == NULL)
        goto error;

    period_reg = (int32_t) (period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    WRITE_REG(SYSALS__INTERMEASUREMENT_PERIOD, period_reg, 1);
    WRITE_REG(SYSALS__START, 0x03, 1);

error:
    return;
}

void vl6180x_StartInterleavedContinuous(vl6180x_t *dev, uint16_t period)
{
    int32_t period_reg;

    if (dev == NULL)
        goto error;

    period_reg = (int32_t) (period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);

    WRITE_REG(INTERLEAVED_MODE__ENABLE, 1, 1);
    WRITE_REG(SYSALS__INTERMEASUREMENT_PERIOD, period_reg, 1);
    WRITE_REG(SYSALS__START, 0x03, 1);

error:
    return;
}

void vl6180x_StopContinuous(vl6180x_t *dev)
{
    if (dev == NULL)
        goto error;

    WRITE_REG(SYSRANGE__START, 0x01, 1);
    WRITE_REG(SYSALS__START, 0x01, 1);
    WRITE_REG(INTERLEAVED_MODE__ENABLE, 0, 1);

error:
    return;
}

uint16_t vl6180x_ReadRangeSingle(vl6180x_t *dev)
{
    uint8_t range = UINT8_MAX;

    if (dev == NULL)
        goto error;

    WRITE_REG(SYSRANGE__START, 0x01, 1);
    internalFunctionUsage = true;
    range = vl6180x_ReadRangeContinuous(dev);
    internalFunctionUsage = false;
    range /= dev->scaling; // ReadRangeContinuous() call scales the result by default

error:
    return (range == UINT8_MAX) ? UINT16_MAX : (uint16_t) dev->scaling * range;
}

uint16_t vl6180x_ReadRangeContinuous(vl6180x_t *dev)
{
    uint32_t timeout;
    uint8_t range = UINT8_MAX;
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    // While computation is not finished
    // only watching if bits 2:0 (mask 0x07) are set to 0b100 (0x04)
    timeout = dev->sampleReadyTimeout > 0 ? dev->sampleReadyTimeout : 1;
    while (true)
    {
        READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);
        if ((interruptStatus & 0x07) == 0x04)
            break;

        dev->interface.delay(1);
        if (--timeout == 0)
            goto error;
    }

    READ_REG(RESULT__RANGE_VAL, range, 1);
    WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x01, 1);

error:
    return (range == UINT8_MAX) ? UINT16_MAX : (uint16_t) dev->scaling * range;
}

uint16_t vl6180x_ReadRangeAsync(vl6180x_t *dev)
{
    uint8_t range = UINT8_MAX;
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);
    if ((interruptStatus & 0x07) != 0x04)
        goto error; // Sample not ready yet

    READ_REG(RESULT__RANGE_VAL, range, 1);
    WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x01, 1);

error:
    return (range == UINT8_MAX) ? UINT16_MAX : (uint16_t) dev->scaling * range;
}

uint16_t vl6180x_ReadAmbientSingle(vl6180x_t *dev)
{
    uint16_t ambient = 0;

    if (dev == NULL)
        goto error;

    WRITE_REG(SYSALS__START, 0x01, 1);
    internalFunctionUsage = true;
    ambient = vl6180x_ReadAmbientContinuous(dev);
    internalFunctionUsage = false;

error:
    return ambient;
}

uint16_t vl6180x_ReadAmbientContinuous(vl6180x_t *dev)
{
    uint32_t timeout;
    uint16_t ambient = 0;
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    // While computation is not finished
    // only watching if bits 5:3 (mask 0x38) are set to 0b100 (0x20)
    timeout = dev->sampleReadyTimeout > 0 ? dev->sampleReadyTimeout : 1;
    while (true)
    {
        READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);
        if ((interruptStatus & 0x38) == 0x20)
            break;

        dev->interface.delay(1);
        if (--timeout == 0)
            goto error;
    }

    READ_REG(RESULT__ALS_VAL, ambient, 2);
    WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x02, 1);

error:
    return ambient;
}

uint16_t vl6180x_ReadAmbientAsync(vl6180x_t *dev)
{
    uint16_t ambient = 0;
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);
    if ((interruptStatus & 0x38) != 0x20)
        goto error; // Sample not ready yet

    READ_REG(RESULT__ALS_VAL, ambient, 2);
    WRITE_REG(SYSTEM__INTERRUPT_CLEAR, 0x02, 1);

error:
    return ambient;
}

bool vl6180x_GetRangeReady(vl6180x_t *dev)
{
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);

error:
    return (interruptStatus & 0x07) == 0x04;
}

bool vl6180x_GetAmbientReady(vl6180x_t *dev)
{
    uint8_t interruptStatus = 0;

    if (dev == NULL)
        goto error;

    READ_REG(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus, 1);

error:
    return (interruptStatus & 0x38) == 0x20;
}

vl6180x_range_error_t vl6180x_ReadRangeStatus(vl6180x_t *dev)
{
    vl6180x_range_error_t rangeStatus = VL6180X_RANGE_ERROR_DRIVER;

    if (dev == NULL)
        goto error;

    READ_REG(RESULT__RANGE_STATUS, rangeStatus, 1);

error:
    return rangeStatus == VL6180X_RANGE_ERROR_DRIVER ? rangeStatus : (vl6180x_range_error_t) (rangeStatus >> 4);
}