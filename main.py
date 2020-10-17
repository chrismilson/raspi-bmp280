import smbus
import time

DEVICE_ADDRESS = 0x76
DEVICE_REG_CALIB = 0x88
DEVICE_REG_DATA = 0xF7

# Measurement control settings register
DEVICE_REG_CTRL_MEAS = 0xF4
# Temperature ovsersmpling setting
OSRS_T_SKIP = 0x0
OSRS_T_X1 = 0x20
OSRS_T_X2 = 0x40
OSRS_T_X4 = 0x60
OSRS_T_X8 = 0x80
OSRS_T_X16 = 0xA0
# Pressure oversampling setting
OSRS_P_SKIP = 0x0
OSRS_P_X1 = 0x04
OSRS_P_X2 = 0x08
OSRS_P_X4 = 0x0C
OSRS_P_X8 = 0x10
OSRS_P_X16 = 0x14
# Power mode
POWER_MODE_NORMAL = 0x03
POWER_MODE_SLEEP = 0x0
POWER_MODE_FORCED = 0x01

DEVICE_REG_CONFIG = 0xF5
# Ciefficients for the IIRC noise reduction filter on the chip
FILTER_COEF_OFF = 0x0
FILTER_COEF_2 = 0x04
FILTER_COEF_4 = 0x08
FILTER_COEF_8 = 0x0C
FILTER_COEF_16 = 0x14
# The time between measurement cycles
STANDBY_TIME_0_5MS = 0x0
STANDBY_TIME_62_5MS = 0x20
STANDBY_TIME_125MS = 0x40
STANDBY_TIME_250MS = 0x60
STANDBY_TIME_500MS = 0x80
STANDBY_TIME_1000MS = 0xA0
STANDBY_TIME_2000MS = 0xC0
STANDBY_TIME_4000MS = 0xE0


def init(bus):
    """
    Initialise the i2c bus for normal measurement
    """

    bus.write_byte_data(
        DEVICE_ADDRESS,
        DEVICE_REG_CTRL_MEAS,
        OSRS_T_X2 | OSRS_P_X16 | POWER_MODE_NORMAL
    )
    bus.write_byte_data(
        DEVICE_ADDRESS,
        DEVICE_REG_CONFIG,
        STANDBY_TIME_0_5MS | FILTER_COEF_16
    )
    time.sleep(0.5)


def getCalibrationData(bus):
    buffer = bus.read_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG_CALIB, 24)

    # The data that we are interested in is in two-byte chunks.
    values = []

    # There are three temperature values and nine pressure values.
    for i in range(12):
        val = (buffer[2 * i] << 8) | buffer[2 * i + 1]
        if i != 0 and i != 3 and val >= 1 << 15:
            val -= 1 << 16

        values.append(val)

    return values


def getCalibratedData(bus, calibrationData=None):
    if calibrationData == None:
        calibrationData = getCalibrationData(bus)
    buffer = bus.read_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG_DATA, 6)
    pressureData = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) >> 4
    temperatureData = ((buffer[3] << 16) | (buffer[4] << 8) | buffer[5]) >> 4

    tempCal, presCal = calibrationData[:3], calibrationData[3:]

    # This process for calculating the temperature and pressure is from the
    # documentation. It is done in such a way that no registers will overflow,
    # and the minimum amount of precision from limited double space will be
    # lost.
    # [here](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
    # I did not decide on this implementation and I believe it to be VERY hard
    # to read, letalone debug. Not the best.
    var1 = (((temperatureData >> 3) - (tempCal[0] << 1)) * tempCal[1]) >> 11
    var2 = (temperatureData >> 4) - tempCal[0]
    var2 = (((var2 * var2) >> 12) * tempCal[2]) >> 14
    t_fine = var1 + var2
    temperature = ((t_fine * 5 + 128) >> 8) / 100

    var1 = t_fine - 128000
    var2 = var1 * var1 * presCal[5]
    var2 = var2 + ((var1 * presCal[4]) << 17)
    var2 = var2 + (presCal[3] << 35)
    var1 = ((var1 * var1 * presCal[2]) >> 8) + ((var1 * presCal[1]) << 12)
    var1 = (((1 << 47) + var1) * presCal[0]) >> 33

    if var1 == 0:
        return temperature, float('nan')

    pressure = 1048576 - pressureData
    pressure = ((pressure << 31) - var2) * 3125 / var1
    var1 = (presCal[8] * (pressure >> 13) * (pressure >> 13)) >> 25
    var2 = (presCal[7] * pressure) >> 19
    pressure = ((pressure + var1 + var2) >> 8) + (presCal[6] << 4)
    pressure = pressure / 100

    return temperature, pressure


bus = smbus.SMBus(1)
init(bus)
calibrationData = getCalibrationData(bus)

while True:
    temperature, pressure = getCalibratedData(bus, calibrationData)

    print(f'The temperature is: {temperature:.2f}°C')
    print(f'The pressure is {pressure:.2f}hPa')
    print()
    time.sleep(0.001)
