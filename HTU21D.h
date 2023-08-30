/*! \file HTU21D.h
    \brief HTU21D Temperature Humidity Sensor Library
*/

#ifndef _HTU21D_H_
#define _HTU21D_H_

#define HTU21D_ADDRESS                          0x40


// Commands
#define HTU21D_TEMP_MEASUREMENT_HOLD_CMD        0xE3
#define HTU21D_TEMP_MEASUREMENT_NO_HOLD_CMD     0xF3

#define HTU21D_HUM_MEASUREMENT_HOLD_CMD         0xE5
#define HTU21D_HUM_MEASUREMENT_NO_HOLD_CMD      0xF5

#define HTU21D_WRITE_USER_REG_CMD               0xE6
#define HTU21D_READ_USER_REG_CMD                0xE7

#define HTU21D_SOFT_RESET_CMD                   0xFE


// Masks and shifts
#define HTU21D_STATUS_BITs_MASK                 0x2

#define HTU21D_USER_REG_RESOLUTION_H_MASK       0x80
#define HTU21D_USER_REG_RESOLUTION_L_MASK       0x01

#define HTU21D_USER_REG_RESOLUTION_H_SHIFT      7

#define HTU21D_USER_REG_HEATER_MASK             0x4
#define HTU21D_USER_REG_HEATER_SHIFT            2

#define HTU21D_RESOLUTION_RH_12_T_14            0
#define HTU21D_RESOLUTION_RH_8_T_12             1
#define HTU21D_RESOLUTION_RH_10_T_13            2
#define HTU21D_RESOLUTION_RH_11_T_11            3


// Values
#define HTU21D_RAW_TEMP_TO_C_FACTOR             (175.72f / 65536.0f)
#define HTU21D_RAW_TEMP_TO_C_OFFSET             -46.85f

#define HTU21D_TEMP_F_TO_C_FACTOR               (9.0f/5.0f)
#define HTU21D_TEMP_F_TO_C_OFFSET               32.0f

#define HTU21D_RAW_HUM_TO_REL_HUM_FACTOR        (125.0f / 65536.0f)
#define HTU21D_RAW_HUM_TO_REL_HUM_OFFSET        -6.0f

#define HTU21D_TEMP_MEASURE_TIME_MS_14_BIT      50
#define HTU21D_TEMP_MEASURE_TIME_MS_13_BIT      25
#define HTU21D_TEMP_MEASURE_TIME_MS_12_BIT      13
#define HTU21D_TEMP_MEASURE_TIME_MS_11_BIT      7

#define HTU21D_HUM_MEASURE_TIME_MS_12_BIT       16
#define HTU21D_HUM_MEASURE_TIME_MS_11_BIT       8
#define HTU21D_HUM_MEASURE_TIME_MS_10_BIT       5
#define HTU21D_HUM_MEASURE_TIME_MS_8_BIT        3

#define HTU21D_HEATER_DISABLE                   0
#define HTU21D_HEATER_ENABLE                    1

#define HTU21D_CHECKSUM_ERROR                   0xFFFF

#define HTU21D_TEMP_HUMIDITY_BYTES              3
#define HTU21D_USER_REG_BYTES                   1


class HTU21D
{
    public:
        uint16_t readRawTemp(void);
        float convertRawTempToF(uint16_t raw_temp);
        float convertRawTempToC(uint16_t raw_temp);
        uint16_t readRawHumidity(void);
        float convertRawHumidityToHumidity(uint16_t raw_humidity);
        bool verifyChecksum(uint16_t data_in, uint8_t checksum);
        uint8_t readUserReg(void);
        uint8_t readUserRegResolution(void);
        uint8_t readUserRegHeater(void);
        void writeUserRegHeater(uint8_t heater_req);
};

#endif
