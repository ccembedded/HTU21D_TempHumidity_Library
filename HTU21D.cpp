/*! \file HTU21D.cpp
    \brief HTU21D Temperature Humidity Sensor Library
*/

#include <Wire.h>
#include <inttypes.h>
#include "HTU21D.h"
#include "Arduino.h"

#define HTU21D_CRC_SIZE        8
#define HTU21D_DATA_BIT_SIZE   16
#define HTU21D_CRC_POLYNOMIAL  0x131

#define HTU21D_HUM_DELAY_MS    0
#define HTU21D_TEMP_DELAY_MS   1

#define HTU21D_RESOLUTION_TO_TIME_TABLE_ROWS    4
#define HTU21D_RESOLUTION_TO_TIME_TABLE_COLS    2

uint8_t resolutionToMeasTimeMs[HTU21D_RESOLUTION_TO_TIME_TABLE_ROWS][HTU21D_RESOLUTION_TO_TIME_TABLE_COLS] =
{ // HTU21D_HUM_DELAY_MS                HTU21D_TEMP_DELAY_MS
    {HTU21D_HUM_MEASURE_TIME_MS_12_BIT, HTU21D_TEMP_MEASURE_TIME_MS_14_BIT}, // HTU21D_RESOLUTION_RH_12_T_14
    {HTU21D_HUM_MEASURE_TIME_MS_8_BIT,  HTU21D_TEMP_MEASURE_TIME_MS_12_BIT}, // HTU21D_RESOLUTION_RH_8_T_12
    {HTU21D_HUM_MEASURE_TIME_MS_10_BIT, HTU21D_TEMP_MEASURE_TIME_MS_13_BIT}, // HTU21D_RESOLUTION_RH_10_T_13
    {HTU21D_HUM_MEASURE_TIME_MS_11_BIT, HTU21D_TEMP_MEASURE_TIME_MS_11_BIT}  // HTU21D_RESOLUTION_RH_11_T_11
};

/*!
    \brief Reads temperature data from sensor
    \return Raw temperature value
            Returns HTU21D_CHECKSUM_ERROR if there is a checksum error
*/
uint16_t HTU21D::readRawTemp(void)
{
    uint16_t data;
    uint8_t checksum;
    uint8_t resolution;
    uint8_t meas_delay_ms;
    bool matches;

    data = 0;
    checksum = 0;
    resolution = readUserRegResolution();
    meas_delay_ms = resolutionToMeasTimeMs[resolution][HTU21D_TEMP_DELAY_MS];

    Wire.beginTransmission(HTU21D_ADDRESS);
    Wire.write(HTU21D_TEMP_MEASUREMENT_HOLD_CMD);
    Wire.endTransmission();
    Wire.requestFrom(HTU21D_ADDRESS, 3);

    delay(meas_delay_ms);

    data = Wire.read() << 8;
    data |= Wire.read();

    checksum = Wire.read();

    matches = verifyChecksum(data, checksum);

    /* Clear status bits */
    data &= ~((uint16_t)HTU21D_STATUS_BITs_MASK);

    if(matches != true)
    {
        data = HTU21D_CHECKSUM_ERROR;
    }

    return (data);
}

/*!
    \brief Converts raw temperature value to Fahernheit
    \param[in] raw_temp Raw temperature value
    \return Temperature in Fahrenheit
*/
float HTU21D::convertRawTempToF(uint16_t raw_temp)
{
    float tempC;
    float tempF;

    tempC = convertRawTempToC(raw_temp);

    tempF = (tempC * HTU21D_TEMP_F_TO_C_FACTOR) + HTU21D_TEMP_F_TO_C_OFFSET;

    return (tempF);
}

/*!
    \brief Converts raw temperature value to Celsius
    \param[in] raw_temp Raw temperature value
    \return Temperature in Celsius
*/
float HTU21D::convertRawTempToC(uint16_t raw_temp)
{
    float tempC;

    tempC = (float)raw_temp;

    tempC = (tempC * HTU21D_RAW_TEMP_TO_C_FACTOR) + HTU21D_RAW_TEMP_TO_C_OFFSET;

    return (tempC);
}

/*!
    \brief Reads humidity data from sensor
    \return Raw humidity value
            Returns HTU21D_CHECKSUM_ERROR if there is a checksum error
*/
uint16_t HTU21D::readRawHumidity(void)
{
    uint16_t data;
    uint8_t result;
    uint8_t checksum;
    uint8_t resolution;
    uint8_t meas_delay_ms;
    bool matches;

    data = 0;
    checksum = 0;
    resolution = readUserRegResolution();
    meas_delay_ms = resolutionToMeasTimeMs[resolution][HTU21D_TEMP_DELAY_MS];

    Wire.beginTransmission(HTU21D_ADDRESS);
    Wire.write(HTU21D_HUM_MEASUREMENT_HOLD_CMD);
    result = Wire.endTransmission();

    Wire.requestFrom(HTU21D_ADDRESS, 3);

    delay(meas_delay_ms);

    data = Wire.read() << 8;
    data |= Wire.read();

    checksum = Wire.read();

    matches = verifyChecksum(data, checksum);

    data &= ~((uint16_t)HTU21D_STATUS_BITs_MASK); /* Clear status bits */

    if(matches != true)
    {
        data = HTU21D_CHECKSUM_ERROR;
    }

    return (data);
}

/*!
    \brief Converts raw humidity value to relative humidity
    \param[in] raw_humidity Raw humidity
    \return Relative humidity value
*/
float HTU21D::convertRawHumidityToHumidity(uint16_t raw_humidity)
{
    float outputHumidity;

    outputHumidity = (float)raw_humidity;

    outputHumidity = (outputHumidity * HTU21D_RAW_HUM_TO_REL_HUM_FACTOR) + HTU21D_RAW_HUM_TO_REL_HUM_OFFSET;

    return (outputHumidity);
}

/*!
    \brief Calculates checksum and compares against received checksum
    \param[in] data_in Sensor data
    \param[in] checksum Received checksum from sensor
    \return true if checksums match, false otherwise
*/
bool HTU21D::verifyChecksum(uint16_t data_in, uint8_t checksum)
{
    uint32_t data;
    uint32_t polynomial;
    uint32_t mask;
    uint8_t poly_msb_pos;
    bool matches;

    data = (uint32_t)data_in << HTU21D_CRC_SIZE;
    polynomial = (uint32_t)HTU21D_CRC_POLYNOMIAL << (HTU21D_DATA_BIT_SIZE - 1);
    mask = 0;
    poly_msb_pos = (HTU21D_DATA_BIT_SIZE - 1) + HTU21D_CRC_SIZE;
    matches = false;

    for(uint8_t count = 0; count < HTU21D_DATA_BIT_SIZE; count++)
    {
        mask = (uint32_t)1 << poly_msb_pos;

        if((data & mask) != 0)
        {
            // Data bit above polynomial MSB is 1
            // Compute XOR
            data ^= polynomial;
            polynomial >>= 1;
            poly_msb_pos--;

        }
        else
        {
            // Data bit above polynomial MSB is 0
            // No XOR
            polynomial >>= 1;
            poly_msb_pos--;
        }
    }

    if(data == checksum)
    {
        matches = true;
    }

    return (matches);
}

/*!
    \brief Reads the user register
    \return User register data
*/
uint8_t HTU21D::readUserReg(void)
{
    uint8_t result;
    uint8_t data;

    Wire.beginTransmission(HTU21D_ADDRESS);
    Wire.write(HTU21D_READ_USER_REG_CMD);
    result = Wire.endTransmission();
    Wire.requestFrom(HTU21D_ADDRESS, 1);
    data = Wire.read();

    return (data);
}

/*!
    \brief Reads resolution value from user register
    \return Measurement resolution
*/
uint8_t HTU21D::readUserRegResolution(void)
{
    uint8_t user_reg;
    uint8_t resolution;

    user_reg = readUserReg();

    resolution = (user_reg & HTU21D_USER_REG_RESOLUTION_H_MASK) >> (HTU21D_USER_REG_RESOLUTION_H_SHIFT - 1);
    resolution |= user_reg & HTU21D_USER_REG_RESOLUTION_L_MASK;

    return (resolution);
}

/*!
    \brief Reads status of on chip heater
    \return Heater status
*/
uint8_t HTU21D::readUserRegHeater(void)
{
    uint8_t user_reg;
    uint8_t heater;

    user_reg = readUserReg();

    heater = (user_reg & HTU21D_USER_REG_HEATER_MASK) >> HTU21D_USER_REG_HEATER_SHIFT;

    return (heater);
}

/*!
    \brief Turns on chip heater on/off
    \param[in] heater_req Value to write to heater status
*/
void HTU21D::writeUserRegHeater(uint8_t heater_req)
{
    uint8_t user_reg;

    user_reg = readUserReg();

    if(user_reg & HTU21D_USER_REG_HEATER_MASK)
    {
        if(heater_req == HTU21D_HEATER_DISABLE)
        {
            // Heater is on
            // Turn off
            user_reg ^= HTU21D_USER_REG_HEATER_MASK;
        }
    }
    else
    {
        if(heater_req == HTU21D_HEATER_ENABLE)
        {
            // Heater is off
            // Turn on
            user_reg |= HTU21D_USER_REG_HEATER_MASK;
        }
    }

    Wire.beginTransmission(HTU21D_ADDRESS);
    Wire.write(HTU21D_WRITE_USER_REG_CMD);
    Wire.write(user_reg);
    Wire.endTransmission();
}
