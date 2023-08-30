#include <Wire.h>
#include <inttypes.h>
#include "HTU21D.h"

void setup()
{
    Wire.begin();
    Serial.begin(9600);
}

void loop()
{
    HTU21D temph;
    uint16_t raw_temp;
    uint16_t raw_h;
    float tempF;
    float h;
    static bool test_heat = true;

    raw_temp = temph.readRawTemp();
    if(raw_temp != HTU21D_CHECKSUM_ERROR)
    {
        tempF = temph.convertRawTempToF(raw_temp);
        Serial.print("Temperature F: ");
        Serial.println(tempF);
    }
    else
    {
        Serial.println("Temperature Checksum Error");
    }

    raw_h = temph.readRawHumidity();
    if(raw_h != HTU21D_CHECKSUM_ERROR)
    {
        h = temph.convertRawHumidityToHumidity(raw_h);
        Serial.print("Humidity: ");
        Serial.println(h);
    }
    else
    {
        Serial.println("Humidity Checksum Error");
    }

    delay(5000);
}

void testHeat(HTU21D *temph)
{
    uint16_t raw_temp;
    uint16_t raw_h;
    uint16_t raw_temp_before, raw_temp_after;
    uint16_t raw_h_before, raw_h_after;
    uint8_t ureg;
    float tempF;
    float h;
    static bool test_heat = true;

    raw_temp = temph->readRawTemp();
    tempF = temph->convertRawTempToF(raw_temp);
    Serial.print("\n\nBefore Heat Temperature F: ");
    Serial.println(tempF);

    raw_h = temph->readRawHumidity();
    h = temph->convertRawHumidityToHumidity(raw_h);
    Serial.print("Before Heat Humidity: ");
    Serial.println(h);

    raw_temp_before = raw_temp;
    raw_h_before = raw_h;

    ureg = temph->readUserReg();

    char buff[100];
    snprintf(buff, 100, "User reg before heater: %x\n", ureg);
    Serial.print(buff);

    temph->writeUserRegHeater(HTU21D_HEATER_ENABLE);
    ureg = temph->readUserReg();
    snprintf(buff, 100, "User reg after heater: %x\n", ureg);
    Serial.print(buff);

    delay(2000);

    raw_temp = temph->readRawTemp();
    tempF = temph->convertRawTempToF(raw_temp);
    Serial.print("After Heat Temperature F: ");
    Serial.println(tempF);

    raw_h = temph->readRawHumidity();
    h = temph->convertRawHumidityToHumidity(raw_h);
    Serial.print("After Heat Humidity: ");
    Serial.println(h);

    raw_temp_after = raw_temp;
    raw_h_after = raw_h;

    temph->writeUserRegHeater(HTU21D_HEATER_DISABLE);
    ureg = temph->readUserReg();
    snprintf(buff, 100, "User reg heat off end: %x\n", ureg);
    Serial.print(buff);

    snprintf(buff, 100, "Temp Before: %x\nTemp After: %x\nHumidity Before: %x\nHumidity After: %x\n\n", raw_temp_before, raw_temp_after, raw_h_before, raw_h_after);
    Serial.print(buff);
}
