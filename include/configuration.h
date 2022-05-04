#ifndef CONFIG_h
#define CONFIG_h

#include <EEPROM.h>

#define MAGIC 0xEBBEAFF0
#define EEPROMADDR 0

struct SConfiguration
{
    // magic header
    unsigned int signature;

    // sensor correction
    float offset_temperature_indoor;
    float offset_humidity_indoor;
    float offset_temperature_outdoor;
    float offset_humidity_outdoor;

    // automatic rules
    float min_temperature_indoor;
    float min_temperature_outdoor;

    float max_humidity_outdoor;
    float min_humidity_indoor;

    float threshold_dewpoint_on;
    float threshold_dewpoint_off;

    unsigned int sleeptime_ms_fan_on;
    unsigned int deepsleeptime_ms_fan_off;

    float min_voltage;

    float voltage_correction_factor;
} configuration;

struct SSensorData
{
  float temperature;
  float humidity;
  float abshumidity;
  float dewpoint;

} sensorDataIndoor, sensorDataOutdoor;


void initconfig(SConfiguration *conf)
{
    conf->signature = MAGIC;

    conf->offset_temperature_indoor = 0.0;
    conf->offset_humidity_indoor = 0.0;
    conf->offset_temperature_outdoor = 0.0;
    conf->offset_humidity_outdoor = 0.0;

    conf->min_temperature_indoor = 8.0;
    conf->min_temperature_outdoor = 0.0;

    conf->max_humidity_outdoor = 70.0;
    conf->min_humidity_indoor = 50.0;

    conf->threshold_dewpoint_on = 5.0;
    conf->threshold_dewpoint_off = 1.0;

    conf->sleeptime_ms_fan_on = 1000 * 60 * 5;
    conf->deepsleeptime_ms_fan_off = 1000 * 60 * 30;

    conf->min_voltage = 12.4;
    conf->voltage_correction_factor = 1.0;
}
bool saveconfig(SConfiguration *conf)
{
    EEPROM.put(EEPROMADDR, *conf);
    EEPROM.commit();
    Serial.println(F("configuration stored"));
    return true;
}

bool loadconfig(SConfiguration *conf)
{
    EEPROM.begin(512);
    EEPROM.get(EEPROMADDR, *conf);

    if (conf->signature != MAGIC)
    {
        initconfig(conf);
        saveconfig(conf);
    }
    Serial.println(F("configuration loaded"));
    return true;
}

#endif