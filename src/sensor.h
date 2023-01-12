
#include <Wire.h>
#include <SoftwareSerial.h>

#include "Adafruit_SGP30.h"
#include <Adafruit_AHTX0.h>
#include <Adafruit_TSL2561_U.h>

#include <driver/i2s.h>

#include <ArduinoJson.h>

#define PM_RX 13 //18
#define PM_TX 12 //19

// Structure in which PM Sensor sends data to Serial port
struct pms5003data
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};


class ENV_Sensor
{
  public:
  ENV_Sensor();

  StaticJsonDocument<256> doc;
  char sensor_data_buffer[256];

  int read();

  int _pm1 = 0;
  int _pm25 = 0;
  int _pm10 = 0;
  int _voc = 0;
  int _co2 = 0;
  int _temp = 0;
  int _humi = 0;
  int _lux = 0;
  int _rawh2 = 0;
  int _raweth = 0;

};


void check_sensors();
boolean readPMSdata(Stream *s);
void read_pm();
void read_lumen();
void read_aht();
void read_voc();
void configureSensor(void);
void displaySensorDetails(void);
void i2s_install();
void i2s_setpin();
void read_mic();
void i2c_scanner(void);
void display_data_oled();