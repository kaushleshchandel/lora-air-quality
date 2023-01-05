/*

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16


  The Writing diagram for ESP32

  I2C : Connected to GPIO04 and GPIO15
  Air Quality : Connected to GPIO18 & GPIO19

  Mic : Connected to GPIO 25, GPIO32, and GPIO33
  MIC_SCK --
  MIC_WS  --
  MIC_LR  --
  MIC_VDD --
  MIC_SD  --

*/

#include "heltec.h"
#include "images.h"
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <Adafruit_AHTX0.h>
#include <Adafruit_TSL2561_U.h>
#include <driver/i2s.h>
#include <SoftwareSerial.h>

// Connections to I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

#define PM_RX 18
#define PM_TX 19

// Use I2S Processor
#define I2S_PORT I2S_NUM_0
#define bufferLen 64
int16_t sBuffer[bufferLen]; // Buffer used for Audio data

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
struct pms5003data data;

SoftwareSerial pmsSerial(PM_RX, PM_TX); // Virtual Serial port for PM Sensor
Adafruit_SGP30 sgp;
Adafruit_AHTX0 aht;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); // Light Sensor

/********************* VARIABLES TO HOLD THE CURRENT VALUES***************/
sensors_event_t event_lux;
sensors_event_t humidity, temp; // Used by teh Temp & Humidity Sensor

/******************** LORA*********************/
#define BAND 868E6 // For Heltech OLED
unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet;

bool is_voc_present = false;
bool is_temp_present = false;
bool is_aq_present = false;
bool is_light_present = false;
bool is_mic_present = false;

/**************************************************************************/
/*
    Reads PM Sensor data from Serial port
    checks the data quality based on checksum
*/
/**************************************************************************/
boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum)
  {
    // Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

/**************************************************************************/
/*
    Install I2S Driver for microphone
*/
/**************************************************************************/
void i2s_install()
{
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 44100,
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = bufferLen,
      .use_apll = false};

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) == ESP_OK)
    Serial.println("I2S Init OK");
  else
    Serial.println("I2S Init Failed");
}

/**************************************************************************/
/*
    Sets the Pins for I2S Microphone
*/
/**************************************************************************/
void i2s_setpin()
{
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" lux");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" lux");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561 Light Sensor
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true); /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print("Gain:         ");
  Serial.println("Auto");
  Serial.print("Timing:       ");
  Serial.println("13 ms");
  Serial.println("------------------------------------");
}

/**************************************************************************/
/*
    Calculates the absolute Humidity based on the current temperature
*/
/**************************************************************************/
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

/**************************************************************************/
/*
    Draw the Logo bitmap. Need to change this to Secure
*/
/**************************************************************************/
void logo()
{
  Heltec.display->clear();
  Heltec.display->drawXbm(0, 5, logo_width, logo_height, logo_bits);
  Heltec.display->display();
}

/**************************************************************************/
/*
    Tries 100 times to read the PM Sensor
    Due to synchronization issue with Virtual Serial
*/
/**************************************************************************/
void read_pm()
{

  for (int i; i < 100; i++)
  {
    bool pmread = readPMSdata(&pmsSerial);
    if (pmread)
    {
      // reading data was successful!
      Serial.print("PM 1.0: ");
      Serial.println(data.pm10_standard);
      Serial.print("\t\tPM 2.5: ");
      Serial.print(data.pm25_standard);
      Serial.print("\t\tPM 10: ");
      Serial.println(data.pm100_standard);
      break;
    }

    else
      delay(10);
  }
}

/**************************************************************************/
/*
    Read the Microphone. This needs to be switched to a Thread
*/
/**************************************************************************/
void read_mic()
{

  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0)
    {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i)
      {
        mean += (sBuffer[i]);
      }

      // Average the data reading
      mean /= samples_read;

      // Print to serial plotter
      Serial.println(mean);
    }
  }
  else
    Serial.println("AUDIO ERROR");
}

/********************************/
// Check Connected sensors
/********************************/
void check_sensors()
{

  Heltec.display->clear();
  pmsSerial.begin(9600);

  for (int i; i < 100; i++)
  {
    is_aq_present = readPMSdata(&pmsSerial);
    if (is_aq_present)
      break;
    else
      delay(10);
  }

  if (is_aq_present)
  {
    Heltec.display->drawString(0, 0, "PM Sensor");
    Heltec.display->drawString(80, 0, "[OK]");
    Heltec.display->display();
    Serial.println(" [ AQM PM Present ] ");
  }
  else
  {
    Heltec.display->drawString(0, 0, "PM Sensor");
    Heltec.display->drawString(80, 0, "[ERR]");
    Heltec.display->display();
    Serial.println(" [ AQM PM Not found ] ");
  }

  // Scanning I2C Sensors
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  int sensorcnt = 1;

  nDevices = 1;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0X");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" ( " + String(address) + " )  !");

      if (address == 56)
        Serial.println(" [ OLED Display] ");
      else if (address == 60)
      {
        is_voc_present = true;
        Heltec.display->drawString(0, 12 * sensorcnt, "VOC/CO2");
        Heltec.display->drawString(80, 12 * sensorcnt, "[OK]");
        Heltec.display->display();
        Serial.println(" [ SGP VOC ] ");
        delay(500);
        sensorcnt++;
      }
      else if (address == 88)
      {
        is_temp_present = true;
        Heltec.display->drawString(0, 12 * sensorcnt, "Temp/Humi");
        Heltec.display->drawString(80, 12 * sensorcnt, "[OK]");
        Heltec.display->display();
        Serial.println(" [ AHT10 Temp Humidity ] ");
        delay(500);
        sensorcnt++;
      }

      else if (address == 57)
      {
        is_light_present = true;
        Heltec.display->drawString(0, 12 * sensorcnt, "Light");
        Heltec.display->drawString(80, 12 * sensorcnt, "[OK]");
        Heltec.display->display();
        Serial.println(" [ TSL2561 Lumen ] ");
        delay(500);
        sensorcnt++;
      }

      else
        Serial.println(" [ Unknown ] ");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  // Set up I2S
  i2s_install();
  i2s_setpin();

  if (!i2s_start(I2S_PORT) == ESP_OK)
  {
    Serial.println("ERROR! Starting I2S Port");
    Serial.println("I2S Port: OK");
    Heltec.display->drawString(0, 12 * 4, "Sound  [ERR]");
    Heltec.display->display();
  }
  else
  {
    Serial.println("I2S Port: OK");
    Heltec.display->drawString(0, 12 * 4, "Sound");
    Heltec.display->drawString(80, 12 * 4, "[OK]");
    Heltec.display->display();
  }

  if (!sgp.begin())
  {
    Serial.println("SGP VOC Sensor not found :(");
  }
  else
  {
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
  }

  if (!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
  }
  else
    Serial.println("AHT10 or AHT20 found");

  if (!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
  }
  else
  {
    displaySensorDetails();
    /* Setup the sensor gain and integration time */
    configureSensor();
  }
}

/********************************/
/********************************/
void setup()
{
  // WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->setFont(ArialMT_Plain_10);
  delay(1500);
  Heltec.display->clear();

  check_sensors();

  delay(1000);
}

/**************************************************************************/
/*
    Read the Lumen sensor for light intesity.
*/
/**************************************************************************/
void read_lumen()
{
  /* Get a new sensor event */

  tsl.getEvent(&event_lux);

  /* Display the results (light is measured in lux) */
  if (event_lux.light)
  {
    Serial.print(event_lux.light);
    Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
}

/**************************************************************************/
/*
    Read teh AHT Temperature Sensor
*/
/**************************************************************************/
void read_aht()
{

  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.print("\t\tdegrees C");
  Serial.print("\t\tHumidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println("% rH");
}

/**************************************************************************/
/*
    Read the VOC & CO2 Values
*/
/**************************************************************************/
void read_voc()
{
  if (!sgp.IAQmeasure())
  {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC ");
  Serial.print(sgp.TVOC);
  Serial.print(" ppb\t");
  Serial.print("eCO2 ");
  Serial.print(sgp.eCO2);
  Serial.print(" ppm");

  if (!sgp.IAQmeasureRaw())
  {
    Serial.println("Raw Measurement failed");
    return;
  }
  Serial.print("H2 : ");
  Serial.print(sgp.rawH2);
  Serial.print(" \t");
  Serial.print("Raw Ethanol ");
  Serial.print(sgp.rawEthanol);
  Serial.println("");

  delay(1000);

  counter++;
  if (counter == 30)
  {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
    {
      Serial.println("Failed to get baseline readings");
      return;
    }
    Serial.print("****Baseline values: eCO2: 0x");
    Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x");
    Serial.println(TVOC_base, HEX);
  }
}

/**************************************************************************/
/*
    Display the valeus on the OLED Screen
*/
/**************************************************************************/
void display()
{

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);

  // Display TVOC & CO2
  Heltec.display->drawString(0, 0, "VOC: ");
  Heltec.display->drawString(25, 0, String(sgp.TVOC));
  Heltec.display->drawString(60, 0, "CO2: ");
  Heltec.display->drawString(100, 0, String(sgp.eCO2));

  if (!sgp.IAQmeasureRaw())
  {
    Serial.println("Raw Measurement failed");
  }
  else
  {

    Heltec.display->drawString(0, 12, "H2 ");
    Heltec.display->drawString(25, 12, String(sgp.rawH2));

    Heltec.display->drawString(60, 12, "ETH  ");
    Heltec.display->drawString(100, 12, String(sgp.rawEthanol));
  }

  // PM 2.5 & PM 10
  Heltec.display->drawString(0, 24, "PM2.5 ");
  Heltec.display->drawString(45, 24, String(data.pm25_standard));

  Heltec.display->drawString(60, 24, "PM10 ");
  Heltec.display->drawString(100, 24, String(data.pm100_standard));

  // Temperature & humidity
  Heltec.display->drawString(0, 36, "Temp.");
  Heltec.display->drawString(45, 36, String(temp.temperature));

  Heltec.display->drawString(60, 36, "Hum.");
  Heltec.display->drawString(100, 36, String(humidity.relative_humidity));

  // Lumens
  Heltec.display->drawString(0, 48, "LUX");
  Heltec.display->drawString(45, 48, String(event_lux.light));

  // Heltec.display->drawString(60, 40, "Hum.");
  // Heltec.display->drawString(100, 40, String(data.pm100_standard));

  Heltec.display->display();
}

void loop()
{
  Serial.println("**********************");

  read_pm();
  read_voc();
  read_aht();
  read_lumen();
  // read_mic(); // Need to move to a seperate thread for Microphone DB Sensor
  display();
  delay(1000);
  counter++;
}
