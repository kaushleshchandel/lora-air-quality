
#ifndef CONFIG_H /* include guards */
#define CONFIG_H

#include <Arduino.h>

// file system format
#include "FS.h"
#include <LITTLEFS.h> // https://github.com/lorol/LITTLEFS
#define FileFS LITTLEFS
#define CONFIG_FILENAME F("/config.dat")


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// #include <U8x8lib.h>

#include <Wire.h>
static const PROGMEM u1_t NWKSKEY[16] = {0xbc, 0xa4, 0x6a, 0x13, 0x0e, 0x9f, 0x7e, 0x4e, 0xba, 0x79, 0xa6, 0xcb, 0x02, 0x6f, 0x8c, 0x3c};
static const PROGMEM u1_t APPSKEY[16] = {0x14, 0x0b, 0xf5, 0x03, 0xa1, 0x2d, 0x31, 0x6b, 0x6b, 0x17, 0x2f, 0xd1, 0x1a, 0xf6, 0xaf, 0xc8};
static const u4_t DEVADDR = 0x00f3a25b; // <-- Change this address for every node!



// software version info
#define SW_VERSION "00-00-00"
#define HW_VERSION "esp32-lora" 

// on board pins
#define HOME_PUSH_BUTTON --     //add
#define PIN_Led --              //add


// Wifi Related
#define DEFAULT_SSID "SECURE_SETUP"
#define DEFAULT_WIFI_PWD "securewifi"


// MQTT related
#define BLANK_STRING ""
//-----------------------------------------------------------------
#define PRIMARY_MQTT_SERVER "mqtt.secure.live" 
#define PRIMARY_MQTT_PORT 8883
#define PRIMARY_MQTT_USER "m5-beacon"
#define PRIMARY_MQTT_PASSWORD "hf*!EYfk4jQQjfG9nGf7SHQbJ5"
//-----------------------------------------------------------------
#define SECONDARY_MQTT_SERVER "mqtts.secure.live"
#define SECONDARY_MQTT_PORT 1883
#define SECONDARY_MQTT_USER "m5-beacon"
#define SECONDARY_MQTT_PASSWORD "hf*!EYfk4jQQjfG9nGf7SHQbJ5"
//-----------------------------------------------------------------
#define DEFAULT_MESSAGE_LENGTH 1024
#define DEFAULT_TOPIC_PREFIX ""
#define DEFAULT_SUB_TOPIC "set/#"

#define DEFAULT_DEVICE_CODE "ZZZZ"

#define DEVICE_NAME_LEN 64 // Captiv portal

#define INTIGER_LEN 8

#define DEFAULT_SCANNING_FREQUENCY 10 // 30 // Data sending frequency in seconds
#define DEFAULT_TIME_ZONE 0000


//wifi default ssid and hardcoded passwords
#define DEFAULT_SSID1 "Secure_Veea"
#define DEFAULT_WIFI_PWD0 "securewifi"
#define DEFAULT_WIFI_PWD1 "securewifi"
#define DEFAULT_WIFI_PWD2 "securewifi"
#define DEFAULT_WIFI_PWD3 "securewifi"
#define DEFAULT_WIFI_PWD4 "securewifi"
#define DEFAULT_WIFI_PWD5 "Secure_ATM"
#define DEFAULT_WIFI_PWD6 "securewifi"
#define DEFAULT_WIFI_PWD7 "securewifi"
#define DEFAULT_WIFI_PWD8 "securewifi"
#define DEFAULT_WIFI_PWD9 "securewifi"


void check_configurations();
void debug_string(String msg, bool debug_msg);
void debug_string(String msg);

void serial_print_config();
void set_defaults();
void save_config_file(void);
bool read_config_file();
void format_config_file(void);

#endif