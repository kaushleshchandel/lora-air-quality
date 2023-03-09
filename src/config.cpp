
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "config.h"
#include "types.h"
#include "mqtt.h"

#include "common.h"


#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())

FS *filesystem = &LITTLEFS;

bool deviceHasConfig;   // Set to true if device EEPROM has stored configuration
bool debugMode = false; // Is device in DebugMode ? when in debug mode, all serial messages are sent to MQTT
bool deviceHasWifiCreds = false;

// Coutners for Timers
int previousDiagnosticsMillis = 0;
int previousDataMillis;
int previousSystemMillis;

// Counters for diagnositcs
unsigned long packetsSentPM = 0; // MQTT packets sent
unsigned long packetsFailPM = 0; // MQTT packet sent failure
unsigned long dataLoopsPM = 0;   // Loops with data
unsigned long loopsPM = 0;       // Counter to track executions
unsigned long lastpacketsSentPM = 1;
unsigned long wifiErrors = 0;       // Wifi Connectivity Errors
unsigned long mqttErrors = 0;       // MQTT Error
unsigned long sensorReadErrors = 0; // Sensor read errors


StaticJsonDocument<256> doc;

const char *mqtt_server = PRIMARY_MQTT_SERVER;
int default_mqtt_port = PRIMARY_MQTT_PORT;
const char *primary_mqtt_user = PRIMARY_MQTT_USER;
const char *primary_mqtt_pass = PRIMARY_MQTT_PASSWORD;
const char *mqtt_subs_topic = DEFAULT_SUB_TOPIC;

const char *mqtts_server = SECONDARY_MQTT_SERVER;
int default_mqtts_port = SECONDARY_MQTT_PORT;
const char *secondary_mqtt_user = SECONDARY_MQTT_USER;
const char *secondary_mqtt_pass = SECONDARY_MQTT_PASSWORD;


String chipID = "Secure-" + String(ESP_getChipId(), HEX);

data_config WM_config;


extern PubSubClient mqttclient;


WiFiClientSecure client;
PubSubClient mqttclient(client);



/******************************************
 * debug_string(string,bool)
 * Prints the message to Serial port and sends to MQTT server when device in debug mode
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21-07-08
 *******************************************/
void debug_string(String msg, bool debug_msg/* = false*/)
{
    if (debug_msg == true)
    {
        Serial.print("##");
        Serial.println(msg);
    }
    else
    {
        Serial.print(">>");
        Serial.println(msg);
    }

    if (debugMode == true && mqttclient.connected())
    {
        send_mqtt_string("DEBUG", msg, false);
    }
}


void debug_string(String msg)
{
    Serial.print(">>");
    Serial.println(msg);
}



/******************************************
 * set_defaults()
 * Loads the default values for all the configurations
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void set_defaults()
{
    debug_string("Setting default values", true);

    WM_config.device_config.data_frequency = DEFAULT_SCANNING_FREQUENCY;
    WM_config.device_config.timeZone = DEFAULT_TIME_ZONE;

    strcpy(WM_config.WiFi_Creds.wifi_ssid_local, DEFAULT_SSID);
    strcpy(WM_config.WiFi_Creds.wifi_pw_local, DEFAULT_WIFI_PWD);

    strcpy(WM_config.device_config.primary_mqtt_server, PRIMARY_MQTT_SERVER);
    strcpy(WM_config.device_config.secondary_mqtt_server, SECONDARY_MQTT_SERVER);
    WM_config.device_config.primary_mqtt_port = PRIMARY_MQTT_PORT;
    WM_config.device_config.secondary_mqtt_port = SECONDARY_MQTT_PORT;

    strcpy(WM_config.device_config.device_code, DEFAULT_DEVICE_CODE);
    strcpy(WM_config.device_config.topic_prefix, DEFAULT_TOPIC_PREFIX); // Use Slast at the end to support the blank topics

    // WM_config.device_config.primaryDNS = DEFAULT_DNS_PRIMARY;
    // WM_config.device_config.secondaryDNS = DEFAULT_DNS_SECONDARY;

    // WM_config.device_config.setupMode = 1;
    WM_config.WiFi_Creds.useStaticIP = false;

    strcpy(WM_config.WiFi_Creds.wifi_ssid_local, BLANK_STRING);
    strcpy(WM_config.WiFi_Creds.wifi_pw_local, BLANK_STRING);

    strcpy(WM_config.WiFi_Creds.wifi_ssid, DEFAULT_SSID1);
    strcpy(WM_config.WiFi_Creds.wifi_pw[0], DEFAULT_WIFI_PWD0);
    strcpy(WM_config.WiFi_Creds.wifi_pw[1], DEFAULT_WIFI_PWD1);
    strcpy(WM_config.WiFi_Creds.wifi_pw[2], DEFAULT_WIFI_PWD2);
    strcpy(WM_config.WiFi_Creds.wifi_pw[3], DEFAULT_WIFI_PWD3);
    strcpy(WM_config.WiFi_Creds.wifi_pw[4], DEFAULT_WIFI_PWD4);
    strcpy(WM_config.WiFi_Creds.wifi_pw[5], DEFAULT_WIFI_PWD5);
    strcpy(WM_config.WiFi_Creds.wifi_pw[6], DEFAULT_WIFI_PWD6);
    strcpy(WM_config.WiFi_Creds.wifi_pw[7], DEFAULT_WIFI_PWD7);
    strcpy(WM_config.WiFi_Creds.wifi_pw[8], DEFAULT_WIFI_PWD8);
    strcpy(WM_config.WiFi_Creds.wifi_pw[9], DEFAULT_WIFI_PWD9);

    WM_config.device_config.calibrate_humidity_a = 1;
    WM_config.device_config.calibrate_humidity_b = 0;

    WM_config.device_config.calibrate_temperature_a = 1;
    WM_config.device_config.calibrate_temperature_b = -8;
}

/******************************************
 * save_config_file(void)
 * Save the configurations to EEPROM
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void save_config_file(void)
{
    File file = FileFS.open(CONFIG_FILENAME, "w");

    if (file)
    {
        file.write((uint8_t *)&WM_config, sizeof(WM_config));
        file.close();
        debug_string("Saved Little FS FIle", true);
    }
    else
    {
        debug_string("FAIL!!! Saving Little FS FIle", true);
    }
}

/******************************************
 * read_config_file()
 * Reads Config from EEPROM. if not found, then creates default config file
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21/07/06
 *******************************************/
bool read_config_file()
{
    bool res = false;

    if (!FileFS.begin(true))
    {
        debug_string(F("File System begin : ERROR"), true);
    }
    else
    {
        debug_string(F("File System begin : OK"), true);
    }

    File file = FileFS.open(CONFIG_FILENAME, "r");

    if (file) // File was Opened Succesfully
    {
        debug_string("File read OK", true);
        file.readBytes((char *)&WM_config, sizeof(WM_config));
        file.close();
        res = true;
    }
    else // New system format & save default config file
    {
        debug_string("New device. Format the File system", true);
        FileFS.format();

        if (!FileFS.begin(true))
        {
            debug_string(F("failed! AutoFormatting."), true);
        }
        else
        {
            debug_string(F("Writing Blank Config File"), true);
            set_defaults();
            save_config_file();
        }

        File file = FileFS.open(CONFIG_FILENAME, "r");
        if (file) // File was Opened Succesfully
        {
            file.readBytes((char *)&WM_config, sizeof(WM_config));
            file.close();
            debug_string("New Device : File read OK", true);
        }

        res = false;
    }

    return res;
}

/******************************************
 * serial_print_config()
 * Prints the Main configuration values to Serial port
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21/07/06
 *******************************************/
void serial_print_config()
{
    Serial.print("$$Code=");
    Serial.println(WM_config.device_config.device_code);

    Serial.print("$$Mac=");
    Serial.println(get_beacon_id());

    // Serial.print("$$Setup=");
    // Serial.println(WM_config.device_config.setupMode);

    Serial.print("$$Prefix=");
    Serial.println(WM_config.device_config.topic_prefix);

    Serial.print("$$Version=");
    Serial.println(SW_VERSION);

    Serial.print("$$Hardware=");
    Serial.println(HW_VERSION);

    Serial.print("$$ChipID=");
    Serial.println(chipID);

    Serial.print("$$WiFi1=");
    Serial.println(WM_config.WiFi_Creds.wifi_ssid);
}

/******************************************
 * check_configurations()
 * Checks if the device has the Config file saved to EEPROM
 * If Blank, then creates default configuration file
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21/07/06
 *******************************************/
void check_configurations()
{
    deviceHasConfig = read_config_file();

    String ssid = WM_config.WiFi_Creds.wifi_ssid_local;
    ssid.trim();

    if (ssid == "")
    {
        debug_string("device doesn't have wifi creds");   
        deviceHasWifiCreds = false;
    }
    else
    {
        debug_string("device has stored wifi creds");
        deviceHasWifiCreds = true;
    }

    String deviceCode = WM_config.device_config.device_code;
    deviceCode.trim();
}


void format_config_file(void)
{
  FileFS.format();
}