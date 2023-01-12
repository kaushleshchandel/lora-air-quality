#ifndef MyTypes_h
#define MyTypes_h

#include <WiFi.h>
#include <Arduino.h>

#define SSID_MAX_LEN 32
// From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN 64

typedef struct
{
    char wifi_ssid_local[SSID_MAX_LEN];
    char wifi_pw_local[PASS_MAX_LEN];

    char wifi_ssid[SSID_MAX_LEN];
    char wifi_pw[10][PASS_MAX_LEN];

    bool useStaticIP;
    IPAddress staticIP;
    IPAddress gatewayIP;
    IPAddress subnetIP;
    char reserved[32];
}WiFi_Credentials;

typedef struct
{
    char primary_mqtt_server[24];
    int primary_mqtt_port;
    char secondary_mqtt_server[24];
    int secondary_mqtt_port;
    char topic_prefix[10];
    char device_code[4];
    int data_frequency;
    int timeZone;
    int setupMode;
    int calibrate_temperature_a;
    int calibrate_temperature_b;
    int calibrate_humidity_a;
    int calibrate_humidity_b;
    char reserved[48];

}device_config_data;

typedef struct
{
  WiFi_Credentials WiFi_Creds;
  device_config_data device_config;
}data_config;

#endif