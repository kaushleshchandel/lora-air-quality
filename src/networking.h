
extern IPAddress DEFAULT_DNS_PRIMARY /*(8, 8, 8, 8)*/;
extern IPAddress DEFAULT_DNS_SECONDARY /*(8, 8, 4, 4)*/;

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _WIFIMGR_LOGLEVEL_ 1

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "types.h"

#include "common.h"

extern String chipID;
extern bool wifi_Online;

bool useStoredtWifi = false;

extern data_config WM_config;

extern bool deviceHasWifiCreds;

void check_WiFi(void)
{
  if ((WiFi.status() != WL_CONNECTED))
  {

    debug_string("WiFi lost....", true);
    wifi_Online = false;
    // connectMultiWiFi(true);
  }
}

bool wifi_scan(String ssid)
{
    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0)
    {
        Serial.println("no networks found");
    }
    else
    {
        Serial.print(n);
        Serial.println(" networks found");

        for (uint8_t i = 0; i < n; i++)
        {
            if (WiFi.SSID(i) == ssid)
            {
                Serial.print(WiFi.SSID(i));
                Serial.print(" (");
                Serial.print(WiFi.RSSI(i));
                Serial.print(")");
                Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
                delay(10);
                return true;
            }
        }
    }

    return false;
}

bool connect_to_stored_wifi(void)
{
    bool res = false;

    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_STA);
    debug_string(WM_config.WiFi_Creds.wifi_ssid_local, true);

    WiFi.setHostname(chipID.c_str()); // Set the device Hostname
    WiFi.begin(WM_config.WiFi_Creds.wifi_ssid_local, WM_config.WiFi_Creds.wifi_pw_local);

    int i = 0;

    int status = WiFi.status();
    IPAddress gateway;
    IPAddress subnet;

    while ((i++ < 10) && (status != WL_CONNECTED))
    {
        status = WiFi.status();

        if (status == WL_CONNECTED)
        {
            debug_string("WiFi Connected");
            gateway = WiFi.gatewayIP();
            Serial.print("##GATEWAY: ");
            subnet = WiFi.subnetMask();
            Serial.print("##SUBNET: ");
            Serial.println(subnet);
            Serial.print("##ENCRYPTION ");
            Serial.println(WiFi.encryptionType(WIFI_STA));
            Serial.print("##RSSI: ");
            Serial.println(WiFi.RSSI());
            debug_string("connected. Local IP: " + WiFi.localIP().toString(), true);
            res = true;
            break;
        }
        else
        {
            debug_string("waiting for WiFi to connect...", true);
            delay(1000);
        }
    }
    return res;
}

bool new_wifi_password(void)
{
    bool res = false;

    int status = WiFi.status();
    IPAddress gateway;
    IPAddress subnet;

    for (int i = 0; (i < 10) && (WiFi.status() != WL_CONNECTED); ++i)
    {
        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_STA);
        debug_string(WM_config.WiFi_Creds.wifi_ssid, true);

        WiFi.setHostname(chipID.c_str()); // Set the device Hostname
        WiFi.begin(WM_config.WiFi_Creds.wifi_ssid, WM_config.WiFi_Creds.wifi_pw[i]);

        for (int j = 0; j < 10; ++j)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                debug_string("WiFi Connected");
                gateway = WiFi.gatewayIP();
                Serial.print("##GATEWAY: ");
                subnet = WiFi.subnetMask();
                Serial.print("##SUBNET: ");
                Serial.println(subnet);
                Serial.print("##ENCRYPTION ");
                Serial.println(WiFi.encryptionType(WIFI_STA));
                Serial.print("##RSSI: ");
                Serial.println(WiFi.RSSI());
                debug_string("connected. Local IP: " + WiFi.localIP().toString(), true);
                res = true;

                strcpy(WM_config.WiFi_Creds.wifi_ssid_local, WM_config.WiFi_Creds.wifi_ssid);
                strcpy(WM_config.WiFi_Creds.wifi_pw_local, WM_config.WiFi_Creds.wifi_pw[i]);
                save_config_file();

                break;
            }
            else
            {
                debug_string("waiting for WiFi to connect...", true);
                delay(1000);
            }
        }
    }
    return res;
}

/********************************
 * connect_to_stored_wifi(int)
 * Connect to a wifi based on ID
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21-07-08
 *******************************/
bool wifi_connect(void)
{
    bool res = false;

    debug_string("Start WiFi", true);

    if (deviceHasWifiCreds)
    {
        res = connect_to_stored_wifi();

        if (res == false)
        {
            res = new_wifi_password();
        }
    }
    else
    {
        res = new_wifi_password();
    }

    return res;
}