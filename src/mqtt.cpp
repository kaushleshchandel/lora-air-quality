// #ifndef MQTT_H /* include guards */
// #define MQTT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <config.h>
#include <ArduinoJson.h>

#include "types.h"
#include "ota.h"

#include "common.h"

// Counters for diagnositcs
unsigned long packetsSentPM = 0; // MQTT packets sent
unsigned long packetsFailPM = 0; // MQTT packet sent failure
unsigned long dataLoopsPM = 0;   // Loops with data
unsigned long loopsPM = 0;       // Counter to track executions
unsigned long lastpacketsSentPM = 1;
unsigned long wifiErrors = 0;       // Wifi Connectivity Errors
unsigned long mqttErrors = 0;       // MQTT Error
unsigned long sensorReadErrors = 0; // Sensor read errors

extern bool debugMode;
extern bool wifi_Online;


extern const char *primary_mqtt_user;
extern const char *primary_mqtt_pass;
extern const char *secondary_mqtt_user;
extern const char *secondary_mqtt_pass;


extern data_config WM_config;

// extern WiFiClient client;
extern WiFiClientSecure client;
extern PubSubClient mqttclient;


long lastMsg = 0;
char msg[DEFAULT_MESSAGE_LENGTH];
int value = 0;

char will_topic[56];
char subs_topic[56];
char topic_prefix[56];

char mqtt_send_buffer[256];

void sendConfig();

const char *ca_cert PROGMEM =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIE0DCCA7igAwIBAgIBBzANBgkqhkiG9w0BAQsFADCBgzELMAkGA1UEBhMCVVMx\n"
    "EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxGjAYBgNVBAoT\n"
    "EUdvRGFkZHkuY29tLCBJbmMuMTEwLwYDVQQDEyhHbyBEYWRkeSBSb290IENlcnRp\n"
    "ZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTExMDUwMzA3MDAwMFoXDTMxMDUwMzA3\n"
    "MDAwMFowgbQxCzAJBgNVBAYTAlVTMRAwDgYDVQQIEwdBcml6b25hMRMwEQYDVQQH\n"
    "EwpTY290dHNkYWxlMRowGAYDVQQKExFHb0RhZGR5LmNvbSwgSW5jLjEtMCsGA1UE\n"
    "CxMkaHR0cDovL2NlcnRzLmdvZGFkZHkuY29tL3JlcG9zaXRvcnkvMTMwMQYDVQQD\n"
    "EypHbyBEYWRkeSBTZWN1cmUgQ2VydGlmaWNhdGUgQXV0aG9yaXR5IC0gRzIwggEi\n"
    "MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQC54MsQ1K92vdSTYuswZLiBCGzD\n"
    "BNliF44v/z5lz4/OYuY8UhzaFkVLVat4a2ODYpDOD2lsmcgaFItMzEUz6ojcnqOv\n"
    "K/6AYZ15V8TPLvQ/MDxdR/yaFrzDN5ZBUY4RS1T4KL7QjL7wMDge87Am+GZHY23e\n"
    "cSZHjzhHU9FGHbTj3ADqRay9vHHZqm8A29vNMDp5T19MR/gd71vCxJ1gO7GyQ5HY\n"
    "pDNO6rPWJ0+tJYqlxvTV0KaudAVkV4i1RFXULSo6Pvi4vekyCgKUZMQWOlDxSq7n\n"
    "eTOvDCAHf+jfBDnCaQJsY1L6d8EbyHSHyLmTGFBUNUtpTrw700kuH9zB0lL7AgMB\n"
    "AAGjggEaMIIBFjAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBBjAdBgNV\n"
    "HQ4EFgQUQMK9J47MNIMwojPX+2yz8LQsgM4wHwYDVR0jBBgwFoAUOpqFBxBnKLbv\n"
    "9r0FQW4gwZTaD94wNAYIKwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8v\n"
    "b2NzcC5nb2RhZGR5LmNvbS8wNQYDVR0fBC4wLDAqoCigJoYkaHR0cDovL2NybC5n\n"
    "b2RhZGR5LmNvbS9nZHJvb3QtZzIuY3JsMEYGA1UdIAQ/MD0wOwYEVR0gADAzMDEG\n"
    "CCsGAQUFBwIBFiVodHRwczovL2NlcnRzLmdvZGFkZHkuY29tL3JlcG9zaXRvcnkv\n"
    "MA0GCSqGSIb3DQEBCwUAA4IBAQAIfmyTEMg4uJapkEv/oV9PBO9sPpyIBslQj6Zz\n"
    "91cxG7685C/b+LrTW+C05+Z5Yg4MotdqY3MxtfWoSKQ7CC2iXZDXtHwlTxFWMMS2\n"
    "RJ17LJ3lXubvDGGqv+QqG+6EnriDfcFDzkSnE3ANkR/0yBOtg2DZ2HKocyQetawi\n"
    "DsoXiWJYRBuriSUBAA/NxBti21G00w9RKpv0vHP8ds42pM3Z2Czqrpv1KrKQ0U11\n"
    "GIo/ikGQI31bS/6kA1ibRrLDYGCD+H1QQc7CoZDDu+8CL9IVVO5EFdkKrqeKM+2x\n"
    "LXY2JtwE65/3YR8V3Idv7kaWKK2hJn0KCacuBKONvPi8BDAB\n"
    "-----END CERTIFICATE-----\n";

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
String get_beacon_id()
{
    String bid = WiFi.macAddress();
    bid.replace(":", "");
    return bid;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
String getFullTopic(String topic)
{
    String t = "";
    String mac_address = get_beacon_id();
    String prefix = WM_config.device_config.topic_prefix;
    prefix.trim();

    if (prefix == "")
    {
        if (topic == "")
            t = mac_address;
        else
            t = mac_address + "/" + topic;
    }
    else
    {
        if (topic == "")
            t = prefix + "/" + mac_address;
        else
            t = prefix + "/" + mac_address + "/" + topic;
    }

    return t;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool send_mqtt_raw_topic(String topic, String value, bool retain)
{
    bool res = false;
    if (client.connected())
    {
        char tempStringTopic[50];
        char tempStringvalue[20];
        topic.toCharArray(tempStringTopic, topic.length() + 1);
        value.toCharArray(tempStringvalue, value.length() + 1);
        res = mqttclient.publish(tempStringTopic, tempStringvalue, retain);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string("MQTT Not connected", true);
    return res;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool send_mqtt_string(String topic, String value, bool retain)
{
    bool res = false;
    if (wifi_Online && client.connected())
    {
        topic = getFullTopic(topic);
        char tempStringTopic[50];
        char tempStringvalue[256];
        topic.toCharArray(tempStringTopic, topic.length() + 1);
        value.toCharArray(tempStringvalue, value.length() + 1);
        res = mqttclient.publish(tempStringTopic, tempStringvalue, retain);

        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);

    return res;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool send_mqtt_int(String topic, long value, bool retain)
{

    bool res = false;

    if (mqttclient.connected())
    {
        topic = getFullTopic(topic);
        char tempStringTopic[50];
        char tempStringvalue[20];

        topic.toCharArray(tempStringTopic, topic.length() + 1);
        dtostrf(value, 1, 0, tempStringvalue);
        res = mqttclient.publish(tempStringTopic, tempStringvalue);

        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);

    return res;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool send_mqtt_float(String topic, double value, bool retain)
{
    bool res = false;
    if (mqttclient.connected())
    {
        topic = getFullTopic(topic);
        char tempStringTopic[50];
        char tempStringvalue[20];
        topic.toCharArray(tempStringTopic, topic.length() + 1);
        dtostrf(value, 1, 2, tempStringvalue);
        res = mqttclient.publish(tempStringTopic, tempStringvalue, retain);

        if (res != true)
        {
            debug_string(" MQTT Send Failure", true);
            packetsFailPM++;
        }
        else
        {
            packetsSentPM++;
        }
    }
    else

        debug_string(" MQTT Not connected", true);

    return res;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void send_setup_config()
{

    StaticJsonDocument<256> doc;
    doc["prefix"] = WM_config.device_config.topic_prefix;
    doc["hw"] = HW_VERSION;
    doc["sw"] = SW_VERSION;
    doc["code"] = WM_config.device_config.device_code;
    doc["setup"] = WM_config.device_config.setupMode;

    char buffer[256];
    serializeJson(doc, buffer);

    if (mqttclient.connected())
    {
        String topic = getFullTopic("");
        char tempStringTopic[50];

        topic.toCharArray(tempStringTopic, topic.length() + 1);

        bool res = mqttclient.publish(tempStringTopic, buffer);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);

    if (mqttclient.connected() && WM_config.device_config.setupMode == 1)
    {
        String topic = "SETUP/" + get_beacon_id();
        char tempStringTopic[50];

        topic.toCharArray(tempStringTopic, topic.length() + 1);

        bool res = mqttclient.publish(tempStringTopic, buffer);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void send_device_config()
{
    StaticJsonDocument<256> doc;

    String topicPrefix = WM_config.device_config.topic_prefix;
    String deviceCode = WM_config.device_config.device_code;

    doc["dataFrequency"] = WM_config.device_config.data_frequency;
    doc["timeZone"] = WM_config.device_config.timeZone;

    char buffer[256];
    serializeJson(doc, buffer);

    if (mqttclient.connected())
    {
        String topic = getFullTopic("scanner");
        char tempStringTopic[50];

        topic.toCharArray(tempStringTopic, topic.length() + 1);

        bool res = mqttclient.publish(tempStringTopic, buffer);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void send_network_config()
{

    StaticJsonDocument<256> doc;
    char buffer[256];

    String topicPrefix = WM_config.device_config.topic_prefix;
    String deviceCode = WM_config.device_config.device_code;

    doc["ssid1"] = WM_config.WiFi_Creds.wifi_ssid;
    doc["usestatic1"] = String(WM_config.WiFi_Creds.useStaticIP);
    doc["static1"] = WM_config.WiFi_Creds.staticIP.toString();
    doc["subnet1"] = WM_config.WiFi_Creds.subnetIP.toString();
    doc["gateway1"] = WM_config.WiFi_Creds.gatewayIP.toString();

    // doc["ssid2"] = WM_config.WiFi_Creds[2].wifi_ssid;
    // doc["usestatic2"] = String(WM_config.WiFi_Creds[2].useStaticIP);
    // doc["static2"] = WM_config.WiFi_Creds[2].staticIP.toString();
    // doc["subnet2"] = WM_config.WiFi_Creds[2].subnetIP.toString();
    // doc["gateway2"] = WM_config.WiFi_Creds[2].gatewayIP.toString();

    // doc["ssid0"] = WM_config.WiFi_Creds[0].wifi_ssid;

    serializeJson(doc, buffer);

    if (mqttclient.connected())
    {
        String topic = getFullTopic("network");
        char tempStringTopic[50];
        topic.toCharArray(tempStringTopic, topic.length() + 1);
        bool res = mqttclient.publish(tempStringTopic, buffer);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string("MQTT Not connected", true);
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool process_command(String command, String argument, bool saveConfig, bool isSerial = false)
{
    command.trim();
    argument.trim();

    debug_string(command + " > " + argument);

    bool iscommandvalid = true;

    if (command == "prefix")
    {
        debug_string("Set topic prefix", true);
        strcpy(WM_config.device_config.topic_prefix, argument.c_str());
    }
    else if (command == "caltempa")
    {
        WM_config.device_config.calibrate_temperature_a = argument.toInt();
    }
    else if (command == "caltempb")
    {
        WM_config.device_config.calibrate_temperature_b = argument.toInt();
    }
    else if (command == "calhumia")
    {
        WM_config.device_config.calibrate_humidity_a = argument.toInt();
    }
    else if (command == "calhumib")
    {
        WM_config.device_config.calibrate_humidity_b = argument.toInt();
    }

    /*else if (command == "usestatic1")
    {
        WM_config.WiFi_Creds[1].useStaticIP = argument.toInt();
    }
    else if (command == "static1")
    {

        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[1].staticIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }
    else if (command == "gateway1")
    {
        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[1].gatewayIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }
    else if (command == "subnet1")
    {
        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[1].subnetIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }
    else if (command == "usestatic2")
    {
        WM_config.WiFi_Creds[2].useStaticIP = argument.toInt();
    }
    else if (command == "static2")
    {
        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[2].staticIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }
    else if (command == "gateway2")
    {
        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[2].gatewayIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }
    else if (command == "subnet2")
    {
        IPAddress local_IP;
        if (local_IP.fromString(argument))
            WM_config.WiFi_Creds[2].subnetIP = local_IP;
        else
            debug_string("Invalid IP", true);
    }*/

    else if (command == "dataFrequency")
    {
        WM_config.device_config.data_frequency = argument.toInt();
    }
    /*else if (command == "calibrateRssi")
    {
        WM_config.device_config.calibrateRssi = argument.toInt();
    }*/

    // DO AN OTA BASED ON PASSED FIRMWARE
    else if (command == "ota")
    {
        updateFirmware(argument);
    }

    else if (command == "restart" || command == "reboot")
    {
        debug_string("Rebooting by mqtt", true);
        ESP.restart();
    }

    else if (command == "timeZone")
    {
        WM_config.device_config.timeZone = argument.toInt();
    }

    // else if (command == "setup")
    // {

    //     WM_config.device_config.setupMode = argument.toInt();
    // }
    else if (command == "setDefaults")
    {
        debug_string("Setting defaults", true);
        set_defaults();
    }
    else if (command == "factoryReset")
    {
        debug_string("Factory Reset", true);
        // set_defaults();
        // save_config_file();
        format_config_file();
        WiFi.disconnect();
        esp_restart();
    }
    else if (command == "debug")
    {
        debug_string("Switch Debug Mode", true);
        debugMode = argument.toInt();
    }
    else if (command == "save")
    {
        save_config_file();
    }
    else if (command == "read")
    {
        read_config_file();
        sendConfig();
    }
    else if (command == "code")
    {
        String cdevice_mac = get_beacon_id();
        // if topic
        int sep = argument.indexOf(":");

        String mqttMac = argument.substring(0, sep);
        String newDeviceCode = argument.substring(sep + 1);
        //  debug_string("My MAC " + cdevice_mac + " MQTT MAC " + mqttMac, true);

        if (cdevice_mac == mqttMac)
        {
            debug_string("Updating Device Code with " + newDeviceCode, true);
            if (isSerial != false)
                send_mqtt_string("DEBUG", "Updating Device Code with " + newDeviceCode, false);
            strcpy(WM_config.device_config.device_code, newDeviceCode.c_str());
        }

        else
        {
            debug_string("This message is not for me");
            if (isSerial != false)
                send_mqtt_string("DEBUG", "Message not for me", false);
        }
    }
    /*else if (command == "ssid0")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[0].wifi_ssid, argument.c_str());
            // strcpy(WM_config.WiFi_Creds[0].wifi_pw, pass1.c_str());
        }
    }
    else if (command == "pass0")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[0].wifi_pw, argument.c_str());
        }
    }
    else if (command == "ssid1")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[1].wifi_ssid, argument.c_str());
        }
    }
    else if (command == "pass1")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[1].wifi_pw, argument.c_str());
        }
    }
    else if (command == "ssid2")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[2].wifi_ssid, argument.c_str());
        }
    }
    else if (command == "pass2")
    {

        if (argument.length() > 0)
        {
            strcpy(WM_config.WiFi_Creds[2].wifi_pw, argument.c_str());
        }
    }
    else
    {
        iscommandvalid = false;
    }*/

    if (iscommandvalid)
    {
        if (isSerial != false)
            send_mqtt_string("debug", "Command OK", false);
        else
            debug_string("Command OK", false);

        if (saveConfig)
            save_config_file();
    }
    else
    {
        if (isSerial != false)
            send_mqtt_string("debug", "Command ERR", false);
        else
            debug_string("Command ERROR", true);
    }

    return iscommandvalid;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void callback(char *topic, byte *message, unsigned int length)
{
    String messageTemp;

    for (int i = 0; i < length; i++)
    {
        messageTemp += (char)message[i];
    }

    bool isjsonCommand = false;

    DynamicJsonDocument doc(200);
    DeserializationError err = deserializeJson(doc, message);
    switch (err.code())
    {
    case DeserializationError::Ok:
        isjsonCommand = true;
        Serial.print(F("Deserialization succeeded"));
        break;
    case DeserializationError::InvalidInput:
        Serial.print(F("Invalid input!"));
        break;
    case DeserializationError::NoMemory:
        Serial.print(F("Not enough memory"));
        break;
    default:
        Serial.print(F("Deserialization failed"));
        break;
    }

    if (isjsonCommand)
    {
        debug_string("JSON Command");
        JsonObject documentRoot = doc.as<JsonObject>();

        for (JsonPair keyValue : documentRoot)
        {
            String key = keyValue.key().c_str();
            String parameter = doc[key];
            process_command(key, parameter, false);
        }
    }
    else // Normal Command.
    {
        String cmd = String(topic);
        String prefix = WM_config.device_config.topic_prefix;
        prefix.trim();

        if (prefix == "")
            cmd = cmd.substring(17);
        else
            cmd = cmd.substring(18 + prefix.length());

        debug_string("Device Command : " + cmd + "=" + messageTemp, true);
        process_command(cmd, messageTemp, false, false);
    }
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void callback_json(char *topic, byte *payload, unsigned int length)
{

    // char str[length + 1];

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    //   int i = 0;
    //   for (i = 0; i < length; i++)
    //   {
    //      Serial.print((char)payload[i]);
    //      str[i] = (char)payload[i];
    //  }
    //  str[i] = 0; // Null termination
    Serial.println();

    StaticJsonDocument<256> doc;

    deserializeJson(doc, payload);
    JsonObject documentRoot = doc.as<JsonObject>();

    for (JsonPair keyValue : documentRoot)
    {
        Serial.println(keyValue.key().c_str());
        String key = keyValue.key().c_str();
        const char *cmd = doc[key];
        Serial.println(cmd);
    }
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void subscribe_to_topic(String topic)
{
    debug_string("Subscribe to " + topic, true);
    char tempStringSubsTopic[50];
    topic.toCharArray(tempStringSubsTopic, topic.length() + 1);
    mqttclient.subscribe(tempStringSubsTopic);
}

/******************************************
 * init_mqtt()
 * Initialize the MQTT server, connect and start listening for commands
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
bool init_mqtt()
{

    bool res = false;
    String topicPrefix = WM_config.device_config.topic_prefix;

    String sbeacon_id = get_beacon_id();
    String swilltopic = topicPrefix + sbeacon_id + "/status";

    char tempStringWillTopic[80];
    char tempStringclientID[80];

    sbeacon_id.toCharArray(will_topic, sbeacon_id.length() + 1);
    sbeacon_id.toCharArray(tempStringclientID, sbeacon_id.length() + 1);
    swilltopic.toCharArray(tempStringWillTopic, swilltopic.length() + 1);

    client.setCACert(ca_cert); /* set SSL/TLS certificate */

    mqttclient.setServer(WM_config.device_config.primary_mqtt_server, WM_config.device_config.primary_mqtt_port);
    mqttclient.setCallback(callback);
    // mqttclient.setCallback(callback_json);

    debug_string(WM_config.device_config.primary_mqtt_server, true);
    debug_string(String(WM_config.device_config.primary_mqtt_port), true);
    debug_string("Connecting To ...", true);

    if (!mqttclient.connect(tempStringclientID, primary_mqtt_user, primary_mqtt_pass, tempStringWillTopic, 0, true, "offline"))
    {
        debug_string("FAILED rc =" + String(mqttclient.state()), true);
    }

    if (!mqttclient.connected())
    {
        mqttclient.setServer(WM_config.device_config.secondary_mqtt_server, WM_config.device_config.secondary_mqtt_port);
        debug_string(WM_config.device_config.secondary_mqtt_server, true);
        debug_string(String(WM_config.device_config.secondary_mqtt_port), true);
        debug_string("Connecting To ...", true);

        if (!mqttclient.connect(tempStringclientID, secondary_mqtt_user, secondary_mqtt_pass, tempStringWillTopic, 0, true, "offline"))
        {
            debug_string("FAILED rc =" + String(mqttclient.state()), true);
        }
    }

    if (mqttclient.connected())
    {
        debug_string("MQTT connected", true);
        StaticJsonDocument<256> doc;
        char buffer[256];

        // Subscribe to the commands
        String subscribeToTopic = getFullTopic(DEFAULT_SUB_TOPIC);

        subscribe_to_topic(subscribeToTopic);

        doc["sub"] = subscribeToTopic;
        doc["setup"] = WM_config.device_config.setupMode;
        doc["will"] = swilltopic;
        doc["ipaddress"] = WiFi.localIP().toString();

        serializeJson(doc, buffer);

        if (mqttclient.connected())
        {
            String topic = getFullTopic("settings");
            char tempStringTopic[50];

            topic.toCharArray(tempStringTopic, topic.length() + 1);

            bool res = mqttclient.publish(tempStringTopic, buffer);
            if (res != true)
                packetsFailPM++;
            else
                packetsSentPM++;
        }
        else
            debug_string(" MQTT Not connected", true);

        send_mqtt_string("status", "online", true);
        sendConfig();
        send_network_config();
        res = true;
    }

    return res;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void mqtt_connect()
{
    String topicPrefix = WM_config.device_config.topic_prefix;

    String sbeacon_id = get_beacon_id();
    String swilltopic = topicPrefix + sbeacon_id + "/status";

    char tempStringWillTopic[80];
    char tempStringclientID[80];

    sbeacon_id.toCharArray(will_topic, sbeacon_id.length() + 1);
    sbeacon_id.toCharArray(tempStringclientID, sbeacon_id.length() + 1);
    swilltopic.toCharArray(tempStringWillTopic, swilltopic.length() + 1);

    debug_string(WM_config.device_config.primary_mqtt_server, true);
    debug_string(String(WM_config.device_config.primary_mqtt_port), true);
    mqttclient.setServer(WM_config.device_config.primary_mqtt_server, WM_config.device_config.primary_mqtt_port);

    for (int i = 0; i < 5; i++)
    {
        debug_string("Attempting MQTT connection...", true);
        if (mqttclient.connect(tempStringclientID, primary_mqtt_user, primary_mqtt_pass, tempStringWillTopic, 0, true, "offline"))
        {
            // Subscribe to the commands
            String subscribeToTopic = getFullTopic(DEFAULT_SUB_TOPIC);
            subscribe_to_topic(subscribeToTopic);
            debug_string("Subscribe to Device commands", true);
            send_mqtt_string("status", "online", true);
            sendConfig();
            i = 5;
        }
        else
        {
            debug_string("failed, rc=" + String(mqttclient.state()), true);
            delay(2000);
        }
    }

    if (!mqttclient.connected())
    {
        debug_string(WM_config.device_config.secondary_mqtt_server, true);
        debug_string(String(WM_config.device_config.secondary_mqtt_port), true);
        mqttclient.setServer(WM_config.device_config.secondary_mqtt_server, WM_config.device_config.secondary_mqtt_port);

        for (int i = 0; i < 5; i++)
        {
            debug_string("Attempting MQTT connection...", true);
            if (mqttclient.connect(tempStringclientID, secondary_mqtt_user, secondary_mqtt_pass, tempStringWillTopic, 0, true, "offline"))
            {
                // Subscribe to the commands
                String subscribeToTopic = getFullTopic(DEFAULT_SUB_TOPIC);
                subscribe_to_topic(subscribeToTopic);
                debug_string("Subscribe to Device commands", true);
                send_mqtt_string("status", "online", true);
                sendConfig();
                i = 5;
            }
            else
            {
                debug_string("failed, rc=" + String(mqttclient.state()), true);
                delay(2000);
            }
        }
    }
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void sendDiagnosticsData(PubSubClient &client)
{
    // Update this to a Json structure
    int uptime = esp_timer_get_time() / 60000000; // Minutes

    StaticJsonDocument<256> doc;
    char buffer[256];

    doc["loop-k"] = loopsPM / 1000;
    doc["reads"] = dataLoopsPM;
    doc["mem-kb"] = ESP.getFreeHeap() / 1024;
    doc["fail"] = packetsFailPM;
    doc["wifi-err"] = wifiErrors;
    doc["mqtt-err"] = mqttErrors;
    doc["sensor-err"] = sensorReadErrors;
    doc["uptime-hrs"] = uptime / 60;
    doc["uptime-min"] = uptime;

    serializeJson(doc, buffer);

    if (mqttclient.connected())
    {
        String topic = getFullTopic("performance");
        char tempStringTopic[50];

        topic.toCharArray(tempStringTopic, topic.length() + 1);

        bool res = mqttclient.publish(tempStringTopic, buffer);
        if (res != true)
            packetsFailPM++;
        else
            packetsSentPM++;
    }
    else
        debug_string(" MQTT Not connected", true);

    loopsPM = 0;
    dataLoopsPM = 0;
    lastpacketsSentPM = packetsSentPM;
    packetsSentPM = 0;
    packetsFailPM = 0;
    mqttErrors += packetsFailPM;
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void sendPing(PubSubClient &client)
{

    send_mqtt_string("status", "online", false);
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void mqtt_debug(PubSubClient &client, String msg)
{
    send_mqtt_string("debug", "msg", false);
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void process_cli()
{
    if (Serial.available())
    {
        String commandString;
        String command;
        String arguments;

        commandString = Serial.readStringUntil('\n');

        bool isjsonCommand = false;

        DynamicJsonDocument doc(200);
        DeserializationError err = deserializeJson(doc, commandString);
        switch (err.code())
        {
        case DeserializationError::Ok:
            isjsonCommand = true;
            Serial.print(F("Deserialization succeeded"));
            break;
        case DeserializationError::InvalidInput:
            Serial.print(F("Invalid input!"));
            break;
        case DeserializationError::NoMemory:
            Serial.print(F("Not enough memory"));
            break;
        default:
            Serial.print(F("Deserialization failed"));
            break;
        }

        if (isjsonCommand)
        {
            debug_string("JSON Command");
            JsonObject documentRoot = doc.as<JsonObject>();

            for (JsonPair keyValue : documentRoot)
            {
                String key = keyValue.key().c_str();
                String parameter = doc[key];
                process_command(key, parameter, false);
            }
        }
        else // Normal Command.
        {
            int sep = commandString.indexOf(" ");

            String command = commandString.substring(0, sep);
            String arguments = commandString.substring(sep + 1);

            debug_string("Device Command : " + command + "=" + arguments, true);
            process_command(command, arguments, false, false);
        }
    }
}

/******************************************
 *
 *
 * Author : Kaushlesh Chandel
 * Last Modified :
 *******************************************/
void sendConfig()
{
    send_device_config();
    send_network_config();
    send_setup_config();
}

// #endif