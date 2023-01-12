#ifndef MQTT_H /* include guards */
#define MQTT_H

#include <Arduino.h>
#include <PubSubClient.h>

String get_beacon_id();
String getFullTopic(String topic);
bool send_mqtt_raw_topic(String topic, String value, bool retain);
bool send_mqtt_string(String topic, String value, bool retain);
bool send_mqtt_int(String topic, long value, bool retain);
bool send_mqtt_float(String topic, double value, bool retain);
void send_setup_config();
void send_device_config();
void send_network_config();
bool process_command(String command, String argument, bool saveConfig, bool isSerial = false);
void callback(char *topic, byte *message, unsigned int length);
void callback_json(char *topic, byte *payload, unsigned int length);
void subscribe_to_topic(String topic);
bool init_mqtt();
void mqtt_connect();
void sendDiagnosticsData(PubSubClient &client);
void sendPing(PubSubClient &client);
void mqtt_debug(PubSubClient &client, String msg);
void process_cli();
void sendConfig();

#endif