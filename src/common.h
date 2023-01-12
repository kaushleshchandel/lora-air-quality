#ifndef COMMON_H /* include guards */
#define COMMON_H


// #include <Arduino.h>

// #include "mqtt.h"


// extern bool debugMode;


// extern PubSubClient mqttclient; 

// /******************************************
//  * debug_string(string,bool)
//  * Prints the message to Serial port and sends to MQTT server when device in debug mode
//  * Author : Kaushlesh Chandel
//  * Last Modified : Build 21-07-08
//  *******************************************/
// void debug_string(String msg, bool debug_msg = false)
// {
//     if (debug_msg == true)
//     {
//         Serial.print("##");
//         Serial.println(msg);
//     }
//     else
//     {
//         Serial.print(">>");
//         Serial.println(msg);
//     }

//     if (debugMode == true && mqttclient.connected())
//     {
//         send_mqtt_string("DEBUG", msg, false);
//     }
// }


// /******************************************
//  * debug_string(string,bool)
//  * Prints the message to Serial port and sends to MQTT server when device in debug mode
//  * Author : Kaushlesh Chandel
//  * Last Modified : Build 21-07-08
//  *******************************************/
// void debug_string(String msg)
// {
//     Serial.print(">>");
//     Serial.println(msg);
// }





#endif