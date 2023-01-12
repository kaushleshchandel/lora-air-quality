#ifndef FX_h
#define FX_h

#include "config.h"
#include "mqtt.h"
#include "types.h"
#include "common.h"

bool shouldreboot = false;


extern bool wifi_Online;
extern bool mqtt_connected;

extern bool deviceHasWifiCreds;

extern data_config WM_config;

// extern WiFiClient client;
extern PubSubClient mqttclient;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_WIFI_READY:
    debug_string("WiFi interface ready");
    break;
  case SYSTEM_EVENT_SCAN_DONE:
    debug_string("Completed scan for access points");
    break;
  case SYSTEM_EVENT_STA_START:
    debug_string("WiFi client started");
    break;
  case SYSTEM_EVENT_STA_STOP:
    debug_string("WiFi clients stopped");
    break;
  case SYSTEM_EVENT_STA_CONNECTED:
    debug_string("Connected to access point");
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    debug_string("Disconnected from WiFi access point");
    // WiFi.begin(ssid, password);
    break;
  case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    debug_string("Authentication mode of access point has changed");
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    debug_string("Obtained IP address");
    break;
  case SYSTEM_EVENT_STA_LOST_IP:
    debug_string("Lost IP address and IP address is reset to 0");
    break;
  case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
    debug_string("WiFi Protected Setup (WPS): succeeded in enrollee mode");
    break;
  case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    debug_string("WiFi Protected Setup (WPS): failed in enrollee mode");
    break;
  case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
    debug_string("WiFi Protected Setup (WPS): timeout in enrollee mode");
    break;
  case SYSTEM_EVENT_STA_WPS_ER_PIN:
    debug_string("WiFi Protected Setup (WPS): pin code in enrollee mode");
    break;
  case SYSTEM_EVENT_AP_START:
    debug_string("WiFi access point started");
    break;
  case SYSTEM_EVENT_AP_STOP:
    debug_string("WiFi access point  stopped");
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    debug_string("Client connected");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    debug_string("Client disconnected");
    break;
  case SYSTEM_EVENT_AP_STAIPASSIGNED:
    debug_string("Assigned IP address to client");
    break;
  case SYSTEM_EVENT_AP_PROBEREQRECVED:
    debug_string("Received probe request");
    break;
  case SYSTEM_EVENT_GOT_IP6:
    debug_string("IPv6 is preferred");
    break;
  case SYSTEM_EVENT_ETH_START:
    debug_string("Ethernet started");
    break;
  case SYSTEM_EVENT_ETH_STOP:
    debug_string("Ethernet stopped");
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    debug_string("Ethernet connected");
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    debug_string("Ethernet disconnected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    debug_string("Obtained IP address");
    break;
  default:
    break;
  }
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  debug_string("WiFi connected");
  debug_string("IP address: " + IPAddress(info.got_ip.ip_info.ip.addr).toString());
}

// Task to Send Diagnositcs Data every 30 seconds

void task_send_mqtt_hearbeatCallback()
{

  debug_string("Heartbeat");

  if (WM_config.device_config.setupMode == 1)
  {
    debug_string("!!! Setup Mode !!!");
  }

  if (deviceHasWifiCreds != true)
    debug_string("!!! Update WiFi !!!");

  if (String(WM_config.device_config.device_code) == "ZZZZ")
    debug_string("!!! Update Device Code !!!");

  sendPing(mqttclient);
  // display.blink(BLINK_SHORT);
}

// Check conbnectivyt every 10 minutes
void task_check_connectivityCallback()
{
  if ((WiFi.status() != WL_CONNECTED))
  {
    debug_string("WiFi Disconnected");
    // check_WiFi();
    wifi_Online = false;
  }
  else
  {
    wifi_Online = true;
    // Check MQTT Connection and reconnect as needed.
    if (mqttclient.connected() != true)
    {
      debug_string("MQTT not connected. Reconnecting");
      mqtt_connect();
    }
    else
    {
      mqtt_connected = true;
    }
  }

  if ((mqttclient.connected()) && (WiFi.status() == WL_CONNECTED))
  {
    sendDiagnosticsData(mqttclient);
    debug_string("All is Well");
  }
  else
  {
    int uptime = esp_timer_get_time() / 60000000; // Minutes
    if (uptime > 5)
    {
      debug_string("Connectivity problem. Reboot");
      esp_restart();
    }
  }

  // Resetting the reboot counter
}

void task_scheduled_rebootCallback()
{
  if (shouldreboot == false) // Variable used to avoid rebooting for the first time callback is done
  {
      shouldreboot = true;
  }
  else
  {
    debug_string("Scheduled reboot");
    esp_restart();
  }
}

#endif