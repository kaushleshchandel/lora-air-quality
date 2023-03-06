## Process of pairing new sensor.

### Wifi based setup 
Does not require physical presense
1. Power on device 
2. Device automatically connects to Secure Wifi 
3. Connects to the Central MQTT ( Registration process ) Server
4. From the device/app, an MQTT Message is sent to the device 
5. Device stores the ID / APP ID... etc. to EEPROM
6. Reboots to pair with Lora using the IDs.

### BLE based setup 
No internet available to device

1. 
