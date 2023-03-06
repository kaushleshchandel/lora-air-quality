#ifndef SECURE_GPIO
#define SECURE_GPIO

#include "Arduino.h"

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

  //GPIO 22 (SCL) and GPIO 21 (SDA)

  Mic : Connected to GPIO 25, GPIO32, and GPIO33
  MIC_SCK --
  MIC_WS  --
  MIC_LR  --
  MIC_VDD --
  MIC_SD  --



*/


// Connections to I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

#define PM_RX 18
#define PM_TX 19


#endif