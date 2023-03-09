
/********************* Lora Heltec ***********************/

#include <Arduino.h>
#include "config.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
// #include <U8x8lib.h>

#include <Wire.h>

// Define a task handle and initialize it to NULL
TaskHandle_t task_handle = NULL;



// static uint8_t mydata[300] = "Hi from Sagar!";
static uint8_t mydata[] = "Hi from Sagar!";

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, 34}}; //{26, 33, 32}

// LED and Button Pins
int buttonPin = 0;
int ledPin = 25; // 12
int boardLED = 25;
int lastState = 0;

bool lora_data_sent = true; // true for first to exceute


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}


void init_lora(void)
{
    // LMIC init &RESET
    os_init();
    LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
    // LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    // LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    // LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // Adaptive Data Rate Mode https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate.html
    // LMIC_setAdrMode(1); //Adaptiert Datenrate nach 64 Paketen

    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    // LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30
    // In my place we are not using 868 MHz
    // LMIC_disableChannel(0);
    // LMIC_disableChannel(1);
    // LMIC_disableChannel(2);
}


void lora_loop()
{
    os_runloop_once();
}


void ledFLash(int flashes)
{
    int lastStateLED = digitalRead(ledPin);
    for (int i = 0; i < flashes; i++)
    {
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
        delay(300);
    }
    digitalWrite(ledPin, lastStateLED);
}

void onEvent(ev_t ev)
{
    // Serial.println(ev);
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        ledFLash(2);
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // u8x8.drawString(0, 2, "Data Sent");
        // u8x8.drawString(0, 4, "Button Released");

        lora_data_sent = true;
        if (LMIC.dataLen)
        {
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));
            //   u8x8.drawString(0, 3, "Data Received: ");
            Serial.print(LMIC.dataLen);
            Serial.print(F(" bytes for downlink: 0x"));
            for (int i = 0; i < LMIC.dataLen; i++)
            {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
                {
                    Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
        }
        // Schedule next transmission
        // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(10), do_send);
        delay(2000);
        // u8x8.clearLine(1);
        // u8x8.clearLine(2);
        // u8x8.clearLine(3);
        // u8x8.clearLine(4);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: No JoinAccept"));
        break;

    default:
        Serial.println(F("Unknown event"));
        // u8x8.drawString(0, 2, "Unknown event");
        break;
    }
}


// void do_send(osjob_t *j)
// {
//   // Check if there is not a current TX/RX job running
// //   u8x8.drawString(0, 4, "Button Pressed");
//   Serial.println(LMIC.opmode);
//   if (LMIC.opmode & OP_TXRXPEND)
//   {
//     Serial.println(F("OP_TXRXPEND, not sending"));
//     // u8x8.drawString(0, 1, "OP_TXRXPEND, not sending");
//   }
//   else
//   {
//     Serial.println(F("Packet sending"));
//     // Prepare upstream data transmission at the next possible time.
//     LMIC_setTxData2(2, mydata, sizeof(mydata) - 1, 0);
//     Serial.println(F("Packet queued"));
//     // u8x8.drawString(0, 1, "Packet queued");
//   }
//   // Next TX is scheduled after TX_COMPLETE event.
// }


void do_send(osjob_t *j , char * data, int sizebytes)
{
    // Check if there is not a current TX/RX job running
    //   u8x8.drawString(0, 4, "Button Pressed");
    // Serial.println(LMIC.opmode);
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
        // u8x8.drawString(0, 1, "OP_TXRXPEND, not sending");
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        void *str = &mydata;
        memcpy(mydata, data, sizebytes);

        // LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        LMIC_setTxData2(1, mydata, sizebytes, 1);
        Serial.println(F("Packet queued"));
        Serial.println(F(str));
        // u8x8.drawString(0, 1, "Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void send_data_over_lora(char * L_dta , int bytes)
{
    do_send(&sendjob , L_dta , bytes);
}

/*********************************************************/