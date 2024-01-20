/*****************************************************************************
*                              SERIAL PORT                                   *
******************************************************************************
*
* To be used with a USB cable connection: uncomment the following to use it. */
#define USE_SERIAL_PROTOCOL
#define BAUDRATE 9600

/*****************************************************************************
 *                               ETHERNET                                    *
 *****************************************************************************
 * To be used with an ethernet shield: uncomment the following to use it.    *
 *                                                                           *
 * - You are using an ethernet shield that is NOT supported:                 *
 *      => open an issue at https://github.com/dclause/hermes/issues         *
 * - You are using a supported shield:                                       *
 *      => force the SHIELD value below if unrecognized properly             *
 *      => force the CS_PIN if unrecognized properly                         *
 *              > 10: for most arduino shields                               *
 *              > 5: MKR ETH Shield                                          *
 *              > 0: Teensy 2.0                                              *
 *              > 20 Teensy++ 2.0                                            *
 *              > 15: ESP8266 with Adafruit FeatherWing Ethernet             *
 *              > 33: ESP32 with Adafruit FeatherWing Ethernet               *
 * - PORT is already in use:                                                 *
 *      => change the PORT variable below.                                   *
 * - IP is already used (or you have two boards connected via ethernet):     *
 *      => assign each of them a different a different IP.                   */
//  #define USE_ETHERNET_PROTOCOL
//  #define IP 192, 168, 1, 20
//  #define PORT 5000
//  #define MAC { (byte)random(255), (byte)random(255), (byte)random(255), (byte)random(255), (byte)random(255), (byte)random(255) }
//  #define CS_PIN 10
//  #define USE_ENC28J60     // Uncomment only for ENC28J60 type shields (most likely when using arduino NANO)

/*****************************************************************************
 *                                 WIFI                                      *
 *****************************************************************************
 * To be used from an ESP32, ESP8266, nodeMCU.                               */
// #define USE_WIFI_PROTOCOL

#include <Arduino.h>
#include <HermesClient.h>

void setup() {


#if ACTIVATE_DEBUG && FORCE_SERIAL_DEBUG && !defined(USE_SERIAL_PROTOCOL)
    Serial.begin(9600);
//    IO::blink(5)
#endif

    IO::begin();
    IO::clear();
}

void loop() {
    Commands::receive_and_process_next_command();
    Devices::update_all_devices();
}
