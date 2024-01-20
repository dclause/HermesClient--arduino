#ifndef HERMESCLIENT_IO_ETHERNET_H
#define HERMESCLIENT_IO_ETHERNET_H

/**
 * ETHERNET protocol for arduino-to-server communication.
 *
 * IO files aim to provide a unified interface to read/write operations. This allows
 * the protocol to be transparently switched from the main .ino code.
 */

#ifdef USE_ETHERNET_PROTOCOL

#include "../helper/debugger.h"
#include "../helper/dictionary.h"

#include <SPI.h>
#ifdef USE_ENC28J60
#include <EthernetENC.h>
#else
#include <Ethernet.h>
#endif


namespace IO {
    EthernetUDP udp; // Create a udp Object

    /**
     * Debugs the data via the protocol.
     *
     * @details
     * All debug strings must be wrapped as follow:
     *  - DEBUG byte (@see Order.h) to indicate the other hand of the communication to avoid this.
     *  - a [SPACE] byte (value 32)
     *  - an arbitrarily long string
     *  - a [EndOfLine] byte (value 10)
     *
     * For instance if debug param info is "Hello World", the sent data are : "# Hello World"
     * # is the byte 32 registered as the DEBUG command which will be received and interpreted both
     * on server and arduino side as pure DEBUG.
     *
     * @param info
     */
    void debug(const String &info) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print("# " + info);
        udp.endPacket();
    }

    /**
     * Initializes the communication protocol.
     */
    void begin() {
        pinMode(9, OUTPUT);
        Ethernet.init(CS_PIN);
        byte mac[] = MAC;
        IPAddress ip(IP);
        Ethernet.begin(mac, ip);
        udp.begin(PORT)
        TRACE("Opening Ethernet IO communication at baudrate " + String(ip));
    }

    /**
     * Clears the receive buffer.
     *
     * This operation is done by reading at everything currently available on the receive buffer, hence, may never end
     * if the sender is spamming at that moment.
     */
    void clear() {
        while (udp.available() > 0) {
            udp.read();
        }
    }

    /**
     * Start processing the next available incoming packet.
     * Note: Because of the nature of udp, this will also discard all existing
     * data in the current packet and start using the new one.
     *
     * @return
     *      Returns the size of the packet in bytes, or 0 if no packets are available
     */
    uint8_t parsePacket() {
        uint8_t truc = udp.parsePacket();
        return truc;
    }

    /**
     * Returns the available bytes in the current packet.
     *
     * @return uint8_t: the size of the remaining bytes to read in the current packet.
     */
    uint8_t available() {
        return udp.available();
    }

    /**
     * Waits for incoming given amount of bytes, or exit if timeout.
     *
     * Note1: This is the number of bytes that is still to be read in
     * the current packet. This is because we do not read all data at once.
     * Note2: The signature of this method is not really meaningfull as of
     * udp socket because this can be obtained in o(1): either we have
     * length-bytes remaning in the current packet or not.
     *
     * @param num_bytes (uint8_t): The number of bytes to wait for.
     * @param timeout (uint32_t): The timeout (in milliseconds) for the whole operation.
     *
     * @return bool: true/false if the number of bytes available matches the expectency.
     */
    bool wait_for_bytes(const uint32_t length, const uint32_t timeout = 100) {
        return (udp.available() >= length);
    }

    /**
     * Receives and casts an 8bit word to a MessageCode.
     *
     * @return MessageCode
     */
    MessageCode read_command() {
        const MessageCode code = static_cast<MessageCode>(udp.read());
        TRACE("Command code received: " + String((uint8_t) code));
        return code;
    }

    /**
     * Reads a given amount of bytes.
     *
     * @note  Because it has no timeout, this method should always be used in conjunction with wait_for_bytes() to ensure
     * the given amount of bytes to read is available.
     *
     * @param buffer (uint8_t*) A buffer array of uint8_t where the result will be stored.
     * @param length (uint8_t) The number of bytes to read.
     */
    void read_bytes(uint8_t *buffer, const uint8_t length) {
        udp.read(buffer, length);
    }

    /**
     * Sends an 8bit word MessageCode.
     *
     * @param command (MessageCode)
     */
    void send_command(const MessageCode command) {
        TRACE("Send command: " + String((uint8_t) command));
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(static_cast<uint8_t>(command));
        udp.endPacket();
    }

    /**
     * Sends the given amount of bytes.
     *
     * @param buffer (uint8_t*) A buffer array of uint8_t of data to send.
     * @param length (uint8_t) The number of bytes to write.
     */
    void send_bytes(const uint8_t *buffer, const uint8_t length) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(buffer, length);
        udp.endPacket();
    }

} // namespace IO

#endif // USE_ETHERNET_PROTOCOL
#endif // HERMESCLIENT_IO_ETHERNET_H