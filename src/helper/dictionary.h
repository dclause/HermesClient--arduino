#ifndef HERMESCLIENT_COMMAND_CODE_H
#define HERMESCLIENT_COMMAND_CODE_H

#include <Arduino.h>


/**
 * @todo export this enum to a single 'knowledge dictionary' file and create a code generator to make it.
 * @see backend/hermes/core/dictionary.py
 * @see arduino/helpers/dictionary.h
 */

/**
 * Defines the byte codes of semantic messages (commands, devices, etc...) that can be received/emitted.
 *
 * @note
 * Command codes are mapped to actual commands via the CommandFactory by registering the command class to the factory
 * keyed by the appropriate command code. This is done via the conjunction usage of the macros `COMMAND_DECLARATION`
 * and `REGISTER_COMMAND(N, T)`.
 * @see CommandFactory.h
 * In the same way, Device codes are mapped to actual devices via the DeviceFactory by registering the device class
 * to the factory keyed by the appropriate device code. This is done via the conjunction usage of the macros
 * `DEVICE_DECLARATION` and `REGISTER_DEVICE(N, T)`.
 * @see DeviceFactory.h
 *
 * @details
 * Each message must cast to an 8bits integer, therefore at most 255 messages can be semantically interpreted.
 * Message codes are (tried to) grouped by logical packages but the numbers are assigned as development go in no particular
 * order.
 *
 * @attention
 * Every number from 0 to 255 can be used for exchanges.
 *
 * A few numbers can be generated as "noise" in the serial pipe and should be thus avoided.
 * These noises mainly happens when debugging the arduino code from the monitor. That is the reason they are
 * for specific purposes:
 *  - 0 is ASCII [NULL] char: it is sent by Arduino IDE monitor when baudrate is changed
 *  - 10 is ASCII [EndOfLine] char: it is sent by Arduino IDE monitor on each data sent.
 *  - 35 is ASCII # char: is used to send debug data that should be ignored.*
 *
 * @todo implement the following
 * The values in this file must match with the values in the MessageCode.h file in the backend project. A script is
 * provided in order to help to main the files in sync.
 * (@see messagecode.py in the scripts folder on the root mono-repo)
 */
enum class MessageCode : uint8_t {

    // //////////
    // Reserved
    END_OF_LINE = 10,  // '\n' Reserved @see attention point above

    // /////////
    // COMMANDS
    VOID = 0,       // Reserved @see attention point above
    DEBUG = 35,     // '#' Reserved @see arduino folder ioserial.h
    ACK = 33,       // '!'
    HANDSHAKE = 63, // '?'
    SETTINGS = 36,  // '$'
    ACTION = 62,    // '>'

    // //////////
    // DEVICES
    DIGITAL_WRITE = 87, // 'W'
    SERVO = 42,

    // //////////
    BOOLEAN_INPUT = 141,
};

typedef enum MessageCode MessageCode;

#endif // HERMESCLIENT_COMMAND_CODE_H
