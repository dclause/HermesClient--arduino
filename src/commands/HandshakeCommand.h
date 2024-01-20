#ifndef HERMESCLIENT_HANDSHAKE_COMMAND_H
#define HERMESCLIENT_HANDSHAKE_COMMAND_H

#include <Arduino.h>
#include "../helper/debugger.h"
#include "../helper/dictionary.h"
#include "AbstractCommand.h"
#include "CommandFactory.h"
#include "../protocols/io.h"

/**
 * HANDSHAKE Command: performs the HANDSHAKE sequence.
 *
 * This sequence is done on the initiative of the remote master and must complete to be considered done.
 *
 * @see MessageCode::HANDSHAKE
 */
class HandshakeCommand : public AbstractCommand {
    COMMAND_DECLARATION
    public:

        HandshakeCommand() : AbstractCommand(1) {}

        String getName() const { return "Handshake"; }

        void executePayload(uint8_t *payload) {
            TRACE("-----------");
            TRACE("Process HANDSHAKE command:");

            for (uint8_t i = 0; i < payload[0]; ++i) {
                TRACE("Handshake: waiting for a SETTINGS now.");
                if (IO::wait_for_bytes(1)) {
                    MessageCode code = IO::read_command();
                    if (code != MessageCode::SETTINGS) {
                        TRACE("ERROR in received code: " + String((uint8_t) code));
                        return;
                    }
                    AbstractCommand *command = CommandFactory::getInstance().createCommand(code);
                    command->process();
                }
            }

            TRACE("Handshake: done");
        }
};

REGISTER_COMMAND(MessageCode::HANDSHAKE, HandshakeCommand)

#endif // HERMESCLIENT_HANDSHAKE_COMMAND_H
