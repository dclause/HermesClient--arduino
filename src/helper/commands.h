#ifndef HERMESCLIENT_COMMANDS_H
#define HERMESCLIENT_COMMANDS_H

#include "../protocols/io.h"
#include "map.h"
#include "dictionary.h"
#include "../commands/AbstractCommand.h"
#include "../commands/CommandFactory.h"

// ! All command includes must be listed here.
#include "../commands/HandshakeCommand.h"
#include "../commands/SettingsCommand.h"
#include "../commands/ActionCommand.h"
#include "../commands/VoidCommand.h"

namespace Commands {

    /**
     * Read the next command in the IO buffer and process it.
     * Once done, an ACK is sent.
     */
    void receive_and_process_next_command() {
        if (IO::parsePacket() > 0) {
//            IO::blink(3);
            // Read incoming byte: this represents an order.
            MessageCode code = IO::read_command();
//            IO::blink(3);
            // Make a command out of it.
            AbstractCommand *command = CommandFactory::getInstance().createCommand(code);
            if (command == NULL) {
                return;
            }

            // @todo support multi-line trace.
            // TRACE(*command);

            // Execute the command
            command->process();

            IO::send_command(MessageCode::ACK);
        }
    }

}  // namespace Commands

#endif // HERMESCLIENT_COMMANDS_H
