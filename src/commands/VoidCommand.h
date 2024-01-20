#ifndef HERMESCLIENT_VOID_COMMAND_H
#define HERMESCLIENT_VOID_COMMAND_H

#include "../helper/dictionary.h"
#include "AbstractCommand.h"
#include "CommandFactory.h"

/**
 * VOID Command: does nothing !
 *
 * It's purpose is to let any none attributed MessageCode be mapped to a real command (this one) by the factory.
 * This ensures that factory always returns a real command that can be processed (and do nothing here).
 *
 * @see MessageCode::VOID
 */
class VoidCommand : public AbstractCommand {
    COMMAND_DECLARATION
    public:

        VoidCommand() : AbstractCommand(0) {}

        String getName() const { return "VOID"; }

        void executePayload(uint8_t *payload) {}
};

REGISTER_COMMAND(MessageCode::VOID, VoidCommand)

#endif // HERMESCLIENT_VOID_COMMAND_H
