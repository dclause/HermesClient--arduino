#ifndef HERMESCLIENT_COMMAND_FACTORY_H
#define HERMESCLIENT_COMMAND_FACTORY_H

#include "../helper/debugger.h"
#include "../helper/map.h"
#include "../helper/dictionary.h"
#include "AbstractCommand.h"

#define COMMAND_DECLARATION protected: \
    static AbstractCommand* getInstance(); \
    static bool isRegistered;
#define GET_COMMAND_INSTANCE(T) AbstractCommand* T::getInstance() { return new T(); }
#define REGISTER_COMMAND(N, T) bool T::isRegistered = CommandFactory::getInstance().registerCommand(N, &T::getInstance);GET_COMMAND_INSTANCE(T);

// Callback for instantiating "on the fly" a command.
using CommandInstance = AbstractCommand *(*)();

/**
 * Command Factory: registers command and instantiates it as necessary from a given command code.
 *
 * @note
 * Command codes are mapped to actual commands via the CommandFactory by registering the command class to the factory
 * keyed by the appropriate command code. This is done via the conjunction usage of the macros `COMMAND_DECLARATION`
 * and `REGISTER_COMMAND(N, T)`.
 */
class CommandFactory {
    private:
        CommandFactory() = default;

        KeyValueMap<MessageCode, CommandInstance> registeredCommands_;

    public:
        CommandFactory(const CommandFactory &) = delete;

        CommandFactory &operator=(const CommandFactory &) = delete;

        static CommandFactory &getInstance() {
            static CommandFactory instance;
            return instance;
        }

        /**
         * Let a MessageCode be associated to an instantiable callback.
         * Stores this association in the internal map.
         *
         * @param code (MessageCode)
         * @param callback (CommandInstance) @see `using` statement at the start of file.
         * @return bool: If the command is properly registered.
         */
        bool registerCommand(MessageCode code, CommandInstance callback) {
            return this->registeredCommands_.add(code, callback);
        }

        /**
         * Instantiates an AbstractCommand of the proper type given a MessageCode.
         *
         * @note
         * A proper command is always instantiated. If the MessageCode is unknown or if something goes wrong, the
         * VOID command is returned.
         * @see VoidCommand
         *
         * @param code (MessageCode)
         * @return AbstractCommand: The instantiated command class.
         */
        AbstractCommand *createCommand(MessageCode code) {
            CommandInstance command = this->registeredCommands_.getValue(code);
            if (command == NULL) { return NULL; }
            return command();
        }

        /**
         * Stringifies the data for debug purpose.
         *
         * @return String
         */
        operator String() {
            String log = "Command Factory " + String(this->registeredCommands_.count()) + ":\n";
            for (uint8_t i = 0; i < this->registeredCommands_.count(); i++) {
                AbstractCommand *command = this->registeredCommands_.get(i)->value();
                log += "# - " + String(*command) + "\n";
            }
            return log;
        }
};

#endif // HERMESCLIENT_COMMAND_FACTORY_H