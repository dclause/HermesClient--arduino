#ifndef HERMESCLIENT_ACTION_COMMAND_H
#define HERMESCLIENT_ACTION_COMMAND_H

#include <Arduino.h>

#include "../helper/debugger.h"
#include "../helper/dictionary.h"
#include "AbstractCommand.h"
#include "CommandFactory.h"
#include "../devices/AbstractDevice.h"
#include "../devices/DeviceManager.h"


/**
 * ACTION Command: sets the value of a device in order to do an action. 
 * For instance, turn on a light, position a servo, etc...
 *
 * @see MessageCode::ACTION
 */
class ActionCommand : public AbstractCommand {
    COMMAND_DECLARATION

    public:

        ActionCommand() : AbstractCommand(1) {}

        String getName() const override { return "ACTION"; }

        void executePayload(uint8_t *payload) override {
            TRACE("-----------");
            TRACE("Process ACTION command:");

            AbstractDevice *device = DeviceManager::getInstance().getDevice(payload[0]);
            if (device == NULL) {
                TRACE("  > Undefined device to mutate: " + String(payload[0]));
                return;
            }

            device->process();
        }
};

REGISTER_COMMAND(MessageCode::ACTION, ActionCommand)

#endif  // HERMESCLIENT_ACTION_COMMAND_H
