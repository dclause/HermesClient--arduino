#ifndef HERMESCLIENT_SETTINGS_COMMAND_H
#define HERMESCLIENT_SETTINGS_COMMAND_H

#include <Arduino.h>

#include "../helper/debugger.h"
#include "../helper/dictionary.h"
#include "AbstractCommand.h"
#include "CommandFactory.h"
#include "../devices/AbstractDevice.h"
#include "../devices/DeviceFactory.h"
#include "../devices/DeviceManager.h"


/**
 * SETTINGS Command: creates or patches a device with the payload settings and store it in the device manager.
 *
 * @see MessageCode::SETTINGS
 */
class SettingsCommand : public AbstractCommand {
    COMMAND_DECLARATION

    public:

        SettingsCommand() : AbstractCommand(-1) {}

        String getName() const override { return "SETTINGS"; }

        void executePayload(uint8_t *payload) override {
            TRACE("-----------");
            TRACE("Process SETTINGS command:");

            // The payload received for a SETTINGS deviceToPatch always starts by the MessageCode of the deviceToPatch to patch.
            const MessageCode messageCode = static_cast<MessageCode>(payload[0]);
            TRACE("  > Received messageCode: " + String((uint8_t) messageCode));
            AbstractDevice *deviceToPatch = DeviceFactory::getInstance().createDevice(messageCode);
            if (deviceToPatch == NULL) {
                TRACE("  => SETTINGS abort: device not found.");
                return;
            }
            TRACE("  > Device to SETTINGS: " + String(*deviceToPatch));

            // The data to use is the SETTINGS payload minus the first bytes (MessageCode)
            uint8_t *data = payload + 1;

            // First check if the device is already declared.
            // NOTE: The payload for a device necessarily starts with ID at offset 1.
            TRACE("  > Search for existing device with ID: " + String(payload[1]));
            AbstractDevice *existingDevice = DeviceManager::getInstance().getDevice(payload[1]);
            if (existingDevice != NULL) {
                TRACE("  > Device needs update.");
                TRACE("  > Updated " + String(*existingDevice));
                // @todo implement the update.
            } else {
                TRACE("  > Device needs create.");
                deviceToPatch->updateSettings(data);
                DeviceManager::getInstance().addDevice(deviceToPatch);
                TRACE("  > Created " + String(*deviceToPatch));
            }
        }
};

REGISTER_COMMAND(MessageCode::SETTINGS, SettingsCommand)

#endif  // HERMESCLIENT_SETTINGS_COMMAND_H
