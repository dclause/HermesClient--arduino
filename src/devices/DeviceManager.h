#ifndef HERMESCLIENT_DEVICE_MANAGER_H
#define HERMESCLIENT_DEVICE_MANAGER_H

#include <Arduino.h>
#include "../helper/map.h"
#include "AbstractDevice.h"

/**
 * Device manager: keeps track of all devices.
 * @see AbstractDevice.h
 */
class DeviceManager {
    private:
        DeviceManager() = default;

        KeyValueMap<uint8_t, AbstractDevice *> devices_;

    public:
        DeviceManager(const DeviceManager &) = delete;

        DeviceManager &operator=(const DeviceManager &) = delete;

        static DeviceManager &getInstance() {
            static DeviceManager instance;
            return instance;
        }

        /**
         * Returns first device in chained list.
         *
         * @return KeyValuePair<uint8_t, AbstractDevice*>*
         */
        KeyValuePair<uint8_t, AbstractDevice *> *getHead() {
            return this->devices_.getHead();
        }

        /**
         * Add a device to the known list.
         *
         * @param instance AbstractDevice: a device.
         * @return bool: If the device as been stored properly.
         */
        bool addDevice(AbstractDevice *instance) {
            return this->devices_.add(instance->getId(), instance);
        }

        /**
         * Gets a device on the list by its ID.
         * @param id
         * @return AbstractDevice or NULL
         */
        AbstractDevice *getDevice(uint8_t id) {
            return this->devices_.getValue(id);
        }

        /**
         * Stringifies the device for debug purpose.
         *
         * @return String
         */
        operator String() {
            String log = "AbstractDevice Manager:\n";
            for (uint8_t i = 0; i < this->devices_.count(); i++) {
                AbstractDevice *device = this->devices_.get(i)->value;
                log += "# - " + String(*device) + "\n";
            }
            return log;
        }
};

#endif // HERMESCLIENT_DEVICE_MANAGER_H
