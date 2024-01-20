#ifndef HERMESCLIENT_DEVICE_FACTORY_H
#define HERMESCLIENT_DEVICE_FACTORY_H

#include "../helper/map.h"
#include "../helper/dictionary.h"
#include "AbstractDevice.h"

// Callback for instantiating "on the fly" a device.
using DeviceInstance = AbstractDevice *(*)();

/**
 * Device Factory: registers device and instantiates it as necessary from a given device code.
 *
 * @note
 * Device codes are mapped to actual devices via the DeviceFactory by registering the device class to the factory
 * keyed by the appropriate device code. This is done via the conjunction usage of the macros `DEVICE_DECLARATION`
 * and `REGISTER_DEVICE(N, T)`.
 */
class DeviceFactory {
    private:
        DeviceFactory() = default;

        KeyValueMap<MessageCode, DeviceInstance> registeredDevices_;

    public:
        DeviceFactory(const DeviceFactory &) = delete;

        DeviceFactory &operator=(const DeviceFactory &) = delete;

        static DeviceFactory &getInstance() {
            static DeviceFactory instance;
            return instance;
        }

        /**
         * Let a MessageCode be associated to an instantiable callback.
         * Stores this association in the internal map.
         *
         * @param code (MessageCode)
         * @param callback (DeviceInstance) @see `using` statement at the start of file.
         * @return bool: If the device is properly registered.
         */
        bool registerDevice(MessageCode code, DeviceInstance callback) {
            return this->registeredDevices_.add(code, callback);
        }

        /**
         * Retrieves the class type associated with a device code.
         *
         * @param code (MessageCode)
         * @return any: The device class.
         */
        AbstractDevice *getDeviceType(MessageCode code) {
            int index = this->registeredDevices_.getPosition(code);
            if (index > -1) {
                return this->registeredDevices_.getValue(code)();
            }
            return NULL;
        }

        /**
         * Instantiates an AbstractDevice of the proper type given a MessageCode.
         *
         * @param code (MessageCode)
         * @return AbstractDevice: The instantiated device class.
         */
        AbstractDevice *createDevice(MessageCode code) {
            int index = this->registeredDevices_.getPosition(code);
            if (index > -1) {
                return this->registeredDevices_.getValue(code)();
            }
            return NULL;
        }

        /**
         * Stringifies the data for debug purpose.
         *
         * @return String
         */
        operator String() {
            String log = "Device Factory " + String(this->registeredDevices_.count()) + ":\n";
            for (uint8_t i = 0; i < this->registeredDevices_.count(); i++) {
                AbstractDevice *device = this->registeredDevices_.get(i)->value();
                log += "# - " + String(*device) + "\n";
            }
            return log;
        }
};

#define DEVICE_DECLARATION protected: \
    static AbstractDevice* getInstance(); \
    static bool isRegistered;
#define GET_DEVICE_INSTANCE(T) AbstractDevice* T::getInstance() { return new T(); }
#define REGISTER_DEVICE(N, T) bool T::isRegistered = DeviceFactory::getInstance().registerDevice(N, &T::getInstance);GET_DEVICE_INSTANCE(T);

#endif // HERMESCLIENT_DEVICE_FACTORY_H
