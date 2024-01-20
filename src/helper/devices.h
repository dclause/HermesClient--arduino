#ifndef HERMESCLIENT_DEVICES_H
#define HERMESCLIENT_DEVICES_H

// ! All device includes must be listed here.
#include "../devices/AbstractDevice.h"
#include "../devices/DeviceManager.h"
#include "../devices/ServoDevice.h"
#include "../devices/DigitalWriteDevice.h"

namespace Devices {

    /**
     * Loop over Devices and update them.
     */
    void update_all_devices() {
        KeyValuePair<uint8_t, AbstractDevice *> *device = DeviceManager::getInstance().getHead();
        while (device) {
            device->value->update();
            device = device->next;
        }
    }

}  // namespace Devices

#endif // HERMESCLIENT_DEVICES_H
