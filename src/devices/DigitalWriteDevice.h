#ifndef HERMESCLIENT_DIGITAL_WRITE_DEVICE_H
#define HERMESCLIENT_DIGITAL_WRITE_DEVICE_H

#include <Arduino.h>

#include "../helper/debugger.h"
#include "../helper/dictionary.h"
#include "AbstractDevice.h"
#include "DeviceFactory.h"

/**
 * DIGITAL_WRITE Device: toggles a digital pin HIGH or LOW (0 or 1).
 *
 * @see MessageCode::DIGITAL_WRITE
 */
class DigitalWriteDevice : public AbstractDevice {
    DEVICE_DECLARATION

    protected:
        uint8_t pin_;
        uint8_t default_;
        uint8_t value_;

    public:

        DigitalWriteDevice() : AbstractDevice(1) {}

        String getName() const { return "DigitalWrite"; }

        void updateSettings(const uint8_t *payload) {
            AbstractDevice::updateSettings(payload);
            this->pin_ = payload[1];
            this->default_ = payload[2];
            this->value_ = this->default_;
            pinMode(this->pin_, OUTPUT);
            digitalWrite(this->pin_, this->value_);
            TRACE(*this);
        };

        void executePayload(uint8_t *payload) {
            TRACE("-----------");
            TRACE("Process DIGITAL_WRITE device:");
            this->value_ = payload[0];
            TRACE("  > Set pin " + String(this->pin_) + " to: " + String(this->value_));
            digitalWrite(this->pin_, this->value_);
            TRACE("-----------");
        }
};

REGISTER_DEVICE(MessageCode::DIGITAL_WRITE, DigitalWriteDevice)

#endif  // HERMESCLIENT_DIGITAL_WRITE_DEVICE_H
