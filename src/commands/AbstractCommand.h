#ifndef HERMESCLIENT_COMMAND_H
#define HERMESCLIENT_COMMAND_H

#include "../helper/debugger.h"
#include "../protocols/io.h"
#include <Arduino.h>


/**
 * Class AbstractCommand: all commands must implement this class.
 *
 * A command ca be:
 *  - An action that is required by the master to be done on the slave board.
 *      Ex.: turn a motor, orientate a servomotor, light a led...
 *  - An input that is required to be read and sent back to the master.
 *      Ex.: value of a push button, a PIR sensor, a distance sensor...
 *  - A generic purpose command, generally for the protocol itself.
 *      Ex.: a heartbeat, a handshake, an ACK, etc...
 *
 * A command can either be received (from the backend) or sent (to the backend).
 *
 * A command is a suite of 8bits words:
 *  - starting with an uint8_t (one of the MessageCode dictionary) that will be mapped to the appropriate class by
 *  the CommandFactory.
 *  - followed by a payload which is an arbitrary number of bytes for arguments that are read by the receivePayload_().
 *
 * @note
 * Command codes are mapped to actual commands via the CommandFactory by registering the command class to the factory
 * keyed by the appropriate command code. This is done via the conjunction usage of the macros `COMMAND_DECLARATION`
 * and `REGISTER_COMMAND(N, T)`.
 * @see CommandFactory.h
 */
class AbstractCommand {
    protected:
        int expected_payload_size_;
        uint8_t effective_payload_size_;
        uint8_t *payload_;

    public:

        AbstractCommand(const int expected_payload_size = 0) :
                expected_payload_size_(expected_payload_size),
                effective_payload_size_(expected_payload_size ? expected_payload_size : 0) {
            if (expected_payload_size > 0) {
                this->payload_ = new uint8_t[expected_payload_size];
            } else {
                // Size indicator in messages is on 8bits, hence the 255 max size when size is unknown.
                this->payload_ = new uint8_t[255];
            }
        }

        virtual ~AbstractCommand() {
            free(this->payload_);
            this->payload_ = NULL;
        };

        /**
         * Returns a human readable name for the command.
         *
         * @return String
         */
        virtual String getName() const = 0;

        /**
         * Processes the command when received from the serial port. That means receive the payload and execute it.
         */
        void process() {
            this->receivePayload_();
            this->executePayload(this->payload_);
        };


        /**
         * Stringifies the command for debug purpose.
         *
         * @return String
         */
        operator String() const {
            return "Command " + this->getName();
        }

    private:

        /**
         * Load the expected data in the internal payload.
         * The amount of data loaded is defined by the expected_payload_size_ variable for the command, or the given data
         * size as first byte sent after command code.
         * ex: 33 26 1 2 3 .... 26 would solve to a HANDSHAKE (33) with 26 bytes or data (1...26)
         *
         * The maximum amount of data sent is 255 bytes.
         */
        void receivePayload_() {

            // When the payload size can vary (negative expected_payload_size), the first byte receive
            // is the effective size of the incoming payload.
            if (this->expected_payload_size_ < 0) {
                IO::wait_for_bytes(1);
                IO::read_bytes(this->payload_, 1);
                this->effective_payload_size_ = this->payload_[0];
            } else {
                this->effective_payload_size_ = this->expected_payload_size_;
            }

            if (this->effective_payload_size_ > 0) {
                IO::wait_for_bytes(this->effective_payload_size_);
                IO::read_bytes(this->payload_, this->effective_payload_size_);
            }
            TRACE("Payload size: " + String(this->effective_payload_size_));
#if ACTIVATE_DEBUG
            String payloadAsInts = "";
            for (uint8_t i = 0; i < this->effective_payload_size_; i++) {
                payloadAsInts += String(this->payload_[i]) + " ";
            }
            TRACE("Payload received: " + payloadAsInts);
#endif
        }

        /**
         * Executes the command using the given payload.
         *
         * @param payload (uint8_t*)
         */
        virtual void executePayload(uint8_t *payload) = 0;
};

#endif// HERMESCLIENT_COMMAND_H
