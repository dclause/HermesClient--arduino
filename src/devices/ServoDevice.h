#ifndef HERMESCLIENT_SERVO_DEVICE_H
#define HERMESCLIENT_SERVO_DEVICE_H

#include <Servo.h>
#include "../helper/dictionary.h"
#include "AbstractDevice.h"
#include "DeviceFactory.h"

/**
 * SERVO Device: turns servo to given angle.
 *
 * @see MessageCode::SERVO
 */
class ServoDevice : public AbstractDevice {
    DEVICE_DECLARATION

    protected:
        uint8_t pin_;
        Servo servo_;

        // @see doUpdate() for values.
        uint8_t phase_ = 0;
        // Expressed in milliseconds (ms).
        uint32_t phase_started_at_ms_ = 0;

        // -1: turn toward 0 (anti-clockwise), 1 turn toward 360° (clockwise)
        int8_t movement_direction_ = 0;

        uint16_t default_position_ms_ = 928;
        uint16_t current_position_ms_ = 928;
        uint16_t target_position_ms_ = 928;
        uint16_t start_deceleration_position_ms_ = 0;

        // "Security" delay_ms_ after we consider to be on target to detach the servo.
        // Expressed in milliseconds (ms).
        uint32_t delay_ms_ = 5000;   // 1sec.

        // tmin / tmax are the theoretical min/max position a servo can reach.
        // Often 0-180, but could be 0-360 in theory, hence it uses 16bits to express.
        uint16_t tmin_ = 0;
        uint16_t tmax_ = 180;
        // min / max are the restricted min/max positions this servo should reach according to
        // user settings. Usually chosen because of hardware robot constraints.
        // It could be 0-360 in theory, hence it uses 16bits to express.
        uint16_t min_ = 0;
        uint16_t max_ = 180;

        // Expressed in degree per second (°/sec)
        // Negative value means infinite.
        int16_t max_speed_ms_ = -1;
        int16_t max_acceleration_ms_ = -1;

        // Expressed in degree per second (°/sec)
        uint16_t current_speed_ms_ = 0;
        uint16_t target_speed_ms_ = 0;
        uint16_t max_reached_speed_ms = 0;

    public:
        // We expect 4bytes payload as:
        //  - a position between 0-360° or 544-2400 microseconds (µs) (needs 16bits of size to express).
        //  - a speed between -1 and 32767°/sec (needs 16 signed bits to express).
        ServoDevice() : AbstractDevice(4) {}

        String getName() const { return "SERVO"; }

        void updateSettings(const uint8_t *payload) {
            AbstractDevice::updateSettings(payload);
            this->pin_ = payload[1];

            // All speed/position/accelerations are two bytes word (0 to 360°) we ensure here to have a non-possible
            // value, even for the case we use Servo ms (544 to 2400).
            uint16_t defaultPosition = this->read16bits(payload[2], payload[3]);
            this->tmin_ = this->read16bits(payload[4], payload[5]);
            this->tmax_ = this->read16bits(payload[6], payload[7]);
            this->min_ = this->read16bits(payload[8], payload[9]);
            this->max_ = this->read16bits(payload[10], payload[11]);
            this->max_speed_ms_ = (int16_t)(this->read16bits(payload[12], payload[13]));
            this->max_acceleration_ms_ = (int16_t)(this->read16bits(payload[14], payload[15]));

            this->default_position_ms_ = map(defaultPosition,
                                             this->tmin_,
                                             this->tmax_,
                                             MIN_PULSE_WIDTH,
                                             MAX_PULSE_WIDTH);

            // Convert max speed °/sec to ms/sec
            if (this->max_speed_ms_ > 0) {
                this->max_speed_ms_ = this->max_speed_ms_ * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (this->tmax_ - this->tmin_);
            }

            // Convert max acceleration °/sec to ms/sec
            if (this->max_acceleration_ms_ > 0) {
                this->max_acceleration_ms_ = this->max_acceleration_ms_ * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (this->tmax_ - this->tmin_);
            }

            this->target_position_ms_ = default_position_ms_;
            this->current_position_ms_ = default_position_ms_;
            this->servo_.attach(this->pin_);
            this->servo_.writeMicroseconds(this->current_position_ms_);
            this->startMovementPhase4_();

            TRACE(*this);
        };

        void executePayload(uint8_t *payload) {
            TRACE("-----------");
            TRACE("Process SERVO command:");
            uint16_t targetPosition = this->read16bits(payload[0], payload[1]);
            int16_t targetSpeed = this->read16bits(payload[2], payload[3]);
            // Cap with the min/max values (this should be done by caller, hence just be a security issue here)
            targetPosition = max(this->min_, targetPosition);
            targetPosition = min(this->max_, targetPosition);
            this->target_position_ms_ = map(targetPosition,
                                            this->tmin_,
                                            this->tmax_,
                                            MIN_PULSE_WIDTH,
                                            MAX_PULSE_WIDTH);

            // Convert max speed °/sec to ms/sec
            this->target_speed_ms_ = (targetSpeed > 0) ? targetSpeed * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (this->tmax_ - this->tmin_) : targetSpeed;
            // Cap with max speed (this should be done by caller, hence just be a security issue here)
            this->target_speed_ms_ = this->max_speed_ms_ > 0 ? min(this->max_speed_ms_, this->target_speed_ms_) : this->target_speed_ms_;

            TRACE("  > Servo on PIN " + String(this->pin_) +
                  " requests position: " + String(targetPosition) + "(" + String(this->target_position_ms_) + ")" +
                  " requests speed: " + String(targetSpeed) + "(" + String(this->target_speed_ms_) + ")"
            );

            this->startMovementPhase1_();
        }

        /**
         * Phase = 0: do nothing
         * Phase = 1: accelerate
         * Phase = 2: run full speed
         * Phase = 3: decelerate
         * Phase = 4: wait before detach
         * @param current_time
         */
        void doUpdate(const uint32_t current_time) {

            // Phase1: acceleration.
            if (this->phase_ == 1) {
                this->current_speed_ms_ = this->getSpeedWithAcceleration_(current_time);
                if (this->current_speed_ms_ > this->target_speed_ms_) {
                    this->startMovementPhase2_();
                }
                this->current_position_ms_ = this->getPositionAt_(current_time);
                if (this->shouldStartPhase3_()) {
                    this->startMovementPhase3_();
                }
                if (this->current_position_ms_ == this->target_position_ms_) {
                    TRACE("~~ Phase1: Error on movement calculation");
                    this->startMovementPhase4_();
                }
                this->servo_.writeMicroseconds((int) this->current_position_ms_);
#if ACTIVATE_DEBUG
                if (this->phase_ != 1) {
                    TRACE(" - current position (ms): " + String(this->current_position_ms_) + "(" + String(this->target_position_ms_) + ")");
                }
#endif
            }

            // Phase2: run at full speed.
            if (this->phase_ == 2) {
                this->current_position_ms_ = this->getPositionAt_(current_time);
                if (this->shouldStartPhase3_()) {
                    this->startMovementPhase3_();
                }
                if (this->current_position_ms_ == this->target_position_ms_) {
                    this->startMovementPhase4_();
                }
                this->servo_.writeMicroseconds((int) this->current_position_ms_);

#if ACTIVATE_DEBUG
                if (this->phase_ != 2) {
                    TRACE(" - current position (ms): " + String(this->current_position_ms_) + "(" + String(this->target_position_ms_) + ")");
                }
#endif
            }

            // Phase3: deceleration.
            if (this->phase_ == 3) {
                this->current_speed_ms_ = this->getSpeedWithDeceleration_(current_time);
                if (this->current_speed_ms_ < 0) {
                    if (this->current_position_ms_ != this->target_position_ms_) {
                        TRACE("~~ Phase3: Error on movement calculation: " + String(this->current_position_ms_) + "(" + String(this->target_position_ms_) + ")");
                    }
                    this->startMovementPhase4_();
                }
                this->current_position_ms_ = this->getPositionAt_(current_time);
                if (this->current_position_ms_ == this->target_position_ms_) {
                    this->startMovementPhase4_();
                }
                this->servo_.writeMicroseconds((int) this->current_position_ms_);
#if ACTIVATE_DEBUG
                if (this->phase_ != 3) {
                    TRACE(" - current position (ms): " + String(this->current_position_ms_) + "(" + String(this->target_position_ms_) + ")");
                }
#endif
            }


            // Phase4: idle.
            if (this->phase_ == 4) {
                if (this->servo_.attached() && (current_time - this->phase_started_at_ms_) > this->delay_ms_) {
                    this->startMovementPhase0_();
                    this->servo_.detach();
                }
                this->servo_.writeMicroseconds((int) this->current_position_ms_);
            }
        };

        operator String() {
            return AbstractDevice::operator String() + "\n" +
                   "\t\tpin:" + String(this->pin_) + "\n" +
                   "\t\tdefault:" + String(this->default_position_ms_) + "\n" +
                   "\t\tcurrent:" + String(this->current_position_ms_) + "\n" +
                   "\t\trequest:" + String(this->target_position_ms_) + "\n" +
                   "\t\ttmin:" + String(this->tmin_) + "\n" +
                   "\t\ttmax:" + String(this->tmax_) + "\n" +
                   "\t\tmin:" + String(this->min_) + "\n" +
                   "\t\tmax:" + String(this->max_) + "\n" +
                   "\t\tmax speed:" + String(this->max_speed_ms_) + "\n" +
                   "\t\tmax acceleration:" + String(this->max_acceleration_ms_);
        }

    private:

        /**
         * Helper: resets data to start a movement by a phase 1: acceleration.
         */

        void startMovementPhase0_() {
            this->phase_ = 0;
            this->phase_started_at_ms_ = 0;
        }

        // Phase1: acceleration.
        void startMovementPhase1_() {
            this->phase_ = 1;
            this->phase_started_at_ms_ = millis();
            this->current_speed_ms_ = 0;
            this->max_reached_speed_ms = 0;
            this->servo_.attach(this->pin_);

            // Compute the movement direction.
            // 1 turn clockwise (toward 360°)
            // -1 turn anti-clockwise (toward 0°)
            this->movement_direction_ = (this->target_position_ms_ > this->current_position_ms_) ? 1 : -1;

            // If no acceleration, we can go directly to phase2:
            if (this->max_acceleration_ms_ < 0) {
                return this->startMovementPhase2_();
            }

            // Find out when to decelerate:
            // - either at the middle on the movement is the rotation is short enough to never reach max speed.
            // - either when the position reach a point before target where it will travel to target during the deceleration time.
            // NOTE: acceleration and deceleration is done via the same formula. Hence, the deceleration duration is the same
            // as acceleration phase.
            // distance spent during acc/dec is: Vmax² / 2a
            uint32_t totalDistance = abs(this->target_position_ms_ - this->current_position_ms_);
            uint32_t distanceDuringAcceleration = this->target_speed_ms_ > 0
                                                          ? pow(this->target_speed_ms_, 2) / (2 * this->max_acceleration_ms_)
                                                          : totalDistance;
            uint32_t halfWay = totalDistance / 2;
            if (distanceDuringAcceleration > halfWay) {
                distanceDuringAcceleration = halfWay;
            }
            this->start_deceleration_position_ms_ = this->target_position_ms_ + distanceDuringAcceleration * this->movement_direction_ * -1;

            TRACE(" > Start phase1");
            TRACE(" - current position (ms): " + String(this->current_position_ms_) + "(" + String(this->target_position_ms_) + ")");
            TRACE(" - degrees (ms) during acceleration phase: " + String(distanceDuringAcceleration));
            TRACE(" - should start decelerate at: " + String(this->start_deceleration_position_ms_));
        }

        // Phase2: run at full speed.
        void startMovementPhase2_() {
            this->phase_ = 2;
            this->phase_started_at_ms_ = millis();
            this->current_speed_ms_ = this->target_speed_ms_;
            TRACE(" > Start phase2");
        }

        // Phase2: decelerate.
        void startMovementPhase3_() {
            this->phase_ = 3;
            this->phase_started_at_ms_ = millis();
            this->max_reached_speed_ms = this->current_speed_ms_;
            TRACE(" > Start phase3");
        }

        // Phase = 4: wait before detach.
        void startMovementPhase4_() {
            this->phase_ = 4;
            this->phase_started_at_ms_ = millis();
            this->current_position_ms_ = this->target_position_ms_;
            TRACE(" > Start phase4");
        }

        bool shouldStartPhase3_() const {
            return (this->current_position_ms_ > this->start_deceleration_position_ms_ && this->movement_direction_ == 1) ||
                   (this->current_position_ms_ < this->start_deceleration_position_ms_ && this->movement_direction_ == -1);
        }

        /**
         * Helper: builds a 16bit word from two 8bits.
         * @param byte1
         * @param byte2
         * @return
         */
        uint16_t read16bits(uint8_t byte1, uint8_t byte2) {
            TRACE(String("Read bits:") + "\n" +
                  "\t\tbyte1:" + String(byte1) + '\n' +
                  "\t\tbyte2:" + String(byte2));
            return (byte1 << 8) | byte2;
        }

        /**
         * Computes the speed at the given time during an acceleration phase.
         * @param time Expressed in ms.
         * @return
         */
        float getSpeedWithAcceleration_(const uint32_t time) {
            return this->max_acceleration_ms_ * (time - this->phase_started_at_ms_) / 1000.0;
        }

        /**
         * Computes the speed at the given time during a deceleration phase.
         * @param time Expressed in ms.
         * @return
         */
        float getSpeedWithDeceleration_(const uint32_t time) {
            return this->max_reached_speed_ms - this->max_acceleration_ms_ * (time - this->phase_started_at_ms_) / 1000.0;
        }


        float getPositionAt_(const uint32_t time) {
            // If no speed limit, we can go directly to target.
            if (this->current_speed_ms_ < 0) {
                return this->target_position_ms_;
            }

            int16_t remainingDeltaMs = this->target_position_ms_ - this->current_position_ms_;
            float step = this->current_speed_ms_ * (time - this->time_of_last_update_) / 1000.0;
            float newPosition = (abs(remainingDeltaMs) > step) ? this->current_position_ms_ + step * this->movement_direction_ : this->target_position_ms_;

            TRACE(" - remainingDeltaMs (ms): " + String(remainingDeltaMs));
            TRACE(" - move by a step of (ms): " + String(step));
            TRACE(" - new position (ms): " + String(newPosition) + "(" + String(this->target_position_ms_) + ")");

            return newPosition;
        }
};

REGISTER_DEVICE(MessageCode::SERVO, ServoDevice)

#endif // HERMESCLIENT_SERVO_DEVICE_H
