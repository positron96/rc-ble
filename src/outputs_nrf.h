#pragma once

#include <nordic/nrfx/hal/nrf_timer.h>
#include <nordic/nrfx/hal/nrf_gpio.h>
#include <nordic/nrfx/hal/nrf_gpiote.h>
#include <nordic/nrfx/hal/nrf_ppi.h>

#include "functions.h"


namespace nrf {

    NRF_TIMER_Type *servo_timer = NRF_TIMER1;
    constexpr IRQn_Type servo_irq = TIMER1_IRQn;
    #define servo_isr TIMER1_IRQHandler

    struct NrfServo: outputs::Servo {

        size_t pin;

        nrf_timer_cc_channel_t timer_channel;

        NrfServo(size_t pin, nrf_timer_cc_channel_t ch): Servo{}, pin{pin}, timer_channel{ch} {

        }

        void set_us(uint16_t us) override {
            nrf_timer_cc_set(servo_timer, timer_channel, us);
            //cycles_left = SERVO_CYCLES;
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
        };
        
        void start() override {

        };
        void stop() override {

        };
    };

    class NrfServoTimer {
        size_t n_servos = 0;

        NrfServo* create_servo(size_t pin) {
            if(n_servos == 3) return nullptr;
            auto ret = new NrfServo(pin, static_cast<nrf_timer_cc_channel_t>(n_servos));
            n_servos ++;
            return ret;
        }
    };

    class HBridge: public outputs::Hbridge {

    };

    class PwmPin: public outputs::PwmPin {

    };

    class PWM {
    public:
        HBridge* createHBridge(size_t pin1, size_t pin2) {
            return new HBridge();
        }

        PwmPin* createPin(size_t pin) {
            return new PwmPin();
        }


    };

};