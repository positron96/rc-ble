/**
 * Implementation of PDM output (aka 1-bit DAC)
 * to be used as an analog pin.
 */
#ifndef NRF_PDM_H_
#define NRF_PDM_H_

#include "functions.h"

#include <nrf_gpio.h>

// # original pseudocode (and below) has values 20 and 32 swapped by mistake
// P = 20     # Number of interrupts in a PWM period where the output is 1.
// T = 32     # Number of interrupts in a PWM period.
// h2 = T
// def timer_interrupt():
//     if h2 < T:
//         output = 1
//         h2 += P - T
//     else:
//         output = 0
//         h2 -= T

// https://matthewearl.github.io/2015/03/05/efficient-pwm/

namespace nrf {

    /**
     * A bit-banging analog pin.
     *
     * Call `tick()` at decent frequency.
     * For LEDs should be at least 1KHz.
     */
    class PdmPin: public fn::BaseAnalogPin {
    private:
        constexpr static uint8_t P = 255;
        size_t pin;
        uint8_t t;
        uint8_t h2;
    public:
        PdmPin(size_t pin): pin{pin}, t{0}, h2{0} {
            nrf_gpio_cfg_output(pin);
        }

        void set_pwm(uint8_t val) override {
            t = val;
        };

        void tick() {
            if (h2 < t) {
                nrf_gpio_pin_write(pin, 1);
                h2 += P - t;
            } else {
                nrf_gpio_pin_write(pin, 0);
                h2 -= t;
            }
        }

    };
};

#endif // NRF_PDM_H_