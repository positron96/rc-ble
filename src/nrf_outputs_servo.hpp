#ifndef NRF_OUTPUTS_SERVO_HPP_
#define NRF_OUTPUTS_SERVO_HPP_

#include <functions/base_functions.hpp>
#include "nrf_outputs.hpp"

namespace nrf {

    /**
     * Uses hardware timer to generate servo events,
     * but controls pins from timer interrupt.
     *
     * Can handle much more servo outputs and does not occupy PPI channels.
     * Timer ISR fires every servo interval (i.e. ~1.5ms)
     */
    struct SWServoTimer: fn::Wakeable {

        struct Servo: outputs::BaseServo {
            size_t pin;
            SWServoTimer *parent = nullptr;
            size_t index = 0;
            size_t current_us = center_us;

            Servo(size_t pin): pin{pin} {
            }

            void set_us(uint16_t) override;
        };

        static constexpr nrf_timer_int_mask_t cc_int_mask = NRF_TIMER_INT_COMPARE1_MASK;
        static constexpr nrf_timer_event_t cc_event = NRF_TIMER_EVENT_COMPARE1;
        static constexpr nrf_timer_cc_channel_t cc_ch = NRF_TIMER_CC_CHANNEL1;
        static constexpr nrf_timer_short_mask_t cc_short_mask = NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK;

        static constexpr size_t MAX_SERVOS = outputs::BaseServo::SERVO_PERIOD_US / 2500;
        etl::vector<Servo*, MAX_SERVOS> servos;

        bool running = false;

        bool add_servo(Servo &servo) {
            if(servos.available()==0) return false;
            servo.index = servos.size();
            servo.parent = this;
            servos.push_back(&servo);
            return true;
        }

        void set_us(size_t idx, uint16_t us) {
            cycles_left = SERVO_CYCLES;
            set_paused(false);
        }

        void init() {
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
            nrf_timer_bit_width_set(servo_timer, NRF_TIMER_BIT_WIDTH_16);
            nrf_timer_mode_set(servo_timer, NRF_TIMER_MODE_TIMER);
            nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_1MHz); // 1us resolution
            // nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_250kHz); // debug

            nrf_timer_int_enable(servo_timer, cc_int_mask);

            nrf_timer_shorts_enable(servo_timer, cc_short_mask);

            nrf_timer_cc_write(
                servo_timer, cc_ch, outputs::BaseServo::SERVO_PERIOD_US);

            for(size_t i=0; i<servos.size(); i++) {
                size_t pin = servos[i]->pin;
                servos[i]->current_us = servos[i]->center_us;
                nrf_gpio_cfg_output(pin);
            }
        }

        void wake() { init(); }

        void sleep() {
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
            nrf_timer_int_disable(servo_timer, cc_int_mask);

            for(size_t i=0; i<servos.size(); i++) {
                size_t pin = servos[i]->pin;
                nrf_gpio_cfg_default(pin);
            }
        }

        void isr() {
            if (nrf_timer_event_check(servo_timer, cc_event)) {
                // clear event flag
                nrf_timer_event_clear(servo_timer, cc_event);

                nrf_gpio_pin_clear(servos[current_servo]->pin);

                current_servo++;
                if(current_servo == servos.size()) {
                    current_servo = 0;
                    // looped through all servoes, check if we've ran out of cycles
                    //cycles_left--;
                    //if(cycles_left == 0) {
                    //    set_paused(true);
                    //}
                }
                nrf_gpio_pin_set(servos[current_servo]->pin);
                nrf_timer_cc_write(servo_timer, cc_ch, servos[current_servo]->current_us);
            }
        }

    private:
        static constexpr size_t SERVO_CYCLES = 5;
        size_t cycles_left;

        size_t current_servo = 0;

        /**
         * Stops/starts timer.
         * Call set_pause(false) only from isr,
         * otherwise GPIOs can be in unpredictable state.
         */
        void set_paused(bool to_pause) {
            if(!running && !to_pause) {
                sd_clock_hfclk_request();
                nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
                NVIC_EnableIRQ(servo_irq);
                running = true;
            } else
            if(running && to_pause) {
                nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
                NVIC_DisableIRQ(servo_irq);
                sd_clock_hfclk_release();
                running = false;
            }
        }
    };

    inline void SWServoTimer::Servo::set_us(uint16_t us)  {
        current_us = us;
        if(parent!=nullptr)
            parent->set_us(index, us);
    };

}

#endif // NRF_OUTPUTS_SERVO_HPP_
