#pragma once

#include <nordic/nrfx/hal/nrf_timer.h>
#include <nordic/nrfx/hal/nrf_gpio.h>
#include <nordic/nrfx/hal/nrf_gpiote.h>
#include <nordic/nrfx/hal/nrf_ppi.h>

#include "functions.h"

#include "log.h"


namespace nrf {

    struct ServoTimer;

    struct Servo: fn::Servo {

        size_t pin;
        ServoTimer *owner;
        size_t index;

        Servo(size_t pin): pin{pin} {
        }

        void set_us(uint16_t us) override;

    };

    NRF_TIMER_Type *servo_timer = NRF_TIMER1;
    constexpr IRQn_Type servo_irq = TIMER1_IRQn;

    struct ServoTimer {
        static constexpr size_t SERVO_PERIOD = 20000;
        static constexpr size_t MAX_SERVOS = 3;

        etl::vector<Servo*, MAX_SERVOS> servos;

        bool add_servo(Servo &servo) {
            if(servos.available()==0) return false;
            servo.index = servos.size();
            servo.owner = this;
            servos.push_back(&servo);
            return true;
        }

        void set_us(size_t idx, uint16_t us) {
            nrf_timer_cc_set(servo_timer, (nrf_timer_cc_channel_t)idx, us);
            logf("set_us %d\n", us);
            cycles_left = SERVO_CYCLES;
            NVIC_EnableIRQ(servo_irq);
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
        }

        void init() {
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
            nrf_timer_bit_width_set(servo_timer, NRF_TIMER_BIT_WIDTH_16);
            nrf_timer_mode_set(servo_timer, NRF_TIMER_MODE_TIMER);
            nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_1MHz); // 1us resolution
            // nrf_timer_frequency_set(servo_timer, NRF_TIMER_FREQ_250kHz); // debug

            nrf_timer_int_enable(servo_timer, NRF_TIMER_INT_COMPARE3_MASK);

            nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK);

            nrf_timer_cc_set(servo_timer, NRF_TIMER_CC_CHANNEL3, SERVO_PERIOD);

            for(size_t i=0; i<servos.size(); i++) {
                size_t pin = servos[i]->pin;
                nrf_gpio_cfg_output(pin);
                size_t gpiote_ch = i;

                nrf_ppi_channel_t ppi_ch_on = ppi_ch(i*2);
                nrf_ppi_channel_t ppi_ch_off = ppi_ch(i*2+1);

                nrf_gpiote_task_configure(NRF_GPIOTE, gpiote_ch,
                    pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW
                );
                nrf_gpiote_task_enable(NRF_GPIOTE, gpiote_ch);
                nrf_ppi_channel_endpoint_setup(
                    NRF_PPI, ppi_ch_on,
                    //(uint32_t)&servo_timer->EVENTS_COMPARE[i],
                    //(uint32_t)&NRF_GPIOTE->TASKS_CLR[gpiote_ch]
                    nrf_timer_event_address_get(
                        servo_timer, nrf_timer_compare_event_get(i)),
                    nrf_gpiote_task_address_get(
                        NRF_GPIOTE, nrf_gpiote_clr_task_get(gpiote_ch))
                );
                nrf_ppi_channel_enable(NRF_PPI, ppi_ch_on);

                nrf_ppi_channel_endpoint_setup(
                    NRF_PPI, ppi_ch_off,
                    nrf_timer_event_address_get(servo_timer, NRF_TIMER_EVENT_COMPARE3),
                    //(uint32_t)&NRF_GPIOTE->TASKS_SET[gpiote_ch]
                    nrf_gpiote_task_address_get(
                        NRF_GPIOTE, nrf_gpiote_set_task_get(gpiote_ch))
                );
                nrf_ppi_channel_enable(NRF_PPI, ppi_ch_off);
            }
        }

        static void isr() {
            if (nrf_timer_event_check(servo_timer, NRF_TIMER_EVENT_COMPARE3)) {
                // clear event flag
                nrf_timer_event_clear(servo_timer, NRF_TIMER_EVENT_COMPARE3);
                cycles_left--;
                if(cycles_left == 0) {
                    nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
                }
            }
        }
    private:
        static constexpr size_t SERVO_CYCLES = 5;
        static size_t cycles_left;

        static constexpr nrf_ppi_channel_t PPI_FIRST_CH = NRF_PPI_CHANNEL6;

        constexpr nrf_ppi_channel_t ppi_ch(size_t i) {
            return nrf_ppi_channel_t(size_t(PPI_FIRST_CH) + i);
        }
    };

    struct HBridge: fn::Hbridge {
        size_t pin1, pin2;
        HBridge(size_t p1, size_t p2): pin1{p1}, pin2{p2} {}
        void set(int8_t val) override {}
    };

    class PwmPin: public fn::PwmPin {

    };

    class PWM {
    public:
        bool setHBridge(HBridge &ret) {
            return true;
        }

        bool setPin(PwmPin &pin) {
            return true;
        }

    };

    class Pin: public fn::Pin {
    public:
        size_t pin;
        Pin(size_t num) : pin{num} {}
        void set(bool v) override {
            nrf_gpio_pin_write(pin, v ? 1 : 0);
        }
    };

};


void nrf::Servo::set_us(uint16_t us) {
    owner->set_us(index, us);
};


extern "C" void TIMER1_IRQHandler() {
    nrf::ServoTimer::isr();
};

size_t nrf::ServoTimer::cycles_left = 0;
