#pragma once

#include <nrf_timer.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrf_ppi.h>
#include <nrf_pwm.h>

#include <etl/vector.h>
#include <etl/array.h>

#include <outputs.hpp>
#include <functions/base_functions.hpp>

#include "log.h"


namespace nrf {

    NRF_TIMER_Type *servo_timer = NRF_TIMER1;
    constexpr IRQn_Type servo_irq = TIMER1_IRQn;

    /**
     * Class to generate servo pulses with hardware timer that (mostly) does not use CPU.
     *
     * It uses 3 CC channels and PPI to set 3 pins high,
     *   and 4th CC channel to set them all low (so 3 servos max).
     * For every servo output it uses 1 CC channel and 2 PPI channels.
     * Timer ISR runs every 20ms to do some house cleaning
     * (it stops outputting pulses after some cycles with the goal to reduce servo jitter).
     */
    struct ServoTimer: fn::Wakeable {

        struct Servo: outputs::BaseServo {
            size_t pin;
            ServoTimer *owner;
            size_t index;
            Servo(size_t pin): pin{pin} {
            }
            void set_us(uint16_t us) override;
        };

        static constexpr size_t MAX_SERVOS = 3;

        etl::vector<Servo*, MAX_SERVOS> servos;

        bool running = false;

        bool add_servo(Servo &servo) {
            if(servos.available()==0) return false;
            servo.index = servos.size();
            servo.owner = this;
            servos.push_back(&servo);
            return true;
        }

        void set_us(size_t idx, uint16_t us) {
            nrf_timer_cc_channel_t ch = (nrf_timer_cc_channel_t)idx;
            uint16_t current = nrf_timer_cc_read(servo_timer, ch);
            if(current == us) return;
            nrf_timer_cc_write(servo_timer, ch, us);
            // logf("set_us(%d) %d\n", idx, us);
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

            nrf_timer_int_enable(servo_timer, NRF_TIMER_INT_COMPARE3_MASK);

            nrf_timer_shorts_enable(servo_timer, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK);

            nrf_timer_cc_write(
                servo_timer, NRF_TIMER_CC_CHANNEL3, outputs::BaseServo::SERVO_PERIOD_US);

            for(size_t i=0; i<servos.size(); i++) {
                size_t pin = servos[i]->pin;
                nrf_gpio_cfg_output(pin);
                size_t gpiote_ch = i;

                nrf_ppi_channel_t ppi_ch_on = ppi_ch(i*2);
                nrf_ppi_channel_t ppi_ch_off = ppi_ch(i*2+1);

                nrf_gpiote_task_configure(gpiote_ch,
                    pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW
                );
                nrf_gpiote_task_enable(gpiote_ch);
                nrf_ppi_channel_endpoint_setup(
                    ppi_ch_on,
                    //(uint32_t)&servo_timer->EVENTS_COMPARE[i],
                    //(uint32_t)&NRF_GPIOTE->TASKS_CLR[gpiote_ch]
                    (uint32_t)nrf_timer_event_address_get(
                        servo_timer,  nrf_timer_compare_event_get(i)),
                    nrf_gpiote_task_addr_get(
                        nrf_gpiote_clr_task_get(gpiote_ch))
                );
                nrf_ppi_channel_enable(ppi_ch_on);

                nrf_ppi_channel_endpoint_setup(
                    ppi_ch_off,
                    (uint32_t)nrf_timer_event_address_get(servo_timer, NRF_TIMER_EVENT_COMPARE3),
                    //(uint32_t)&NRF_GPIOTE->TASKS_SET[gpiote_ch]
                    nrf_gpiote_task_addr_get(
                        nrf_gpiote_set_task_get(gpiote_ch))
                );
                nrf_ppi_channel_enable(ppi_ch_off);
            }
        }

        void wake() { init(); }

        void sleep() {
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_CLEAR);
            nrf_timer_int_disable(servo_timer, NRF_TIMER_INT_COMPARE3_MASK);

            for(size_t i=0; i<servos.size(); i++) {
                size_t pin = servos[i]->pin;

                size_t gpiote_ch = i;

                nrf_gpiote_task_disable(gpiote_ch);
                nrf_ppi_channel_t ppi_ch_on = ppi_ch(i*2);
                nrf_ppi_channel_t ppi_ch_off = ppi_ch(i*2+1);

                nrf_ppi_channel_enable(ppi_ch_on);
                nrf_ppi_channel_enable(ppi_ch_off);

                nrf_gpio_cfg_default(pin);
            }
        }

        void isr() {
            if (nrf_timer_event_check(servo_timer, NRF_TIMER_EVENT_COMPARE3)) {
                // clear event flag
                nrf_timer_event_clear(servo_timer, NRF_TIMER_EVENT_COMPARE3);
                cycles_left--;
                if(cycles_left == 0) {
                    set_paused(true);
                }
            }
        }
    private:
        static constexpr size_t SERVO_CYCLES = 5;
        size_t cycles_left;

        static constexpr nrf_ppi_channel_t PPI_FIRST_CH = NRF_PPI_CHANNEL6;

        constexpr nrf_ppi_channel_t ppi_ch(size_t i) {
            return nrf_ppi_channel_t(size_t(PPI_FIRST_CH) + i);
        }

        /**
         * Stops/starts timer.
         * Call set_pause(false) only from isr,
         * otherwise GPIOs can be in unpredictable state.
         */
        void set_paused(bool to_pause) {
            if(!running && !to_pause) {
                nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_START);
                NVIC_EnableIRQ(servo_irq);
                running = true;
            } else
            if(running && to_pause) {
                nrf_timer_task_trigger(servo_timer, NRF_TIMER_TASK_STOP);
                NVIC_DisableIRQ(servo_irq);
                running = false;
                // release GPIO to that it can be controlled with GPIO
                //nrf_gpiote_task_disable(NRF_GPIOTE, gpiote_ch);

            }
        }
    };

    inline void ServoTimer::Servo::set_us(uint16_t us) {
        //logf("%d\n", us);
        if(owner!=nullptr)
            owner->set_us(index, us);
    };



    NRF_PWM_Type *pwm = NRF_PWM0;

    /**
     * Class that works with one NRF PWM peripheral.
     *
     * Provides 4 PWM pins either as AnalogPins or as HBridges (in any combination).
     */
    class PWM: public fn::Wakeable {
    public:
        static constexpr size_t NUM_PINS = NRF_PWM_CHANNEL_COUNT;
        static constexpr size_t MAX_PWM = 255;

        struct HBridge: outputs::BaseHbridge {
            size_t pin1, pin2;

            HBridge(size_t p1, size_t p2): pin1{p1}, pin2{p2} {}

            void set_raw(uint8_t val, bool fwd) override;
        private:
            PWM *owner;
            size_t index;
            friend class PWM;
        };

        struct Pin: outputs::BaseAnalogPin {
            size_t pin;
            Pin(size_t p): pin{p} {}
            void set_pwm(uint8_t val) override;
        private:
            size_t idx;
            PWM *owner;
            friend class PWM;
        };

        bool add_hbridge(HBridge &hbr) {
            if(available_pins()<2) return false;
            hbr.index = pins.size();
            hbr.owner = this;
            pins.push_back(hbr.pin1);
            pins.push_back(hbr.pin2);
            data[hbr.index] = PWM_ZERO;
            data[hbr.index+1] = PWM_ZERO;
            return true;
        }

        bool add_pin(Pin &pin) {
            if(available_pins()<1) return false;
            pin.idx = pins.size();
            pin.owner = this;
            pins.push_back(pin.pin);
            data[pin.idx] = PWM_ZERO;
            return true;
        }

        void init() {
            uint32_t pins[] {0xFFFF'FFFF, 0xFFFF'FFFF, 0xFFFF'FFFF, 0xFFFF'FFFF};
            for(size_t i=0; i<this->pins.size(); i++) pins[i] = this->pins[i];
            nrf_pwm_pins_set(pwm, pins);
            nrf_pwm_enable(pwm);
            nrf_pwm_configure(pwm, NRF_PWM_CLK_8MHz, NRF_PWM_MODE_UP, MAX_PWM); // ~20khz

            nrf_pwm_decoder_set(pwm, NRF_PWM_LOAD_INDIVIDUAL, NRF_PWM_STEP_AUTO); //NRF_PWM_STEP_TRIGGERED
            nrf_pwm_seq_ptr_set(pwm, 0, data.data());
            nrf_pwm_seq_cnt_set(pwm, 0, data.size());
            nrf_pwm_seq_refresh_set(pwm, 0, 1);
            nrf_pwm_seq_end_delay_set(pwm, 0, 0);
            nrf_pwm_loop_set(pwm, 0);

            nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
        }

        void wake() { init(); };

        void sleep() {
            nrf_pwm_disable(pwm);
        };

        void set_hbridge(uint8_t val, bool fwd, size_t idx1) {
            // if(val!=0) logf("%d=%c%d\n", idx1, fwd?'+':'-', val);
            size_t idx2 = idx1+1;
            if(val==0) { // coast
                data[idx1] = PWM_ZERO;
                data[idx2] = PWM_ZERO;
            } else
            if(fwd) {
                data[idx1] = PWM_ZERO;
                data[idx2] = add_edge(val);
            } else {
                data[idx1] = add_edge(val);
                data[idx2] = PWM_ZERO;
            }
            nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
        }

        void set_pwm(uint8_t val, size_t idx) {
            uint16_t v = add_edge(val);
            if(data[idx] == v)
                return;
            data[idx] = v;
            nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
        }

    private:
        constexpr static uint16_t add_edge(const uint16_t v) { return v | 0x8000; }
        const static uint16_t PWM_ZERO;

        etl::vector<size_t, NUM_PINS> pins;

        etl::array<uint16_t, NUM_PINS> data;

        size_t available_pins() { return pins.available(); }
    };

    inline const uint16_t PWM::PWM_ZERO = PWM::add_edge(0);

    inline void PWM::HBridge::set_raw(uint8_t val, bool fwd) {
        if(owner!=nullptr)
            owner->set_hbridge(val, this->inverted?!fwd:fwd, index);
    };

    inline void PWM::Pin::set_pwm(uint8_t val) {
        if(owner!=nullptr)
            owner->set_pwm(val, idx);
    };

    /** A simple GPIO output. */
    class Pin: public outputs::BasePin {
    public:
        size_t pin;
        Pin(size_t num) : pin{num} { nrf_gpio_cfg_output(pin); }
        void set(bool v) override {
            nrf_gpio_pin_write(pin, v ? 1 : 0);
        }
    };

};
