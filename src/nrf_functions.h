#pragma once

#include <nrf_timer.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrf_ppi.h>
#include <nrf_pwm.h>

#include <etl/vector.h>
#include <etl/array.h>

#include "functions.h"

#include "log.h"


namespace nrf {

    struct ServoTimer;

    struct Servo: fn::BaseServo {

        size_t pin;
        ServoTimer *owner;
        size_t index;

        Servo(size_t pin): pin{pin} {
        }

        void set_us(uint16_t us) override;

    };

    NRF_TIMER_Type *servo_timer = NRF_TIMER1;
    constexpr IRQn_Type servo_irq = TIMER1_IRQn;

    struct ServoTimer: fn::Wakeable {
        static constexpr size_t SERVO_PERIOD = 20000;
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

            nrf_timer_cc_write(servo_timer, NRF_TIMER_CC_CHANNEL3, SERVO_PERIOD);

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

    class PWM;
    struct HBridge: fn::BaseHbridge {
        size_t pin1, pin2;
        PWM *owner;
        size_t index;
        HBridge(size_t p1, size_t p2): pin1{p1}, pin2{p2} {}

        void set(uint8_t val, bool fwd) override;
    };

    struct PwmPin: fn::BaseAnalogPin {
        size_t pin;
        size_t idx;
        PWM *owner;
        PwmPin(size_t p): pin{p} {}
        void set_pwm(uint8_t val) override;
    };

    NRF_PWM_Type *pwm = NRF_PWM0;
    constexpr uint16_t add_edge(const uint16_t v) { return v | 0x8000; }
    constexpr uint16_t PWM_ZERO = add_edge(0);

    class PWM: public fn::Wakeable {
    public:
        static constexpr size_t NUM_PINS = NRF_PWM_CHANNEL_COUNT;
        static constexpr size_t MAX_PWM = 255;
        etl::array<uint16_t, NUM_PINS> data;

        bool add_hbridge(HBridge &hbr) {
            if(available_pins()<2) return false;
            hbr.index = pins.size();
            hbr.owner = this;
            pins.push_back(hbr.pin1);
            pins.push_back(hbr.pin2);
            return true;
        }

        bool add_pin(PwmPin &pin) {
            if(available_pins()<1) return false;
            pin.idx = pins.size();
            pin.owner = this;
            pins.push_back(pin.pin);
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

        etl::vector<size_t, 4> pins;

        size_t available_pins() { return NUM_PINS - pins.size(); }
    };

    class Pin: public fn::BasePin {
    public:
        size_t pin;
        Pin(size_t num) : pin{num} { nrf_gpio_cfg_output(pin); }
        void set(bool v) override {
            nrf_gpio_pin_write(pin, v ? 1 : 0);
        }
    };

};

void nrf::Servo::set_us(uint16_t us) {
    owner->set_us(index, us);
};

void nrf::HBridge::set(uint8_t val, bool fwd) {
    owner->set_hbridge(val, this->inverted?!fwd:fwd, index);
};

void nrf::PwmPin::set_pwm(uint8_t val) {
    owner->set_pwm(val, idx);
};
