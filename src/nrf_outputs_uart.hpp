#ifndef NRF_FUNCTIONS_UART_H_
#define NRF_FUNCTIONS_UART_H_

#include <outputs.hpp>
#include <timed_utils.hpp>
#include <functions/base_functions.hpp>

#include "uart.hpp"

namespace nrf {

    struct UartAnalogPin;
    class BaseUartPinOutput {
        virtual void send_pin(UartAnalogPin *);
        friend class UartAnalogPin;
    };

#ifdef NO_UART_PINS
    struct UartAnalogPin: outputs::BaseAnalogPin {
        UartAnalogPin(size_t pin) {}
        void set_pwm(uint8_t val) override { }
        void force_update() {}
    }
#else
    /**
     * A pin that sends its state to UART.
     *
     * The output format is "P1=200\n", where 1 is pin nimber and 200 is PWM value.
     */
    struct UartAnalogPin: outputs::BaseAnalogPin {
        const size_t remote_pin;
        int16_t last_val;

        UartAnalogPin(size_t pin): remote_pin{pin}, last_val{-1} {};

        void set_pwm(uint8_t val) override {
            // don't send duplicates
            if(val==last_val) return;
            last_val = val;
            ticks_left = 0;
            if(owner!=nullptr) owner->send_pin(this);
        }

        void force_update() {
            int16_t v = last_val;
            if(v<0) return;
            last_val = -1;
            set_pwm(last_val);
        }

    private:
        template<size_t MAX_PINS> friend class UartOutputs;
        size_t ticks_left = 0;
        BaseUartPinOutput *owner = nullptr;

        size_t add_data(char* dst, const size_t size) {
            int r = snprintf(dst, size, "P%d=%d", remote_pin, last_val);
            if(r>=0 && r<(int)size) return r;
            return 0;
        }
    };
#endif

#ifdef NO_UART_PINS

    template<size_t MAX_PINS>
    class UartOutputs: public fn::Tickable, public BaseUartPinOutput {
    public:
        constexpr static size_t REFRESH_INTERVAL = ms_to_ticks(1000);

        bool add_pin(UartAnalogPin &pin) { return true;  }
        void tick() override { }
        void send_pin(UartAnalogPin *p) override {  }

    };

#else

    template<size_t MAX_PINS>
    class UartOutputs: public timed::Tickable, public BaseUartPinOutput {
    public:
        constexpr static size_t REFRESH_INTERVAL = ms_to_ticks(1000);

        //UartOutputs() {}

        bool add_pin(UartAnalogPin &pin) {
            if(pins.available()==0) { logs("!OUT OF UART PINS"); return false; }
            pins.push_back(&pin);
            pin.owner = this;
            return true;
        }

        void tick() override {
            if(ticks_left == 0) {
                char msg[7*MAX_PINS];
                size_t pos = 0;
                ticks_left = REFRESH_INTERVAL;
                for(auto& p: pins) {
                    if(p->ticks_left == 0) {
                        //send_pin(p);
                        //logf("printing at %d\n", pos-msg);
                        pos += p->add_data(msg+pos, sizeof(msg)-pos);
                        msg[pos] = ';'; pos++;
                        p->ticks_left = REFRESH_INTERVAL;
                    }
                }
                if(pos!=0) {
                    pos--;
                    msg[pos] = '\n';
                    uart::write(msg, pos+1);
                }
            }
            ticks_left--;
            for(auto& p: pins) {
                if(p->ticks_left > 0) p->ticks_left --;
            }
        }

        void send_pin(UartAnalogPin *p) override {
            char msg[16];
            size_t l = p->add_data(msg, sizeof(msg));
            msg[l] = '\n';
            uart::write(msg, l+1);
            p->ticks_left = REFRESH_INTERVAL;

            // nrf_uarte_tx_buffer_set(uart, (uint8_t*)msg, l);
            // nrf_uarte_task_trigger(uart, NRF_UARTE_TASK_STARTTX);
            // while(!nrf_uarte_event_check(uart, NRF_UARTE_EVENT_ENDTX)) {}
            // nrf_uarte_event_clear(uart, NRF_UARTE_EVENT_ENDTX);
            // p->ticks_left = REFRESH_INTERVAL;
        }

    private:
        etl::vector<UartAnalogPin*, MAX_PINS> pins;
        size_t ticks_left = REFRESH_INTERVAL;
        //NRF_UARTE_Type *uart;
    };

#endif

};

#endif // NRF_FUNCTIONS_UART_H_
