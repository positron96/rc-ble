#ifndef NRF_FUNCTIONS_UART_H_
#define NRF_FUNCTIONS_UART_H_

#include <outputs.hpp>

#include <nrf_uarte.h>

namespace nrf {

    /**
     * A pin that sends its state to UART.
     *
     * The output format is "P1=200\n", where 1 is pin nimber and 200 is PWM value.
     */
    struct UartAnalogPin: outputs::BaseAnalogPin {
        size_t remote_pin;
        NRF_UARTE_Type *uart;
        int16_t last_val;

        UartAnalogPin(size_t pin, NRF_UARTE_Type *uart): remote_pin{pin}, uart{uart}, last_val{-1} {};

        void set_pwm(uint8_t val) override {
            // don't send duplicates
            if(val==last_val) return;
            send_pin(uart, remote_pin, val);
        }

        void force_update() {
            int16_t v = last_val;
            if(v<0) return;
            last_val = -1;
            set_pwm(last_val);
        }

        static void send_pin(NRF_UARTE_Type *uart, size_t pin, uint8_t val) {
            char msg[16];
            snprintf(msg, sizeof(msg), "P%d=%d\n", pin, val);
            size_t l = strlen(msg);
            nrf_uarte_tx_buffer_set(uart, (uint8_t*)msg, l);
            nrf_uarte_task_trigger(uart, NRF_UARTE_TASK_STARTTX);
            while(!nrf_uarte_event_check(uart, NRF_UARTE_EVENT_ENDTX)) {}
            nrf_uarte_event_clear(uart, NRF_UARTE_EVENT_ENDTX);
        }

    };

};

#endif // NRF_FUNCTIONS_UART_H_