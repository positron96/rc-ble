#include "uart.hpp"

#include <nrf_uarte.h>

#include <cstring>
#include <cstdarg>
#include <cstdio>


namespace uart {

    NRF_UARTE_Type *uart_inst = NRF_UARTE0;

    void init(size_t pin_tx, size_t pin_rx) {
        nrf_uarte_txrx_pins_set(uart_inst, pin_tx, pin_rx);
        nrf_uarte_baudrate_set(uart_inst, NRF_UARTE_BAUDRATE_115200);
        nrf_uarte_configure(uart_inst, NRF_UARTE_PARITY_EXCLUDED, NRF_UARTE_HWFC_DISABLED);
        nrf_uarte_enable(uart_inst);
    }

    bool is_inited() {
        return uart_inst->ENABLE == UARTE_ENABLE_ENABLE_Enabled;
    }

    void write(const char* msg, size_t len) {
        nrf_uarte_tx_buffer_set(uart_inst, (uint8_t*)msg, len);
        nrf_uarte_task_trigger(uart_inst, NRF_UARTE_TASK_STARTTX);
        while(!nrf_uarte_event_check(uart_inst, NRF_UARTE_EVENT_ENDTX)) {}
        nrf_uarte_event_clear(uart_inst, NRF_UARTE_EVENT_ENDTX);
    }

    void puts(const char* msg) {
        size_t l = strlen(msg);
        // if(nrf_uarte_event_check(log_uart, NRF_UARTE_EVENT_TXSTARTED)) {
        //     nrf_uarte_event_clear(log_uart, NRF_UARTE_EVENT_TXSTARTED);
        // }
        write(msg, l);
    }

}