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

    /** Sends data to UART using EasyDMA.
     * Make sure data is in RAM!
     */
    void write_ram(const char* msg, size_t len) {
        nrf_uarte_tx_buffer_set(uart_inst, (uint8_t*)msg, len);
        nrf_uarte_task_trigger(uart_inst, NRF_UARTE_TASK_STARTTX);
        while(!nrf_uarte_event_check(uart_inst, NRF_UARTE_EVENT_ENDTX)) {}
        nrf_uarte_event_clear(uart_inst, NRF_UARTE_EVENT_ENDTX);
    }

    void write_cpy(const char* msg, size_t len) {
        constexpr size_t bufsize = 128;
        char buf[bufsize];
        if (len>bufsize) len = bufsize;
        memcpy(buf, msg, len);
        write_ram(buf, len);
    }

    void write(const char* msg, size_t len) {
        const uintptr_t  p = (const uintptr_t)msg;
        if(p>=0x2000000 && p<0x3FFFFFFF) { // full 0.5GB of SRAM
            write_ram(msg, len);
        } else {
            write_cpy(msg, len);
        }
    }

    void puts(const char* msg) {
        size_t l = strlen(msg);
        write(msg, l);
    }

    void puts(const etl::string_view &msg) {
        constexpr size_t bufsize = 64;
        char buf[bufsize];
        snprintf(buf, sizeof(buf), "%.*s", msg.length(), msg.data());
        size_t l = strlen(buf);
        write(buf, l);
    }

    void vprintf(const char * fmt, va_list args) {
        constexpr size_t bufsize = 64;
        char buf[bufsize];
        vsnprintf(buf, bufsize, fmt, args);
        write(buf, strlen(buf));
    }

    void printf(const char * fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }

}