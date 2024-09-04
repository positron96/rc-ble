#include "log.h"

#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>

#define LOG_RTT_
#define LOG_UART

#ifdef LOG_UART

#include <nrf_uarte.h>
#include <app_uart.h>

NRF_UARTE_Type *log_uart = NRF_UARTE0;

constexpr size_t bufsize = 128;
char txbuf[bufsize];

void log_init() {

    nrf_uarte_txrx_pins_set(log_uart, 18, 15);//15, 18);
    nrf_uarte_baudrate_set(log_uart, NRF_UARTE_BAUDRATE_115200);
    nrf_uarte_configure(log_uart, NRF_UARTE_PARITY_EXCLUDED, NRF_UARTE_HWFC_DISABLED);
    nrf_uarte_enable(log_uart);

}

void logs(const char* msg) {
    size_t l = strlen(msg);
    if(l>bufsize) l = bufsize;
    if(msg != txbuf) {
        memcpy(txbuf, msg, l);
    }
    if(nrf_uarte_event_check(log_uart, NRF_UARTE_EVENT_TXSTARTED)) {
        nrf_uarte_event_clear(log_uart, NRF_UARTE_EVENT_TXSTARTED);
        while(!nrf_uarte_event_check(log_uart, NRF_UARTE_EVENT_ENDTX)) {}
    }
    nrf_uarte_event_clear(log_uart, NRF_UARTE_EVENT_ENDTX);
    nrf_uarte_tx_buffer_set(log_uart, (uint8_t*)txbuf, l);
    nrf_uarte_task_trigger(log_uart, NRF_UARTE_TASK_STARTTX);
}

void logln(const char* msg) {
    logs(msg);
    logs("\n");
}

/** Not reentrant! */
void vlogf(const char * fmt, va_list args) {
    vsnprintf(txbuf, bufsize, fmt, args);
    logs(txbuf);
}

void logf(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlogf(fmt, args);
    va_end(args);
}


// void logs(const char* msg) {
//     puts(msg);

// }

// void logln(const char* msg) {
//     printf("%s", msg);
// }

// /** Not reentrant! */
// void vlogf(const char * fmt, va_list args) {
//     vprintf(fmt, args);
// }

// void logf(const char * fmt, ...) {
//     va_list args;
//     va_start(args, fmt);
//     vlogf(fmt, args);
//     va_end(args);
// }

#elif defined(LOG_RTT)

#include "SEGGER_RTT.h"

void log_init() {
    SEGGER_RTT_Init();
}

void logs(const char* msg) {
    SEGGER_RTT_WriteString(0, msg);
}

void logln(const char* msg) {
    SEGGER_RTT_WriteString(0, msg);
    SEGGER_RTT_PutChar(0, '\n');
}

extern "C" int SEGGER_RTT_vprintf(unsigned , const char *, va_list *);

void logf(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
}

#else

void logs(const char* msg) {}
void logln(const char* msg) {}
void vlogf(const char * fmt, va_list args) {}
void logf(const char * fmt, ...) {}

#endif
