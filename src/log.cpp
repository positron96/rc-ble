#include "log.h"

#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>

#define LOG_UART  1
#define LOG_RTT   2
#define LOG_BLE   3
#define LOG_NONE  0

#ifndef LOG_TARGET
  #define LOG_TARGET  LOG_UART
#endif

void _logln(const char* msg) {
    logs(msg);
    logs("\n");
}

#if (LOG_TARGET == LOG_UART)

    #include "uart.hpp"

    constexpr size_t bufsize = 128;
    char txbuf[bufsize];

    void log_init() {
        if(!uart::is_inited()) {
            uart::init();
        }
    }

    void logs(const char* msg) {
        if(!uart::is_inited()) return;
        size_t l = strlen(msg);
        if(l>bufsize) l = bufsize;
        if(msg != txbuf) {
            memcpy(txbuf, msg, l);
        }
        uart::write(txbuf, l);
    }

    void logln(const char* msg) {
        logs("!");
        logs(msg);
        logs("\n");
    }

    /** Not reentrant! */
    void vlogf(const char * fmt, va_list args) {
        vsnprintf(txbuf, bufsize, fmt, args);
        logs("!");
        logs(txbuf);
    }

    void logf(const char * fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vlogf(fmt, args);
        va_end(args);
    }


#elif (LOG_TARGET == LOG_RTT)

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

    constexpr size_t bufsize = 128;
    char txbuf[bufsize];

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

#elif (LOG_TARGET == LOG_BLE)

    #include "ble_sd.hpp"

    void log_init() {
    }

    //extern void send_ble(const char* msg, const size_t len);

    void logs(const char* msg) {
        ble::transmit(msg, strlen(msg));
    }

    void logln(const char* msg) {
        _logln(msg);
    }

    constexpr size_t bufsize = 128;
    char txbuf[bufsize];

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

#else

    void log_init() {}
    void logs(const char* msg) {}
    void logln(const char* msg) {}
    void logf(const char * fmt, ...) {}

#endif
