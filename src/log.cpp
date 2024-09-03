#include "log.h"

#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>

#define LOG_RTT
#define LOG_UART_

#ifdef LOG_UART

void logs(const char* msg) {
    puts(msg);

}

void logln(const char* msg) {
    printf("%s", msg);
}

/** Not reentrant! */
void vlogf(const char * fmt, va_list args) {
    vprintf(fmt, args);
}

void logf(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlogf(fmt, args);
    va_end(args);
}

#elif defined(LOG_RTT)

#include "SEGGER_RTT.h"


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
