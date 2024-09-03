#include "log.h"

#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>


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

#else

void logs(const char* msg) {}
void logln(const char* msg) {}
void vlogf(const char * fmt, va_list args) {}
void logf(const char * fmt, ...) {}

#endif
