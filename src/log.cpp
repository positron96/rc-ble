#include "log.h"

#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>

#include <Arduino.h>


void logs(const char* msg) {
    puts(msg);
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
