#ifndef UART_HPP_
#define UART_HPP_

#include <cstddef>

namespace uart {
    void init();
    void puts(const char* msg);
    void write(const char* msg, size_t len);
}

#endif // UART_HPP_