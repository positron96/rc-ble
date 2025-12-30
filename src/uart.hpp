#ifndef UART_HPP_
#define UART_HPP_

#include <etl/string_view.h>

#include <cstddef>

namespace uart {
    // only present on dev module, not on ble-rc board.
    // set to 0xFFFFFFFF to not use pin
    constexpr size_t PIN_TX = 18;
    constexpr size_t PIN_RX = 15;
    constexpr size_t PIN_DISCONNECT = 0xFFFF'FFFF;

    void init(size_t pin_tx = PIN_TX, size_t pix_rx = PIN_DISCONNECT);
    bool is_inited();
    void puts(const char* msg);
    void write(const char* msg, size_t len);

    void puts(const etl::string_view &msg);
    void printf(const char * fmt, ...);
}

#endif // UART_HPP_
