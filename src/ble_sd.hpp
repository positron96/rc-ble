#ifndef BLE_SD_HPP_
#define BLE_SD_HPP_

#include <stdint.h>

#include "nrf_sdh.h"


namespace ble {
    void start();

    size_t get_connected_clients_count();

    void set_bas(uint8_t);

    void update_dev_name();

    void transmit(const char* msg, const size_t len);
}

#endif // BLE_SD_HPP_