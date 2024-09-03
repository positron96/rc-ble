#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <nrf_bootloader_info.h>
#include <nrf_pwr_mgmt.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>

__attribute__((noreturn))
inline void reboot_to_bootloader() {
    NRF_LOG_INFO("Rebooting to bootloader");
    NRF_LOG_FLUSH();
    uint32_t err_code;
    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    APP_ERROR_CHECK(err_code);
    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    APP_ERROR_CHECK(err_code);
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
    __builtin_unreachable();
}

#endif // BOOTLOADER_H_