#include "storage.h"

#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"

#include "app_timer.h"
#include "app_util.h"
#include "nrf_fstorage.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
  #error "Designed to be used with softdevice. see example flash_fstorage"
#endif

#include "log.h"

namespace storage {

    constexpr static uint32_t START_ADDR = 0x3e000;
    constexpr static uint32_t END_ADDR = 0x3ffff;
    constexpr static uint32_t SIZE = END_ADDR - START_ADDR;

    extern "C" {

        static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

        NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
        {
            /* Set a handler for fstorage events. */
            .evt_handler = fstorage_evt_handler,

            /* These below are the boundaries of the flash space assigned to this instance of fstorage.
            * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
            * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
            * last page of flash available to write data. */
            .start_addr = START_ADDR,
            .end_addr   = END_ADDR,
        };

    }


    /**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
     *          can be used to write user data.
     */
    static uint32_t nrf5_flash_end_addr_get()
    {
        uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
        uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
        uint32_t const code_sz         = NRF_FICR->CODESIZE;

        return (bootloader_addr != 0xFFFFFFFF ?
                bootloader_addr : (code_sz * page_sz));
    }


    extern "C" {
        static  void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt) {
            if (p_evt->result != NRF_SUCCESS) {
                logln("--> Event received: ERROR while executing an fstorage operation");
                return;
            }

            switch (p_evt->id) {
                case NRF_FSTORAGE_EVT_WRITE_RESULT: {
                    logf("--> Event received: wrote %d bytes at address 0x%x\n",
                                p_evt->len, p_evt->addr);
                } break;

                case NRF_FSTORAGE_EVT_ERASE_RESULT: {
                    logf("--> Event received: erased %d page from address 0x%x\n",
                                p_evt->len, p_evt->addr);
                } break;

                default: break;
            }
        }
    }


    static void print_flash_info(nrf_fstorage_t * p_fstorage) {
        logln("========| flash info |========");
        logf("erase unit: \t%d bytes\n",      p_fstorage->p_flash_info->erase_unit);
        logf("program unit: \t%d bytes\n",    p_fstorage->p_flash_info->program_unit);
        logf("==============================");
    }

    void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)  {
        /* While fstorage is busy, sleep and wait for an event. */
        while (nrf_fstorage_is_busy(p_fstorage))  {
            //power_manage();
        }
    }


    void init() {

        ret_code_t rc;

        uint32_t end_flash = nrf5_flash_end_addr_get();
        logf("Flash end is at %X\n", end_flash);

        static uint32_t m_data = 0xBADC0FFE;

        nrf_fstorage_api_t * p_fs_api;

        p_fs_api = &nrf_fstorage_sd;
        rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
        APP_ERROR_CHECK(rc);

        print_flash_info(&fstorage);

        /* Let's write to flash. */
        logf("Writing \"%x\" to flash.", m_data);
        rc = nrf_fstorage_write(&fstorage, 0x3e000, &m_data, sizeof(m_data), NULL);
        APP_ERROR_CHECK(rc);

        wait_for_flash_ready(&fstorage);
        logln("Done.");

        rc = nrf_fstorage_write(&fstorage, 0x3e100, &m_data, sizeof(m_data), NULL);
        APP_ERROR_CHECK(rc);

        wait_for_flash_ready(&fstorage);
        logln("Done.");
    }
}
