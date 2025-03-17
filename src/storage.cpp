#include "storage.h"

#include <cstring>

#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "app_util.h"
#include "fds.h"


#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#else
  #error "Designed to be used with softdevice. see example flash_fstorage"
#endif

#include "log.h"

namespace storage {

    struct configuration_t {
        uint32_t boot_count;
        char     device_name[16];
        bool     config1_on;
        bool     config2_on;
    };

    static configuration_t m_dummy_cfg =
    {
        .boot_count  = 0x0,
        {.device_name = "device"},  //https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55227
        .config1_on  = false,
        .config2_on  = true,
    };

    #define CONFIG_FILE     (0x8010)
    #define CONFIG_REC_KEY  (0x7010)

    /* A record containing dummy configuration data. */
    static fds_record_t const m_dummy_record =
    {
        .file_id           = CONFIG_FILE,
        .key               = CONFIG_REC_KEY,
        .data = {
            .p_data       = &m_dummy_cfg,
            /* The length of a record is always expressed in 4-byte units (words). */
            .length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
        },
    };

    static bool volatile m_fds_initialized;

    extern "C" {

        static void fds_evt_handler(fds_evt_t const * p_evt) {
            if (p_evt->result == NRF_SUCCESS) {
                logf("fds event %d received (NRF_SUCCESS)\n", p_evt->id);
            } else {
                logf("fds event %d received (err %d)\n", p_evt->id, p_evt->result);
            }

            switch (p_evt->id) {
                case FDS_EVT_INIT:
                   if (p_evt->result == NRF_SUCCESS) { m_fds_initialized = true; }
                    break;

                case FDS_EVT_WRITE:
                case FDS_EVT_UPDATE: {
                    if (p_evt->result == NRF_SUCCESS) {
                        logf("Record ID:\t0x%04x\n",  p_evt->write.record_id);
                        logf("File ID:\t0x%04x\n",    p_evt->write.file_id);
                        logf("Record key:\t0x%04x\n", p_evt->write.record_key);
                    }
                } break;

                case FDS_EVT_DEL_FILE:
                case FDS_EVT_DEL_RECORD: {
                    if (p_evt->result == NRF_SUCCESS) {
                        logf("Record ID:\t0x%04x\n",  p_evt->del.record_id);
                        logf("File ID:\t0x%04x\n",    p_evt->del.file_id);
                        logf("Record key:\t0x%04x\n", p_evt->del.record_key);
                    }
                } break;

                default:
                    break;
            }
        }

    }


    void init() {

        ret_code_t rc;

        //ble_stack_init();

        /* Register first to receive an event when initialization is complete. */
        rc = fds_register(fds_evt_handler);

        rc = fds_init();
        APP_ERROR_CHECK(rc);
        while (!m_fds_initialized)  {
            #ifdef SOFTDEVICE_PRESENT
                (void) sd_app_evt_wait();
            #else
                __WFE();
            #endif
        }

        logln("Reading flash usage statistics...");

        fds_stat_t stat = {0};

        rc = fds_stat(&stat);
        APP_ERROR_CHECK(rc);

        logf("Found %d valid records\n", stat.valid_records);
        logf("Found %d dirty records (ready to be garbage collected)\n", stat.dirty_records);

        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};

        rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

        if (rc == NRF_SUCCESS)  {
            /* A config file is in flash. Let's update it. */
            fds_flash_record_t config = {0};

            /* Open the record and read its contents. */
            rc = fds_record_open(&desc, &config);
            APP_ERROR_CHECK(rc);

            /* Copy the configuration from flash into m_dummy_cfg. */
            memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

            logf("Config file found, updating boot count to %d\n", m_dummy_cfg.boot_count);

            /* Update boot count. */
            m_dummy_cfg.boot_count++;

            /* Close the record when done reading. */
            rc = fds_record_close(&desc);
            APP_ERROR_CHECK(rc);

            /* Write the updated record to flash. */
            // rc = fds_record_update(&desc, &m_dummy_record);
            // if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH)) {
            //     logln("No space in flash, delete some records to update the config file");
            // } else {
            //     APP_ERROR_CHECK(rc);
            // }
        } else {
            /* System config not found; write a new one. */
            logln("Creating config file...\n");

            // strncpy(m_dummy_cfg.device_name, "device", sizeof(m_dummy_cfg.device_name));

            rc = fds_record_write(&desc, &m_dummy_record);
            if (rc == FDS_ERR_NO_SPACE_IN_FLASH)  {
                logln("No space in flash, delete some records to update the config file");
            } else {
                APP_ERROR_CHECK(rc);
            }
        }

    }
}
