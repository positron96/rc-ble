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

    using devname_t = char[DEVNAME_LEN];

    constexpr uint16_t CONFIG_FILE_ID = 0x1;
    constexpr uint16_t REC_DEVNAME_KEY = 0x1;

    devname_t m_devname;

    /* A record containing dummy configuration data. */
    static fds_record_t const m_devname_rec{
        .file_id           = CONFIG_FILE_ID,
        .key               = REC_DEVNAME_KEY,
        .data = {
            .p_data       = &m_devname,
            .length_words = (sizeof(m_devname) + 3) / sizeof(uint32_t),
        },
    };

    static volatile int32_t event_result;

    static void fds_evt_handler(fds_evt_t const * evt) {
        if (evt->result == NRF_SUCCESS) {
            logf("fds event %d = NRF_SUCCESS\n", evt->id);
        } else {
            logf("fds event %d err %d\n", evt->id, evt->result);
        }

        event_result = evt->result;

        switch (evt->id) {
            case FDS_EVT_INIT:
                break;
            case FDS_EVT_WRITE:
            case FDS_EVT_UPDATE: {
                if (evt->result == NRF_SUCCESS) {
                    logf("Record ID:\t0x%04x\n",  evt->write.record_id);
                    logf("File ID:\t0x%04x\n",    evt->write.file_id);
                    logf("Record key:\t0x%04x\n", evt->write.record_key);
                }
            } break;

            case FDS_EVT_DEL_FILE:
            case FDS_EVT_DEL_RECORD: {
                if (evt->result == NRF_SUCCESS) {
                    logf("Record ID:\t0x%04x\n",  evt->del.record_id);
                    logf("File ID:\t0x%04x\n",    evt->del.file_id);
                    logf("Record key:\t0x%04x\n", evt->del.record_key);
                }
            } break;

            default:  break;
        }
    }


    void init() {
        ret_code_t rc;

        rc = fds_register(fds_evt_handler);

        event_result = -1;
        rc = fds_init();
        APP_ERROR_CHECK(rc);
        while (event_result==-1)  { (void) sd_app_evt_wait(); }

        logln("Reading flash usage statistics...");
        fds_stat_t stat = {0};
        rc = fds_stat(&stat);
        APP_ERROR_CHECK(rc);
        logf("Stats: fsok=%c\n Avail=%dpg\n Valid=%drec\n Dirty=%drec\n",
            stat.corruption?'N':'Y',
            stat.pages_available, stat.valid_records, stat.dirty_records
        );
        logf(" Res: %dw\n Used: %dw\n Freeable: %dw\n",
            stat.words_reserved, stat.words_used, stat.freeable_words);

    }

    bool clean() {
        event_result = -1;
        ret_code_t rc = fds_gc();
        if(rc==NRF_SUCCESS) while (event_result==-1)  { (void) sd_app_evt_wait(); }
        return rc == NRF_SUCCESS && event_result == NRF_SUCCESS;
    }

    bool wipe() {
        fds_record_desc_t desc{0};
        fds_find_token_t tok{0};
        ret_code_t rc;
        while (fds_record_iterate(&desc, &tok) != FDS_ERR_NOT_FOUND) {
            rc = fds_record_delete(&desc);
            logf("deleting %d = %d\n", desc.record_id, rc);
        }
        return rc == NRF_SUCCESS;
    }


    etl::optional<etl::string_view> get_dev_name() {
        fds_record_desc_t desc = {0};
        fds_find_token_t tok  = {0};

        ret_code_t rc = fds_record_find(CONFIG_FILE_ID, REC_DEVNAME_KEY, &desc, &tok);

        if(rc == FDS_ERR_NOT_FOUND) {
            return etl::nullopt;
        }
        if (rc != NRF_SUCCESS)  {
            logf("fds_record_find failed: %d\n", rc);
            return etl::nullopt;
        }

        fds_flash_record_t config = {0};
        rc = fds_record_open(&desc, &config);
        if (rc != NRF_SUCCESS) {
            logf("fds_record_open failed: %d\n", rc);
            return etl::nullopt;
        }

        memcpy(&m_devname, config.p_data, sizeof(devname_t));
        m_devname[DEVNAME_LEN-1] = 0; // ensure it's zero-terminated in case it got corrupted

        rc = fds_record_close(&desc);
        if (rc != NRF_SUCCESS) logf("fds_record_close failed: %d\n", rc);

        return etl::string_view{m_devname};
    }

    bool set_dev_name(const etl::string_view name) {

        if(name.length() < DEVNAME_LEN-1) {
            memset(m_devname, 0, sizeof(devname_t));
            memcpy(m_devname, name.data(), name.length());
        } else {
            memcpy(m_devname, name.data(), DEVNAME_LEN-1);
            m_devname[DEVNAME_LEN-1] = 0;
        }

        fds_record_desc_t desc{0};
        fds_find_token_t tok{0};
        ret_code_t rc = fds_record_find(CONFIG_FILE_ID, REC_DEVNAME_KEY, &desc, &tok);

        if (rc == NRF_SUCCESS)  {
            rc = fds_record_update(&desc, &m_devname_rec);
            if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH)) {
                logln("No space in flash");
                return false;
            } else {
                logf("fds_record_update failed: %d\n", rc);
                return false;
            }
        } else {
            logln("Creating config file...\n");

            rc = fds_record_write(&desc, &m_devname_rec);
            if (rc == FDS_ERR_NO_SPACE_IN_FLASH)  {
                logln("No space in flash");
                return false;
            } else {
                logf("fds_record_write failed: %d\n", rc);
                return false;
            }
        }

    }
}
