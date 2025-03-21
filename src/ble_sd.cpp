#include "ble_sd.hpp"

#include "nordic_common.h"
#include "nrf.h"

#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"

#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "ble_nus.h"
#include "ble_bas.h"
#include "app_timer.h"
#include <app_util_platform.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>

#include "storage.hpp"
#include "line_processor.h"
#include "log.h"

// NORDIC UART service UUID
// #define NRF_UART_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
// #define NRF_UART_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
// #define NRF_UART_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// #define NRF_UART_CHAR_SIZE 20
#define NRF_UART_RX_BUFFER_SIZE 128


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

// fast advertising: every 50ms for 1min
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define APP_ADV_DURATION                MSEC_TO_UNITS(60000, UNIT_10_MS)
// slow advertising: every 1s for 3 min.
// 3min does not really matter as it will be restarted infinitely.
#define APP_ADV_INTERVAL2               MSEC_TO_UNITS(1000, UNIT_0_625_MS)
#define APP_ADV_DURATION2               MSEC_TO_UNITS(180000, UNIT_10_MS)

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_BAS_DEF(m_bas);
NRF_BLE_GATT_DEF(m_gatt);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t   m_conn_handle = BLE_CONN_HANDLE_INVALID;
 /** Maximum length of data (in bytes) that can be transmitted to the peer. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;
static ble_uuid_t m_adv_uuids[]          = {
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

// void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
//     logf("!! Err %d %s:%d", error_code, p_file_name, line_num);
//     while(1){ }
// }

extern void process_str(etl::string_view);

namespace ble {


size_t get_connected_clients_count() {
    return m_conn_handle == BLE_CONN_HANDLE_INVALID ? 0 : 1;
}


using BLELineProcessor = line_processor::LineProcessor<NRF_UART_RX_BUFFER_SIZE>;
BLELineProcessor rx(line_processor::callback_t::create<process_str>());

void set_bas(uint8_t battery_level) {
    ret_code_t err_code;
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

void send_ble(const char* msg, const size_t len) {
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        return;
    }
    uint16_t l = len;
    uint8_t* d = (uint8_t*)msg;
    uint32_t err_code = ble_nus_data_send(&m_nus, d, &l, m_conn_handle);
    if ((err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_NOT_FOUND))
    {
        APP_ERROR_CHECK(err_code);
    }
}

static void nus_data_handler(ble_nus_evt_t * p_evt) {

    if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint16_t i = 0; i < p_evt->params.rx_data.length; i++) {
            char ch = (char)p_evt->params.rx_data.p_data[i];
            if(ch!=0) rx.add(ch);
        }
    }
}


static void services_init(void)  {
    uint32_t err_code;

    // Initialize NUS.
    ble_nus_init_t nus_init;
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize BAS
    ble_bas_init_t bas_init_obj;
    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 0;

    bas_init_obj.bl_rd_sec        = SEC_OPEN;
    bas_init_obj.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init_obj.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)  {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}


static void conn_params_init(void) {
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    uint32_t err_code;

    NRF_LOG_INFO("Adv event: %d", (int)ble_adv_evt);

    switch (ble_adv_evt)  {
        case BLE_ADV_EVT_FAST:
            // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            // APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}


static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)  {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            // APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
                NRF_LOG_DEBUG("PHY update request.");
                const ble_gap_phys_t phys = {
                    .tx_phys = BLE_GAP_PHY_AUTO,
                    .rx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
                break;
            }

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static void stack_init(void) {
    ret_code_t err_code;

    if(!nrf_sdh_is_enabled()) {
        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);
    }

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void update_dev_name() {
    const auto devname_opt = storage::get_dev_name();
    const char* devname;
    if(devname_opt) {
        devname = devname_opt.value().data();
    } else {
        devname = storage::DEFAULT_DEVNAME;
    }
    //logf("Dev name: '%s'\n", devname);

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    uint32_t  err_code;
    err_code = sd_ble_gap_device_name_set(
        &sec_mode,
        (const uint8_t *) devname,
        strlen(devname));
    APP_ERROR_CHECK(err_code);

    if(m_advertising.initialized) {
        // recreate advertising data with new name
        logf("Updating adv data\n");
        ble_advdata_t advdata, srdata;
        memset(&advdata, 0, sizeof(advdata));
        memset(&srdata, 0, sizeof(srdata));

        advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance = false;
        advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

        srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        srdata.uuids_complete.p_uuids  = m_adv_uuids;

        err_code = ble_advertising_advdata_update(&m_advertising, &advdata, &srdata);
        if(err_code!=NRF_SUCCESS) {
            logf("Update adv failed: %d\n", err_code);
        }
    }
}

static void gap_params_init() {
    uint32_t  err_code;
    ble_gap_conn_params_t   gap_conn_params;

    update_dev_name();

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


void gatt_init(void) {
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}



static void advertising_init(void) {
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.config.ble_adv_slow_enabled = true;
    init.config.ble_adv_slow_interval = APP_ADV_INTERVAL2;
    init.config.ble_adv_slow_timeout = APP_ADV_DURATION2;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);

    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


void start() {
    ret_code_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Starting BLE");
    logln("Starting BLE/mylog");

    stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("BLE started");
}

}

extern "C" {

    static void on_error(void) {
        #ifdef DEBUG
            NRF_BREAKPOINT_COND;
        #endif
        NVIC_SystemReset();
    }

    void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
        logf("error err_code:%d %s:%d\n", error_code, p_file_name, line_num);
        on_error();
    }

    void app_error_handler_bare(uint32_t error_code) {
        logf("error_bare: 0x%08x!\n", error_code);
        on_error();
    }

    void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
        logf("fault 0x%08x, pc: 0x%08x, info: 0x%08x\n", id, pc, info);
        on_error();
    }

}
