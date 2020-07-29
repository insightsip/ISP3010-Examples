
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "ble_dis.h"

#include "boards.h"
#include "twi_manager.h"
#include "m_batt_meas.h"
#include "m_uwb_range.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#if defined(BOARD_ISP3010_UX_TAG_LP) || defined(BOARD_ISP3010_UX_TAG)
#define DEVICE_NAME                     "ISP3010_TG"                            /**< Name of device. Will be included in the advertising data. */
#elif defined(BOARD_ISP3010_UX_AN)
#define DEVICE_NAME                     "ISP3010_AN"                            /**< Name of device. Will be included in the advertising data. */
#endif

#define MANUFACTURER_NAME               "Insight SiP"                           /**< Manufacturer. Will be passed to Device Information Service. */
#define FW_REVISION                     "1.1.0"                                 /**< Version. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                1000                                     /**< The advertising interval (in units of 0.625 ms). */
#define APP_ADV_DURATION                0                                       /**< The advertising duration in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(1000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, NRF_SDH_BLE_GATT_MAX_MTU_SIZE)  /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                60                                                                   /**< Maximum number of events in the scheduler queue. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[1];                                               /**< Universally unique service identifiers. */


static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_info_t * err_info = (error_info_t*)info;
    NRF_LOG_ERROR(" id = %d, pc = %d, file = %s, line number: %d, error code = %d = %s", \
        id, pc, nrf_log_push((char*)err_info->p_file_name), err_info->line_num, err_info->err_code, nrf_log_push((char*)nrf_strerror_find(err_info->err_code)));  
    NRF_LOG_FINAL_FLUSH();
    nrf_delay_ms(5);

    // Blink red LED
    nrf_gpio_pin_clear(PIN_LED_RED);
    nrf_gpio_cfg_output(PIN_LED_RED);
    nrf_delay_ms(1000);
    nrf_gpio_cfg_input(PIN_LED_RED, NRF_GPIO_PIN_NOPULL);

    // On assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#endif

    app_error_save_and_stop(id, pc, info);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
   APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    ble_uuid_t ble_range_uuid;
    ble_range_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    m_range_ble_uuid_get(&ble_range_uuid.uuid);
    m_adv_uuids[0] = ble_range_uuid;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type                  = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance         = false;
    init.advdata.flags                      = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt     = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids      = m_adv_uuids;
    init.config.ble_adv_fast_enabled        = true;
    init.config.ble_adv_fast_interval       = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout        = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling battery measurement events.

 * @param[in] p_event  batt_meas event.
 */
void batt_meas_event_handler(m_batt_meas_event_t const * p_event)
{
    if (p_event->type == M_BATT_MEAS_EVENT_DATA)
    {
        NRF_LOG_INFO("batt_meas_event_handler : U = %d mV", p_event->voltage_mv);
    }
}


/**@brief Function for initializing the battery measurement module
 */
void batt_meas_module_init(void)
{
    uint32_t err_code;

    batt_meas_param_t batt_meas_param;

    batt_meas_param.pin_batt = PIN_BATT_MEAS;
    batt_meas_param.pin_batt_chg = PIN_WPC_CHG;
    batt_meas_param.voltage_div = BATT_VOLTAGE_DIV;
    batt_meas_param.batt_type = BATT_TYPE;
    batt_meas_init_t batt_meas_init;
    batt_meas_init.evt_handler = batt_meas_event_handler;
    batt_meas_init.batt_meas_param = batt_meas_param;

    err_code = m_batt_meas_init(&batt_meas_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Device Information Service.
 */
static void dis_init(void)
{
    uint32_t err_code;
    ble_dis_init_t dis_init_obj;

    memset (&dis_init_obj, 0, sizeof(ble_dis_init_t));
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, (char*)FW_REVISION);	
    dis_init_obj.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init (&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the uwb range module
 */
void uwb_range_module_init(void)
{
    uint32_t err_code;
    m_uwb_range_init_t uwb_range_init;
    m_uwb_range_param_t uwb_range_param;

    uwb_range_param.pin_wakeup  = PIN_DW1000_WAKEUP;
    uwb_range_param.pin_irq     = PIN_DW1000_IRQ;
    uwb_range_param.pin_rst     = PIN_DW1000_RST;
    uwb_range_param.pin_clk     = PIN_DW1000_CLK;
    uwb_range_param.pin_mosi    = PIN_DW1000_MOSI;
    uwb_range_param.pin_miso    = PIN_DW1000_MISO;
    uwb_range_param.pin_cs      = PIN_DW1000_CS;
    uwb_range_param.pin_reset   = PIN_DW1000_RST;
#if defined(BOARD_ISP3010_UX_TAG_LP) || defined(BOARD_ISP3010_UX_TAG)
    uwb_range_param.role        = TWR_INITIATOR;
#elif defined(BOARD_ISP3010_UX_AN)
    uwb_range_param.role        = TWR_RESPONDER;
#endif
    uwb_range_init.uwb_range_param = uwb_range_param;

    err_code = m_range_init(&uwb_range_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds = false;

    // Blink yellow LED
    nrf_gpio_pin_clear(PIN_LED_YELLOW);
    nrf_gpio_cfg_output(PIN_LED_YELLOW);
    nrf_delay_ms(500);
    nrf_gpio_cfg_input(PIN_LED_YELLOW, NRF_GPIO_PIN_NOPULL);

    // Initialize.
    log_init();
    twi_manager_init(APP_IRQ_PRIORITY_LOW);
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    peer_manager_init();

    dis_init();
#if defined(BOARD_ISP3010_UX_TAG_LP) || defined(BOARD_ISP3010_UX_TAG)
    batt_meas_module_init();
#endif
    uwb_range_module_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("ISP3010 TWR Demo started.");
    application_timers_start();
    advertising_start(erase_bonds);
    m_range_start();


    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        idle_state_handle();
    }
}

