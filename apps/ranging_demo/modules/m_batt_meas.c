 /******************************************************************************
 * @file    m_batt_meas.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    08-07-2019
 * @brief   Battery measurement module implementation file.
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <stdint.h>
#include <string.h>


#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_saadc.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble_bas.h"

#include "boards.h"

#include "m_batt_meas.h"


#define SAMPLES_IN_BUFFER           10
#define INVALID_BATTERY_LEVEL       (0xFF)                                                  /**<  Invalid/default battery level. */


BLE_BAS_DEF(m_bas);                                                                         /**< Structure used to identify the battery service. */
APP_TIMER_DEF(batt_meas_app_timer_id);                                                      /**< Timer for periodic battery measurement. */
static nrf_saadc_value_t            m_buffer_pool[2][SAMPLES_IN_BUFFER];
static m_batt_meas_event_handler_t  m_evt_handler;                                          /**< Battery event handler function pointer. */
static batt_meas_param_t            m_batt_meas_param;                                      /**< Battery parameters. */
static uint8_t                      m_initial_batt_level_percent = INVALID_BATTERY_LEVEL;   /**< Initial battery level in percent. */


static uint8_t lipo_voltage_to_battery_level(uint16_t voltage_mv)
{
    uint8_t battery_level;

    if (voltage_mv >= 4200)
    {
        battery_level = 100;
    }
    else if (voltage_mv > 4050)
    {
        battery_level = 100 - ((4200 - voltage_mv) * 41) / 100;
    }
    else if (voltage_mv > 3830)
    {
        battery_level = 42 - ((4060 - voltage_mv) * 17) / 160;
    }
    else if (voltage_mv > 3410)
    {
        battery_level = 18 - ((3836 - voltage_mv) * 8) / 300;
    }
    else if (voltage_mv > 3000)
    {
        battery_level = 6 - ((3416 - voltage_mv) * 4) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}


/** @brief Function for converting digital pin number to analog pin number
 *
 * @param[in]  digital_pin     Digital pin number
 *
 * @return Analog pin number
 */
static nrf_saadc_input_t digital_to_analog_pin (uint32_t digital_pin)
{
    nrf_saadc_input_t analog_pin;

    switch(digital_pin)
    {
        case 2:
            analog_pin = NRF_SAADC_INPUT_AIN0;
            break;
        case 3: 
            analog_pin = NRF_SAADC_INPUT_AIN1;
            break;
        case 4: 
            analog_pin = NRF_SAADC_INPUT_AIN2;
            break;
        case 5: 
            analog_pin = NRF_SAADC_INPUT_AIN3;
            break;
        case 28: 
            analog_pin = NRF_SAADC_INPUT_AIN4;
            break;	
        case 29: 
            analog_pin = NRF_SAADC_INPUT_AIN5;
            break;
        case 30: 
            analog_pin = NRF_SAADC_INPUT_AIN6;
            break;
        case 31: 
            analog_pin = NRF_SAADC_INPUT_AIN7;
            break;
        default:
            analog_pin = NRF_SAADC_INPUT_VDD;
    }

    return analog_pin;
}


void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint16_t soc;
        uint16_t voltage_mv;    

        nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);

        // Compute mean value
        int32_t saadc_result = 0;
        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
             saadc_result += p_event->data.done.p_buffer[i];
        }
        saadc_result /= SAMPLES_IN_BUFFER;
        if (saadc_result < 0) saadc_result = 0;

        // convert to voltage
        voltage_mv = (saadc_result * 3600 / 1024);      // Voltage read by saadc, Input range = (0.6 V)/(1/6) = 3.6 V, resulution 10bits
        voltage_mv *= m_batt_meas_param.voltage_div;    // Compensate the voltage divider

        // Convert to SoC
        if (m_batt_meas_param.batt_type == 0) // CR2032
        {
            soc = battery_level_in_percent(voltage_mv);
        }
        else if (m_batt_meas_param.batt_type == 1) // LiPo
        {
            soc = lipo_voltage_to_battery_level(voltage_mv);
        }

        // Update BLE characteristic
        err_code = ble_bas_battery_level_update(&m_bas, soc, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                // APP_ERROR_HANDLER(err_code);
            }
        else
        {
            m_initial_batt_level_percent = soc;
        }

        // Call Battery event handler
        m_batt_meas_event_t batt_meas_evt;
        batt_meas_evt.voltage_mv = voltage_mv;
        batt_meas_evt.level_percent = soc;
        batt_meas_evt.type = M_BATT_MEAS_EVENT_DATA;
        m_evt_handler(&batt_meas_evt);
    }
}

/**@brief Initalizes SAADC for battery measurement
 *
 * @return NRF_SUCCESS
 * @return Other codes from the underlying driver.
 */
static uint32_t saadc_init(void)
{
    uint32_t err_code;

    static const nrfx_saadc_config_t default_config = NRFX_SAADC_DEFAULT_CONFIG;
    err_code = nrfx_saadc_init(&default_config, saadc_callback);
    VERIFY_SUCCESS(err_code);

    nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(digital_to_analog_pin(m_batt_meas_param.pin_batt));
    channel_config.acq_time = NRF_SAADC_ACQTIME_15US;
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    VERIFY_SUCCESS(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


/** @brief  The WPC events, executed in main context.
 */
static void wpc_event_handler_charge(void * event_data, uint16_t unused)
{
    m_batt_meas_event_t batt_meas_evt;
    batt_meas_evt.type = *(m_batt_meas_event_type_t*)event_data;
    batt_meas_evt.valid_voltage = false;
    batt_meas_evt.voltage_mv = 0;
    batt_meas_evt.level_percent = 0;
    m_evt_handler(&batt_meas_evt);
}


/** @brief WPC GPIOTE event handler for charge status.
 */
static void wpc_gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t unused)
{
    uint32_t err_code;
    m_batt_meas_event_t batt_meas_evt;

    m_batt_meas_event_type_t event_source;

    /* Read status of battery charge associated pins. */
    if (nrf_gpio_pin_read(m_batt_meas_param.pin_batt_chg))
    {
        event_source = M_BATT_MEAS_EVENT_CHARGING_FINISHED;
    }
    else
    {
        event_source = M_BATT_MEAS_EVENT_CHARGING;
    }

    NRF_LOG_DEBUG("GPIOTE handler, event source %d\r\n", event_source);

    // Call event handler
    err_code = app_sched_event_put((void*)&event_source, sizeof(event_source), wpc_event_handler_charge);
    APP_ERROR_CHECK(err_code);
}


/** @brief GPIO task and event config for detecting WPC and battery charge status.
 */
static uint32_t wpc_gpiote_init(void)
{
    uint32_t err_code;

    if (m_batt_meas_param.pin_batt_chg == 0xFF)
        return NRF_SUCCESS;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher  = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull        = NRF_GPIO_PIN_NOPULL;
    gpiote_in_config.sense       = NRF_GPIOTE_POLARITY_TOGGLE;

    err_code = nrf_drv_gpiote_in_init (m_batt_meas_param.pin_batt_chg, &gpiote_in_config, wpc_gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    nrf_drv_gpiote_in_event_enable(m_batt_meas_param.pin_batt_chg, true);

    return NRF_SUCCESS;
}


/** @brief Periodic timer handler.
 */
static void app_timer_periodic_handler(void * unused)
{
    nrfx_saadc_sample();
}


/** @brief Checks validity of supplied parameters.
 */
static uint32_t param_check(batt_meas_init_t const * const p_batt_meas_init)
{
    VERIFY_PARAM_NOT_NULL(p_batt_meas_init);
    VERIFY_PARAM_NOT_NULL(p_batt_meas_init->evt_handler);

    if (p_batt_meas_init->batt_meas_param.voltage_div == 0) return NRF_ERROR_INVALID_PARAM;
    if (p_batt_meas_init->batt_meas_param.batt_type > 2) return NRF_ERROR_INVALID_PARAM;

    // PlaceHolder if paramaters need to be checked

    return M_BATT_STATUS_CODE_SUCCESS;
}


/**@brief Event handler, handles events in the Battery Service.
 *
 * @details This callback function is often used to enable a service when requested over BLE,
 * and disable when not requested to save power. 
 */
static void ble_bas_evt_handler(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            m_batt_meas_enable(1000);
            NRF_LOG_DEBUG("BLE_BAS_EVT_NOTIFICATION_ENABLED");
            break;

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            m_batt_meas_disable();
            NRF_LOG_DEBUG("BLE_BAS_EVT_NOTIFICATION_DISABLED");
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Battery Service.
 *
 * @details This callback function will be called from the ble handling module to initialize the Battery service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t battery_service_init(void)
{
    uint32_t              err_code;
    ble_bas_init_t        bas_init;

    memset(&bas_init, 0, sizeof(bas_init));

    // Security level for the Battery Service
    bas_init.bl_report_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = ble_bas_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t m_batt_meas_enable(uint32_t meas_interval_ms)
{
    uint32_t err_code;

    if (meas_interval_ms < MEAS_INTERVAL_LOW_LIMIT_MS)
    {
        return M_BATT_STATUS_CODE_INVALID_PARAM;
    }

    err_code = app_timer_start(batt_meas_app_timer_id, APP_TIMER_TICKS(meas_interval_ms), NULL);
    VERIFY_SUCCESS(err_code);

    return M_BATT_STATUS_CODE_SUCCESS;
}


uint32_t m_batt_meas_disable(void)
{
    uint32_t err_code;

    err_code = app_timer_stop(batt_meas_app_timer_id);
    VERIFY_SUCCESS(err_code);

    return M_BATT_STATUS_CODE_SUCCESS;
}


uint32_t m_batt_meas_init (batt_meas_init_t const * const p_batt_meas_init)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_batt_meas_init);

    err_code = param_check(p_batt_meas_init);
    VERIFY_SUCCESS(err_code);

    m_evt_handler = p_batt_meas_init->evt_handler;
    m_batt_meas_param = p_batt_meas_init->batt_meas_param;

    // Initialize gpiote for WPC charger events
    if (m_batt_meas_param.pin_batt_chg != 255)
    {
        err_code = wpc_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    // Initialize SAADC for battery measurement
    err_code = saadc_init();
    VERIFY_SUCCESS(err_code);

    // Initialize timer
    err_code = app_timer_create(&batt_meas_app_timer_id, APP_TIMER_MODE_REPEATED, app_timer_periodic_handler);
    VERIFY_SUCCESS(err_code);

    // Initialize BLE BAS service
    err_code = battery_service_init();
    VERIFY_SUCCESS(err_code);

    return M_BATT_STATUS_CODE_SUCCESS;
}
