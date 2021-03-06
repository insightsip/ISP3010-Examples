 /******************************************************************************
 * @file    m_batt_meas.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    08-07-2019
 * @brief   Battery measurement module header file.
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

#ifndef __M_BATT_MEAS_H__
#define __M_BATT_MEAS_H__

#include <stdint.h>
#include <stdbool.h>

#define MEAS_INTERVAL_LOW_LIMIT_MS 50


/** @brief The m_batt_meas return status codes.
 */
enum
{
    M_BATT_STATUS_CODE_SUCCESS,                     ///< Successfull
    M_BATT_STATUS_CODE_INVALID_PARAM,               ///< Invalid parameters
};

/** @brief Battery and charge event codes.
 */
typedef enum
{
    M_BATT_MEAS_EVENT_DATA,                          /**< New battery SoC available.  */
    M_BATT_MEAS_EVENT_LOW,                           /**< Low battery.  */
    M_BATT_MEAS_EVENT_CHARGING,                      /**< Battery charging active.  */
    M_BATT_MEAS_EVENT_CHARGING_FINISHED,             /**< Battery charging finished/not charging.  */
    M_BATT_MEAS_EVENT_ERROR,                         /**< Error state detected signalled by the charger (not implemeted, CHG and CHG finished will toggle in case of error).  */
}m_batt_meas_event_type_t;

/** @brief The struct passed to the handler with relevant battery information.
 */
typedef struct
{
    m_batt_meas_event_type_t    type;               /**< Given event type.  */
    uint16_t                    voltage_mv;         /**< Battery voltage given in millivolts.  */
    uint8_t                     level_percent;      /**< Remaining battery capacity percent.  */
    bool                        valid_voltage;      /**< True if an event is generated by a ADC conversion.  */
}m_batt_meas_event_t;

/** @brief m_batt sensor event handler type. Should be implemented by user e.g. in main()
 */
typedef void (*m_batt_meas_event_handler_t)(m_batt_meas_event_t const * p_event);


/** @brief Input parameters for m_batt_meas_init.
 */
typedef struct
{
    uint32_t        pin_batt;             /**< Pin connected to SAADC */
    uint32_t        pin_batt_chg;         /**< Pin connected to "Charging status output pin" (CSO) of the wireles battery charger. */
    uint8_t         voltage_div;          /**< Value of the voltage divider if any */
    uint8_t         batt_type;            /**< Type of battery 0=CR2032 and 1=Lipo */
}batt_meas_param_t;

/** @brief Init parameters for m_batt_meas_init.
 */
typedef struct
{
    m_batt_meas_event_handler_t     evt_handler;        ///< Function pointer to the event handler
    batt_meas_param_t               batt_meas_param;    ///< Input parameters.
}batt_meas_init_t;

/**@brief Initalizes the battery driver.
 *
 * @param[out] p_handle             Pointer to the location to store the service handle.
 * @param[in]  p_batt_meas_init     Struct containing the configuration parameters.
 *
 * @return M_BATT_STATUS_CODE_SUCCESS
 * @return M_BATT_STATUS_CODE_INVALID_PARAM
 * @return NRF_ERROR_NULL
 * @return Other codes from the underlying driver.
 */
uint32_t m_batt_meas_init (batt_meas_init_t const * const p_batt_meas_init);

/**@brief Enables battery measurement at the given interval.
 *
 * @param meas_interval_ms  Sampling interval given in milliseconds.
 *
 * @note This will call the handler supplied in p_batt_meas_init at the given interval
 * which is supplied with a m_batt_meas_event_t struct containing the information.
 *
 * @return M_BATT_STATUS_CODE_SUCCESS
 * @return Other codes from the underlying driver
 */
uint32_t m_batt_meas_enable (uint32_t meas_interval_ms);

/**@brief Stops the battery measurement.
 *
 * @return M_BATT_STATUS_CODE_SUCCESS
 * @return Other codes from the underlying driver
 */
uint32_t m_batt_meas_disable (void);

#endif

