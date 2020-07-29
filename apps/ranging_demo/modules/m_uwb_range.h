 /******************************************************************************
 * @file    m_range.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    09-07-2019
 * @brief   Range module header file.
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

#ifndef __M_RANGE_H__
#define __M_RANGE_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_spi.h"
#include "drv_uwb_range.h"


typedef drv_uwb_range_role_t m_uwb_range_role_t;

/** @brief Input parameters for m_uwb_range_init.
 */
typedef struct
{
    uint32_t             pin_wakeup;            /**<  Pin connected to dw1000 wakeup  */
    uint32_t             pin_reset;             /**<  Pin connected to dw1000 reset  */
    uint32_t             pin_irq;               /**<  Pin connected to dw1000 irq   */
    uint32_t             pin_rst;               /**<  Pin connected to dw1000 rst   */
    uint32_t             pin_clk;               /**<  Pin connected to dw1000 clk   */
    uint32_t             pin_mosi;              /**<  Pin connected to dw1000 mosi  */
    uint32_t             pin_miso;              /**<  Pin connected to dw1000 miso  */
    uint32_t             pin_cs;                /**<  Pin connected to dw1000 miso  */
    m_uwb_range_role_t   role;                  /**< role. */
}m_uwb_range_param_t;

/** @brief Init parameters for m_uwb_range_init.
 */
typedef struct
{
    //m_uwb_range_event_handler_t     evt_handler;        ///< Function pointer to the event handler
    const nrf_drv_spi_t   * p_spi_instance;
    m_uwb_range_param_t     uwb_range_param;        /**< Input parameters. */
   
}m_uwb_range_init_t;


/**@brief Function for starting the uwb module.
 *
 * @details This function should be called after m_uwb_init to start the uwb module.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_range_start (void);

/**@brief Function for stopping the uwb module.
 *
 * @details This function should be called after m_uwb_start to stop the uwb module.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_range_stop (void);

/**@brief Function for initializing the uwb module.
 *
 * @param[in] p_params    Pointer to the init parameters.
 */
uint32_t m_range_init (m_uwb_range_init_t * p_params);

uint32_t m_range_ble_uuid_get(uint16_t *uuid);

#endif

