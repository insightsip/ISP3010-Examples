 /******************************************************************************
 * @file    drv_acc.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-07-2019
 * @brief   accelerometer driver implementation file.
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

#include "nrf_delay.h"
#include "sdk_macros.h"

#include "drv_acc.h"
#include "drv_lis2de12.h"

#include "nrf_log.h"


#define RETURN_IF_INV_ERROR(PARAM)              \
        if ((PARAM) != INV_SUCCESS)             \
        {                              		\
            return NRF_ERROR_INTERNAL;		\
        }
		
#define ACC_SCALE_2G 	0.015625f
#define ACC_SCALE_4G 	0.031250f 
#define ACC_SCALE_8G 	0.062500f 
#define ACC_SCALE_16G 	0.125000f 


/**@brief Motion configuration struct.
 */
typedef struct
{
    bool                        enabled;       ///< Driver enabled.
    drv_lis2de12_twi_cfg_t      cfg;           ///< TWI configuraion.
    bool                        running;
    drv_acc_evt_handler_t       evt_handler;
} drv_acc_t;

/**@brief configuration.
 */
static drv_acc_t m_drv_acc;

uint32_t drv_acc_init(drv_acc_init_t * p_params)
{
    uint32_t err_code;
	
    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_acc.evt_handler           = p_params->evt_handler;
    m_drv_acc.cfg.twi_addr          = p_params->twi_addr;
    m_drv_acc.cfg.pin_int1          = p_params->pin_int1;
    m_drv_acc.cfg.pin_int2          = p_params->pin_int2;
    m_drv_acc.cfg.p_twi_instance    = p_params->p_twi_instance;
    m_drv_acc.cfg.p_twi_cfg         = p_params->p_twi_cfg;
    m_drv_acc.enabled               = false;

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_verify();
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_reboot();
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t drv_acc_enable(void)
{
    uint32_t err_code;

    if (m_drv_acc.enabled)
    {
        return NRF_SUCCESS;
    }
    m_drv_acc.enabled = true;
	
    const drv_lis2de12_cfg_t lis2de12_cfg =
    {
        .reg_vals =
        {
            .ctrl_reg1 = BITS_XEN | BITS_YEN | BITS_ZEN | BITS_LPEN | BITS_ODR_10HZ
        },
        .reg_selects =
        {
            .ctrl_reg1      = true,
            .ctrl_reg2      = false,
            .ctrl_reg3      = false,
            .ctrl_reg4      = false,
            .ctrl_reg5      = false,
            .ctrl_reg6      = false,
            .temp_cfg_reg   = false,
            .fifo_ctrl_reg  = false,
            .int1_cfg       = false,
            .int2_cfg       = false,
            .click_cfg      = false
        }
    };

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);
	
    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);
	
    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_acc_disable(void)
{
    uint32_t err_code = NRF_SUCCESS;

    m_drv_acc.enabled = false;
		
    const drv_lis2de12_cfg_t lis2de12_cfg =
    {
        .reg_vals =
        {
            .ctrl_reg1 = CTRL_REG1_DEFAULT
        },
        .reg_selects =
        {
            .ctrl_reg1      = true,
            .ctrl_reg2      = false,
            .ctrl_reg3      = false,
            .ctrl_reg4      = false,
            .ctrl_reg5      = false,
            .ctrl_reg6      = false,
            .temp_cfg_reg   = false,
            .fifo_ctrl_reg  = false,
            .int1_cfg       = false,
            .int2_cfg       = false,
            .click_cfg      = false
        }
    };

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);
		
    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}


uint32_t drv_acc_get(float * p_acc)
{
    uint32_t err_code = NRF_SUCCESS;
    uint8_t raw_val[3];

    VERIFY_PARAM_NOT_NULL(p_acc);
	
    // Request twi bus
    drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);
	
    err_code = drv_lis2de12_accelerometer_get(raw_val);
    VERIFY_SUCCESS(err_code);

    // Currently we use the accelerometer with +/-2g 
    // TODO manage automatically full scale selection
    p_acc[0] = (float)(raw_val[0]) * ACC_SCALE_2G;
    p_acc[1] = (float)(raw_val[1]) * ACC_SCALE_2G;
    p_acc[2] = (float)(raw_val[2]) * ACC_SCALE_2G;
	
    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;	
}


