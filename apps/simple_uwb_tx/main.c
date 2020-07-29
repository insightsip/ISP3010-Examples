 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-07-2019
 * @brief   simple uwb tx main file
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"

#include "boards.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static nrf_drv_spi_t m_spi_instance = NRF_DRV_SPI_INSTANCE(0);
APP_TIMER_DEF(m_uwb_timer_id); 

/**@brief Table specifying the default TX power parameters 
*/
const uint32_t default_tx_power[14] =
{
    0x15355575,     // Channel 1 - 16M prf power
    0x07274767,     // Channel 1 - 64M prf power
    0x15355575,     // Channel 2 - 16M prf power
    0x07274767,     // Channel 2 - 64M prf power
    0x0f2f4f6f,     // Channel 3 - 16M prf power
    0x2b4b6b8b,     // Channel 3 - 64M prf power
    0x1f1f3f5f,     // Channel 4 - 16M prf power
    0x3a5a7a9a,     // Channel 4 - 64M prf power
    0x0E082848,     // Channel 5 - 16M prf power
    0x25456585,     // Channel 5 - 64M prf power 
    0x0,            // Channel 6 - this is just a place holder
    0x0,            // Channel 6 - this is just a place holder
    0x32527292,     // Channel 7 - 16M prf power
    0x5171b1d1      // Channel 7 - 64M prf power
};


/**@brief Table specifying the default pg delay parameters 
*/
const uint8_t default_pg_delay[7] =
{
    0xc9,     // Channel 1 
    0xc2,     // Channel 2
    0xc5,     // Channel 3
    0x95,     // Channel 4
    0xc0,     // Channel 5
    0x0,      // Channel 6 - this is just a place holder
    0x93      // Channel 7
};


int readfromspi (uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    uint32 length = headerLength + readlength;

    uint8 m_spi_rx_buffer[length];
    uint8 m_spi_tx_buffer[length];
    memset(m_spi_rx_buffer, 0, length);
    memset(m_spi_tx_buffer, 0, length); 

    memcpy(m_spi_tx_buffer, headerBuffer, headerLength);
    nrf_drv_spi_transfer(&m_spi_instance, m_spi_tx_buffer, length, m_spi_rx_buffer, length);
    memcpy(readBuffer, m_spi_rx_buffer+headerLength, readlength);

    return 0;    
}


int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    uint32 length = headerLength + bodylength;

    uint8 m_spi_rx_buffer[length];
    uint8 m_spi_tx_buffer[length];
    memset(m_spi_rx_buffer, 0, length);
    memset(m_spi_tx_buffer, 0, length); 

    memcpy(m_spi_tx_buffer, headerBuffer, headerLength);
    memcpy(m_spi_tx_buffer + headerLength, bodyBuffer, bodylength);
    nrf_drv_spi_transfer(&m_spi_instance, m_spi_tx_buffer, length, NULL, 0);
    
    return 0;        
}


void decamutexoff(decaIrqStatus_t s)
{ }


decaIrqStatus_t decamutexon(void)
{
    return 0;
}


void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}


/**@brief DW1000 TX done event
 */
static void cb_tx_done (const dwt_cb_data_t *txd)
{
    NRF_LOG_INFO("UWB TX done.");
}


/**@brief DW1000 RX done event
 */
static void cb_rx_done (const dwt_cb_data_t *rxd)
{
    // Nothing to do
}


/**@brief DW1000 RX timeout event
 */
static void cb_rx_to (const dwt_cb_data_t *rxd)
{    
    // Nothing to do
}


/**@brief DW1000 RX error event
 */
static void cb_rx_err (const dwt_cb_data_t *rxd)
{
    // Nothing to do
}


/**@brief Function for waking up the dw1000
 */
static uint32_t wake_up (void)
{
    uint32 x = 0;
    
    // Wake up device
    nrf_gpio_pin_set(PIN_DW1000_WAKEUP);
    nrf_delay_us(500);   //500 us to wake up 
    nrf_gpio_pin_clear(PIN_DW1000_WAKEUP);  

    // Wait for DW1000 RST pin to go high
    while(!nrf_gpio_pin_read(PIN_DW1000_RST))
     {
        x++;
        nrf_delay_us(50);
        if (x > 100) break;
     }
    
    // Check if it is alive
    for (int i=0; i<30; i++)
    {    
        x = dwt_readdevid();
        if(x == DWT_DEVICE_ID) break;    
    }
    if(x != DWT_DEVICE_ID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    
    dwt_entersleepaftertx(0);
    dwt_setinterrupt(DWT_INT_TFRS, 1); //re-enable the TX/RX interrupt

    return NRF_SUCCESS;    
}



/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    if ((pin == PIN_DW1000_IRQ) && (nrf_gpio_pin_read(PIN_DW1000_IRQ) == 1))
    {
        dwt_isr();
    }
}


/**@brief Initialize the GPIO tasks and events system to catch pin data ready interrupts.
 */
static uint32_t gpiote_init(void)
{
    uint32_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher  = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull        = NRF_GPIO_PIN_NOPULL;
    gpiote_in_config.sense       = NRF_GPIOTE_POLARITY_LOTOHI;
    err_code = nrf_drv_gpiote_in_init(PIN_DW1000_IRQ, &gpiote_in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


/**@brief Initialize the SPI peripheral to communicate with dw1000.
 */
static uint32_t spi_init(void)
{
    uint32_t err_code;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.frequency = NRF_DRV_SPI_FREQ_2M;
    spi_config.miso_pin = PIN_DW1000_MISO;
    spi_config.mosi_pin = PIN_DW1000_MOSI;
    spi_config.sck_pin  = PIN_DW1000_CLK;
    spi_config.ss_pin   = PIN_DW1000_CS;

    err_code = nrf_drv_spi_init(&m_spi_instance, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for handling the uwb timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void uwb_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    uint8_t tx_msg[] = {'H', 'E', 'L', 'L', 'O', ' ', 'W', 'O', 'R', 'L', 'D', 0, 0};

    // Wake up dw1000
    wake_up();

    // Configure dw1000 to go to low power mode after tx
    dwt_entersleepaftertx(1);

    // Write frame data to DW1000 
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); 
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0);

    // Start transmission.
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Start LF clock
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while(!NRF_CLOCK->EVENTS_LFCLKSTARTED)
    {}

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_uwb_timer_id, APP_TIMER_MODE_REPEATED, uwb_timeout_handler);
    APP_ERROR_CHECK(err_code);
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
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
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


/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    power_management_init();    
    timers_init();

    // Configure Wake up pin
    nrf_gpio_pin_clear(PIN_DW1000_WAKEUP);
    nrf_gpio_cfg_output(PIN_DW1000_WAKEUP);  
      
    // Configure IRQ pin 
    gpiote_init();

    // Configure SPI
    spi_init();

    // Wake up
    nrf_gpio_pin_set(PIN_DW1000_WAKEUP);
    nrf_delay_us(500);   //500 us to wake up 
    nrf_gpio_pin_clear(PIN_DW1000_WAKEUP);  
    nrf_delay_ms(2);

    // Reset dw1000
    nrf_gpio_pin_clear(PIN_DW1000_RST);  
    nrf_gpio_cfg_output(PIN_DW1000_RST);   
    nrf_delay_ms(2); 
    nrf_gpio_cfg_input(PIN_DW1000_RST, NRF_GPIO_PIN_NOPULL); 
    nrf_delay_ms(2); 

    // Initialize dw1000
    APP_ERROR_CHECK(dwt_initialise(DWT_LOADUCODE));

    // SPI can now be set to 8M
    nrf_spim_frequency_set((NRF_SPIM_Type *)&m_spi_instance.u.spi.p_reg, NRF_SPIM_FREQ_8M);

    // Configure dw1000 interrupts
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setcallbacks(cb_tx_done, cb_rx_done, cb_rx_to, cb_rx_err);

    // Configure dw1000 uwb signal
    static dwt_config_t config = 
    {
        5,                /* Channel number. */
        DWT_PRF_64M,      /* Pulse repetition frequency. */
        DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
        9,                /* TX preamble code. Used in TX only. */
        9,                /* RX preamble code. Used in RX only. */
        0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_6M8,       /* Data rate. */
        DWT_PHRMODE_STD,  /* PHY header mode. */
        (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };
    dwt_configure(&config);

    // Configure dw1000 tx power
    uint32_t tx_power[14];
    dwt_otpread(0x10, tx_power, 12);
    uint32_t power = config.chan == 7 ? tx_power[2*(config.chan-2) + config.prf-1] : tx_power[2*(config.chan-1) + config.prf-1];
    if ((power == 0x0) || (power == 0xFFFFFFFF)) 
    {
        power = default_tx_power[2*(config.chan-1) + config.prf-1];
    }
    dwt_txconfig_t tx_config;
    tx_config.PGdly = default_pg_delay[config.chan-1];
    tx_config.power = power;
    dwt_configuretxrf(&tx_config);

    // Configure Sleep mode
    dwt_configuresleep(DWT_LOADUCODE | DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV, DWT_SLP_EN | DWT_WAKE_WK); 
    dwt_entersleep();
    
    // Start execution.
    NRF_LOG_INFO("ISP3010 simple uwb tx started.");
    nrf_drv_gpiote_in_event_enable(PIN_DW1000_IRQ, true);
    APP_ERROR_CHECK(app_timer_start(m_uwb_timer_id, APP_TIMER_TICKS(1000), NULL)); // Every sec
   
    // Enter main loop.
    while(1)
    {
        idle_state_handle();    
    }
}
