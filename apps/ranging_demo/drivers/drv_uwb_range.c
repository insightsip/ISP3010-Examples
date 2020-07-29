/******************************************************************************
 * @file    drv_range.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    09-07-2019
 * @brief   Range driver implementation
 *
 * @attention
 *    THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <string.h>

#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "sdk_macros.h"
#include "nrf_log.h"

#include "drv_uwb_range.h"
#include "deca_device_api.h"
#include "deca_regs.h"

static nrf_drv_spi_t m_spi_instance = NRF_DRV_SPI_INSTANCE(0);

//The table below specifies the default TX spectrum configuration parameters 
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

/**@brief Initialization struct for range hw driver.
 */
typedef struct
{
    uint32_t                     pin_irq;           /**< Interrupt pin number. */
    uint32_t                     pin_wakeup;        /**< Wake up pin number. */
    uint32_t                     pin_reset;         /**< Reset pin number. */
    uint32_t                     pin_mosi;          /**< mosi pin number. */
    uint32_t                     pin_miso;          /**< miso pin number. */
    uint32_t                     pin_clk;           /**< clk pin number. */
    uint32_t                     pin_cs;            /**< cs pin number. */
} drv_uwb_range_hw_cfg_t;

/**@brief Initialization struct for uwb sw driver.
 */
typedef struct
{
    drv_uwb_range_role_t role;                  /**< Role of the device */
    bool sleep_enabled;                         /**< is sleep mode enabled */
    bool filter_enabled;                        /**< is frame filter enabled */
    bool busy;                                  /**< is driver busy ie not allowing configuration */
    uint32_t frame_sn;                          /**< Frame counter */
    uint8_t src_address[8];                     /**< Address of the device */
    uint8_t dest_address[8];                    /**< Address of the device to range with */
    uint32_t poll_tx_timestamp_u32;
    uint32_t poll_rx_timestamp_u32;
    uint32_t resp_tx_timestamp_u32;
    uint32_t resp_rx_timestamp_u32;
    uint64_t poll_rx_timestamp_u64;
    uint64_t resp_tx_timestamp_u64;
    uint32 tx_power[12];                        /**< Tx power configuration read from OTP (6 channels consecutively with PRF16 then 64, e.g. Ch 1 PRF16 is index 0 and 64 index 1) */
    uint32_t ant_delay;
} drv_uwb_range_sw_cfg_t;

/**@brief uwb configuration struct.
 */
typedef struct
{
    drv_uwb_range_hw_cfg_t            hw_cfg;           /**< HW configuration. */
    drv_uwb_range_sw_cfg_t            sw_cfg;           /**< SW configuration. */
    drv_uwb_range_evt_handler_t       evt_handler;      /**< Event handler called by gpiote_evt_sceduled. */
    
} drv_uwb_range_t;

static drv_uwb_range_t m_uwb_drv_range;     /**< Stored configuration. */
static double m_last_range = -1.0; 
static uint8_t m_spi_rx_buffer[128];
static uint8_t m_spi_tx_buffer[128];

// DW implemtations
void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}

    
void decamutexoff(decaIrqStatus_t s)
{
}


decaIrqStatus_t decamutexon(void)
{
    return 0;
}


int readfromspi (uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    uint32_t length = headerLength + readlength;

    memset(m_spi_rx_buffer, 0, 128);
    memset(m_spi_tx_buffer, 0, 128); 

    memcpy(m_spi_tx_buffer, headerBuffer, headerLength);
    nrf_drv_spi_transfer(&m_spi_instance, m_spi_tx_buffer, length, m_spi_rx_buffer, length);
    memcpy(readBuffer, m_spi_rx_buffer+headerLength, readlength);

    return 0;    
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    uint32 length = headerLength + bodylength;

    memset(m_spi_rx_buffer, 0, 128);
    memset(m_spi_tx_buffer, 0, 128); 

    memcpy(m_spi_tx_buffer, headerBuffer, headerLength);
    memcpy(m_spi_tx_buffer + headerLength, bodyBuffer, bodylength);
    nrf_drv_spi_transfer(&m_spi_instance, m_spi_tx_buffer, length, NULL, 0);	

    return 0;        
}

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled (void * p_event_data, uint16_t event_size)
{    
    dwt_isr();
}

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    if ((pin == m_uwb_drv_range.hw_cfg.pin_irq) && (nrf_gpio_pin_read(m_uwb_drv_range.hw_cfg.pin_irq) == 1))
    {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Initialize the GPIO tasks and events system to catch pin data ready interrupts.
 */
static uint32_t gpiote_init (uint32_t pin)
{
    uint32_t err_code;

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
    err_code = nrf_drv_gpiote_in_init(pin, &gpiote_in_config, gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Uninitialize the GPIO tasks and events system.
 */
static void gpiote_uninit (uint32_t pin)
{
    nrf_drv_gpiote_in_uninit(pin);
}


static uint64_t get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void resp_msg_get_ts (uint8_t *ts_field, uint32_t *ts)
{
    int i;
    
    *ts = 0;
    for (i = 0; i < 4; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

static void resp_msg_set_ts (uint8_t *ts_field, const uint64_t ts)
{
    int i;
    
    for (i = 0; i < 4; i++)
    {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}

static float calculate_range (uint32 init_tx_timestamp, uint32 init_rx_timestamp, uint32 resp_tx_timestamp, uint32 resp_rx_timestamp)
{
    int32 rtd_init, rtd_resp;
    volatile float clock_offset_ratio;
    volatile float tof; 
    volatile float distance;
    
    /* Read carrier integrator value and calculate clock offset ratio */
    clock_offset_ratio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
    
    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
    rtd_init = resp_rx_timestamp - init_tx_timestamp;
    rtd_resp = resp_tx_timestamp - init_rx_timestamp;
    
    tof = ((rtd_init - rtd_resp * (1.0f - clock_offset_ratio)) / 2.0f) * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;
    
    return distance;
}

static uint32_t wake_up (void)
{
    uint32 x = 0;
    
    // Wake up device
    if (m_uwb_drv_range.hw_cfg.pin_wakeup != DRV_UWB_RANGE_PIN_NOT_USED)
    {
        nrf_gpio_pin_set(m_uwb_drv_range.hw_cfg.pin_wakeup);
        nrf_delay_ms(1);   //200 us to wake up 
        nrf_gpio_pin_clear(m_uwb_drv_range.hw_cfg.pin_wakeup);
    }    
    
    nrf_delay_ms(2);
    
    // Check if it is alive
    for (int i=0; i<30; i++)
    {    
        x = dwt_readdevid();
        if (x == DWT_DEVICE_ID) break;    
    }
    if (x != DWT_DEVICE_ID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    
    // TX antenna delay needs reprogramming as it is not preserved after DEEP SLEEP
    dwt_settxantennadelay(m_uwb_drv_range.sw_cfg.ant_delay);
    
    //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
    dwt_entersleepaftertx(0);
    dwt_setinterrupt(DWT_INT_TFRS, 1); //re-enable the TX/RX interrupts
    
    dwt_seteui(m_uwb_drv_range.sw_cfg.src_address);
    
    return NRF_SUCCESS;    
}


/**@brief TX done event
 */
static void cb_tx_done (const dwt_cb_data_t *txd)
{
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_RESPONDER: // TWR_RESPONDER
            m_last_range = -1;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            break;

        default:
            break;
    }
}


/**@brief RX done event
 */
static void cb_rx_done (const dwt_cb_data_t *rxd)
{
    srd_msg_dlsl rxmsg_ll;
    uint16_t rx_msg_length;
            
    // Read Data Frame
    rx_msg_length = rxd->datalength;
    dwt_readrxdata((uint8 *)&rxmsg_ll, rx_msg_length, 0);
    
    // Check frame control bytes - must be 0x41CC
    if (rxmsg_ll.frameCtrl[0] != 0x41 || rxmsg_ll.frameCtrl[1] != 0xCC)
    {
        // unexpected frame control
        if (m_uwb_drv_range.sw_cfg.role == TWR_RESPONDER)
        {
            //immediate rx enable
            dwt_rxenable(DWT_START_RX_IMMEDIATE); 
        }
        else
        {
            //put device into low power mode
            if (m_uwb_drv_range.sw_cfg.sleep_enabled) dwt_entersleep();    
        }
        return;
    }    

    // Action depending on the role.....
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_INITIATOR: // TWR_INITIATOR
        {
            if (rxmsg_ll.messageData[0] == SIMPLE_MSG_ANCH_RESP) // SIMPLE_MSG_ANCH_RESP received
            {               
                // Retrieve poll transmission and response reception timestamps.
                m_uwb_drv_range.sw_cfg.poll_tx_timestamp_u32 = dwt_readtxtimestamplo32();
                m_uwb_drv_range.sw_cfg.resp_rx_timestamp_u32 = dwt_readrxtimestamplo32();
                
                // Get timestamps embedded in response message
                resp_msg_get_ts(&rxmsg_ll.messageData[POLL_RX_TS], &m_uwb_drv_range.sw_cfg.poll_rx_timestamp_u32);
                resp_msg_get_ts(&rxmsg_ll.messageData[RESP_TX_TS], &m_uwb_drv_range.sw_cfg.resp_tx_timestamp_u32);

                // Calculate range
                m_last_range = calculate_range(m_uwb_drv_range.sw_cfg.poll_tx_timestamp_u32,
                                        m_uwb_drv_range.sw_cfg.poll_rx_timestamp_u32,
                                        m_uwb_drv_range.sw_cfg.resp_tx_timestamp_u32,
                                        m_uwb_drv_range.sw_cfg.resp_rx_timestamp_u32);        

                // Go to sleep
                if (m_uwb_drv_range.sw_cfg.sleep_enabled) 
                {
                    dwt_entersleep();
                }
                m_uwb_drv_range.sw_cfg.busy = false;
                
                // Generate data event    
                if (m_uwb_drv_range.evt_handler != NULL)
                {
                    drv_uwb_range_evt_t evt;
                    evt.type = DRV_RANGE_EVT_DATA;
                    evt.range = m_last_range;
                    m_uwb_drv_range.evt_handler(&evt);
                }
            }
            else    // No expected message received
            {
                // Go to sleep
                if (m_uwb_drv_range.sw_cfg.sleep_enabled) 
                {
                    dwt_entersleep();
                }
            }
        }
        break;

        case TWR_RESPONDER: // TWR_RESPONDER
        {
            if (rxmsg_ll.messageData[0] == SIMPLE_MSG_TAG_POLL) // SIMPLE_MSG_TAG_POLL received
            {
                // Retrieve poll reception timestamp.
                m_uwb_drv_range.sw_cfg.poll_rx_timestamp_u64 = get_rx_timestamp_u64();
            
                // Compute final message transmission time.
                uint32_t resp_tx_time = (m_uwb_drv_range.sw_cfg.poll_rx_timestamp_u64 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            
                // Response TX timestamp is the transmission time we programmed plus the antenna delay.
                m_uwb_drv_range.sw_cfg.resp_tx_timestamp_u64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;        

                // Prepare and send ANCHOR RESP msg to TWR_INITIATOR                        
                uint16 length;
                srd_msg_dlsl msg;
        
                msg.frameCtrl[0]             = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
                msg.frameCtrl[1]             = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
                msg.seqNum                   = m_uwb_drv_range.sw_cfg.frame_sn++;
                msg.panID[0]                 = (0xdeca) & 0xff;
                msg.panID[1]                 = (0xdeca) >> 8;
                memcpy(&msg.sourceAddr[0],      m_uwb_drv_range.sw_cfg.src_address,     ADDR_BYTE_SIZE_L);
                memcpy(&msg.destAddr[0],        m_uwb_drv_range.sw_cfg.dest_address,    ADDR_BYTE_SIZE_L); 
                msg.messageData[FCODE_POS]      = SIMPLE_MSG_ANCH_RESP;
                resp_msg_set_ts(&msg.messageData[POLL_RX_TS], m_uwb_drv_range.sw_cfg.poll_rx_timestamp_u64);    // Poll message reception timestamp
                resp_msg_set_ts(&msg.messageData[RESP_TX_TS], m_uwb_drv_range.sw_cfg.resp_tx_timestamp_u64);    // Response message transmission timestamp
            
                length = SIMPLE_MSG_ANCH_RESP_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
        
                // Write the frame data
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                dwt_writetxdata(length, (uint8_t*)&msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(length, 0, 1);             /* Zero offset in TX buffer, ranging. */
        
                // Send frame
                dwt_setdelayedtrxtime(resp_tx_time);
                if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
                {
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }   

                // Now we can check if there is a valid Range in the payload
                if (rx_msg_length == (SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC))
                {
                    memcpy(&m_last_range, &rxmsg_ll.messageData[POLL_MSG_TOF_POS], 8);

                    // Generate data event    
                    if ((m_uwb_drv_range.evt_handler != NULL) && (m_last_range != -1))
                    {
                        drv_uwb_range_evt_t evt;
                        evt.type = DRV_RANGE_EVT_DATA;
                        evt.range = m_last_range;
                        m_uwb_drv_range.evt_handler(&evt);
                    }
                }
            }
            else    // No expected message received
            {
                //immediate rx enable
                dwt_rxenable(DWT_START_RX_IMMEDIATE); 
            }
        }
        break;

        default:
            break;
    }     
}

/**@brief RX timeout event
 */
static void cb_rx_to (const dwt_cb_data_t *rxd)
{    
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_INITIATOR: // TWR_INITIATOR
            // go to sleep mode
            if (m_uwb_drv_range.sw_cfg.sleep_enabled) 
            {
                dwt_entersleep();
            }
            m_uwb_drv_range.sw_cfg.busy = false;
            
            // generate timeout event    
            if (m_uwb_drv_range.evt_handler != NULL)
            {
                drv_uwb_range_evt_t evt;
                evt.type = DRV_RANGE_EVT_TIMEOUT;
                m_uwb_drv_range.evt_handler(&evt);
            }
            break;

        case TWR_RESPONDER: // TWR_RESPONDER
            //immediate rx enable
            dwt_rxenable(DWT_START_RX_IMMEDIATE); 
            break;
        
        default:
            break;
    }
}

/**@brief RX error event
 */
static void cb_rx_err (const dwt_cb_data_t *rxd)
{
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_INITIATOR: // TWR_INITIATOR
            // go to sleep mode
            if (m_uwb_drv_range.sw_cfg.sleep_enabled) dwt_entersleep();
             m_uwb_drv_range.sw_cfg.busy = false;
            
            // generate event        
            if (m_uwb_drv_range.evt_handler != NULL)
            {
                drv_uwb_range_evt_t evt;
                evt.type = DRV_RANGE_EVT_ERROR;
                m_uwb_drv_range.evt_handler(&evt);
            }
           
            break;

        case TWR_RESPONDER: // TWR_RESPONDER
            //immediate rx enable
            dwt_rxenable(DWT_START_RX_IMMEDIATE); 
            break;
        
        default:
            break;
    }
}

uint32_t drv_uwb_range_init (drv_uwb_range_init_t * p_params)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler); 
    
    m_uwb_drv_range.evt_handler                 = p_params->evt_handler;
    m_uwb_drv_range.hw_cfg.pin_irq              = p_params->pin_irq;
    m_uwb_drv_range.hw_cfg.pin_wakeup           = p_params->pin_wakeup;
    m_uwb_drv_range.hw_cfg.pin_mosi             = p_params->pin_mosi;
    m_uwb_drv_range.hw_cfg.pin_miso             = p_params->pin_miso;
    m_uwb_drv_range.hw_cfg.pin_clk              = p_params->pin_clk;
    m_uwb_drv_range.hw_cfg.pin_cs               = p_params->pin_cs;
    m_uwb_drv_range.hw_cfg.pin_reset            = p_params->pin_reset;
    m_uwb_drv_range.sw_cfg.role                 = p_params->role;
    m_uwb_drv_range.sw_cfg.sleep_enabled        = p_params->enable_sleep;
    m_uwb_drv_range.sw_cfg.filter_enabled       = p_params->enable_filter;
    m_uwb_drv_range.sw_cfg.busy                 = false;
    memcpy(&m_uwb_drv_range.sw_cfg.src_address, p_params->own_address, 8);
    memcpy(&m_uwb_drv_range.sw_cfg.dest_address, p_params->reply_address, 8);

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.mosi_pin         = m_uwb_drv_range.hw_cfg.pin_mosi;
    spi_config.miso_pin         = m_uwb_drv_range.hw_cfg.pin_miso;
    spi_config.sck_pin          = m_uwb_drv_range.hw_cfg.pin_clk;
    spi_config.ss_pin           = m_uwb_drv_range.hw_cfg.pin_cs;
    spi_config.frequency        = NRF_SPIM_FREQ_2M;
    spi_config.mode             = NRF_SPIM_MODE_0;

    // Configure Wake up pin
    if (m_uwb_drv_range.hw_cfg.pin_wakeup != DRV_UWB_RANGE_PIN_NOT_USED)
    {
        nrf_gpio_pin_clear (m_uwb_drv_range.hw_cfg.pin_wakeup);
        nrf_gpio_cfg_output (m_uwb_drv_range.hw_cfg.pin_wakeup);   
    }

    // Configure IRQ pin 
    if (m_uwb_drv_range.hw_cfg.pin_irq != DRV_UWB_RANGE_PIN_NOT_USED)
    {
        err_code = gpiote_init(m_uwb_drv_range.hw_cfg.pin_irq);
        VERIFY_SUCCESS(err_code);
    }

    // Configure SPIM
    err_code = nrf_drv_spi_init(&m_spi_instance, &spi_config, NULL, NULL);
    VERIFY_SUCCESS(err_code);

    // wake up
    if (m_uwb_drv_range.hw_cfg.pin_wakeup != DRV_UWB_RANGE_PIN_NOT_USED)
    {
        nrf_gpio_pin_set (m_uwb_drv_range.hw_cfg.pin_wakeup);
        nrf_delay_us(500);   //500 us to wake up 
        nrf_gpio_pin_clear(m_uwb_drv_range.hw_cfg.pin_wakeup);  
        nrf_delay_ms(2);
    }

    // Reset dw1000
    if (m_uwb_drv_range.hw_cfg.pin_reset != DRV_UWB_RANGE_PIN_NOT_USED)
    {
        nrf_gpio_pin_clear (m_uwb_drv_range.hw_cfg.pin_reset);
        nrf_gpio_cfg_output (m_uwb_drv_range.hw_cfg.pin_reset);    
        nrf_delay_ms(2);
        nrf_gpio_cfg_input (m_uwb_drv_range.hw_cfg.pin_reset, NRF_GPIO_PIN_NOPULL);    
        nrf_delay_ms(2);
    }

    // Initialize dw1000
    err_code = dwt_initialise(DWT_LOADUCODE);
    if (err_code == DWT_ERROR)
    {
        return NRF_ERROR_INTERNAL;
    }

    nrf_spim_frequency_set((NRF_SPIM_Type *)&m_spi_instance.u.spi.p_reg, NRF_SPIM_FREQ_8M);
    
    // Configure interrupts
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
    dwt_otpread(0x10, m_uwb_drv_range.sw_cfg.tx_power, 12);
    uint32_t power =  m_uwb_drv_range.sw_cfg.tx_power[2*(config.chan-1) + config.prf-1];
    if ((power == 0x0) || (power == 0xFFFFFFFF)) 
    {
        power = default_tx_power[2*(config.chan-1) + config.prf-1];
    }
    dwt_txconfig_t tx_config;
    tx_config.PGdly = default_pg_delay[config.chan-1];
    tx_config.power = power;
    dwt_configuretxrf(&tx_config);
    NRF_LOG_DEBUG("DW1000 Tx power: 0x%08x", power);
    
    // Configure antenna delays
    dwt_otpread(0x1C, &m_uwb_drv_range.sw_cfg.ant_delay, 1);
    if (m_uwb_drv_range.sw_cfg.ant_delay == 0x0 || m_uwb_drv_range.sw_cfg.ant_delay == 0xFFFFFFFF)
    {
        m_uwb_drv_range.sw_cfg.ant_delay = ANT_DLY;
    }
    dwt_setrxantennadelay(m_uwb_drv_range.sw_cfg.ant_delay & 0xFFFF);
    dwt_settxantennadelay(m_uwb_drv_range.sw_cfg.ant_delay & 0xFFFF);
    NRF_LOG_DEBUG("DW1000 Ant delay: 0x%08x", m_uwb_drv_range.sw_cfg.ant_delay);
    
    // Configure filter 
    if (m_uwb_drv_range.sw_cfg.filter_enabled)
    {
        dwt_setpanid(0xdeca);            
        dwt_seteui(m_uwb_drv_range.sw_cfg.src_address);
        dwt_enableframefilter(DWT_FF_DATA_EN);
    }
    NRF_LOG_DEBUG("DW1000 Filter enabled: %d", m_uwb_drv_range.sw_cfg.filter_enabled);
    
    // Configure Sleep mode
    if (m_uwb_drv_range.sw_cfg.sleep_enabled)
    {
        uint8_t wake = DWT_SLP_EN;
        int sleepmode = (DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);
        
        if (m_uwb_drv_range.hw_cfg.pin_wakeup != DRV_UWB_RANGE_PIN_NOT_USED)
        {
            wake |= DWT_WAKE_WK;
        }
        dwt_configuresleep(sleepmode, wake); 
        
        dwt_entersleep();
    }

    // Enable IRQ
    nrf_drv_gpiote_in_event_enable(m_uwb_drv_range.hw_cfg.pin_irq, true);

    return NRF_SUCCESS;
}

uint32_t drv_uwb_range_request (void)
{
    if (m_uwb_drv_range.sw_cfg.role != TWR_INITIATOR)
    {
        return NRF_ERROR_INVALID_STATE;
    }
        
    srd_msg_dlsl msg; 
    uint16_t length;
        
    msg.frameCtrl[0]     = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
    msg.frameCtrl[1]     = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
    msg.seqNum           = m_uwb_drv_range.sw_cfg.frame_sn++;
    msg.panID[0]         = (0xdeca) & 0xff;
    msg.panID[1]         = (0xdeca) >> 8;
    memcpy(&msg.sourceAddr[0], m_uwb_drv_range.sw_cfg.src_address, ADDR_BYTE_SIZE_L);
    memcpy(&msg.destAddr[0], m_uwb_drv_range.sw_cfg.dest_address, ADDR_BYTE_SIZE_L); 
    msg.messageData[FCODE_POS] = SIMPLE_MSG_TAG_POLL;
    if (m_last_range != -1)
    {
        memcpy(&msg.messageData[POLL_MSG_TOF_POS], &m_last_range, 8);
        length = SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
    }
    else
    {
         length = SIMPLE_MSG_TAG_POLL_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
    }
        
    m_uwb_drv_range.sw_cfg.busy = true;
                
    // wake up
    if (m_uwb_drv_range.sw_cfg.sleep_enabled) 
    {
        wake_up();
    }
        
    // Write the frame data
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(length, (uint8_t*)&msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(length, 0, 1);             /* Zero offset in TX buffer, ranging. */
        
    // Set the delayed rx on time (the response message will be sent after this delay)
    dwt_setrxaftertxdelay(600);
    dwt_setrxtimeout(500);        
        
    // Send frame
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
 
    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_scan_start(void)
{    
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_RESPONDER:
        {
            m_uwb_drv_range.sw_cfg.busy = true;
        
            // wake up
            if (m_uwb_drv_range.sw_cfg.sleep_enabled) wake_up();
        
            //immediate rx enable
            dwt_rxenable(0);     
        }
        break;

        case TDOA_RECEIVER:
        {
            //TODO
        }
        break;
        
        case TWR_INITIATOR:
        case TDOA_TRANSMITTER:
        default:
            return NRF_ERROR_INVALID_STATE;

    }

    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_scan_stop (void)
{
    switch (m_uwb_drv_range.sw_cfg.role)
    {
        case TWR_RESPONDER:
        {
            // Stop rx
            dwt_rxreset();     
        
            // go to sleep
            if (m_uwb_drv_range.sw_cfg.sleep_enabled)    dwt_entersleep();    
        
            m_uwb_drv_range.sw_cfg.busy = false; 
        }
        break;

        case TDOA_RECEIVER:
        {
            //TODO
        }
        break;
        
        case TWR_INITIATOR:
        case TDOA_TRANSMITTER:
        default:
            return NRF_ERROR_INVALID_STATE;

    }

    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_cfg_set (drv_uwb_range_uwb_cfg_t *p_config)
{
    uint32_t err_code;
    dwt_config_t config;
    
    config.chan             = p_config->channel;
    config.dataRate         = p_config->data_rate;
    config.prf              = p_config->prf;
    config.txPreambLength   = p_config->preamble_length;
    config.txCode           = p_config->preamble_code;
    config.rxCode           = p_config->preamble_code;
    config.nsSFD            = p_config->nsSFD;
    config.phrMode          = p_config->phrMode;
    config.rxPAC            = p_config->rx_pac;
    config.sfdTO            = p_config->sfdTO;
    
    // check if driver busy
    if (m_uwb_drv_range.sw_cfg.busy) return NRF_ERROR_FORBIDDEN;
    
    // wake up
    if (m_uwb_drv_range.sw_cfg.sleep_enabled) wake_up();
    
    // Configure dw1000 uwb signal
    dwt_configure(&config);
    
    // Configure dw1000 tx power
    uint32_t power = m_uwb_drv_range.sw_cfg.tx_power[2*(config.chan-1) + config.prf-1];
    if((power == 0x0) || (power == 0xFFFFFFFF)) 
    {
        power = default_tx_power[2*(config.chan-1) + config.prf-1];
    }
    dwt_txconfig_t tx_config;
    tx_config.PGdly = default_pg_delay[config.chan-1];
    tx_config.power = power;
    dwt_configuretxrf(&tx_config);
    
    // go to sleep
    if (m_uwb_drv_range.sw_cfg.sleep_enabled)    dwt_entersleep();    
    
    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_destination_address_set (uint8_t *p_address)
{
    VERIFY_PARAM_NOT_NULL(p_address);
    
    memcpy(&m_uwb_drv_range.sw_cfg.dest_address, p_address, 8);
    
    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_destination_address_get (uint8_t *p_address)
{
    VERIFY_PARAM_NOT_NULL(p_address);
    
    memcpy(p_address, &m_uwb_drv_range.sw_cfg.dest_address, 8);
    
    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_source_address_set (uint8_t *p_address)
{
    VERIFY_PARAM_NOT_NULL(p_address);
    
    memcpy(&m_uwb_drv_range.sw_cfg.src_address, p_address, 8);
    
    return NRF_SUCCESS;    
}

uint32_t drv_uwb_range_source_address_get (uint8_t *p_address)
{
    VERIFY_PARAM_NOT_NULL(p_address);
    
    memcpy(p_address, &m_uwb_drv_range.sw_cfg.src_address, 8);
    
    return NRF_SUCCESS;    
}