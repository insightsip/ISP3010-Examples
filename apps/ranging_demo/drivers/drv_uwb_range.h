/******************************************************************************
 * @file    drv_uwb_range.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    09-07-2019
 * @brief   Range driver header
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
 
#ifndef __DRV_UWB_RANGE_H__
#define __DRV_UWB_RANGE_H__

#include "nrf_drv_spi.h"
#include <stdint.h>


#define DRV_UWB_RANGE_PIN_NOT_USED  0xFF

// Simple 2WR function codes
#define SIMPLE_MSG_TAG_POLL         (0x51)      // ISP Tag poll message
#define SIMPLE_MSG_ANCH_RESP        (0x52)      // ISP Anchor response to poll

// lengths including the ranging Message Function Code byte
#define SIMPLE_MSG_TAG_POLL_LEN             1           // FunctionCode(1),
#define SIMPLE_MSG_TAG_POLL_WITH_RG_LEN     9           // FunctionCode(1), Range(8),
#define SIMPLE_MSG_ANCH_RESP_LEN            9           // FunctionCode(1), poll message reception timestamp(4), response message transmission timestamp(4)

#define STANDARD_FRAME_SIZE         127
#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)
#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC                   2
#define FRAME_SOURCE_ADDRESS_S      (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L      (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP                 (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)         //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)     //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS   (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC)      //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC)      //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS  (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC)     //127 - 15 - 16 - 2 = 94

// Function code byte offset (valid for all message types).
#define FCODE_POS                   0               // Function code is 1st byte of messageData
#define POLL_MSG_TOF_POS            1               // ToF is 2nd to 6th byte of messageData

// Simple anchor response byte offsets.
#define POLL_RX_TS                  1               // Poll message reception timestamp(2)
#define RESP_TX_TS                  5               // Response message transmission timestamp(2)

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT              299702547

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME             65536

#define POLL_RX_TO_RESP_TX_DLY_UUS  1000


/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY          0x4030
#define RX_ANT_DLY          0x4030 
#define ANT_DLY             0x4030
// value of internal antenna of ips1510

/* configuration defines */
#define DRV_RANGE_PRF_16M         1       /**< equals to DWT_PRF_16M */
#define DRV_RANGE_PRF_64M         2       /**< equals to DWT_PRF_64M */
#define DRV_RANGE_PLEN_4096       0x0C    /**< equals to DWT_PLEN_4096 */
#define DRV_RANGE_PLEN_2048       0x28    /**< equals to DWT_PLEN_2048 */
#define DRV_RANGE_PLEN_1536       0x18    /**< equals to DWT_PLEN_1536 */
#define DRV_RANGE_PLEN_1024       0x08    /**< equals to DWT_PLEN_1024 */
#define DRV_RANGE_PLEN_512        0x34    /**< equals to DWT_PLEN_512 */
#define DRV_RANGE_PLEN_256        0x24    /**< equals to DWT_PLEN_256 */
#define DRV_RANGE_PLEN_128        0x14    /**< equals to DWT_PLEN_128 */
#define DRV_RANGE_PLEN_64         0x04    /**< equals to DWT_PLEN_64 */
#define DRV_RANGE_BR_110K         0       /**< equals to DWT_BR_110K */
#define DRV_RANGE_BR_850K         1       /**< equals to DWT_BR_850K */
#define DRV_RANGE_BR_6M8          2       /**< equals to DWT_BR_6M8 */
#define DRV_RANGE_PHRMODE_STD     0x0     /**< equals to DWT_PHRMODE_STD */
#define DRV_RANGE_PHRMODE_EXT     0x3     /**< equals to DWT_PHRMODE_STD */
#define DRV_RANGE_PAC8            0       /**< equals to DWT_PAC8 */
#define DRV_RANGE_PAC16           1       /**< equals to DWT_PAC16 */
#define DRV_RANGE_PAC32           2       /**< equals to DWT_PAC32 */
#define DRV_RANGE_PAC64           3       /**< equals to DWT_PAC64 */


typedef struct
{
    uint8_t channel;                /**< channel number {1, 2, 3, 4, 5, 7 }    */
    uint8_t prf;                    /**< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}    */
    uint8_t preamble_length;        /**< DWT_PLEN_64..DWT_PLEN_4096    */
    uint8_t preamble_code ;         /**< TX preamble code    */
    uint8_t data_rate ;             /**< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}    */
    uint8_t nsSFD ;                 /**< Boolean should we use non-standard SFD for better performance    */
    uint8_t rx_pac ;                /**< Acquisition Chunk Size (Relates to RX preamble length)    */
    uint8_t sfdTO ;                 /**< SFD timeout value (in symbols)    */
    uint8_t phrMode ;               /**< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT} */
} drv_uwb_range_uwb_cfg_t ;

// simple 802.15.4 frame structure - using long addresses
typedef struct
{
    uint8_t frameCtrl[2];            /**<  frame control bytes 00-01 */
    uint8_t seqNum;                  /**<  sequence_number 02 */
    uint8_t panID[2];                /**<  PAN ID 03-04 */
    uint8_t destAddr[8];             /**<  05-12 using 64 bit addresses */
    uint8_t sourceAddr[8];           /**<  13-20 using 64 bit addresses */
    uint8_t messageData[88] ;        /**<  22-124 (application data and any user payload) */
    uint8_t fcs[2] ;                 /**<  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes. */
} srd_msg_dlsl ;

/**@brief uwb roles.
 */
typedef enum 
{
    TWR_INITIATOR, 
    TWR_RESPONDER, 
    TDOA_TRANSMITTER,
    TDOA_RECEIVER,
    NUM_ROLES
} drv_uwb_range_role_t;

/**@brief uwb event types.
 */
typedef enum
{
    DRV_RANGE_EVT_DATA,     /**< new range */
    DRV_RANGE_EVT_TIMEOUT,  /**< Time out */    
    DRV_RANGE_EVT_ERROR     /**< Error in frame*/
}drv_uwb_range_evt_type_t;

/**@brief uwb event struct.
 */
typedef struct
{
    drv_uwb_range_evt_type_t type;
    double range;
}drv_uwb_range_evt_t;

/**@brief range driver event handler callback type.
 */
typedef void (*drv_uwb_range_evt_handler_t)(drv_uwb_range_evt_t const * p_evt);

/**@brief Initialization struct for uwb driver.
 */
typedef struct
{
    uint32_t                         pin_irq;           /**< Interrupt pin. */
    uint32_t                         pin_wakeup;        /**< Wake up pin. */
    uint32_t                         pin_reset;         /**< Reset pin. */
    uint32_t                         pin_mosi;          /**< Mosi pin. */
    uint32_t                         pin_miso;          /**< Miso up pin. */
    uint32_t                         pin_clk;           /**< Clk up pin. */
    uint32_t                         pin_cs;            /**< CS up pin. */
    drv_uwb_range_evt_handler_t      evt_handler;      /**< Event handler - called after a pin interrupt has been detected. */
    drv_uwb_range_role_t             role;
    uint8_t                 const *  own_address;
    uint8_t                 const *  reply_address;
    bool                             enable_sleep;
    bool                             enable_filter;
}drv_uwb_range_init_t;

/**@brief Function for initializing the uwb driver.
 *
 * @param[in] p_params                  Pointer to init parameters.
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid state.
 */
uint32_t drv_uwb_range_init (drv_uwb_range_init_t *p_params);

/**@brief Function for requesting a range (TAG only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_FORBIDDEN          If the driver is in invalid role.
 */
uint32_t drv_uwb_range_request(void);

/**@brief Function for starting a scan for range request (ANCHOR only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid role.
 */
uint32_t drv_uwb_range_scan_start(void);

/**@brief Function for stopping a scan for range request (ANCHOR only function)
 *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid role.
 */
uint32_t drv_uwb_range_scan_stop(void);

/**@brief Function for setting new uwb parameters
 *
 * @param[in] p_config                  Pointer to config parameters.
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE      If the driver is in invalid role.
 */
uint32_t drv_uwb_range_cfg_set(drv_uwb_range_uwb_cfg_t *p_config);

/**@brief Function for setting a new destination address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_destination_address_set(uint8_t *p_address);

/**@brief Function for getting a new destination address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_destination_address_get(uint8_t *p_address);


/**@brief Function for setting a new source address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_source_address_set(uint8_t *p_address);

/**@brief Function for getting a new source address
 *
 * @param[in] p_config                  Pointer to the address
  *
 * @retval NRF_SUCCESS                  If initialization was successful.
 */
uint32_t drv_uwb_range_source_address_get(uint8_t *p_address);
#endif
