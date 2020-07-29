 /******************************************************************************
 * @file    ISP3010_UX_TAG_LP.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    08-07-2019
 * @brief   ISP3010_UX_TAG board specific file.
 * @version C
 *
 * @attention
 *  THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef ISP3010_UX_TAG
#define ISP3010_UX_TAG

// ISP3010 internal pinout
#define PIN_DW1000_CLK          6
#define PIN_DW1000_IRQ          7
#define PIN_DW1000_RST          25
#define PIN_DW1000_MOSI         26
#define PIN_DW1000_WAKEUP       28
#define PIN_DW1000_CS           29
#define PIN_DW1000_MISO         30

// Boards specific pinout
#define PIN_WPC_CHG             255
#define PIN_LED_YELLOW          9
#define PIN_LED_RED             10
#define PIN_UART_RX             27
#define PIN_UART_TX             31
#define PIN_BATT_MEAS           3

// other
#define BATT_VOLTAGE_DIV        1
#define BATT_TYPE               0   //0 for CR2032, 1 for LiPo

#endif
