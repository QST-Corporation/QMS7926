/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/


/************************************************************** 
 * COPYRIGHT(c)2020, QST
 * All rights reserved. 
 *
 *
 * Module Name:	GPS
 * File name:	gps.c 
 * Brief description:
 *    UART GPS module
 * Author:	QST AE team
 * Data:	2020-12-22
 * Revision:V0.01

****************************************************************/
#include "types.h"
#include "app_wrist.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "gpio.h"
#include "uart.h"
#include "gps.h"
#include "error.h"
#include "log.h"


/*********************************************************************
 * TYPEDEFS
 */
typedef struct _uart_Context{
  bool        enable;
  uint8_t     tx_state;
  uart_Tx_Buf_t tx_buf;
  uart_Cfg_t  cfg;
}uart_Ctx_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
gps_ctx_t mGpsCtx; 

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uart_Ctx_t m_uartCtx;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gps_uart_evt_handler(uart_Evt_t* pev)
{
    gps_ctx_t* pctx = &mGpsCtx;
    //LOG("uart evt: %x\n", pev->type);
    switch(pev->type){
    case UART_EVT_TYPE_RX_DATA:
        if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
          break;
        osal_stop_timerEx(pctx->task_id, WRIST_GPS_RX_TIMEOUT_EVT);
        osal_start_timerEx(pctx->task_id, WRIST_GPS_RX_TIMEOUT_EVT, 10);
        memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
        pctx->hal_uart_rx_size += pev->len;
        break;
    case UART_EVT_TYPE_RX_DATA_TO:
        if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
          break;
        osal_stop_timerEx(pctx->task_id, WRIST_GPS_RX_TIMEOUT_EVT);
        osal_start_timerEx(pctx->task_id, WRIST_GPS_RX_TIMEOUT_EVT, 10);
        memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
        pctx->hal_uart_rx_size += pev->len;
        break;
    case UART_EVT_TYPE_TX_COMPLETED:
        pctx->tx_state = GPS_TX_ST_IDLE;
        break;
    default:
        break;
    }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
int gps_data_transmit(uint8_t* pdata, uint8_t size)
{
  gps_ctx_t* pctx =  &mGpsCtx;
  switch(pctx->tx_state){
  case GPS_TX_ST_IDLE:
    memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
    pctx->tx_size += size;
    hal_uart_send_buff(pctx->tx_buf, pctx->tx_size);
    pctx->tx_size = 0;
    pctx->tx_state = GPS_TX_ST_SENDING;

    break;
  case GPS_TX_ST_DELAY_SLOT:
  case GPS_TX_ST_SENDING:
  {
    memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
    pctx->tx_size += size;
    break;
  }
  default:
    pctx->tx_state = GPS_TX_ST_IDLE;
    break;
  }
  return PPlus_SUCCESS;
}

int gps_receive_handler(void)
{
  gps_ctx_t* pctx =  &mGpsCtx;

  memcpy(pctx->rx_buf + pctx->rx_size, pctx->hal_uart_rx_buf, pctx->hal_uart_rx_size);
  pctx->rx_size += pctx->hal_uart_rx_size;
  pctx->hal_uart_rx_size = 0;
  //use_tx_buf, should disable the DEBUG_INFO, the LOG print will be unavailable:
  /*LOG("gps_rx(%d): ", pctx->rx_size);
  for(uint8_t i=0; i<pctx->rx_size; i++) {
    LOG("0x%X,", pctx->rx_buf[i]);
  }
  LOG("\n");*/
  //gps_data_process();
  //memset(pctx->rx_buf, 0x00, UART_RX_BUF_SIZE);
  pctx->rx_size = 0;
  return PPlus_SUCCESS;
}

int gps_uart_config(uint8 task_id)
{
  gps_ctx_t* pctx =  &mGpsCtx;
  uart_Cfg_t gps_uart_cfg = {
    .tx_pin = P9,
    .rx_pin = P10,
    .rts_pin = GPIO_DUMMY,
    .cts_pin = GPIO_DUMMY,
    .baudrate = 115200,
    .use_fifo = TRUE,
    .hw_fwctrl = FALSE,
    .use_tx_buf = TRUE,
    .parity     = FALSE,
    .evt_handler = gps_uart_evt_handler,
  };

  memset(&mGpsCtx, 0, sizeof(mGpsCtx));
  m_uartCtx.enable = FALSE;
  hal_uart_init(gps_uart_cfg);//uart re-init

  hal_uart_set_tx_buf(pctx->hal_uart_tx_buf, UART_TX_BUF_SIZE);
  pctx->task_id = task_id;

  LOG("gps uart config...\n");
  return PPlus_SUCCESS;
}


/*********************************************************************
*********************************************************************/
