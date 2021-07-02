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
 * Module Name:	host communication
 * File name:	host_comm.c 
 * Brief description:
 *    host communication processing module
 * Author:	QST AE team
 * Data:	2020-12-18
 * Revision:V0.01

****************************************************************/
#include "types.h"
#include "app_wrist.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "gpio.h"
#include "spi.h"
#include "host_comm.h"
#include "pwrmgr.h"
#include "error.h"
#include "log.h"

static hal_spi_t host_spi = {.spi_index = SPI0};
static host_ctx_t s_host_ctx;

int spis_write_tx_buf(uint8 *data,uint16 len)
{
  uint8 ret = 0;
  host_ctx_t *pctx = &s_host_ctx;

  hal_spi_set_tx_buffer(&host_spi, pctx->tx_buf, BUF_LEN);
  ret = hal_spi_transmit(&host_spi, data, pctx->rx_buf, len);
  LOG("tx len:%d, ret:%d\n\n",len, ret);
  return ret;
}

void spis_int_handler(spi_evt_t* evt)
{
  host_ctx_t *pctx = &s_host_ctx;
  switch(evt->evt){
  case SPI_TX_COMPLETED:
    osal_set_event(AppWrist_TaskID, SLAVE_TX_COMPLETED_EVT);
    break;
  case SPI_RX_COMPLETED:
    osal_set_event(AppWrist_TaskID, SLAVE_RX_COMPLETED_EVT);
    break;
  case SPI_RX_DATA_S:
    memcpy(pctx->rx_buf+pctx->rx_len, evt->data, evt->len);
    pctx->rx_len += evt->len;
    //LOG("rx %d %d\n", evt->len, pctx->rx_len);
    if(pctx->rx_len == 6){ //Host command len should less than 8
      osal_set_event(AppWrist_TaskID, HOST_CMD_EVT);
      //spi_int_disable(&host_spi);
      //pctx->rx_len = 0;
    }else if(pctx->rx_len > 7){
      pctx->rx_len = 0;
    }
    break;
  }
}

void host_spi_deinit(void)
{
  hal_spi_bus_deinit(&host_spi);
}

void send_fixed_data(void)
{
  uint8_t array_send[20] = {0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC};
  spis_write_tx_buf(array_send, sizeof(array_send));
}

void host_spi_command_handler(void)
{
  host_ctx_t *pctx = &s_host_ctx;
  uint8_t rx_len = pctx->rx_len;

  if((pctx->rx_buf[0] == 0x00) &&
    (pctx->rx_buf[1] == 0x01) &&
    (pctx->rx_buf[2] == 0x02) &&
    (pctx->rx_buf[3] == 0x03) &&
    (pctx->rx_buf[4] == 0x04) &&
    (pctx->rx_buf[5] == 0x05))
   {
      send_fixed_data(); // for test purpose
   }
  LOG("\n==== Host command(%d):", rx_len);
  for(uint8_t i=0; i<6; i++) {
    LOG("0x%02x ",pctx->rx_buf[i]);
  }
  LOG("\n");

  switch(pctx->rx_buf[0])
  {
    case HOST_STEP_COUNT:
    break;
    case HOST_SLEEP_MONITOR:
    break;
    case HOST_GPS:
    break;
    case HOST_HEARTRATE:
    break;
    case HOST_SPO2:
    break;
    case HOST_BLOOD_PRESSURE:
    break;
    case HOST_STEP_CONTROL:
    break;
    case HOST_SLEEP_CONTROL:
    break;
    default:
      LOG("Unsupported command:0x%x\n", pctx->rx_buf[0]);
    break;
  }
  pctx->rx_len = 0;
}

int host_spi_init(void)
{
  spi_Cfg_t cfg = {
      .sclk_pin = P34,
      .ssn_pin = P31,
      .MOSI = P32,
      .MISO = P33,
      .baudrate = 1000000,
      .spi_tmod = SPI_TRXD,
      .spi_scmod = SPI_MODE3, //Only mode3 and mode1 could work with slave mode.
      .force_cs = true,

      .int_mode = true,
      .evt_handler = spis_int_handler,
    };
//LOG("h_init\n");
hal_gpioin_unregister(P7);
  hal_spis_bus_init(&host_spi, cfg);
  memset(&s_host_ctx, 0x00, sizeof(s_host_ctx));
  return PPlus_SUCCESS;
}

void host_wakeup_register(hostwakeCB_t cb)
{
  hal_gpio_pin_init(P7, IE);
  hal_gpio_pull_set(P7, PULL_DOWN);
  hal_gpioin_register(P7, cb, NULL);
  hal_pwrmgr_register(MOD_USR1, NULL, NULL);
}


