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

void host_spi_deinit(void)
{
  hal_spi_bus_deinit(&host_spi);
}

void host_spi_command_handler(void)
{
  int len;
  uint8_t data[8];

  len = hal_spis_rx_len(&host_spi);
  if(len){
    hal_spis_read_rxn(&host_spi, data, len);
  }

  LOG("\n==== Host command:");
  for(uint8_t i=0; i<len; i++)
  LOG("0x%02x ",data[i]);
  LOG("\n");

  switch(data[0])
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
      LOG("Unsupported command:0x%x\n", data[0]);
    break;
  }
}

int host_spi_init(void)
{
  spi_Cfg_t cfg = {
      .sclk_pin = P34,
      .ssn_pin = P31,
      .MOSI = P32,
      .MISO = P33,
      .baudrate = 12000000,
      .spi_tmod = SPI_TRXD,
      .spi_scmod = SPI_MODE0,
      .force_cs = false,

      .int_mode = false,
      .evt_handler = NULL,
    };
  hal_spis_bus_init(&host_spi, cfg);

  return PPlus_SUCCESS;
}

void host_wakeup_register(hostwakeCB_t cb)
{
  hal_gpio_pin_init(P7, IE);
  hal_gpio_pull_set(P7, PULL_DOWN);
  hal_gpioin_register(P7, cb, NULL);
  hal_pwrmgr_register(MOD_USR1, NULL, NULL);
}


