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
 *
 * Module Name:	host communication
 * File name:	host_comm.h 
 * Brief description:
 *    host communication processing module
 * Author:	QST AE team
 * Revision:V0.01

****************************************************************/

#ifndef _HOST_COMM_H
#define _HOST_COMM_H

#include "types.h"

typedef void(* hostwakeCB_t)(GPIO_Pin_e pin,IO_Wakeup_Pol_e type);

#define BUF_LEN             256

typedef enum{
    HOST_TX_COMPLETED = 1,
    HOST_RX_COMPLETED,
    HOST_RX_S
}ev_t;

typedef struct
{
    ev_t      ev;
    uint16_t* data;
}host_ev_t;

typedef void(* host_spis_cb_t)(host_ev_t* pev);

typedef struct _host_ctx_t{
    bool            module_valid;
    uint8_t         tx_buf[BUF_LEN];
    uint8_t         rx_buf[BUF_LEN];
    uint16_t        rx_len;
    host_spis_cb_t  host_spis_cb;
}host_ctx_t;

typedef enum {
    HOST_STEP_COUNT = 0x01,  //0x01
    HOST_SLEEP_MONITOR,      //0x02
    HOST_GPS,                //0x03
    HOST_HEARTRATE,          //0x04
    HOST_SPO2,               //0x05
    HOST_BLOOD_PRESSURE,     //0x06
    HOST_STEP_CONTROL = 0x31,
    HOST_SLEEP_CONTROL = 0x32
} host_cmd_t;

void host_spi_deinit(void);
void host_spi_command_handler(void);
int host_spi_init(void);
void host_wakeup_register(hostwakeCB_t cb);
#endif


