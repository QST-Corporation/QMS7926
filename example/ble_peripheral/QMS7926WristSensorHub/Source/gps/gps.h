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
 * Module Name:	GPS
 * File name:	gps.h 
 * Brief description:
 *    UART GPS module
 * Author:	QST AE team
 * Revision:V0.01

****************************************************************/

#ifndef _GPS_H
#define _GPS_H

/*********************************************************************
 * INCLUDES
 */
#include "types.h"

/*********************************************************************
 * CONSTANTS
 */
#define UART_RX_BUF_SIZE  512
#define UART_TX_BUF_SIZE  512

enum{
  GPS_RX_ST_IDLE = 0,
  GPS_RX_ST_DELAY_SLOT,
  GPS_RX_ST_SENDING
};

enum{
  GPS_TX_ST_IDLE = 0,
  GPS_TX_ST_DELAY_SLOT,
  GPS_TX_ST_SENDING
};

typedef struct{
  uint8_t task_id;
  //uart_rx
  uint8_t rx_state;
  uint8_t rx_size;
  uint8_t rx_buf[UART_RX_BUF_SIZE];

  //uart tx
  uint8_t tx_state;
  uint8_t tx_size;
  uint8_t tx_buf[UART_TX_BUF_SIZE];

  uint8_t hal_uart_rx_size;
  uint8_t hal_uart_rx_buf[UART_RX_BUF_SIZE];
  uint8_t hal_uart_tx_buf[UART_TX_BUF_SIZE];
}gps_ctx_t;

int gps_receive_handler(void);
int gps_data_transmit(uint8_t* pdata, uint8_t size);
int gps_uart_config(uint8 task_id);

#endif


