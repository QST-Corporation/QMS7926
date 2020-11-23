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
 *
 * Module Name:	key 
 * File name:	key.h 
 * Brief description:
 *    key driver module
 * Author:	QST AE team
 * Data:	2017-07-01
 * Revision:V0.01
****************************************************************/

#ifndef __KEY_H__
#define __KEY_H__

#include "types.h"
#include "gpio.h"

#define KEY_NUM 2

typedef enum{
	STATE_KEY_IDLE = 0x00,
	STATE_KEY_PRESS_DEBONCE = 0x01,
	STATE_KEY_PRESS = 0x02,
	STATE_KEY_RELEASE_DEBONCE = 0x03,
}key_state_e;

typedef enum{
	KEY_EVT_IDLE = 0x0000,
	KEY_EVT_DEBONCE_PROCESS = 0x0001,
	KEY_EVT_PRESS   = 0x0002,
	KEY_EVT_RELEASE = 0x0004,
	KEY_EVT_SHORT_PRESS = 0x0008,
	KEY_EVT_LONG_PRESS = 0x0010,
	KEY_EVT_LONG_RELEASE = 0x0020,
} key_evt_t;

typedef enum{
	LOW_IDLE = 0x00,
	HIGH_IDLE = 0x01,
}idle_level_e;

typedef void (* key_callbank_hdl_t)(uint8_t,key_evt_t);

typedef struct gpio_key_t{
	GPIO_Pin_e 			pin;
	key_state_e    	state;
	idle_level_e 		idle_level;
	
}gpio_key;

typedef struct gpio_internal_t{
	uint32_t 	timer_tick;	
	bool     	in_enable;
	key_evt_t info_inter;
	
}gpio_internal;

typedef struct key_state{
	gpio_key 						key[KEY_NUM];
	gpio_internal 			temp[KEY_NUM];
	uint8_t  						task_id;
	key_callbank_hdl_t  key_callbank;
	
}key_contex_t;


void key_init(void);
void gpio_key_timer_handler(uint8 index);
extern key_contex_t key_state;
#endif

