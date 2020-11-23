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

#ifndef _GSENSOR_H
#define _GSENSOR_H


#include <stdbool.h>
#include <stdint.h>


//BUF_M1, BUF_M0 selects the operating mode of the sample buffer per Table 23.



enum{
	wufs_event = 0x01,	//
	stap_event = 0x02,	//single tap
	dtap_event = 0x04,	//double tap
	tps_event  = 0x08,
	wmi_event  = 0x10,	//acceleration data
	tilt_event  = 0x20	//tilt event
};



typedef struct _gsensor_ev_t{
	uint8_t	ev;
	uint8_t	flg;
	uint8_t size;
	void* 	data;
}gsensor_ev_t;

typedef void (*gsensor_evt_hdl_t)	(gsensor_ev_t* pev);


uint16_t gsensor_fetch_acc_data(void);
uint8_t drv_gsensor_event_handle(void);
int gsensor_enable_tilt(bool en);
int gsensor_enable(void);
int gsensor_disable(void);
int gsensor_init(gsensor_evt_hdl_t evt_hdl);


#endif   /* _drv_GSENSOR_H_ */











