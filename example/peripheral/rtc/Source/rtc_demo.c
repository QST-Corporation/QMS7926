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

/**************************************************************************************************
  Filename:       rtc_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "rtc_demo.h"
#include "log.h"

#include "datetime/rtc_datetime.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"

/*********************************************************************
 * Rtc_Task: RTC demo
 * 
 */
static uint8 Rtc_TaskID;

void Rtc_Demo_Init(uint8 task_id)
{

  Rtc_TaskID = task_id;
  LOG("RTC demo start...\n");

  rtc_datetime_init();
  osal_start_timerEx(task_id, RTC_TIMER_DT_EVT, RTC_DATETIME_SYNC_INTERVAL);
}

uint16 Rtc_ProcessEvent( uint8 task_id, uint16 events )
{

  if(task_id != Rtc_TaskID){
    return 0;
  }

  if ( events & RTC_TIMER_DT_EVT ) {
    rtc_datetime_sync_handler();
    // reload datetime sync timer
    osal_start_timerEx(task_id, RTC_TIMER_DT_EVT, RTC_DATETIME_SYNC_INTERVAL);

    return (events ^ RTC_TIMER_DT_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
