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

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"


/*********************************************************************
 * timer_Task
 * Task timer sample code
 */
static uint8 timer_TaskID; 
#define TIMER_1S_ONCE                                0x0001
#define TIMER_2S_CYCLE                               0x0004

void Timer_Demo_Init( uint8 task_id ){
  timer_TaskID = task_id;

  osal_start_timerEx( timer_TaskID, TIMER_1S_ONCE , 1000);
  osal_start_reload_timer( timer_TaskID, TIMER_2S_CYCLE , 2000);
}

uint16 Timer_Demo_ProcessEvent( uint8 task_id, uint16 events ){
  static uint8 count1 = 0,count2 = 0;
  static bool  timer_cycle_enable = TRUE;

  if(task_id != timer_TaskID){
    return 0;
  }

  if ( events & TIMER_1S_ONCE ){
    LOG("1s:once only mode\n");
    osal_start_timerEx( timer_TaskID, TIMER_1S_ONCE , 1000);
    
    if(timer_cycle_enable == FALSE){
      if(++count1 >= 10 ){
        osal_start_reload_timer( timer_TaskID, TIMER_2S_CYCLE , 2000);
        
        LOG("2s:recycle mode start\n");
        timer_cycle_enable = TRUE;
        count1 = 0;
      }
    }
    return (events ^ TIMER_1S_ONCE);
  }  

    if ( events & TIMER_2S_CYCLE ){
      LOG("2s:recycle mode\n");
      if(++count2 >= 5 ){
        osal_stop_timerEx(timer_TaskID, TIMER_2S_CYCLE);
        
        LOG("2s:recycle mode stop\n");
        timer_cycle_enable = FALSE;
        count2 = 0;
      }
      
    return (events ^ TIMER_2S_CYCLE);
  }  

  return 0;
}

/*********************************************************************
 * Rtc_Task: RTC demo
 * 
 */
static uint8 Rtc_TaskID;

void Rtc_Demo_Init(uint8 task_id)
{

  Rtc_TaskID = task_id;
  LOG("rtc demo start...\n");

  /*
  1, remove onboard XTAL 32K, use the internal RCOSC 32K;
  2, config the RTC clock as CLK_32K_XTAL, then check the RTC performence
  */
}

uint16 Rtc_ProcessEvent( uint8 task_id, uint16 events )
{

  if(task_id != Rtc_TaskID){
    return 0;
  }
/*
  if( events & KEY_EVT_PRESS){
    LOG("\n%-22s","KEY_EVT_PRESS:");
      for (i = 0; i < KEY_NUM; ++i){
      if(key_state.temp[i].info_inter & KEY_EVT_PRESS){
        LOG("key:%d gpio:%d	",i,key_state.key[i].pin);
        key_state.temp[i].info_inter &= ~KEY_EVT_PRESS;
      }
    }
    return (events ^ KEY_EVT_PRESS);
  }
*/
  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
