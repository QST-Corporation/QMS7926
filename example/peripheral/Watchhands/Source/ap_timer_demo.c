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
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "ap_timer_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "spi.h"
#include "error.h"
#include "spiflash.h"
#include "pwrmgr.h"
#include "ap_timer.h"

static uint8 ap_timer_TaskID; 
uint8_t cycle_num = 0;

static uint16 duty_tim[] = {310, 160};
void timer_int_handler(uint8_t evt)
{
    if(cycle_num == 12)
        ap_timer_set(3, 28000);
    else
        ap_timer_set(3, duty_tim[cycle_num & 1]);
    
    if(cycle_num < 12)
        hal_gpio_write(P14, !(cycle_num & 1));
    else if(cycle_num > 12 && cycle_num < 26)
        hal_gpio_write(P15, !(cycle_num & 1));
    else if(cycle_num == 26 ){
        ap_timer_set(3, 1000000);
        cycle_num = 0xFF;
    }
    cycle_num++;

}

/*
static uint16 duty_tim[] = {220, 220, 1080, 2300, 2800, 220, 220, 26000};
void timer_int_handler(uint8_t evt)
{
    ap_timer_set(3, duty_tim[cycle_num & 7]);
    if(cycle_num & 8){
        cycle_num++;
        if ((cycle_num & 7) == 4)
            hal_gpio_write(P15, 1);
        else hal_gpio_write(P15, 0);
        hal_gpio_write(P14, (cycle_num & 1));
    }
    else{
        cycle_num++;
        if ((cycle_num & 7) == 4)
            hal_gpio_write(P14, 1);
        else hal_gpio_write(P14, 0);
        hal_gpio_write(P15, (cycle_num & 1));
    }

}

*/

/*********************************************************************
 * @fn      AP_TIMER_Demo_Init
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 */
 void AP_TIMER_Demo_Init( uint8 task_id ){
	ap_timer_TaskID = task_id;
	 
	LOG("when test this case,you can uncomment comment which in timer int function\n");
  osal_start_reload_timer( ap_timer_TaskID, TIMER_1000_MS_EVT, 1000);
}



uint16 AP_TIMER_Demo_ProcessEvent( uint8 task_id, uint16 events )
{    
    hal_gpio_write(P14, 0);
    hal_gpio_write(P15, 0);
    hal_gpio_pull_set(P14, STRONG_PULL_UP);
    hal_gpio_pull_set(P15, STRONG_PULL_UP);
    
    if (events & TIMER_1000_MS_EVT )
	{
		ap_timer_init(timer_int_handler);
        ap_timer_clear(3);
        ap_timer_set(3, 28000);
        
        while(1);
		
		return (events ^ TIMER_1000_MS_EVT);
	}  

	return 0;
}
