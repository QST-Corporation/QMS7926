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

/*******************************************************************************
* @file			i2c_slave Raw Pass Proj
* @brief		Data transmission use IIC Slave
* @version		1.0
* @date			2018/7/5  
* 
* Copyright(C) 2016, QST Corporation All rights reserved.
*******************************************************************************/

/*******************************************************************************
*@ Module    		:  Head File Include
*@ Description    :  
*******************************************************************************/
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "ll.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "uart.h"
#include "i2c.h"
#include "log.h"

/*******************************************************************************
*@ Module    		:  Functions Statement
*@ Description    :  NULL
*******************************************************************************/
extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);

/*******************************************************************************
*@ Module    		:  Variable Statement
*@ Description    :  NULL
*******************************************************************************/
extern int         pclk;
volatile uint8 g_current_advType;

/*******************************************************************************
*@ Description    :  RF wakeuo handler
*@ Input          :  NULL
*@ Output         :  NULL
*@ Return         :  None
*******************************************************************************/
static void rf_wakeup_handler(void)
{
  NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
  NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);
}

/*******************************************************************************
*@ Description    :  MCU RF Initialization
*@					 WirelessTxPower Datarate ROMCode GPIO DC IRQ(BB,TIMER)
*@                   Pwrmgr
*@ Input          :  NULL
*@ Output         :  NULL
*@ Return         :  None
*******************************************************************************/
static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;

    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    
    hal_rom_code_ini();
    
    //Quick Boot setting and 
    //  3'b1xx: 62.5us.  control bits for main digital part reset 
    //  wait time after power up charge pump. 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;      

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw  

    DCDC_CONFIG_SETTING(0x0d);
	
    NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);

    hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);
}


/*******************************************************************************
*@ Description    :  Hal Initialization(System Clock,LOG,RTC,GPIO,)
*@ Input          :  NULL
*@ Output         :  NULL
*@ Return         :  None
*******************************************************************************/
static void hal_init(void)
{
    hal_system_init(g_system_clk);
    LOG_INIT();
    hal_rtc_clock_config(CLK_32K_XTAL);
    hal_gpio_init();
    hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);
}

/*******************************************************************************
*@ Description    :  main()
*@ Input          :  NULL
*@ Output         :  NULL
*@ Return         :  Should not return from here
*******************************************************************************/
int  main(void)  
{     
    g_system_clk = SYS_CLK_XTAL_16M;
    
    init_config();

    hal_pwrmgr_init();

    hal_rfphy_init();
    
    hal_init();

    app_main();	
    
}
