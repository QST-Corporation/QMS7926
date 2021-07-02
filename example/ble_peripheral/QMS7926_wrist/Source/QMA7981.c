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

/*!
*date 2018-03-20
*yufeng.lao
*version V1.0
* @brief this file is used for acceleration sensor drive and interference*/
#include <stdio.h>
#include <stdlib.h>
#include "osal.h"
#include "QMA7981.h"
#include "types.h"
#include "hal_mcu.h"
#include "gpio.h"
#include "error.h"
#include "i2c.h"
#include "log.h"
#include "app_wrist.h"

#define QMA7981_SLAVE_ADDR 0x12

typedef struct _QMA7981_ctx_t{
	bool			module_valid;
	uint32_t		evt_cnt;
	QMA7981_evt_hdl_t evt_hdl;
}QMA7981_ctx_t;

// store come from pedometer parm config
static QMA7981_ctx_t s_QMA7981_ctx;


// store ACC data;
uint8_t  acc_data[1*6];

void QMA7981_timer_stop(void)
{
}
void QMA7981_timer_start(int ms)
{
  osal_start_timerEx(AppWrist_TaskID, ACC_DATA_EVT, ms);
}


void QMA7981_delay_ms(int ms)
{	
	volatile int i = 4500;
	volatile int loop = ms;
	
  while(loop) 
  { 
		loop--; 
		for(; i; i--);
	} 
}


static void* QST_i2c_init(void)
{
  void* pi2c;
	hal_i2c_pin_init(I2C_0, P28, P26);
  pi2c = hal_i2c_init(I2C_0,I2C_CLOCK_400K);
  return pi2c;
}
static int QST_i2c_deinit(void* pi2c)
{
  int ret;
  ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P26,IE);
  hal_gpio_pin_init(P28,IE);
	return ret;
}


static int QST_i2c_read(void* pi2c, uint8_t reg, uint8_t* data, uint8_t size)
{
 	return hal_i2c_read(pi2c, QMA7981_SLAVE_ADDR, reg, data, size);
}

static int QST_i2c_write(void* pi2c, uint8_t reg, uint8_t val)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = val;
  hal_i2c_addr_update(pi2c, QMA7981_SLAVE_ADDR);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 2);
    HAL_EXIT_CRITICAL_SECTION();
  }
  return hal_i2c_wait_tx_completed(pi2c);
}


uint16_t QMA7981_fetch_acc_data(void)
{
	QMA7981_ev_t ev;
	uint8_t val;

	void* pi2c = QST_i2c_init();
    
    
    QST_i2c_read(pi2c, QMA7981_DATA_LOWBYTE_X,acc_data,6);
    
	QST_i2c_deinit(pi2c);
    
    acc_data[0] &= 0xFE;    
    acc_data[2] &= 0xFE;
    acc_data[4] &= 0xFE;
    
	ev.ev = wmi_event;
	ev.size = 6;
	ev.data = acc_data;
	s_QMA7981_ctx.evt_hdl(&ev);
	return val;
}


uint8_t drv_QMA7981_event_handle()
{
	uint8_t val;

	void* pi2c;
  QMA7981_timer_start(400);
  pi2c = QST_i2c_init();
    
    QST_i2c_read(pi2c, QMA7981_INT_ST2,&val, 1);
	QST_i2c_deinit(pi2c);

	QMA7981_fetch_acc_data();
    return 0;
}


	
void QMA7981_int_hdl(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(type == POSEDGE)
  {
		s_QMA7981_ctx.evt_cnt ++;
    osal_set_event(AppWrist_TaskID, ACC_DATA_EVT);
	}
}



void QMA7981_timeout_hdl(void *parm)
{
  QMA7981_fetch_acc_data();
}			
	

/*!
*	@brief this funtion is used for initialize QMA7981 register,and initialize GPIOE，and regeister event handler
*
*/

int QMA7981_config(void* pi2c)
{
	uint8_t  val;
	/***************verify proper integraterd circuit******************************/
	QST_i2c_read(pi2c, QMA7981_CHIP_ID,&val,1);
	if(val!=0xE7)
		return PPlus_ERR_IO_FAIL;
	QMA7981_delay_ms(50);

	val = 0xF0;    				// Enable sensor  range  8g  low power mode
	QST_i2c_write(pi2c, QMA7981_PM,val); 

	val = 0x04;    				// Range  8g
	QST_i2c_write(pi2c, QMA7981_FSR,val); 
    
	val = 0x05;			//Configure ODR, 500KHz / 15375
	QST_i2c_write(pi2c, QMA7981_BW,val);
	

    val = 0xF0 ;			// Enable data ready interrupt
	QST_i2c_write(pi2c, QMA7981_INT_EN1,val);
    
    val = 0x72;			// Map interrupt pin 1
	QST_i2c_write(pi2c, QMA7981_INT_MAP1,val);

    val = 0x05;			// Rising edge interrupt.
	QST_i2c_write(pi2c, QMA7981_INT_PIN_CFG,val);
    
    
    val = 0xDD;			// clear interrupt flag when read any status, enable shadow, latch mode.
	QST_i2c_write(pi2c, QMA7981_INT_CFG,val);


	QST_i2c_write(pi2c, QMA7981_PM, 0xF0);
	QMA7981_delay_ms(50);

    QST_i2c_read(pi2c, QMA7981_INT_ST2,&val, 1);      /* Read data ready interrupt status */
	
	QMA7981_delay_ms(50);
  QMA7981_fetch_acc_data();

	return PPlus_SUCCESS;

}


int QMA7981_enable(void)
{
	int ret = 0;
	void* pi2c = QST_i2c_init();
    ret = QMA7981_config(pi2c);
	LOG("QMA7981_enable is %d\n",ret);
	QST_i2c_deinit(pi2c);
	return ret;
}

int QMA7981_disable(void)
{
	void* pi2c = QST_i2c_init();
    QST_i2c_write(pi2c, QMA7981_PM, 0x40);
	QST_i2c_deinit(pi2c);
	return PPlus_SUCCESS;
}


int QMA7981_init(QMA7981_evt_hdl_t evt_hdl)
{
	int ret = PPlus_SUCCESS;
	//copy pcfg
	s_QMA7981_ctx.evt_hdl = evt_hdl;
	
									
	ret = hal_gpioin_register(P5, QMA7981_int_hdl, NULL );//pin_event_handler);
	
	ret = QMA7981_enable();
  QMA7981_timer_start(400);

	if(ret != PPlus_SUCCESS){
		s_QMA7981_ctx.module_valid = false;
		return ret;
	}
	s_QMA7981_ctx.module_valid = true;

  
	return PPlus_SUCCESS;
}
