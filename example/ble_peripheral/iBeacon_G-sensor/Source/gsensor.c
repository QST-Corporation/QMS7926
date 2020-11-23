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
#include "gsensor.h"
#include "types.h"
#include "hal_mcu.h"
#include "gpio.h"
#include "error.h"
#include "i2c.h"
#include "log.h"

#include "simpleBLEPeripheral.h"

#define GSENSOR_SLAVE_ADDR 0x12                   //QMS7926

#define I2C_SDA_PIN    P28
#define I2C_CLK_PIN    P26

#define GRAVITY_EARTH_1000          9807
#define ACC_lSB_SIV                 2048 /*2G,4096;4G,2048;8G,1024;16G,512;32G,120*/

typedef struct __ctx_t{
	bool			module_valid;
	uint32_t		evt_cnt;
	gsensor_evt_hdl_t evt_hdl;
}gsensor_ctx_t;

// store come from pedometer parm config
static gsensor_ctx_t s_gsensor_ctx;


// store ACC data;
int32_t  acc_data[4];
uint8_t  acc_printf_data[64];

void gsensor_delay_ms(int ms)
{	
	volatile int i = 4500;
	volatile int loop = ms;
	
	while(loop) 
	{ 
		loop--; 
		for(; i; i--);
	} 
}

static void* gsensor_i2c_init(void)
{
  void* pi2c;
	hal_i2c_pin_init(I2C_0, I2C_SDA_PIN, I2C_CLK_PIN);
	pi2c = hal_i2c_init(I2C_0,I2C_CLOCK_400K);
  return pi2c;
}

static int gsensor_i2c_deinit(void* pi2c)
{
  int ret;
	ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(I2C_SDA_PIN, IE);
	hal_gpio_pin_init(I2C_CLK_PIN, IE);
	return ret;
}

static int gsensor_i2c_read(void* pi2c, uint8_t reg, uint8_t* data, uint8_t size)
{
 	return hal_i2c_read(pi2c, GSENSOR_SLAVE_ADDR, reg, data, size);
}

static int gsensor_i2c_write(void* pi2c, uint8_t reg, uint8_t val)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = val;
  hal_i2c_addr_update(pi2c, GSENSOR_SLAVE_ADDR);
  {
//    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 2);
//    HAL_EXIT_CRITICAL_SECTION();
  }
  return hal_i2c_wait_tx_completed(pi2c);
}

uint8_t gsensor_read_acc_data(void)
{
	uint8_t ret;
	gsensor_ev_t ev;
	uint8_t reg_data[6];
	int16_t raw[3];
	int32_t raw_32[3];
	uint8_t pin_status;
	float   x_y_z_off[3];
	int32_t   x_y_z_off_error[3];

	void* pi2c = gsensor_i2c_init();
	
	ret = gsensor_i2c_read(pi2c, 0x01, reg_data, 6);
	if(ret != PPlus_SUCCESS)
	{
		gsensor_i2c_deinit(pi2c);
		return PPlus_ERR_NOT_FOUND;
	}

 	raw[0] = (int16_t)((int16_t)(reg_data[1]<<8)|(int16_t)(reg_data[0]));
	raw[1] = (int16_t)((int16_t)(reg_data[3]<<8)|(int16_t)(reg_data[2]));
	raw[2] = (int16_t)((int16_t)(reg_data[5]<<8)|(int16_t)(reg_data[4]));
	
	raw_32[0] = raw[0]>>2;
	raw_32[1] = raw[1]>>2;
	raw_32[2] = raw[2]>>2;

	acc_data[0] = (raw_32[0] * GRAVITY_EARTH_1000) / (ACC_lSB_SIV);
	acc_data[1] = (raw_32[1] * GRAVITY_EARTH_1000) / (ACC_lSB_SIV);
	acc_data[2] = (raw_32[2] * GRAVITY_EARTH_1000) / (ACC_lSB_SIV);

	LOG("acc data:%8d\t%8d\t%8d\n", acc_data[0], acc_data[1], acc_data[2]);
	
	osal_memset((char *)acc_printf_data, 0, 40);
	
	hal_gpio_pin_init(P13,IE);
	hal_gpio_pull_set(P13,WEAK_PULL_UP);
	
	x_y_z_off[0] = acc_data[0] / 1000.0;
	x_y_z_off[1] = acc_data[1] / 1000.0;
	x_y_z_off[2] = acc_data[2] / 1000.0;	
	
	x_y_z_off_error[0] = (raw_32[0] * 100) / (ACC_lSB_SIV);
	x_y_z_off_error[1] = (raw_32[1] * 100) / (ACC_lSB_SIV);
	x_y_z_off_error[2] = ((raw_32[2] - ACC_lSB_SIV) * 100) / (ACC_lSB_SIV);
	
	pin_status = hal_gpio_read(P13);
	if(pin_status)
	{
		sprintf((char *)acc_printf_data,"%-6.2f_%-6.2f_%-6.2f%+03d%%_%+03d%%_%+03d%%S%d_OFF", x_y_z_off[0], x_y_z_off[1],  x_y_z_off[2], x_y_z_off_error[0],x_y_z_off_error[1],x_y_z_off_error[2],acc_data[3]);
	}
	else
	{
		sprintf((char *)acc_printf_data,"%-6.2f_%-6.2f_%-6.2f%+03d%%_%+03d%%_%+03d%%S%d_ON.", x_y_z_off[0], x_y_z_off[1],  x_y_z_off[2], x_y_z_off_error[0],x_y_z_off_error[1],x_y_z_off_error[2],acc_data[3]);		
	}	
	
	gsensor_i2c_deinit(pi2c);
	ev.ev = wmi_event;
	ev.size = 2;
	ev.data = acc_printf_data;
	s_gsensor_ctx.evt_hdl(&ev);
	
	return PPlus_SUCCESS;
}

void gsensor_step_read_counter(uint32_t * step)
{
	uint8_t reg_data[3];	
	void* pi2c;

	if(NULL != step)
	{
		pi2c = gsensor_i2c_init();
		gsensor_i2c_read(pi2c, 0x07, reg_data, 2);	
		gsensor_i2c_read(pi2c, 0x0e, &reg_data[2], 1);
		*step = ((int)reg_data[2]<<16)|((int)reg_data[1]<<8)|reg_data[0];
		LOG("step data:%8d\n", *step);
		gsensor_i2c_deinit(pi2c);
	}	
}

void gsensor_step_reset_counter(void)
{
	void* pi2c = gsensor_i2c_init();

	gsensor_i2c_write(pi2c, 0x13, 0x80);		// clear step
	gsensor_i2c_write(pi2c, 0x13, 0x00);		// 

	gsensor_i2c_deinit(pi2c);	
}

int gsensor_standby(void)
{
	void* pi2c = gsensor_i2c_init();
	gsensor_i2c_write(pi2c, 0x11,0);
	gsensor_i2c_deinit(pi2c);
	return PPlus_SUCCESS;
}

uint8_t gsensor_get_chipid(uint8_t * chip_id)
{
	uint8_t  ret = PPlus_ERR_NOT_FOUND;
	void* pi2c;	
	
	if(NULL != chip_id)
	{
		pi2c = gsensor_i2c_init();
		gsensor_i2c_read(pi2c, 0x00, chip_id, 2);
		LOG("chip_id:0x%02X\n", *chip_id);		
		if((*chip_id >= 0xe7) && (*chip_id <= 0xe8))
		{
			LOG("gsensor i2c success\n");
			ret = PPlus_SUCCESS;
		}
		else
		{
			LOG("gsensor i2c failed\n");
		}
		gsensor_i2c_deinit(pi2c);
	}
	else
	{
		ret = PPlus_ERR_NULL;
	}
	
	return ret;
}

/*!
*	@brief this funtion is used for initialize gsensor register,and initialize GPIOE，and regeister event handler
*
*/
int gsensor_step_config(void)
{
	uint8_t chip_id;
	uint8_t reg_0x10 = 0;
	void* pi2c;
	
	if(gsensor_get_chipid(&chip_id))
	{
		return PPlus_ERR_NOT_FOUND;
	}
	else
	{
		pi2c = gsensor_i2c_init();
		
		gsensor_i2c_write(pi2c, 0x36,0xb6);
		gsensor_delay_ms(50);	
		gsensor_i2c_write(pi2c, 0x36, 0x00);		
		gsensor_i2c_write(pi2c, 0x0f, 0x02);
		
		reg_0x10 = 0xe0;
		gsensor_i2c_write(pi2c, 0x10, reg_0x10);

//		gsensor_i2c_write(pi2c, 0x4a, 0x08);	//Force I2C I2C interface
		gsensor_i2c_write(pi2c, 0x11, 0x80);
		gsensor_i2c_write(pi2c, 0x5f, 0x80);
		gsensor_i2c_write(pi2c, 0x5f, 0x00);

		if(reg_0x10 == 0xe0)
		{
			// ODR: 65hz 15.48 ms
			gsensor_i2c_write(pi2c, 0x12, 0x94);
			gsensor_i2c_write(pi2c, 0x13, 0x80);		// clear step
			gsensor_i2c_write(pi2c, 0x13, 0x00);		// 
			gsensor_i2c_write(pi2c, 0x14, 0x12);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			gsensor_i2c_write(pi2c, 0x15, 0x10);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}
		else if(reg_0x10 == 0xe1)
		{
			// ODR: 130hz 7.74 ms
			gsensor_i2c_write(pi2c, 0x12, 0x94);
			gsensor_i2c_write(pi2c, 0x13, 0x80);		// clear step
			gsensor_i2c_write(pi2c, 0x13, 0x00);		// 
			gsensor_i2c_write(pi2c, 0x14, 0x24);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			gsensor_i2c_write(pi2c, 0x15, 0x20);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}
		else if(reg_0x10 == 0xe2)
		{
			// ODR: 258Hz 3.87 ms
			gsensor_i2c_write(pi2c, 0x12, 0x94);
			gsensor_i2c_write(pi2c, 0x13, 0x80);		// clear step
			gsensor_i2c_write(pi2c, 0x13, 0x00);		// 
			gsensor_i2c_write(pi2c, 0x14, 0x48);		// STEP_TIME_LOW<7:0>*(1/ODR) 
			gsensor_i2c_write(pi2c, 0x15, 0x40);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
		}
		
		gsensor_i2c_write(pi2c, 0x1f, 0x00);          // 

		// step int
		#if defined(QMA7981_STEP_INT)
		reg_0x16 |= 0x08;
		reg_0x19 |= 0x08;
		gsensor_i2c_write(pi2c, 0x16, &reg_0x16, 1);
		gsensor_i2c_write(pi2c, 0x19, &reg_0x19, 1);
		#endif
		#if defined(QMA7981_SIGNIFICANT_STEP)
		gsensor_i2c_write(pi2c, 0x1d, 0x26);		//every 30 step
		reg_0x16 |= 0x40;
		reg_0x19 |= 0x40;
		gsensor_i2c_write(pi2c, 0x16, &reg_0x16, 1);
		gsensor_i2c_write(pi2c, 0x19, &reg_0x19, 1);
		#endif

		gsensor_i2c_write(pi2c, 0x20, 0x00);
		
		gsensor_i2c_deinit(pi2c);
		
	}	

	return PPlus_SUCCESS;

}

void gsensor_timer_stop(void)
{
}

void gsensor_timer_start(int ms)
{
	osal_start_timerEx(simpleBLEPeripheral_TaskID, ACC_DATA_EVT, ms);
}

uint8_t drv_gsensor_event_handle()
{

	gsensor_timer_start(1000);    		
	gsensor_step_read_counter((uint32_t *)&acc_data[3]);
	gsensor_read_acc_data();
	
	return 0;
}
	
void gsensor_int_hdl(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(type == POSEDGE)
	{
		s_gsensor_ctx.evt_cnt ++;
		osal_set_event(simpleBLEPeripheral_TaskID, ACC_DATA_EVT);
	}
}

void gsensor_timeout_hdl(void *parm)
{
	gsensor_read_acc_data();
}			

int gsensor_init(gsensor_evt_hdl_t evt_hdl)
{
	int ret = PPlus_SUCCESS;
	//copy pcfg
	s_gsensor_ctx.evt_hdl = evt_hdl;	
									
//	ret = hal_gpioin_register(P14, gsensor_int_hdl, NULL );//pin_event_handler);
	
	ret = gsensor_step_config();
	gsensor_timer_start(1000);

	if(ret != PPlus_SUCCESS)
	{
		s_gsensor_ctx.module_valid = false;
		return ret;
	}
	s_gsensor_ctx.module_valid = true;

  
	return PPlus_SUCCESS;
}































