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
#define STEP_W_TIME_L	300
#define STEP_W_TIME_H	250
#define QMA7981_LAYOUT  3

typedef struct _QMA7981_ctx_t{
	bool			module_valid;
	uint32_t		evt_cnt;
	QMA7981_evt_hdl_t evt_hdl;
}QMA7981_ctx_t;

// store come from pedometer parm config
static QMA7981_ctx_t s_QMA7981_ctx;

/*	
qma7981 odr setting
0x10<2:0>		ODR(Hz)				Time(ms)	|	RANGE 0x0f<3:0>
000				43.3125				23.088		|	0001	2g  		244ug/LSB
001				86.4453				11.568		|	0010	4g  		488ug/LSB
002				172.1763			5.808		|	0100	8g  		977ug/LSB
003				341.5300			2.928		|	1000	16g  	1.95mg/LSB
004				672.0430			1.488		|	1111	32g  	3.91mg/LSB
005				32.5013				30.768		|	Others	2g  		244ug/LSB
006				129.3995			7.728		|
007				257.2016			3.888		|
*/

const uint8_t qma7981_init_tbl[][2] = 
{
  {0x11, 0x80},
  {0x36, 0xb6},
  {QMA7981_DELAY, 0x05},
  {0x36, 0x00},
  {0x0f, QMA7981_RANGE_4G},
  {0x10, 0xe1},		// ODR 130hz	
  //{0x4a, 0x08},	//Force I2C I2C s32erface.SPI is disabled,SENB can be used as ATB
  {0x20, 0x05},     // Rising edge interrupt.
  {0x11, 0x80},
  {0x5f, 0x80},		// enable test mode,take control the FSM
  {0x5f, 0x00},		//normal mode
  {QMA7981_DELAY, 20}
};


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

    //acc_data[0] &= 0xFE;    
    //acc_data[2] &= 0xFE;
    //acc_data[4] &= 0xFE;
	ev.ev = wmi_event;
	ev.size = 6;
	ev.data = acc_data;
	s_QMA7981_ctx.evt_hdl(&ev);
	return val;
}

#if defined(QMA7981_STEPCOUNTER)
uint32_t QMA7981_read_stepcounter(void)
{
	uint8_t data[3];
	int ret;
	uint32_t step_num=0;
	void* pi2c = QST_i2c_init();

	ret = QST_i2c_read(pi2c, QMA7981_STEP_CNT_L, data, 2);
	ret = QST_i2c_read(pi2c, QMA7981_STEP_CNT_M, &data[2], 1);
	if (ret == PPlus_SUCCESS) {
		step_num = (uint32_t)(((uint32_t)data[2]<<16)|((uint32_t)data[1]<<8)|data[0]);
	}

	QST_i2c_deinit(pi2c);
#if defined(QMA7981_CHECK_ABNORMAL_DATA)
	ret=qma7981_check_abnormal_data(step_num, &step_num);
	if(ret != 0)
	{
		return -1;
	}
#endif
#if defined(QMA7981_STEP_DEBOUNCE_IN_INT)
	step_num = qma7981_step_debounce_read_data(step_num);
#endif

	return step_num;
}

 void QMA7981_clear_step(void)
{
	void* pi2c = QST_i2c_init();
	QST_i2c_write(pi2c, 0x13, 0x80);		// clear step	
	QMA7981_delay_ms(10);
	QST_i2c_write(pi2c, 0x13, 0x80);		// clear step
	QMA7981_delay_ms(10);
	QST_i2c_write(pi2c, 0x13, 0x80);		// clear step
	QMA7981_delay_ms(10);
	QST_i2c_write(pi2c, 0x13, 0x80);		// clear step		
	QST_i2c_write(pi2c, 0x13, 0x7f);		// clear step
	QST_i2c_deinit(pi2c);	
}
#endif

uint8_t drv_QMA7981_event_handle(void)
{
	//uint8_t val;

	//void* pi2c;
    QMA7981_timer_start(400);
    //pi2c = QST_i2c_init();

    //QST_i2c_read(pi2c, QMA7981_INT_ST2,&val, 1);
	//QST_i2c_deinit(pi2c);
	//LOG("QMA7981_INT_ST2[%d]\n",val);

	QMA7981_fetch_acc_data();
    return 0;
}

void QMA7981_int_hdl(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    QMA7981_ev_t evt;
    uint8_t r_data[4]={0x00,};
    //uint8_t reg_0x18 = 0;
    uint8_t reg_0x1a = 0;
	uint8_t int_type = 0xff;
	int ret = -1;

	if(type == POSEDGE)
    {
        //s_QMA7981_ctx.evt_cnt ++;
        //osal_set_event(AppWrist_TaskID, ACC_DATA_EVT);

		void* pi2c = QST_i2c_init();
		ret = QST_i2c_read(pi2c, 0x09, r_data, 3);
		LOG("r_data:%x,%x,%x,ret[%d]\n",r_data[0],r_data[1],r_data[2],ret);
		if(r_data[0] & 0xF)
		{
			int_type = 1;
		}
		else if(r_data[0] & 0x80)
		{	
			//bsp_stop_timer(0);
			QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
			reg_0x1a &= 0x7f;
			QST_i2c_write(pi2c, 0x1a, reg_0x1a);		// disable nomotion
			int_type = 2;
			//LOG(" no motion!\n");
		}
		else if(r_data[1] & 0x01)
		{	
			int_type = 3;

			QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
			reg_0x1a |= 0x80;			// enable nomotion
			//reg_0x1a &= 0xfe;			// disable anymotion
			QST_i2c_write(pi2c, 0x1a, reg_0x1a);
			//LOG(" significant motion!\n");
		}
		else if(r_data[1] & 0x40)
		{	
			int_type = 4;
			//LOG("  significant step int!\n");
		}
		else if(r_data[1] & 0x08)
		{
			int_type = 5;
			//LOG(" step int!\n");
		}
#if defined(QMA7981_HAND_UP_DOWN)
		else if(r_data[1] & 0x02)
		{
			int_type = 6;
			evt.ev = handUp_event;
			//LOG(" hand raise!\n");
		}
		else if(r_data[1] & 0x04)
		{
			int_type = 7;
			evt.ev = handDown_event;
			//LOG(" hand down!\n");
		}
		s_QMA7981_ctx.evt_hdl(&evt);
#endif
		LOG("int_type:%d\n", int_type);
		QST_i2c_deinit(pi2c);
	}
}

#if 0
void QMA7981_int_event_handler(void)
{
    QMA7981_ev_t evt;
    uint8_t r_data[4]={0x00,};
    //uint8_t reg_0x18 = 0;
    uint8_t reg_0x1a = 0;
	uint8_t int_type = 0xff;
	int ret = -1;

	void* pi2c = QST_i2c_init();
	ret = QST_i2c_read(pi2c, 0x09, r_data, 3);
	LOG("r_data:%x,%x,%x,ret[%d]\n",r_data[0],r_data[1],r_data[2],ret);
	if(r_data[0] & 0xF)
	{
		int_type = 1;
	}
	else if(r_data[0] & 0x80)
	{	
		//bsp_stop_timer(0);
		QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
		reg_0x1a &= 0x7f;
		QST_i2c_write(pi2c, 0x1a, reg_0x1a);		// disable nomotion
		int_type = 2;
		//LOG(" no motion!\n");
	}
	else if(r_data[1] & 0x01)
	{	
		int_type = 3;

		QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
		reg_0x1a |= 0x80;			// enable nomotion
		//reg_0x1a &= 0xfe;			// disable anymotion
		QST_i2c_write(pi2c, 0x1a, reg_0x1a);
		//LOG(" significant motion!\n");
	}
	else if(r_data[1] & 0x40)
	{	
		int_type = 4;
		//LOG("  significant step int!\n");
	}
	else if(r_data[1] & 0x08)
	{
		int_type = 5;
		//LOG(" step int!\n");
	}
#if defined(QMA7981_HAND_UP_DOWN)
	else if(r_data[1] & 0x02)
	{
		int_type = 6;
		evt.ev = handUp_event;
		//LOG(" hand raise!\n");
	}
	else if(r_data[1] & 0x04)
	{
		int_type = 7;
		evt.ev = handDown_event;
		//LOG(" hand down!\n");
	}
	s_QMA7981_ctx.evt_hdl(&evt);
#endif
	LOG("int_type:%d\n", int_type);
	QST_i2c_deinit(pi2c);

}
#endif

void QMA7981_timeout_hdl(void *parm)
{
    QMA7981_fetch_acc_data();
}

#if defined(QMA7981_HAND_UP_DOWN)
static void QMA7981_set_hand_up_down(void* pi2c, int8_t layout)
{
#if 1//defined(QMA7981_SWAP_XY)
	uint8_t reg_0x42 = 0;
#endif
	uint8_t reg_0x1e = 0;
	uint8_t reg_0x34 = 0;
	uint8_t yz_th_sel = 4;
	int8_t y_th = -3; //-2;				// -16 ~ 15
	uint8_t x_th = 6;		// 0--7.5
	int8_t z_th = 6;				// -8--7

#if 1//defined(QMA7981_SWAP_XY)	// swap xy
	if(layout%2)
	{
		QST_i2c_read(pi2c, 0x42, &reg_0x42, 1);
		reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
		QST_i2c_write(pi2c, 0x42, reg_0x42);
	}
#endif

	if((layout >=0) && (layout<=3))
	{
		z_th = 3;
		if((layout == 2)||(layout == 3))
			y_th = 3; 
		else if((layout == 0)||(layout == 1))	
			y_th = -3;
	}
	else if((layout >=4) && (layout<=7))
	{
		z_th = -3;
		
		if((layout == 6)||(layout == 7))
			y_th = 3; 
		else if((layout == 4)||(layout == 5))	
			y_th = -3;
	}

	// 0x34 YZ_TH_SEL[7:5]	Y_TH[4:0], default 0x9d  (YZ_TH_SEL   4   9.0 m/s2 | Y_TH  -3  -3 m/s2)
	//QST_i2c_write(pi2c, 0x34, 0x9d);	//|yz|>8 m/s2, y>-3 m/m2
	if((y_th&0x80))
	{
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= (y_th&0x0f)|0x10;
		QST_i2c_write(pi2c, 0x34, reg_0x34);
	}
	else
	{	
		reg_0x34 |= yz_th_sel<<5;
		reg_0x34 |= y_th;
		QST_i2c_write(pi2c, 0x34, reg_0x34);	//|yz|>8m/s2, y<3 m/m2
	}
	//Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)	X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
	//QST_i2c_write(pi2c, 0x1e, 0x68);	//6 m/s2, 4 m/m2

	QST_i2c_write(pi2c, 0x2a, (0x19|(0x03<<6)));			// 12m/s2 , 0.5m/s2
	QST_i2c_write(pi2c, 0x2b, (0x7c|(0x03>>2)));
	//QST_i2c_write(pi2c, 0x2a, (0x19|(0x02<<6)));			// 12m/s2 , 0.5m/s2
	//QST_i2c_write(pi2c, 0x2b, (0x7c|(0x02)));

	//QST_i2c_read(pi2c, 0x1e, &reg_0x1e, 1);
	if((z_th&0x80))
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= ((z_th<<4)|0x80);
		QST_i2c_write(pi2c, 0x1e, reg_0x1e);
	}
	else
	{
		reg_0x1e |= (x_th&0x0f);
		reg_0x1e |= (z_th<<4);
		QST_i2c_write(pi2c, 0x1e, reg_0x1e);
	}
}
#endif

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
#if 1
	uint8_t reg_0x10 = 0, reg_0x11 = 0;
#if defined(QMA7981_STEPCOUNTER)
	uint8_t reg_0x14 = 0, reg_0x15 = 0;
#endif
	uint8_t reg_0x16 = 0, reg_0x18 = 0, reg_0x19 = 0, reg_0x1a = 0;
    uint8_t regCount = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	uint8_t index;
	for (index=0; index<regCount; index++) {
		if (qma7981_init_tbl[index][0] == QMA7981_DELAY) {
           QMA7981_delay_ms(qma7981_init_tbl[index][1]);
		} else {
           QST_i2c_write(pi2c, qma7981_init_tbl[index][0], qma7981_init_tbl[index][1]);
		}
	}

	QST_i2c_read(pi2c, 0x16, &reg_0x16, 1);
	QST_i2c_read(pi2c, 0x18, &reg_0x18, 1);
	QST_i2c_read(pi2c, 0x19, &reg_0x19, 1);
	QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);

	reg_0x10 = 0xe1;
	QST_i2c_write(pi2c, 0x10, reg_0x10);
	reg_0x11 = 0x80;
	QST_i2c_write(pi2c, 0x11, reg_0x11);

#if defined(QMA7981_STEPCOUNTER)
	if(reg_0x11 == 0x80)		// 500K
	{
		reg_0x10 = 0xe1;
		QST_i2c_write(pi2c, 0x10, reg_0x10);
	
		reg_0x14 = (((STEP_W_TIME_L*100)/771)+1);		// odr 129.7hz, 7.71ms
		reg_0x15 = (((STEP_W_TIME_H*100)/771)+1);
		if(reg_0x10 == 0xe0)		// odr 65hz
		{
			reg_0x14 = (reg_0x14>>1);
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe5)	// odr 32.5hz
		{
			reg_0x14 = (reg_0x14>>2);
			reg_0x15 = (reg_0x15>>2);
		}
	}
	else if(reg_0x11 == 0x81)	// 333K
	{		
		reg_0x10 = 0xe1;
		QST_i2c_write(pi2c, 0x10, reg_0x10);
	
		reg_0x14 = (((STEP_W_TIME_L*100)/581)+1);	// odr 172.0930233 hz, 5.81ms
		reg_0x15 = (((STEP_W_TIME_H*100)/581)+1);
		if(reg_0x10 == 0xe1)	// 86.38132296 hz
		{			
			reg_0x14 = (reg_0x14>>1);
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe0)		// 43.2748538
		{
			reg_0x14 = (reg_0x14>>2);
			reg_0x15 = (reg_0x15>>2);
		}
	}
	else if(reg_0x11 == 0x82)		// 200K
	{
		reg_0x10 = 0xe2;
		QST_i2c_write(pi2c, 0x10, reg_0x10);
	
		reg_0x14 = (((STEP_W_TIME_L*100)/967)+1);	// 103.3591731 hz, 9.675 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/967)+1);
		if(reg_0x10 == 0xe1)
		{			
			reg_0x14 = (reg_0x14>>1);		// 51.88067445 hz
			reg_0x15 = (reg_0x15>>1);
		}
		else if(reg_0x10 == 0xe3)
		{				
			reg_0x14 = (reg_0x14<<1);		// 205.1282051 hz				
			reg_0x15 = (reg_0x15<<1);
		}
	}		
	else if(reg_0x11 == 0x83)		// 100K
	{
		reg_0x10 = 0xe3;
		QST_i2c_write(pi2c, 0x10, reg_0x10);
	
		reg_0x14 = (((STEP_W_TIME_L*100)/975)+1);	// 102.5641026 hz, 9.75 ms
		reg_0x15 = (((STEP_W_TIME_H*100)/975)+1);
		if(reg_0x10 == 0xe2)
		{
			reg_0x14 = (reg_0x14>>1);		// 51.67958656 hz
			reg_0x15 = (reg_0x15>>1);
		}
	}
	else if(reg_0x11 == 0x84)		// 50K
	{
		reg_0x10 = 0xe3;
		QST_i2c_write(pi2c, 0x10, reg_0x10);
	
		reg_0x14 = (((STEP_W_TIME_L)/20)+1);	// 50hz
		reg_0x15 = (((STEP_W_TIME_H)/20)+1);

	}

	LOG("0x14[%X] 0x15[%X] \n", reg_0x14, reg_0x15);
	QST_i2c_write(pi2c, 0x12, 0x94);
	QST_i2c_write(pi2c, 0x13, 0x80);		// clear step
	QST_i2c_write(pi2c, 0x13, 0x7f);		// 0x7f(1/16) 0x00(1/8)
	QST_i2c_write(pi2c, 0x14, reg_0x14);		// STEP_TIME_LOW<7:0>*(1/ODR) 
	QST_i2c_write(pi2c, 0x15, reg_0x15);		// STEP_TIME_UP<7:0>*8*(1/ODR) 

	//QST_i2c_write(pi2c, 0x1f, 0x09);		// 0 step
	//QST_i2c_write(pi2c, 0x1f, 0x29);		// 4 step
	//QST_i2c_write(pi2c, 0x1f, 0x49);		// 8 step
	//QST_i2c_write(pi2c, 0x1f, 0x69);		// 12 step
	//QST_i2c_write(pi2c, 0x1f, 0x89);		// 16 step
	QST_i2c_write(pi2c, 0x1f, 0xa9);		// 24 step
	//QST_i2c_write(pi2c, 0x1f, 0xc9);		// 32 step
	//QST_i2c_write(pi2c, 0x1f, 0xe9);		// 40 step
#endif

#if defined(QMA7981_HAND_UP_DOWN)
	QMA7981_set_hand_up_down(pi2c, QMA7981_LAYOUT);
	reg_0x16 |= 0x02;
	reg_0x19 |= 0x02;

	QST_i2c_write(pi2c, 0x16, reg_0x16);	// hand up
	QST_i2c_write(pi2c, 0x19, reg_0x19);
	//reg_0x16 |= 0x04;
	//reg_0x19 |= 0x04;
	//QST_i2c_write(pi2c, 0x16, reg_0x16);	// hand down
	//QST_i2c_write(pi2c, 0x19, reg_0x19);

	QST_i2c_read(pi2c, 0x16, &reg_0x16, 1);//for debug
	QST_i2c_read(pi2c, 0x19, &reg_0x19, 1);//for debug
	LOG("0x16[%X] 0x19[%X] \n", reg_0x16, reg_0x19);//for debug
#endif

#else
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
#endif
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
