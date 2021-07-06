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
*Date: 2021-06-29
*Author:
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
#include "pwrmgr.h"

typedef struct _QMA7981_ctx_t{
  bool              module_valid;
  uint8_t           pm_value;
  uint32_t          acc_report_intval;
  uint32_t          step_report_intval;
  QMA7981_evt_hdl_t evt_hdl;
}QMA7981_ctx_t;

// store come from pedometer parm config
static QMA7981_ctx_t s_QMA7981_ctx;
uint16_t g_qma7981_lsb_1g;
float  g_qma7981_cust_lsb_mg;

/*  
qma7981 odr setting
0x10<2:0>    ODR(Hz)      Time(ms)  |  RANGE 0x0f<3:0>
000        43.3125        23.088    |  0001  2g     244ug/LSB
001        86.4453        11.568    |  0010  4g     488ug/LSB
002        172.1763       5.808     |  0100  8g     977ug/LSB
003        341.5300       2.928     |  1000  16g    1.95mg/LSB
004        672.0430       1.488     |  1111  32g    3.91mg/LSB
005        32.5013        30.768    |  Others  2g   244ug/LSB
006        129.3995       7.728     |
007        257.2016       3.888     |
*/
const uint8_t qma7981_init_tbl[][2] = 
{
  {0x11, 0x80},
  {0x36, 0xb6},
  {QMA7981_DELAY, 5},
  {0x36, 0x00},
  {0x0f, QMA7981_RANGE_4G},
  {0x10, 0xe1},   // ODR 130hz
  //{0x4a, 0x08}, //Force I2C interface.SPI is disabled,SENB can be used as ATB
  {0x20, 0x05},   // Rising edge interrupt.
  {0x11, 0x80},
  {0x5f, 0x80},   // enable test mode,take control the FSM
  {0x5f, 0x00},   //normal mode
  {QMA7981_DELAY, 20}
};

static void QMA7981_delay_ms(int ms)
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

static void QMA7981_int_handler(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    uint8_t r_data[4]={0x00,};
    //uint8_t reg_0x18 = 0;
    uint8_t reg_0x1a = 0;
    uint8_t int_type = 0xff;
    void* pi2c = QST_i2c_init();

  //if(type == POSEDGE)
  //{
    hal_pwrmgr_lock(MOD_USR1);

    QST_i2c_read(pi2c, QMA7981_INT_ST0, r_data, 3);
    //LOG("r_data:%x,%x,%x\n",r_data[0],r_data[1],r_data[2]);
    if(r_data[0] & 0xF)
    {
      int_type = 1;
    }
    else if(r_data[0] & 0x80)
    {
      //bsp_stop_timer(0);
      QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
      reg_0x1a &= 0x7f;
      QST_i2c_write(pi2c, 0x1a, reg_0x1a);    // disable nomotion
      int_type = 2;
      //LOG(" no motion!\n");
    }
    else if(r_data[1] & 0x01)
    {
      int_type = 3;

      QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
      reg_0x1a |= 0x80;     // enable nomotion
      //reg_0x1a &= 0xfe;     // disable anymotion
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
      //LOG(" hand raise!\n");
    }
    else if(r_data[1] & 0x04)
    {
      int_type = 7;
      //LOG(" hand down!\n");
    }
#endif
    //LOG("int_type:%d\n", int_type);
    osal_set_event(AppWrist_TaskID, ACC_INT_EVT);
    QST_i2c_deinit(pi2c);
// }
}

static uint8_t QMA7981_set_range(void* pi2c, uint8_t range)
{
    if(range == QMA7981_RANGE_4G) {
      g_qma7981_lsb_1g = 2048;
      g_qma7981_cust_lsb_mg = 7.8;
    }
    else if(range == QMA7981_RANGE_8G) {
      g_qma7981_lsb_1g = 1024;
      g_qma7981_cust_lsb_mg = 15.6;
    }
    else if(range == QMA7981_RANGE_16G) {
      g_qma7981_lsb_1g = 512;
      g_qma7981_cust_lsb_mg = 31.2;
    }
    else if(range == QMA7981_RANGE_32G) {
      g_qma7981_lsb_1g = 256;
      g_qma7981_cust_lsb_mg = 62.5;
    }
    else {
      g_qma7981_lsb_1g = 4096;
      g_qma7981_cust_lsb_mg = 3.9;
    }

    return QST_i2c_write(pi2c, QMA7981_REG_RANGE, range);
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
    int8_t y_th = -3; //-2;        // -16 ~ 15
    uint8_t x_th = 6;    // 0--7.5
    int8_t z_th = 6;        // -8--7

#if 1//defined(QMA7981_SWAP_XY)  // swap xy
    if(layout%2)
    {
      QST_i2c_read(pi2c, 0x42, &reg_0x42, 1);
      reg_0x42 |= 0x80;    // 0x42 bit 7 swap x and y
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

    // 0x34 YZ_TH_SEL[7:5]  Y_TH[4:0], default 0x9d  (YZ_TH_SEL   4   9.0 m/s2 | Y_TH  -3  -3 m/s2)
    //QST_i2c_write(pi2c, 0x34, 0x9d);  //|yz|>8 m/s2, y>-3 m/m2
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
      QST_i2c_write(pi2c, 0x34, reg_0x34);  //|yz|>8m/s2, y<3 m/m2
    }
    //Z_TH<7:4>: -8~7, LSB 1 (unit : m/s2)  X_TH<3:0>: 0~7.5, LSB 0.5 (unit : m/s2) 
    //QST_i2c_write(pi2c, 0x1e, 0x68);  //6 m/s2, 4 m/m2

    QST_i2c_write(pi2c, 0x2a, (0x19|(0x02<<6)));      // 12m/s2 , 0.5m/s2
    QST_i2c_write(pi2c, 0x2b, (0x7c|(0x03>>2)));
    //QST_i2c_write(pi2c, 0x2a, (0x19|(0x02<<6)));      // 12m/s2 , 0.5m/s2
    //QST_i2c_write(pi2c, 0x2b, (0x7c|(0x02)));
    QST_i2c_write(pi2c, 0x35, (50));//50ms
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
*  @brief this funtion is used for initialize QMA7981 register,and initialize GPIOE，and regeister event handler
*
*/
static int QMA7981_config(void)
{
    uint8_t chip_id, regCount, index;
    uint8_t dieId_L, dieId_H, waferId;
    uint8_t reg_0x10 = 0, reg_0x11 = 0;
    uint8_t reg_0x16 = 0, reg_0x18 = 0, reg_0x19 = 0, reg_0x1a = 0;
#if defined(QMA7981_STEPCOUNTER)
    uint8_t reg_0x14 = 0, reg_0x15 = 0;
#endif
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
    uint8_t reg_0x2c = 0;
#endif
    void* pi2c = QST_i2c_init();
    /***************verify proper integraterd circuit******************************/
    QST_i2c_read(pi2c, QMA7981_CHIP_ID, &chip_id, 1);
    if(chip_id != 0xE7) {
      LOG("ERR: read QMA7981 chin id(0x%x) failed!\n", chip_id);
      QST_i2c_deinit(pi2c);
      return PPlus_ERR_IO_FAIL;
    }
    QMA7981_delay_ms(50);

    regCount = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
    for (index=0; index<regCount; index++) {
      if (qma7981_init_tbl[index][0] == QMA7981_DELAY) {
          QMA7981_delay_ms(qma7981_init_tbl[index][1]);
      } else {
          QST_i2c_write(pi2c, qma7981_init_tbl[index][0], qma7981_init_tbl[index][1]);
      }
    }
    QMA7981_set_range(pi2c, QMA7981_RANGE_4G);

    QST_i2c_read(pi2c, 0x47, &dieId_L, 1);
    QST_i2c_read(pi2c, 0x48, &dieId_H, 1);
    QST_i2c_read(pi2c, 0x5a, &waferId, 1);
    LOG("QMA7981 DieId_L:%d  DieId_H:%d  WaferId:%d \n", dieId_L, dieId_H, waferId&0x1f);

    QST_i2c_read(pi2c, 0x16, &reg_0x16, 1);
    QST_i2c_read(pi2c, 0x18, &reg_0x18, 1);
    QST_i2c_read(pi2c, 0x19, &reg_0x19, 1);
    QST_i2c_read(pi2c, 0x1a, &reg_0x1a, 1);
#if defined(QMA7981_ANY_MOTION)||defined(QMA7981_NO_MOTION)
    QST_i2c_read(pi2c, 0x2c, &reg_0x2c, 1);
#endif

    //0xe0  [MCLK/7695]
    //0xe1  [MCLK/3855]
    //0xe2  [MCLK/1935]
    //0xe3  [MCLK/975]
    reg_0x10 = 0xe1;
    QST_i2c_write(pi2c, 0x10, reg_0x10);
    reg_0x11 = 0x80;
    QST_i2c_write(pi2c, 0x11, reg_0x11);
    s_QMA7981_ctx.pm_value = reg_0x11;

#if defined(QMA7981_STEPCOUNTER)
    if(reg_0x11 == 0x80)  // 500K
    {
      reg_0x10 = 0xe1;
      QST_i2c_write(pi2c, 0x10, reg_0x10);

      reg_0x14 = (((STEP_W_TIME_L*100)/771)+1);   // odr 129.7hz, 7.71ms
      reg_0x15 = (((STEP_W_TIME_H*100)/771)+1);
      if(reg_0x10 == 0xe0)    // odr 65hz
      {
        reg_0x14 = (reg_0x14>>1);
        reg_0x15 = (reg_0x15>>1);
      }
      else if(reg_0x10 == 0xe5) // odr 32.5hz
      {
        reg_0x14 = (reg_0x14>>2);
        reg_0x15 = (reg_0x15>>2);
      }
    }
    else if(reg_0x11 == 0x81)  // 333K
    {
      reg_0x10 = 0xe1;
      QST_i2c_write(pi2c, 0x10, reg_0x10);

      reg_0x14 = (((STEP_W_TIME_L*100)/581)+1);  // odr 172.0930233 hz, 5.81ms
      reg_0x15 = (((STEP_W_TIME_H*100)/581)+1);
      if(reg_0x10 == 0xe1)  // 86.38132296 hz
      {
        reg_0x14 = (reg_0x14>>1);
        reg_0x15 = (reg_0x15>>1);
      }
      else if(reg_0x10 == 0xe0)    // 43.2748538
      {
        reg_0x14 = (reg_0x14>>2);
        reg_0x15 = (reg_0x15>>2);
      }
    }
    else if(reg_0x11 == 0x82)    // 200K
    {
      reg_0x10 = 0xe2;
      QST_i2c_write(pi2c, 0x10, reg_0x10);

      reg_0x14 = (((STEP_W_TIME_L*100)/967)+1);  // 103.3591731 hz, 9.675 ms
      reg_0x15 = (((STEP_W_TIME_H*100)/967)+1);
      if(reg_0x10 == 0xe1)
      {
        reg_0x14 = (reg_0x14>>1);    // 51.88067445 hz
        reg_0x15 = (reg_0x15>>1);
      }
      else if(reg_0x10 == 0xe3)
      {
        reg_0x14 = (reg_0x14<<1);    // 205.1282051 hz
        reg_0x15 = (reg_0x15<<1);
      }
    }
    else if(reg_0x11 == 0x83)    // 100K
    {
      reg_0x10 = 0xe3;
      QST_i2c_write(pi2c, 0x10, reg_0x10);

      reg_0x14 = (((STEP_W_TIME_L*100)/975)+1);  // 102.5641026 hz, 9.75 ms
      reg_0x15 = (((STEP_W_TIME_H*100)/975)+1);
      if(reg_0x10 == 0xe2)
      {
        reg_0x14 = (reg_0x14>>1);    // 51.67958656 hz
        reg_0x15 = (reg_0x15>>1);
      }
    }
    else if(reg_0x11 == 0x84)    // 50K
    {
      reg_0x10 = 0xe3;
      QST_i2c_write(pi2c, 0x10, reg_0x10);
      reg_0x14 = (((STEP_W_TIME_L)/20)+1);  // 50hz
      reg_0x15 = (((STEP_W_TIME_H)/20)+1);
    }

    QST_i2c_write(pi2c, 0x12, 0x94);
    QST_i2c_write(pi2c, 0x13, 0x80);    // clear step
    QST_i2c_write(pi2c, 0x13, 0x7f);    // 0x7f(1/16) 0x00(1/8)
    QST_i2c_write(pi2c, 0x14, reg_0x14);    // STEP_TIME_LOW<7:0>*(1/ODR) 
    QST_i2c_write(pi2c, 0x15, reg_0x15);    // STEP_TIME_UP<7:0>*8*(1/ODR) 

    //QST_i2c_write(pi2c, 0x1f, 0x09);    // 0 step
    //QST_i2c_write(pi2c, 0x1f, 0x29);    // 4 step
    //QST_i2c_write(pi2c, 0x1f, 0x49);    // 8 step
    //QST_i2c_write(pi2c, 0x1f, 0x69);    // 12 step
    //QST_i2c_write(pi2c, 0x1f, 0x89);    // 16 step
    QST_i2c_write(pi2c, 0x1f, 0xa9);    // 24 step
    //QST_i2c_write(pi2c, 0x1f, 0xc9);    // 32 step
    //QST_i2c_write(pi2c, 0x1f, 0xe9);    // 40 step

  // step int
#if defined(QMA7981_STEP_INT)
    reg_0x16 |= 0x08;
    reg_0x19 |= 0x08;
    QST_i2c_write(pi2c, 0x16, reg_0x16);
    QST_i2c_write(pi2c, 0x19, reg_0x19);
#endif
#if defined(QMA7981_SIGNIFICANT_STEP)
    QST_i2c_write(pi2c, 0x1d, 0x26);    //every 30 step
    reg_0x16 |= 0x40;
    reg_0x19 |= 0x40;
    QST_i2c_write(pi2c, 0x16, reg_0x16);
    QST_i2c_write(pi2c, 0x19, reg_0x19);
#endif
#endif

//RANGE<3:0> Acceleration range Resolution
//0001 2g 244ug/LSB
//0010 4g 488ug/LSB
//0100 8g 977ug/LSB
//1000 16g 1.95mg/LSB
//1111 32g 3.91mg/LSB
//Others 2g 244ug/LSB

//0x2c
//Duration = (NO_MOT_DUR<3:0> + 1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0> + 4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0> + 10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0> + 1) samples 

//0x2e ANY MOTION MOT_CONF2
//TH= ANY_MOT_TH<7:0> * 16 * LSB 

#if defined(QMA7981_ANY_MOTION)
    reg_0x18 |= 0x07;
    reg_0x1a |= 0x01;
    reg_0x2c |= 0x00;  //BIT[0-1]   (ANY_MOT_DUR<1:0> + 1) samples 
    
    QST_i2c_write(pi2c, 0x18, reg_0x18);
    QST_i2c_write(pi2c, 0x1a, reg_0x1a);
    QST_i2c_write(pi2c, 0x2c, reg_0x2c);
    //QST_i2c_write(pi2c, 0x2e, 0x14);    // 0.488*16*20 = 156mg
    //QST_i2c_write(pi2c, 0x2e, 0x80);    // 0.488*16*128 = 1g
    //QST_i2c_write(pi2c, 0x2e, 0xa0);    // 0.488*16*160 = 1.25g
    //QST_i2c_write(pi2c, 0x2e, 0x60);    // 0.488*16*96 = 750mg
    //QST_i2c_write(pi2c, 0x2e, 0x40);    // 0.488*16*64 = 500mg
    //QST_i2c_write(pi2c, 0x2e, 0x20);    // 0.488*16*32 = 250mg
    QST_i2c_write(pi2c, 0x2e, 0x40);    // 0.488*16*64 = 500mg

#if defined(QMA7981_ABNORMAL_SHAKE_CHECK)
    reg_0x10 = 0xe0;    // ODR: 65hz 15.48 ms
    QST_i2c_write(pi2c, 0x10, reg_0x10);
    qmaX981_set_range(QMAX981_RANGE_8G);
    QST_i2c_write(pi2c, 0x2e, 0x60);    // 0.977*16*96 = 1500mg
#endif
  
#if defined(QMA7981_SIGNIFICANT_MOTION)
//SIG_MOT_TPROOF [BIT4-5]<1:0>: 00: T_PROOF=0.25s,  01: T_PROOF=0.5s,  10: T_PROOF=1s,  11: T_PROOF=2s 
// significant motion interrupt ,  0: select any motion interrupt
//SIG_MOT_TSKIP[BIT2-3]<1:0>: 00: T_SKIP=1.5s,  01: T_SKIP=3s,  10: T_SKIP=6s,  11: T_SKIP=12s 
//SIG_MOT_SEL: 1: select
//QST_i2c_write(pi2c, 0x2f, 0x0c|0x01);
    QST_i2c_write(pi2c, 0x2f, 0x01);    // bit0   1 significant motion, 0: any motion.

    reg_0x19 |= 0x01;
    QST_i2c_write(pi2c, 0x19, reg_0x19);
#endif
#endif
#if defined(QMA7981_NO_MOTION)
    reg_0x18 |= 0xe0;
    reg_0x1a |= 0x80;
    reg_0x2c |= 0x00;  //1s   //0x24;

    QST_i2c_write(pi2c, 0x18, reg_0x18);
    QST_i2c_write(pi2c, 0x1a, reg_0x1a);
    QST_i2c_write(pi2c, 0x2c, reg_0x2c);
    QST_i2c_write(pi2c, 0x2d, 0x14);
#endif

#if defined(QMA7981_HAND_UP_DOWN)
    QMA7981_set_hand_up_down(pi2c, QMA7981_LAYOUT);
    reg_0x16 |= 0x02;
    reg_0x19 |= 0x02;
    QST_i2c_write(pi2c, 0x16, reg_0x16);  // hand up
    QST_i2c_write(pi2c, 0x19, reg_0x19);
    //reg_0x16 |= 0x04;
    //reg_0x19 |= 0x04;
    //QST_i2c_write(pi2c, 0x16, reg_0x16);  // hand down
    //QST_i2c_write(pi2c, 0x19, reg_0x19);
#endif

#if defined(QMA7981_DATA_READY)
    reg_0x1a |= 0x10;
    QST_i2c_write(pi2c, 0x17, 0x10);
    QST_i2c_write(pi2c, 0x1a, reg_0x1a);
#endif

#if defined(QMA7981_INT_LATCH)
    QST_i2c_write(pi2c, 0x21, 0x1f);  // default 0x1c, INT latch mode
#endif
    // int default level set
    //QST_i2c_write(pi2c, 0x20, 0x00);
    //-------------------------------------------------------------
    // X offset 2102, 4G range, CUST_X = 2102/9.807/7.8 = 27
    int16_t Xoffset = 2102;
    uint8_t XcustPostv = Xoffset*1000/GRAVITY_EARTH_1000/g_qma7981_cust_lsb_mg;
    LOG("Xoffset:%d, Xcust:%d\n", Xoffset, XcustPostv);
    uint8_t XcustNegav = ~XcustPostv + 1;
    QST_i2c_write(pi2c, 0x27, XcustNegav);
    //-------------------------------------------------------------
    QMA7981_delay_ms(50);

    QST_i2c_deinit(pi2c);
    return PPlus_SUCCESS;
}

static uint8_t QMA7981_read_raw_xyz(int16_t *data)
{
    uint8_t databuf[6] = {0};
    int16_t data_raw[3];
    uint8_t ret;
    void* pi2c = QST_i2c_init();

    ret = QST_i2c_read(pi2c, QMA7981_DATA_LOWBYTE_X,databuf,6);
    QST_i2c_deinit(pi2c);
    if(ret != PPlus_SUCCESS){
        LOG("ERR: 7981 read xyz error!!!\n");
        return ret;
    }

    data_raw[0] = (int16_t)((databuf[1]<<8)|(databuf[0]));
    data_raw[1] = (int16_t)((databuf[3]<<8)|(databuf[2]));
    data_raw[2] = (int16_t)((databuf[5]<<8)|(databuf[4]));
    //LOG("RAW:X%d,Y%d,Z%d\n", data_raw[0], data_raw[1], data_raw[2]);
    data[0] = data_raw[0]>>2;
    data[1] = data_raw[1]>>2;
    data[2] = data_raw[2]>>2;
    //LOG("X%d,Y%d,Z%d\n", data[0], data[1], data[2]);

    return ret;
}

uint8_t QMA7981_read_acc(float *accData)
{
    uint8_t ret;
    int16_t rawData[3];

    ret = QMA7981_read_raw_xyz(rawData);

    accData[0] = ((float)rawData[0]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g); //unit: m/s2
    accData[1] = ((float)rawData[1]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g);
    accData[2] = ((float)rawData[2]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g);

    return ret;
}

#if defined(QMA7981_STEPCOUNTER)
uint32_t QMA7981_read_stepcounter(void)
{
    uint8_t data[3];
    int ret;
    uint32_t step_num=0;
    void* pi2c = QST_i2c_init();

    ret = QST_i2c_read(pi2c, QMA7981_STEPCNT_LOWBYTE, data, 2);
    ret = QST_i2c_read(pi2c, QMA7981_STEPCNT_HIGHBYTE, &data[2], 1);
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
    QST_i2c_write(pi2c, 0x13, 0x80);  // clear step
    QMA7981_delay_ms(10);
    QST_i2c_write(pi2c, 0x13, 0x80);  // clear step
    QMA7981_delay_ms(10);
    QST_i2c_write(pi2c, 0x13, 0x80);  // clear step
    QMA7981_delay_ms(10);
    QST_i2c_write(pi2c, 0x13, 0x80);  // clear step
    QST_i2c_write(pi2c, 0x13, 0x7f);  // clear step
    QST_i2c_deinit(pi2c);
}
#endif

void QMA7981_acc_report_start(uint32_t report_intval_ms)
{
    s_QMA7981_ctx.acc_report_intval = report_intval_ms; //for timer reload
    osal_start_timerEx(AppWrist_TaskID, ACC_DATA_REPORT_EVT, report_intval_ms);
}

void QMA7981_acc_report_stop(void)
{
    osal_stop_timerEx(AppWrist_TaskID, ACC_DATA_REPORT_EVT);
}

void QMA7981_step_report_start(uint32_t report_intval_ms)
{
    s_QMA7981_ctx.step_report_intval = report_intval_ms; //for timer reload
    osal_start_timerEx(AppWrist_TaskID, ACC_STEP_REPORT_EVT, report_intval_ms);
}

void QMA7981_step_report_stop(void)
{
    osal_stop_timerEx(AppWrist_TaskID, ACC_STEP_REPORT_EVT);
}

uint8_t QMA7981_report_acc(void)
{
    QMA7981_ev_t ev;
    int16_t acc_raw[3];
    int32_t acc_data[3];
    uint8_t ret;

    osal_start_timerEx(AppWrist_TaskID, ACC_DATA_REPORT_EVT, s_QMA7981_ctx.acc_report_intval);
    ret = QMA7981_read_raw_xyz(acc_raw);  //ret = QMA7981_read_acc(acc_data);
    acc_data[0] = ((int32_t)acc_raw[0]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g); //unit: m/s2
    acc_data[1] = ((int32_t)acc_raw[1]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g);
    acc_data[2] = ((int32_t)acc_raw[2]*GRAVITY_EARTH_1000)/(g_qma7981_lsb_1g);

    ev.ev = acc_event;
    ev.size = 6;// 3;
    ev.data = acc_data; //acc_raw
    s_QMA7981_ctx.evt_hdl(&ev);
    return ret;
}

#if defined(QMA7981_STEPCOUNTER)
uint8_t QMA7981_report_stepcounter(void)
{
    QMA7981_ev_t ev;
    uint32_t stepCount;

    osal_start_timerEx(AppWrist_TaskID, ACC_STEP_REPORT_EVT, s_QMA7981_ctx.step_report_intval);
    stepCount = QMA7981_read_stepcounter();

    ev.ev = step_event;
    ev.size = 1;
    ev.data = &stepCount;
    s_QMA7981_ctx.evt_hdl(&ev);
    return 0;
}
#endif

void QMA7981_report_handup(void)
{
    QMA7981_ev_t evt;
    evt.ev = handUp_event;
    evt.size = 0;
    s_QMA7981_ctx.evt_hdl(&evt);
}

void QMA7981_deep_sleep(void)
{
    uint8_t val = 0;
    void* pi2c = QST_i2c_init();
    QST_i2c_read(pi2c, 0x11, &val, 1);
    s_QMA7981_ctx.pm_value = val;
    QST_i2c_write(pi2c, 0x11, 0x87); //G-sensor will work with ultra-low MCLK
    QST_i2c_deinit(pi2c);
}

void QMA7981_wake_up(void)
{
    void* pi2c = QST_i2c_init();
    QST_i2c_write(pi2c, 0x11, s_QMA7981_ctx.pm_value);
    QST_i2c_deinit(pi2c);
}

void QMA7981_disable(void)
{
    void* pi2c = QST_i2c_init();
    QST_i2c_write(pi2c, 0x11, 0x40);
    QST_i2c_deinit(pi2c);

    QMA7981_acc_report_stop();
    QMA7981_step_report_stop();
}

int QMA7981_init(QMA7981_evt_hdl_t evt_hdl)
{
    int ret = PPlus_SUCCESS;

    s_QMA7981_ctx.evt_hdl = evt_hdl;

    hal_gpio_pin_init(P5, IE);
    hal_gpio_pull_set(P5, PULL_DOWN);
    ret = hal_gpioin_register(P5, QMA7981_int_handler, NULL );
    hal_gpio_wakeup_set(P5, POSEDGE);
    hal_pwrmgr_register(MOD_USR1, NULL, NULL);

    ret = QMA7981_config();
    LOG("QMA7981 initialize: %d\n",ret);

    if(ret != PPlus_SUCCESS){
      s_QMA7981_ctx.module_valid = false;
      return ret;
    }
    s_QMA7981_ctx.module_valid = true;
    //QMA7981_acc_report_start(1000);

    return PPlus_SUCCESS;
}
