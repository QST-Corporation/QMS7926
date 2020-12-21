
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <stdlib.h>
#include "types.h"
#include "app_wrist.h"
#include "hal_mcu.h"
#include "app_err.h"
#include "OSAL_Timers.h"
#include "i2c.h"
#include "gpio.h"
#include "log.h"
#include "error.h"

#include "hx3690l.h"
#include "hx3690q_factory_test.h"

#ifdef SPO2_VECTOR
#include "spo2_vec.h" 
uint32_t spo2_send_cnt = 0;
uint32_t red_buf_vec[8];
uint32_t ir_buf_vec[8];
#endif

#ifdef HR_VECTOR
#include "hr_vec.h" 
uint32_t spo2_send_cnt = 0;
uint32_t PPG_buf_vec[8];
#endif

#ifdef GSENSER_DATA
#include "lis3dh_drv.h"
#endif

#define HX3690_I2C_ADDR     0x44
const uint8_t  hx3690_accurate_first_shot = 0;
const uint8_t  hx3690_up_factor1 = 3;
const uint8_t  hx3690_up_shift1 = 2;
const uint8_t  hx3690_up_factor2 = 31;
const uint8_t  hx3690_up_shift2 = 5;

// configure for JUDGE_HUMAN_ENABLE
const uint8_t  hx3690_human_delta1_thrs     = 40;   //The smaller value easy enter human status,recommand vaule 40 to 60
const uint8_t  hx3690_human_delta1_thrs2    = 20;   //The smaller value easy enter human status,recommand vaule 20 to 30
const uint8_t  hx3690_human_thrs            = 4;    //The smaller value easy enter human status,recommand vaule 2 to 6 or to 10               
const uint16_t  hx3690_human_static_thrs    = 4000; //The bigger value easy enter human status,recommand value 2000 to 9000,

//SPO2 agc
const uint8_t  hx3690l_spo2_agc_red_idac = 28;  //110 6,7,8...
const uint8_t  hx3690l_spo2_agc_ir_idac = 28;  //110 6,7,8...
//hrs agc
const uint8_t  hx3690l_hrs_agc_idac = 6;  //default=14 6,7,8...

//HRS_INFRARED_THRES
const int32_t  hrs_ir_unwear_thres = 8000;
const int32_t  hrs_ir_wear_thres = 15000; 
//SPO2_INFRARED_THRES
const int32_t  spo2_ir_unwear_thres = 10000; 
const int32_t  spo2_ir_wear_thres = 30000;

static hx3690lCB_t hx3690lCB = NULL;
void hx3690l_agc_Int_handle(GPIO_Pin_e pin,IO_Wakeup_Pol_e type);

#if defined(MALLOC_MEMORY)
uint8_t alg_ram[1*1024] = {0};
#else
uint8_t alg_ram[13*1024] = {0};
#endif


//

#ifdef GSENSER_DATA
volatile int16_t gsen_fifo_x[64];  //ppg time 330ms..330/40 = 8.25
volatile int16_t gsen_fifo_y[64];
volatile int16_t gsen_fifo_z[64];
#else
int16_t gen_dummy[64] = {0};
#endif

WORK_MODE_T work_mode_flag = HRS_MODE;
//////// para and switches
const  uint8_t   COUNT_BLOCK_NUM = 50;            // delay the block of some single good signal after a series of bad signal 
const  int32_t   SPO2_LOW_XCORR_THRE = 40;        //(64*xcorr)'s square below this threshold, means error signal
const  int32_t   XCORR_MODE = 1;                  //xcorr mode switch
const  int32_t   QUICK_RESULT = 1;                //come out the spo2 result quickly ;0 is normal,1 is quick
const  int32_t   MEAN_NUM = 256;                  // the length of smooth-average ;the value of MEAN_NUM can be given only 256 and 512
const  int32_t   G_SENSOR = 1;                    // if =1, open the gsensor mode
#ifdef MALLOC_MEMORY 
void *hx_malloc(size_t size)
{
    return (void*)malloc(size);
}

void hx_free(void * ptr)
{
    free(ptr);
}
#endif

void hx3690l_delay_us(uint32_t us)
{
    WaitUs(us);
}

void hx3690l_delay(uint32_t ms)
{
	volatile int i = 4500;
	volatile int loop = ms;

    while(loop) 
    { 
		loop--; 
		for(; i; i--);
	} 
}

static bool hr_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(AppWrist_TaskID, TIMER_HR_EVT, intval_ms);
    return true;
}

static bool hr_timer_stop(void)
{
    osal_stop_timerEx(AppWrist_TaskID, TIMER_HR_EVT);
    return true;
}

static void* tyhx_i2c_init(void)
{
    hal_gpio_pull_set(P28, STRONG_PULL_UP);
    hal_gpio_pull_set(P26, STRONG_PULL_UP);
    hal_i2c_pin_init(I2C_0, P28, P26);
    return hal_i2c_init(I2C_0, I2C_CLOCK_400K);
}

static int tyhx_i2c_deinit(void* pi2c)
{
    int ret = hal_i2c_deinit(pi2c);
    hal_gpio_pin_init(P28,IE);
    hal_gpio_pin_init(P26,IE);
    return ret;
}

bool hx3690l_write_reg(uint8_t addr, uint8_t data) 
{
    uint8_t data_buf[2];    
    data_buf[0] = addr;
    data_buf[1] = data;
    void* pi2c = tyhx_i2c_init();
    hal_i2c_addr_update(pi2c, HX3690_I2C_ADDR);
    {
        HAL_ENTER_CRITICAL_SECTION();
        hal_i2c_tx_start(pi2c);
        hal_i2c_send(pi2c, data_buf, 2);
        HAL_EXIT_CRITICAL_SECTION();
    }
    if(hal_i2c_wait_tx_completed(pi2c))
        LOG("hx3690l write func invalid para!");
    tyhx_i2c_deinit(pi2c);
    return true;
}

uint8_t hx3690l_read_reg(uint8_t addr) 
{
    uint8_t data_buf;
    void* pi2c = tyhx_i2c_init();
    hal_i2c_read(pi2c, HX3690_I2C_ADDR, addr, &data_buf, 1);
    tyhx_i2c_deinit(pi2c);
    return data_buf;
}

bool hx3690l_brust_read_reg(uint8_t addr , uint8_t *buf, uint8_t length) 
{
    void* pi2c = tyhx_i2c_init();
    hal_i2c_read(pi2c, HX3690_I2C_ADDR, addr, buf, length);
    tyhx_i2c_deinit(pi2c);
    return true;
}

void hx3690l_LEDpower_cfg(bool en)
{
    if(en)
    {
        hal_gpio_write(P18,1);
    }
    else
    {
        hal_gpio_write(P18,0);
    }
}

uint8_t chip_id = 0;
bool hx3690l_chip_check(void)
{
    uint8_t i = 0;

    for(i=0;i<10;i++)
    {
        hx3690l_write_reg(0x02, 0x30);
        hx3690l_delay(5);
        chip_id = hx3690l_read_reg(0x00);
        AGC_LOG("readout hx3690l id: 0x%X\n", chip_id);
        if (chip_id == 0x69)
        {
            AGC_LOG("r0x3E =0x%x\r\n",hx3690l_read_reg(0x3e));
            AGC_LOG("r0x3F =0x%x\r\n",hx3690l_read_reg(0x3f));
            AGC_LOG("r0x40 =0x%x\r\n",hx3690l_read_reg(0x40));
            AGC_LOG("r0x41 =0x%x\r\n",hx3690l_read_reg(0x41));
            return true;
        }
    }

    return false;
}

uint8_t hx3690l_read_fifo_size(void) // 20200615 ericy read fifo data number
{
    uint8_t fifo_num_temp = 0;
    fifo_num_temp = hx3690l_read_reg(0x14)&0x7f;

    return fifo_num_temp;
}

void hx3690l_ppg_off(void) // 20200615 ericy chip sleep enable
{
    hx3690l_write_reg(0x02, 0x31);
    hx3690l_LEDpower_cfg(false); //power off LED VBAT.
}

void hx3690l_ppg_on(void)
{
    //hx3690l_write_reg(0x6a, 0x00);   //reset fifo
    hx3690l_write_reg(0x02, 0x30);
}

#ifdef HRS_ALG_LIB
void hx3690l_hrs_ppg_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
    uint16_t sample_rate = 25; /*config the data rate of chip alps2_fm ,uint is Hz*/

    uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */

    uint8_t cic_mode_en =0;
    uint8_t cic_b2_en = 0;
    uint8_t samp_delay_leden_num = 0; /* 00=8,01=16,10=32,11=64*/
    uint8_t samp_copy_avg = 0;        /* 0=1, 1=2, 2=4 ,3=8, 4=16*/
    uint8_t data_avg_num = 0;         /* 0 = 1 ; 1 = 2; 2 =4 ; 3 =8 ; 4 =16 ;*/
    uint8_t phase3_4_internal = 0;    /* phase3 and phase4 prf internal cfg */

    uint8_t phase1_enable = 1;     /*phase1_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase2_enable = 1;     /*phase2_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase3_enable = 1;     /*phase3_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase4_enable = 1;     /*phase4_enable  , 1 mean enable ; 0 mean disable */

    uint8_t phase1_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase2_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase3_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase4_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    //green
    uint8_t phase1_inner_avg = 0;   /* phase1 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase1_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase1_ldr_sel = 1;     /*ball led 1 = ldr1(red); 2 = ldr2(ir); 4 = ldr3(green); 8 = ldr4 ;
                                    * 3in1 led 1 = ldr1(red); 2 = ldr2(green); 4 = ldr3(ir); 8 = ldr4 ;
                                    * 205U led 1 = ldr1(green); 2 = ldr2(red); 4 = ldr3(ir); 8 = ldr4 ;
                                    * GT01 led 1 = ldr1(green); 2 = ldr2(IR); 4 = ldr3(red); 8 = ldr4 ;
                                    */
    uint8_t phase1_pd_sel = 1;      /* 1 = pd1; 2 = pd2; */
    uint8_t phase1_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase1_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase1_led_en = 1;      /* phase1 led enable*/
    //als(green)
    uint8_t phase2_inner_avg = 0;   /* phase2 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase2_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase2_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase2_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase2_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase2_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase2_led_en = 0;      /* phase2 led enable*/
    //ir
    uint8_t phase3_inner_avg = 0;   /* phase3 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase3_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase3_ldr_sel = 4;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase3_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase3_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase3_ldr_cur = 64;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase3_led_en = 1;      /* phase3 led enable*/
    //als(ir)
    uint8_t phase4_inner_avg = 0;   /* phase4 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase4_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase4_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase4_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase4_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase4_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase4_led_en = 0;      /* phase4 led enable*/

    uint8_t init_wait_delay = 5 ; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
                                     4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */

    uint8_t afe_reset = 3;        /* 0 = 15clk ; 1 = 31clk ; 2 = 63clk ; 3 = 127clk(d) ;
                                     4 = 255clk; 5 = 511clk; 6 = 1024; 7 = 2048clk */

    uint8_t led_on_time = 3;      /* 0 = 32clk=8us ; 1 = 64clk=16us; 2=128clk=32us ; 3 = 256clk=64us ;
                                     4 = 512clk=128us ; 5 = 1024clk=256us; 6= 2048clk=512us; 7 = 4096clk=1024us */
    hx3690l_write_reg(0x02, 0x30);
    hx3690l_delay(5);
    hx3690l_write_reg(0X6a, 0X00);	//rest int
    hx3690l_delay(5);

    hx3690l_write_reg(0X1a, (uint8_t)prf_clk_num);    // prf bit<7:0>
    hx3690l_write_reg(0X1b, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>
    hx3690l_write_reg(0X1c, (uint8_t)(prf_clk_num>>16)); // prf bit<23:16>

    hx3690l_write_reg(0X1d, phase3_4_internal);
    hx3690l_write_reg(0X1e, ((afe_reset<<3)| 0x00) );
    hx3690l_write_reg(0X1f, (led_on_time<<4| phase1_led_en<<3 | phase3_led_en<<2 | phase4_led_en<<1 | phase2_led_en) );

    hx3690l_write_reg(0X26, (init_wait_delay<<4 | 0x0f));
    hx3690l_write_reg(0X27, (phase1_inner_avg | (phase2_inner_avg<<4)));
    hx3690l_write_reg(0X28, (phase3_inner_avg | (phase4_inner_avg<<4)));
    hx3690l_write_reg(0X29, cic_mode_en<<7 | cic_b2_en<<6 | samp_delay_leden_num<<4 | samp_copy_avg);

    hx3690l_write_reg(0X2c, phase1_tia_res);
    hx3690l_write_reg(0X2d, phase3_tia_res);
    hx3690l_write_reg(0X2e, phase4_tia_res);
    hx3690l_write_reg(0X2f, phase2_tia_res);

    hx3690l_write_reg(0X30, phase1_ldr_cur);
    hx3690l_write_reg(0X31, phase3_ldr_cur);
    hx3690l_write_reg(0X32, phase4_ldr_cur);
    hx3690l_write_reg(0X33, phase2_ldr_cur);

    hx3690l_write_reg(0X34, (phase1_pd_sel<<4 |  phase1_ldr_sel));
    hx3690l_write_reg(0X35, (phase3_pd_sel<<4 |  phase3_ldr_sel));
    hx3690l_write_reg(0X36, (phase4_pd_sel<<4 |  phase4_ldr_sel));
    hx3690l_write_reg(0X37, (phase2_pd_sel<<4 |  phase2_ldr_sel));

    hx3690l_write_reg(0X38, phase1_offset_idac);
    hx3690l_write_reg(0X39, phase3_offset_idac);
    hx3690l_write_reg(0X3a, phase4_offset_idac);
    hx3690l_write_reg(0X3b, phase2_offset_idac);
    hx3690l_write_reg(0X3c, data_avg_num<<4 | data_avg_num );
    hx3690l_write_reg(0X3d, data_avg_num<<4 | data_avg_num );

// analog circuit cfg
    hx3690l_write_reg(0X60, 0x0a);	//1a= adc self test
    hx3690l_write_reg(0X66, 0x92);	//0x92= r2r idac en; 0x91= mos idac en; 0x93= two idac en;
    hx3690l_write_reg(0X67, 0xbf);	//32k osc cfg relate
    hx3690l_write_reg(0X69, 0xa0);	//bit<0>: rc_comb_en bits<1>=rc_rbp_en bits<7>= vcom_clamp_en bits<6:4>= LED_vdesl
    hx3690l_write_reg(0X6a, 0X02);	//02= u_low_pow, INT cmos output

/////////FIFO and adc conversion ready config////////
    hx3690l_write_reg(0X12, 0x32);   // fifo almostfull cfg ,max=0x40;
    hx3690l_write_reg(0X13, 0x31);   /* bits<7:4> fifo data sel, 0000 = p1-p2;0001= p1,p2;0010=p3,p4;
                                       0011=p1,p2,p3,p4;0100=p3-(p2+p4)/2;0101=p1-p2,p3-p4;0110=p2-(p1+p3)/2;
                                       bits<3:2> fifo int clear mode, 00 = selfclear;01=reserve;10=manual clear;
									   bits<1:0> fifo mode sel, 00=bypass,01=fifo,10=stream,11=reserve;*/
    hx3690l_write_reg(0X20, 0x03);   // int width
    hx3690l_write_reg(0X23, 0x20);   // phase int sel  80=p1 / 10=p2 / 40=p3 / 20 =p4
    hx3690l_write_reg(0X24, 0x00);   // fifo int output sel
///////FIFO//////////

    hx3690l_write_reg(0X18,(phase1_enable<<3)|(phase1_adc_osr)|(phase3_enable<<7)|(phase3_adc_osr<<4) );
    hx3690l_write_reg(0X19,(phase4_enable<<3)|(phase4_adc_osr)|(phase2_enable<<7)|(phase2_adc_osr<<4) );

    hx3690l_write_reg(0X51, 0x02);
    hx3690l_delay(5);
	hx3690l_write_reg(0X13, 0x30);
	hx3690l_delay(5);
	hx3690l_write_reg(0X13, 0x31);	
    hx3690l_write_reg(0X51, 0x00);
}
#endif

#ifdef SPO2_ALG_LIB
void hx3690l_spo2_ppg_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
    uint16_t sample_rate = 50; /*config the data rate of chip alps2_fm ,uint is Hz*/

    uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */

    uint8_t cic_mode_en =0;
    uint8_t cic_b2_en = 0;
    uint8_t samp_delay_leden_num = 0; /* 00=8,01=16,10=32,11=64*/
    uint8_t samp_copy_avg = 0;        /* 0=1, 1=2, 2=4 ,3=8, 4=16*/
    uint8_t data_avg_num = 1;         /* 0 = 1 ; 1 = 2; 2 =4 ; 3 =8 ; 4 =16 ;*/
    uint8_t phase3_4_internal = 0;    /* phase3 and phase4 prf internal cfg */

    uint8_t phase1_enable = 1;     /*phase1_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase2_enable = 0;     /*phase2_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase3_enable = 1;     /*phase3_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase4_enable = 1;     /*phase4_enable  , 1 mean enable ; 0 mean disable */

    uint8_t phase1_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase2_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase3_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase4_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    //red
    uint8_t phase1_inner_avg = 0;   /* phase1 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase1_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase1_ldr_sel = 2;     /*ball led 1 = ldr1(red); 2 = ldr2(ir); 4 = ldr3(green); 8 = ldr4 ;
                                    * 3in1 led 1 = ldr1(red); 2 = ldr2(green); 4 = ldr3(ir); 8 = ldr4 ;
                                    * 205U led 1 = ldr1(green); 2 = ldr2(red); 4 = ldr3(ir); 8 = ldr4 ;
                                    * GT01 led 1 = ldr1(green); 2 = ldr2(IR); 4 = ldr3(red); 8 = ldr4 ;
                                    */
    uint8_t phase1_pd_sel = 1;      /* 1 = pd1; 2 = pd2; */
    uint8_t phase1_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase1_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase1_led_en = 1;      /* phase1 led enable*/
    //no use
    uint8_t phase2_inner_avg = 0;   /* phase2 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase2_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase2_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase2_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase2_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase2_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase2_led_en = 0;      /* phase2 led enable*/
    //als(for red and ir)
    uint8_t phase3_inner_avg = 0;   /* phase3 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase3_tia_res = 6;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase3_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase3_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase3_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase3_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase3_led_en = 0;      /* phase3 led enable*/
    //IR
    uint8_t phase4_inner_avg = 0;   /* phase4 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase4_tia_res = 6;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase4_ldr_sel = 4;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase4_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase4_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase4_ldr_cur = 64;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase4_led_en = 1;      /* phase4 led enable*/

    uint8_t init_wait_delay = 5 ; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
                                     4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */

    uint8_t afe_reset = 4;        /* 0 = 15clk ; 1 = 31clk ; 2 = 63clk ; 3 = 127clk(d) ;
                                     4 = 255clk; 5 = 511clk; 6 = 1024; 7 = 2048clk */

    uint8_t led_on_time = 5;      /* 0 = 32clk=8us ; 1 = 64clk=16us; 2=128clk=32us ; 3 = 256clk=64us ;
                                     4 = 512clk=128us ; 5 = 1024clk=256us; 6= 2048clk=512us; 7 = 4096clk=1024us */

    hx3690l_write_reg(0x02, 0x30);
    hx3690l_delay(5);
    hx3690l_write_reg(0X6a, 0X00);	//rest int
    hx3690l_delay(5);

    hx3690l_write_reg(0X1a, (uint8_t)prf_clk_num);    // prf bit<7:0>
    hx3690l_write_reg(0X1b, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>
    hx3690l_write_reg(0X1c, (uint8_t)(prf_clk_num>>16)); // prf bit<23:16>

    hx3690l_write_reg(0X1d, phase3_4_internal);
    hx3690l_write_reg(0X1e, ((afe_reset<<3)| 0x00) );
    hx3690l_write_reg(0X1f, (led_on_time<<4| phase1_led_en<<3 | phase3_led_en<<2 | phase4_led_en<<1 | phase2_led_en) );

    hx3690l_write_reg(0X26, (init_wait_delay<<4 | 0x0f));
    hx3690l_write_reg(0X27, (phase1_inner_avg | (phase2_inner_avg<<4)));
    hx3690l_write_reg(0X28, (phase3_inner_avg | (phase4_inner_avg<<4)));
    hx3690l_write_reg(0X29, cic_mode_en<<7 | cic_b2_en<<6 | samp_delay_leden_num<<4 | samp_copy_avg);

    hx3690l_write_reg(0X2c, phase1_tia_res);
    hx3690l_write_reg(0X2d, phase3_tia_res);
    hx3690l_write_reg(0X2e, phase4_tia_res);
    hx3690l_write_reg(0X2f, phase2_tia_res);

    hx3690l_write_reg(0X30, phase1_ldr_cur);
    hx3690l_write_reg(0X31, phase3_ldr_cur);
    hx3690l_write_reg(0X32, phase4_ldr_cur);
    hx3690l_write_reg(0X33, phase2_ldr_cur);

    hx3690l_write_reg(0X34, (phase1_pd_sel<<4 |  phase1_ldr_sel));
    hx3690l_write_reg(0X35, (phase3_pd_sel<<4 |  phase3_ldr_sel));
    hx3690l_write_reg(0X36, (phase4_pd_sel<<4 |  phase4_ldr_sel));
    hx3690l_write_reg(0X37, (phase2_pd_sel<<4 |  phase2_ldr_sel));

    hx3690l_write_reg(0X38, phase1_offset_idac);
    hx3690l_write_reg(0X39, phase3_offset_idac);
    hx3690l_write_reg(0X3a, phase4_offset_idac);
    hx3690l_write_reg(0X3b, phase2_offset_idac);
    hx3690l_write_reg(0X3c, data_avg_num<<4 | data_avg_num );
    hx3690l_write_reg(0X3d, data_avg_num<<4 | data_avg_num );

// analog circuit cfg
    hx3690l_write_reg(0X60, 0x0a);	//1a= adc self test
    hx3690l_write_reg(0X66, 0x92);	//0x92= r2r idac en; 0x91= mos idac en; 0x93= two idac en;
    hx3690l_write_reg(0X67, 0xbf);	//32k osc cfg relate
    hx3690l_write_reg(0X69, 0xa0);	//bit<0>: rc_comb_en bits<1>=rc_rbp_en bits<7>= vcom_clamp_en bits<6:4>= LED_vdesl
    hx3690l_write_reg(0X6a, 0X02);	//02= u_low_pow, INT cmos output

/////////FIFO and adc conversion ready config////////
    hx3690l_write_reg(0X12, 0x32);   // fifo almostfull cfg ,max=0x40;
    hx3690l_write_reg(0X13, 0x31);   /* bits<7:4> fifo data sel, 0000 = p1-p2;0001= p1,p2;0010=p3,p4;
                                       0011=p1,p2,p3,p4;0100=p3-(p2+p4)/2;0101=p1-p2,p3-p4;0110=p2-(p1+p3)/2;
                                       bits<3:2> fifo int clear mode, 00 = selfclear;01=reserve;10=manual clear;
									   bits<1:0> fifo mode sel, 00=bypass,01=fifo,10=stream,11=reserve;*/
    hx3690l_write_reg(0X20, 0x03);   // int width
    hx3690l_write_reg(0X23, 0x20);   // phase int sel  80=p1 / 10=p2 / 40=p3 / 20 =p4
    hx3690l_write_reg(0X24, 0x00);   // fifo int output sel
///////FIFO//////////

    hx3690l_write_reg(0X18,(phase1_enable<<3)|(phase1_adc_osr)|(phase3_enable<<7)|(phase3_adc_osr<<4) );
    hx3690l_write_reg(0X19,(phase4_enable<<3)|(phase4_adc_osr)|(phase2_enable<<7)|(phase2_adc_osr<<4) );

    hx3690l_write_reg(0X51, 0x02);
    hx3690l_delay(5);
    hx3690l_write_reg(0X13, 0x30);
    hx3690l_delay(5);
    hx3690l_write_reg(0X13, 0x31);
    hx3690l_write_reg(0X51, 0x00);

   // while(1);
}
#endif

void hx3690l_320ms_timer_cfg(bool en)
{
    if(en)
    {
        hr_timer_start(320);
    }
    else
    {
        hr_timer_stop();
    }
}

void hx3690l_40ms_timer_cfg(bool en)
{
    if(en)
    {
        #if defined(GSENSER_DATA)||!defined(EXT_INT_AGC)  
        gsen_read_timers_start();   
        #endif 
    }
    else
    {
        #if defined(GSENSER_DATA)||!defined(EXT_INT_AGC)  
        gsen_read_timers_stop();   
        #endif 
    }
}

void hx3690l_gpioint_cfg(bool en)
{
    if(en)
    {
         hal_gpioin_register(P4, NULL, hx3690l_agc_Int_handle);
    }
    else
    {
         hal_gpioin_unregister(P4);
    }
}

bool hx3690l_init(WORK_MODE_T mode)
{
    hal_gpioin_register(P4, NULL, hx3690l_agc_Int_handle);
    hal_gpio_pin_init(P18, OEN);
    hx3690l_LEDpower_cfg(true); //power on LED VBAT.

    work_mode_flag = mode;//HRS_MODE,SPO2_MODE

    if(work_mode_flag == HRS_MODE)
    {
        #ifdef HRS_ALG_LIB
        #ifdef SPO2_ALG_LIB
        hx3690_spo2_alg_close();
        #endif
        if(hx3690l_hrs_enable()== SENSOR_OP_FAILED) 
        {
            return false;
        }
        #endif
    }
    else  
    if(work_mode_flag == SPO2_MODE)
    {
        #ifdef SPO2_ALG_LIB
        #ifdef HRS_ALG_LIB
        hx3690_alg_close();
        #endif
        if(hx3690l_spo2_enable()== SENSOR_OP_FAILED) 
        {
            return false;
        }
        #endif
    }
    else 
    if(work_mode_flag == WEAR_MODE)
    {
        #ifdef HRS_ALG_LIB           
        #ifdef SPO2_ALG_LIB
        hx3690_spo2_alg_close();
        #endif
        if(hx3690l_hrs_enable()== SENSOR_OP_FAILED) 
        {
            return false;
        }
        hx3690l_hrs_low_power();
        #else
        #ifdef HRS_ALG_LIB
        hx3690_alg_close();
        #endif
        if(hx3690l_spo2_enable()== SENSOR_OP_FAILED) 
        {
            return false;
        }
        hx3690l_spo2_low_power();
        #endif
    }
    else 
    if(work_mode_flag == FT_HRS_MODE)
    {
        hx3690l_hrs_factory_test_cfg();
    }
    else 
    if(work_mode_flag == FT_SPO2_MODE)
    {
        hx3690l_spo2_factory_test_cfg();
    }
    
    return true;
}

//I/O interrupt falling edge
void hx3690l_agc_Int_handle(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{       
#ifdef EXT_INT_AGC
    
    if(work_mode_flag == HRS_MODE)
    {   
        #ifdef HRS_ALG_LIB
        HRS_CAL_SET_T cal;
        cal = PPG_hrs_agc();
        
        if(cal.work)
        {
            AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
            cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
        }
        #endif
    }
    else  
    if(work_mode_flag == SPO2_MODE)
    {
        #ifdef SPO2_ALG_LIB 
        SPO2_CAL_SET_T cal;
        cal = PPG_spo2_agc();

        if(cal.work)
        {
            AGC_LOG("AGC. Rled_drv=%d,Irled_drv=%d,RledDac=%d,IrledDac=%d,ambDac=%d,Rledstep=%d,Irledstep=%d,Rrf=%d,Irrf=%d,\r\n",\
            cal.R_LED, cal.IR_LED,cal.R_LEDDAC,cal.IR_LEDDAC,cal.AMBDAC,\
            cal.R_led_step,cal.IR_led_step,cal.R_RF,cal.IR_RF);
        }
        #endif
    }   
        
#endif 
}
//Timer interrupt 40ms repeat mode
void gsen_read_timeout_handler(void * p_context)
{
#ifndef EXT_INT_AGC
    
    if(work_mode_flag == HRS_MODE)
    {
        #ifdef HRS_ALG_LIB
        HRS_CAL_SET_T cal;
        cal = PPG_hrs_agc();
        
        if(cal.work)
        {
            AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
            cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
        }
        #endif
    }
    else  
    if(work_mode_flag == SPO2_MODE)
    {
        #ifdef SPO2_ALG_LIB 
        SPO2_CAL_SET_T cal;
        cal = PPG_spo2_agc();
        
        if(cal.work)
        {
            AGC_LOG("AGC. Rled_drv=%d,Irled_drv=%d,RledDac=%d,IrledDac=%d,ambDac=%d,Rledstep=%d,Irledstep=%d,Rrf=%d,Irrf=%d,\r\n",\
            cal.R_LED, cal.IR_LED,cal.R_LEDDAC,cal.IR_LEDDAC,cal.AMBDAC,\
            cal.R_led_step,cal.IR_led_step,cal.R_RF,cal.IR_RF);
        }
        #endif
    }    
#endif 
   hx3690l_gesensor_Int_handle();
}

//Timer interrupt 320ms repeat mode
void heart_rate_meas_timeout_handler(void)
{
    hx3690l_320ms_timer_cfg(true); //reload 320ms timer.

    if(work_mode_flag == HRS_MODE)
    {
        #ifdef HRS_ALG_LIB
        hx3690l_hrs_ppg_Int_handle();
        #endif
    }
    else       
    if(work_mode_flag == SPO2_MODE)
    {
        #ifdef SPO2_ALG_LIB
        hx3690l_spo2_ppg_Int_handle();
        #endif
    }
    else   
    if(work_mode_flag == WEAR_MODE)
    {
        hx3690l_wear_ppg_Int_handle();
    }
    else   
    if(work_mode_flag == FT_HRS_MODE)
    {
        #ifdef HRS_ALG_LIB
        hx3690l_ft_hrs_Int_handle();
        #endif
    }
    else   
    if(work_mode_flag == FT_SPO2_MODE)
    {
        #ifdef SPO2_ALG_LIB
        hx3690l_ft_spo2_Int_handle();
        #endif
    }
}


void hx3690l_gesensor_Int_handle(void)
{
#ifdef GSENSER_DATA
    uint8_t ii = 0;
    AxesRaw_t gsen_buf;
    if(work_mode_flag == WEAR_MODE)
    {
        return;
    }
    
    LIS3DH_GetAccAxesRaw(&gsen_buf);

    for(ii=0;ii<9;ii++)
    {
        gsen_fifo_x[ii] = gsen_fifo_x[ii+1];
        gsen_fifo_y[ii] = gsen_fifo_y[ii+1];
        gsen_fifo_z[ii] = gsen_fifo_z[ii+1];
    }
    gsen_fifo_x[9] = gsen_buf.AXIS_X>>1;
    gsen_fifo_y[9] = gsen_buf.AXIS_Y>>1;
    gsen_fifo_z[9] = gsen_buf.AXIS_Z>>1;
    //SEGGER_RTT_printf(0,"gsen_x=%d gsen_y=%d gsen_z=%d\r\n", \
    gsen_fifo_x[9],gsen_fifo_y[9],gsen_fifo_z[9]);
#endif 
}

#ifdef HRS_ALG_LIB
hrs_sensor_data_t hrs_s_dat;
void hx3690l_hrs_ppg_Int_handle(void)
{
    uint8_t        ii=0;   
    hx3690_results_t alg_results = {MSG_HRS_ALG_NOT_OPEN,MSG_HRS_NO_WEAR,0,0,0,false};

    uint32_t *PPG_buf = &(hrs_s_dat.ppg_data[0]);
    uint32_t *als = &(hrs_s_dat.als);
    uint32_t *ir_buf = &(hrs_s_dat.ir_data[0]);
    uint8_t *count = &(hrs_s_dat.count);
    int32_t *s_buf = &(hrs_s_dat.s_buf[0]);

    if(hx3690l_hrs_read(&hrs_s_dat) == NULL)
    {
        
        return;
    }
    for(ii=0;ii<*count;ii++)
    {
        AGC_LOG("%d/%d %d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
        PPG_buf[ii]-524288,PPG_buf[ii],ir_buf[ii],s_buf[ii*4],s_buf[ii*4+1],s_buf[ii*4+2], \
        s_buf[ii*4+3],hrs_s_dat.agc_green);
    }

    #ifdef HR_VECTOR
      for(ii=0;ii<8;ii++)
      {
        PPG_buf_vec[ii] = hrm_input_data[spo2_send_cnt+ii];
        gsen_fifo_x[ii] = gsen_input_data_x[spo2_send_cnt+ii];
        gsen_fifo_y[ii] = gsen_input_data_y[spo2_send_cnt+ii];
        gsen_fifo_z[ii] = gsen_input_data_z[spo2_send_cnt+ii];
      }
      spo2_send_cnt = spo2_send_cnt+8;
      *count = 8;
      hx3690_alg_send_data(PPG_buf_vec,*count, 0, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z); 
    #endif
    
    #ifndef HR_VECTOR
      #ifdef GSENSER_DATA
      hx3690_alg_send_data(PPG_buf,*count, *als, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);   
      #else
      hx3690_alg_send_data(PPG_buf,*count,*als, gen_dummy, gen_dummy, gen_dummy);
      #endif 
    #endif      

    //display part                                                                                                                 
    alg_results = hx3690_alg_get_results();
    hr_ev_t ev;
    ev.ev = HR_EV_HR_VALUE;
    ev.value = alg_results.hr_result;
    ev.data = NULL;
    hx3690lCB(&ev);
    AGC_LOG("hr_result:%d, alg_status:%d\n", alg_results.hr_result, alg_results.hrs_alg_status);

    #ifdef HRS_BLE_APP
    {
        rawdata_vector_t rawdata;
        
        HRS_CAL_SET_T cal= get_hrs_agc_status();
        for(ii=0;ii<*count;ii++)
        {
            rawdata.vector_flag = HRS_VECTOR_FLAG;
            rawdata.data_cnt = alg_results.data_cnt-*count+ii;
            rawdata.hr_result = alg_results.hr_result;           
            rawdata.red_raw_data = PPG_buf[ii];
            rawdata.ir_raw_data = ir_buf[ii];
            rawdata.gsensor_x = gsen_fifo_x[ii];
            rawdata.gsensor_y = gsen_fifo_y[ii];
            rawdata.gsensor_z = gsen_fifo_z[ii];
            rawdata.red_cur = cal.LED;
            rawdata.ir_cur = alg_results.hrs_alg_status;
            
            ble_rawdata_vector_push(rawdata);   
        }
    }
    #endif    
}
#endif

#ifdef SPO2_ALG_LIB
spo2_sensor_data_t spo2_s_dat;
void hx3690l_spo2_ppg_Int_handle(void)
{ 
    uint8_t        ii=0;   
    hx3690_spo2_results_t alg_results = {MSG_SPO2_ALG_NOT_OPEN,MSG_SPO2_NO_WEAR,0,0,0};
    
    

    int32_t *red_buf = &(spo2_s_dat.red_data[0]);
    int32_t *ir_buf = &(spo2_s_dat.ir_data[0]);
    uint8_t *count = &(spo2_s_dat.count);
    int32_t *s_buf = &(spo2_s_dat.s_buf[0]);

    if(hx3690l_spo2_read(&spo2_s_dat) == NULL)
    {
        return;
    }
    for(ii=0;ii<*count;ii++)
    {
        DEBUG_PRINTF(0,"%d/%d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
        red_buf[ii],ir_buf[ii],s_buf[ii*3],s_buf[ii*3+1],s_buf[ii*3+2],\
        spo2_s_dat.agc_red,spo2_s_dat.agc_ir);
    }
   
    #ifdef SPO2_VECTOR
      for(ii=0;ii<8;ii++)
      {
        red_buf_vec[ii] = vec_red_data[spo2_send_cnt+ii];
        ir_buf_vec[ii] = vec_ir_data[spo2_send_cnt+ii];
      }
      spo2_send_cnt = spo2_send_cnt+8;
      *count = 8;
      for(ii=0;ii<10;ii++)
      {
        gsen_fifo_x[ii] = 0;
        gsen_fifo_y[ii] = 0;
        gsen_fifo_z[ii] = 0;
      }
      hx3690_spo2_alg_send_data(red_buf_vec, ir_buf_vec, *count, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);
    #endif
      
    #ifndef SPO2_VECTOR
      #ifdef GSENSER_DATA
      hx3690_spo2_alg_send_data(red_buf, ir_buf, *count, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);   
      #else
      hx3690_spo2_alg_send_data(red_buf, ir_buf, *count, gen_dummy, gen_dummy, gen_dummy);
      #endif  
    #endif      

    //display part
    alg_results = hx3690_spo2_alg_get_results();

    hr_ev_t ev;
    ev.ev = HR_EV_SPO2_VALUE;
    ev.value = alg_results.spo2_result;
    ev.data = NULL;
    hx3690lCB(&ev);
    AGC_LOG("spo2_result:%d, spo2_alg_status:%d\n", alg_results.spo2_result, alg_results.spo2_alg_status);
}
#endif

void hx3690l_wear_ppg_Int_handle(void)
{
    int32_t ir_buf[2] = {0};   //als ir
    int32_t ir = 0;
#ifdef HRS_ALG_LIB
    hx3690_results_t alg_results = {MSG_HRS_ALG_NOT_OPEN,MSG_HRS_NO_WEAR,0,0,0,false};
    read_hrs_ir_packet(ir_buf);      
    ir =  ir_buf[0]-ir_buf[1];
    AGC_LOG("ir_data =%d\r\n" ,ir); 
    alg_results.hrs_wear_status = hx3690_hrs_wear_mode_check(WEAR_MODE,ir);
    AGC_LOG("alg_results.hrs_wear_status =%d\r\n" ,alg_results.hrs_wear_status);
#else
    hx3690_spo2_results_t alg_results = {MSG_SPO2_ALG_NOT_OPEN,MSG_SPO2_NO_WEAR,0,0,0};
    read_spo2_ir_packet(ir_buf);      
    ir =  ir_buf[1]-ir_buf[0];
    AGC_LOG("ir_data =%d\r\n" ,ir); 
    alg_results.spo2_wear_status = hx3690_spo2_check_unwear(WEAR_MODE,ir);
    alg_results.spo2_wear_status = hx3690_spo2_check_wear(WEAR_MODE,ir);  
    AGC_LOG("alg_results.hrs_wear_status =%d\r\n" ,alg_results.spo2_wear_status); 
#endif
}

#ifdef HRS_ALG_LIB
void hx3690l_ft_hrs_Int_handle(void)
{
    uint8_t        ii=0; 
    bool ret = false;    


    uint32_t *PPG_buf = &(hrs_s_dat.ppg_data[0]);
    uint32_t *ir_buf = &(hrs_s_dat.ir_data[0]);
    uint8_t *count = &(hrs_s_dat.count);
    int32_t *s_buf = &(hrs_s_dat.s_buf[0]);

    if(hx3690l_ft_hrs_read(&hrs_s_dat) == NULL)
    {
        return;
    }
    for(ii=0;ii<*count;ii++)
    {
        AGC_LOG("%d/%d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
        PPG_buf[ii]-1047600,PPG_buf[ii],ir_buf[ii],s_buf[ii*4],s_buf[ii*4+1],s_buf[ii*4+2], \
        s_buf[ii*4+3]);
    }
    ret = hx3693_factroy_test(HR_LEAK_LIGHT_TEST,*count,(int32_t *)PPG_buf,(int32_t *)ir_buf);  //yorke
    AGC_LOG("HR_LEAK_LIGHT_TEST = %s\n", (ret==true)?"OK":"Fail");
    
    ret = hx3693_factroy_test(HR_GRAY_CARD_TEST,*count,(int32_t *)PPG_buf,(int32_t *)ir_buf);       
    AGC_LOG("HR_GRAY_CARD_TEST = %s\n", (ret==true)?"OK":"Fail");
}
#endif

#ifdef SPO2_ALG_LIB
void hx3690l_ft_spo2_Int_handle(void)
{ 
    uint8_t        ii=0; 
    bool ret = false;     

    int32_t *red_buf = &(spo2_s_dat.red_data[0]);
    int32_t *ir_buf = &(spo2_s_dat.ir_data[0]);
    uint8_t *count = &(spo2_s_dat.count);
    int32_t *s_buf = &(spo2_s_dat.s_buf[0]);

    if(hx3690l_ft_spo2_read(&spo2_s_dat) == NULL)
    {
        return;
    }
    
    for(ii=0;ii<*count;ii++)
    {
        DEBUG_PRINTF(0,"%d/%d %d %d %d %d %d\r\n" ,1+ii,*count,\
        red_buf[ii],ir_buf[ii],s_buf[ii*3],s_buf[ii*3+1],s_buf[ii*3+2]);
    }
    ret = hx3693_factroy_test(SPO2_LEAK_LIGHT_TEST,*count,red_buf,ir_buf);
    AGC_LOG("HR_LEAK_LIGHT_TEST = %s\n", (ret==true)?"OK":"Fail");
    
    ret = hx3693_factroy_test(SPO2_GRAY_CARD_TEST,*count,red_buf,ir_buf);       
    AGC_LOG("HR_GRAY_CARD_TEST = %s\n", (ret==true)?"OK":"Fail");
    
}
#endif

int hx3690l_register(hx3690lCB_t cb)
{
    //hx3690l_init(HRS_MODE);

    //hx3690l_ppg_off();
    hx3690lCB = cb;
	return PPlus_SUCCESS;
}
