//#include <stdio.h>
#include <stdbool.h>
//#include "SEGGER_RTT.h"
//#include "hrs3300_i2c.h"
//#include "app_timer.h"


//////////////////////////////
#include "hrs3300.h"
//#include "hrs3300_alg.h"
#include "hrs3300_reg_init.h"
//////////////////////////////
//#define GSENSER_DATA

#include <stdlib.h>
#include "types.h"
#include "app_wrist.h"
#include "hal_mcu.h"
#include "app_err.h"
//#include "em70xx.h"
#include "hrs3300.h"
#include "OSAL_Timers.h"
#include "i2c.h"
#include "gpio.h"
#include "log.h"
#include "error.h"

// hrs3300 customer config
const uint8_t  HRS3300_SLAVE_ADDR = 0x44;
const uint8_t  hrs3300_bp_timeout_grade = 0;  // max 15
const uint8_t  hrs3300_agc_init_stage = 0x04;  // init AGC state  
const uint8_t  hrs3300_bp_power_grade = 0;
const uint8_t  hrs3300_accurate_first_shot = 0;
const uint8_t  hrs3300_up_factor = 3;
const uint8_t  hrs3300_up_shift = 2;
const uint16_t hrs3300_AMP_LTH = 120;
const uint16_t hrs3300_hr_AMP_LTH = 150;
const uint16_t hrs3300_hr_PVAR_LTH = 10;
// hrs3300 customer config end

//20161117 added by ericy for "low power in no_touch state"
static bool hrs3300_power_up_flg = 0 ;
uint8_t reg_0x7f ;
uint8_t reg_0x80 ;
uint8_t reg_0x81 ;
uint8_t reg_0x82 ;
//20161117 added by ericy for "low power in no_touch state"

static int heartrate_value = 0;
static hrs3300CB_t hrs3300CB = NULL;
static uint8_t hr_started = 0;
static uint16_t hr_rawdata[8];
static uint32 hr_rawdata_cnt = 0;

void nrf_delay_ms(int ms)
{	
	volatile int i = 4500;
	volatile int loop = ms;
	
  while(loop) 
  { 
		loop--; 
		for(; i; i--);
	} 
}

static int hr_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(AppWrist_TaskID, TIMER_HR_EVT, intval_ms);
    return 0;
}

static void* tyhx_init(void)
{
    hal_gpio_pull_set(P18, STRONG_PULL_UP);
    hal_gpio_pull_set(P19, STRONG_PULL_UP);
	hal_i2c_pin_init(I2C_0, P18, P19);
  return hal_i2c_init(I2C_0, I2C_CLOCK_400K);
}

static int tyhx_deinit(void* pi2c)
{
  int ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P19,IE);
  hal_gpio_pin_init(P18,IE);
	return ret;
}


int hrs3300_register(hrs3300CB_t cb)
{
    Hrs3300_chip_init();
    hr_timer_start(40);
  Hrs3300_chip_disable();
  hrs3300CB = cb;
	return PPlus_SUCCESS;
}



bool Hrs3300_write_reg(uint8_t addr, uint8_t data)    
{
	

  uint8 data_buf[2];
  data_buf[0] = addr;
  data_buf[1] = data;
    
  void* pi2c = tyhx_init();
  hal_i2c_addr_update(pi2c, HRS3300_SLAVE_ADDR);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data_buf, 2);
    HAL_EXIT_CRITICAL_SECTION();
  }
  if(hal_i2c_wait_tx_completed(pi2c))
      LOG("HRS3300 write func invalid para!");
  tyhx_deinit(pi2c);
  return 0;

}

uint8_t Hrs3300_read_reg(uint8_t addr)
{
    uint8_t data_buf;
    void* pi2c = tyhx_init();
    hal_i2c_read(pi2c, HRS3300_SLAVE_ADDR, addr, &data_buf, 1);
    tyhx_deinit(pi2c);
    return data_buf;
}



#ifdef MALLOC_MEMORY
void *hr_malloc(size_t size)
{
	return (void*)malloc(size);
}

void hr_free(void * ptr)
{
	free(ptr);
}
#endif

uint16_t Hrs3300_read_hrs(void)
{
	uint8_t  databuf[3];
	uint16_t data;
   
	databuf[0] = Hrs3300_read_reg(0x09);	// addr09, bit
    databuf[1] = Hrs3300_read_reg(0x0a);	// addr0a, bit
    databuf[2] = Hrs3300_read_reg(0x0f);	// addr0f, bit
	
	data = ((databuf[0]<<8)|((databuf[1]&0x0F)<<4)|(databuf[2]&0x0F));

	return data;
}

uint16_t Hrs3300_read_als(void)
{
	uint8_t  databuf[3];
	uint16_t data;

	databuf[0] = Hrs3300_read_reg(0x08);	// addr09, bit [10:3]
    databuf[1] = Hrs3300_read_reg(0x0d);	// addr0a, bit [17:11]
    databuf[2] = Hrs3300_read_reg(0x0e);	// addr0f, bit [2:0]
	
	data = ((databuf[0]<<3)|((databuf[1]&0x3F)<<11)|(databuf[2]&0x07));
	
	if (data > 32767) data = 32767;  // prevent overflow of other function

	return data;
}



bool Hrs3300_chip_init()
{
	int i =0 ;
	uint8_t id =0;
	
	for(i = 0; i<5; i++){
		id = Hrs3300_read_reg(0x00);
		if (0x21==id){
			break;
		}
		else{
			nrf_delay_ms(10);
		}
	}
	
	if (id != 0x21){
			return false;
	}
	
	for(i = 0; i < INIT_ARRAY_SIZE;i++)
	{
	    if ( Hrs3300_write_reg( init_register_array[i][0],
                                init_register_array[i][1]) != 0 )
	    {
	       return false;
	    }
  	}	
	
		
		//20161117 added by ericy for "low power in no_touch state"		
		if(hrs3300_power_up_flg == 0){
          reg_0x7f=Hrs3300_read_reg(0x7f) ;
		  reg_0x80=Hrs3300_read_reg(0x80) ;
		  reg_0x81=Hrs3300_read_reg(0x81) ;
		  reg_0x82=Hrs3300_read_reg(0x82) ;		
			hrs3300_power_up_flg =  1; 
		}
		//20161117 added by ericy for "low power in no_touch state"
	#ifdef RTT_PRINT
	DEBUG_PRINTF(0,">>> hrs3300 init id = %d\r\n", id);
	#endif

	  return true;
}

void Hrs3300_chip_enable()
{
  
    Hrs3300_write_reg( 0x16, 0x78 );
    Hrs3300_write_reg( 0x01, 0xd0 );	
	Hrs3300_write_reg( 0x0c, 0x2e );	
   
    heartrate_value = 0;
    hr_started = 1;
    hr_timer_start(40);
	return ;	
}


void Hrs3300_chip_disable()
{
    heartrate_value = 0;
    hr_started = 0;
    
	Hrs3300_write_reg( 0x01, 0x08 );
	Hrs3300_write_reg( 0x0c, 0x4e );

	return ;	
}


void heart_rate_meas_timeout_handler(void)
{
    /*
    uint32_t        err_code;
    uint16_t        heart_rate;
    uint8_t gsen_data;
    */
	uint16_t hrm_raw_data;
	uint16_t als_raw_data;
	
	  hrs3300_results_t alg_results;
    hr_timer_start(40);
#ifdef BP_CUSTDOWN_ALG_LIB		
	hrs3300_bp_results_t	bp_alg_results ;	
#endif	
	  static uint16_t timer_index =0;
#ifdef GSENSER_DATA
	  AxesRaw_t gsen_buf;
#endif

//    UNUSED_PARAMETER(p_context);
#ifdef GSENSER_DATA
	  LIS3DH_GetAccAxesRaw(&gsen_buf);
#endif
      if(!hr_started)
          return;
	  hrm_raw_data = Hrs3300_read_hrs();
	  als_raw_data = Hrs3300_read_als();  // 20170430
      
#ifdef GSENSER_DATA
    Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, gsen_buf.AXIS_X, gsen_buf.AXIS_Y, gsen_buf.AXIS_Z, 0); 
#else
      hr_rawdata[hr_rawdata_cnt%8] = hrm_raw_data;
      if(hr_rawdata_cnt%8 == 7){
    hr_ev_t ev;
    ev.ev = HR_EV_RAW_DATA;
    ev.value = 8;
    ev.data = hr_rawdata;
    hrs3300CB(&ev);
  }
  hr_rawdata_cnt++;
  
	  Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, 0, 0, 0,0);
#endif
		#if 0
	   alg_results = Hrs3300_alg_get_results();	
  LOG("HRS3300 VAR: %d %d %d %d\n", hrm_raw_data, als_raw_data,alg_results.hr_result,alg_results.alg_status);
	   //SEGGER_RTT_printf(0,"%d %d %d %d %d %d %d\n", hrm_raw_data, als_raw_data,gsen_buf.AXIS_X,gsen_buf.AXIS_Y,gsen_buf.AXIS_Z,alg_results.hr_result,alg_results.alg_status);
	   #endif

	  timer_index ++;
    if (timer_index >= 25)  {    // get result per second
			  timer_index =0;
		alg_results = Hrs3300_alg_get_results();

        
		if (alg_results.alg_status == MSG_NO_TOUCH)
		{
			//opr_display_hr(0);    // customer can print no touch information here
            LOG("HRS3300 no touch");
		}
		else if (alg_results.alg_status == MSG_PPG_LEN_TOO_SHORT)
		{
			//opr_display_wait( ((alg_results.data_cnt/100)%3)+1);  // customer can print waiting information here
            LOG("HRS3300 TOO SHORT");
		}
		else
		{
#ifdef BP_CUSTDOWN_ALG_LIB					
        bp_alg_results = Hrs3300_alg_get_bp_results();  
        if (bp_alg_results.sbp!= 0){
           opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);
        }
#endif				
            //opr_display_hr(alg_results.hr_result);  // customer can print real heart rate here
            if(alg_results.hr_result != heartrate_value){
                LOG("HRS3300 %d\n", alg_results.hr_result);
            heartrate_value = alg_results.hr_result;
            if(hrs3300CB){
                hr_ev_t ev;
                ev.ev = HR_EV_HR_VALUE;
                ev.value = alg_results.hr_result;
                ev.data = NULL;
                hrs3300CB(&ev);
                         }
                    }
		}
	}
}

