#ifndef _hx3690l_H_
#define _hx3690l_H_
#include <stdint.h>
#include <stdbool.h>

#define HRS_ALG_LIB
#define SPO2_ALG_LIB

//#define MALLOC_MEMORY

//#define GSENSER_DATA
//#define HRS_BLE_APP
//#define  SPO2_VECTOR
//#define HR_VECTO
         
#define  EXT_INT_AGC


#define NEW_GSEN_SCHME      // New Gsen Schme after 2017-9-20

#define AGC_DEBUG
#ifdef AGC_DEBUG
#define  AGC_LOG(...)     SEGGER_RTT_printf(0,__VA_ARGS__)
#else
#define	 AGC_LOG(...)

#endif

#define HRS_DEBUG
#ifdef HRS_DEBUG
#define  DEBUG_PRINTF(...)     SEGGER_RTT_printf(__VA_ARGS__)
#else
#define	 DEBUG_PRINTF(...)
#endif



//add ericy 20180428
typedef enum {
	DEFAULT_MOTION = 0,
	STATIC_MOTION = 1,
	WALKING_MOTION = 2,
	RUNING_MOTION = 3,
	CYCLING_MOTION = 4,
	SWINMING_MOTION = 5,
	TENNIS_MOTION = 6,
	HRM_MONITOR_MODE = 7
} MOIION_STATUS;


typedef enum {    
    PPG_INIT, 
    PPG_OFF, 
    PPG_LED_OFF,
    CAL_INIT,
    CAL_OFF,
    RECAL_INIT    
} hx3690l_mode_t;



typedef enum {    
    SENSOR_OK, 
    SENSOR_OP_FAILED,
   
} SENSOR_ERROR_T;

typedef enum {    
    DATA_TYPE_PPG, 
    DATA_TYPE_GSEN,
   
} SENSOR_TYPE_T;

typedef enum {    
    HRS_MODE, 
    SPO2_MODE,
    WEAR_MODE,
    FT_HRS_MODE,//HRS factory test mode
    FT_SPO2_MODE
} WORK_MODE_T;


typedef struct {
    SENSOR_TYPE_T type;
    uint8_t count;
    int32_t red_data[64];
    int32_t ir_data[64];  
    int32_t s_buf[64]; 
    uint8_t agc_red; 
    uint8_t agc_ir;    

}spo2_sensor_data_t;

typedef struct {
    SENSOR_TYPE_T type;
    uint8_t count;
    uint32_t ppg_data[64];
    uint32_t ir_data[64];
    int32_t s_buf[64]; 
    uint32_t als;
    uint32_t agc_green;
}hrs_sensor_data_t;

typedef enum {
	MSG_HRS_NO_WEAR,
	MSG_HRS_WEAR
} hx3690_hrs_wear_msg_code_t;
typedef enum {
	MSG_HRS_ALG_NOT_OPEN,
    MSG_HRS_ALG_OPEN,
    MSG_HRS_ALG_OPEN_DEEP,
	MSG_HRS_READY,
	MSG_HRS_ALG_TIMEOUT,
	MSG_HRS_SETTLE
} hx3690_hrs_alg_msg_code_t;

typedef struct {
    hx3690_hrs_alg_msg_code_t  hrs_alg_status;
    hx3690_hrs_wear_msg_code_t  hrs_wear_status;
    uint32_t           data_cnt;
    uint8_t            hr_result;
    uint8_t            hr_result_qual; // ericy add20170111
    bool               object_flg;
} hx3690_results_t;

typedef enum {
	MSG_SPO2_NO_WEAR,
	MSG_SPO2_WEAR
} hx3690_spo2_wear_msg_code_t;
typedef enum {
	MSG_SPO2_ALG_NOT_OPEN,
    MSG_SPO2_ALG_OPEN,
    MSG_SPO2_ALG_OPEN_DEEP,
	MSG_SPO2_READY,
	MSG_SPO2_ALG_TIMEOUT,
	MSG_SPO2_SETTLE
} hx3690_spo2_alg_msg_code_t;

typedef struct {
  hx3690_spo2_alg_msg_code_t  spo2_alg_status;
  hx3690_spo2_wear_msg_code_t  spo2_wear_status;
  uint32_t                 data_cnt;
  uint8_t                 spo2_result;
  uint8_t                  hr_result; 
	
} hx3690_spo2_results_t;
typedef struct {
  uint8_t                 red_offset_idac;
  uint8_t                 ir_offset_idac; 
  uint16_t                spo2_start_cnt;
	
} hx3690_spo2_agcpara_t;


typedef struct {
    uint8_t flag;
	bool work;
    uint8_t int_cnt;
    uint8_t cur255_cnt;
    uint8_t led_idac;  // 6,7,8...
    uint8_t LED;     // phasex led driver config
    uint8_t LEDDAC;  // phasex led offset idac cfg
    uint8_t AMBDAC;  // phasex offset idac cfg
    uint8_t RF;      // phasex tia feed back resister cfg
    uint32_t led_step;
    uint8_t state;
} HRS_CAL_SET_T;

typedef struct {
    uint8_t flag;
    bool work;
    uint8_t int_cnt;
    uint8_t cur255_cnt;
    uint8_t red_idac;  // 6,7,8...
    uint8_t ir_idac;  // 6,7,8...
    uint8_t R_LED;     // phasex led driver config
    uint8_t IR_LED;     // phasex led driver config
    uint8_t R_LEDDAC;  // phasex led offset idac cfg
    uint8_t IR_LEDDAC;  // phasex led offset idac cfg
    uint8_t AMBDAC;  // phasex offset idac cfg
    uint8_t R_RF;      // phasex tia feed back resister cfg
    uint8_t IR_RF;      // phasex tia feed back resister cfg
    uint32_t R_led_step;
    uint32_t IR_led_step;
    uint8_t state;
} SPO2_CAL_SET_T;


#ifdef HRS_BLE_APP
typedef enum {
    START_VECTOR_FLAG = 0,
	HRS_VECTOR_FLAG = 0x55,
	SPO2_VECTOR_FLAG = 0x66
} vector_flag_t;
typedef struct {
	vector_flag_t vector_flag;
    uint32_t data_cnt;
	uint8_t  hr_result;
	int32_t red_raw_data; //hrs:green,spo2:red
	int32_t ir_raw_data;  //hrs:no use,spo2:ir
	int16_t gsensor_x;
	int16_t gsensor_y;
	int16_t gsensor_z;
    uint8_t red_cur;       //hrs:green cur drv,spo2:red cur drv
    uint8_t ir_cur;        //hrs:no use,spo2:ir cur drv
    
    
}rawdata_vector_t ;
#endif
//add ericy 20180428

#if defined(MALLOC_MEMORY)
extern uint8_t alg_ram[1*1024];
#else
extern uint8_t alg_ram[13*1024];
#endif


extern WORK_MODE_T work_mode_flag;

void hx3690l_delay_us(uint32_t us);
void hx3690l_delay(uint32_t ms);

bool hx3690l_write_reg(uint8_t addr, uint8_t data); 
uint8_t hx3690l_read_reg(uint8_t addr); 
bool hx3690l_brust_read_reg(uint8_t addr , uint8_t *buf, uint8_t length);
bool hx3690l_chip_check(void);
uint8_t hx3690l_read_fifo_size(void);
void hx3690l_ppg_off(void);
void hx3690l_ppg_on(void);  
void hx3690l_320ms_timer_cfg(bool en);
void hx3690l_40ms_timer_cfg(bool en);
void hx3690l_gpioint_cfg(bool en);
bool hx3690l_init(WORK_MODE_T mode);
void hx3690l_agc_Int_handle(void); 
void hx3690l_gesensor_Int_handle(void);

void hx3690l_spo2_ppg_init(void);
void hx3690l_spo2_ppg_Int_handle(void);
void hx3690l_wear_ppg_Int_handle(void);
void hx3690l_ft_hrs_Int_handle(void);
void hx3690l_ft_spo2_Int_handle(void);


void hx3690l_hrs_ppg_init(void);
void hx3690l_hrs_ppg_Int_handle(void);

void display_refresh(void);
void ble_rawdata_clear(void);


#ifdef HRS_BLE_APP
void ble_rawdata_vector_push(rawdata_vector_t rawdata);
uint32_t ble_hrs_heart_rate_send_ext(uint8_t flag,uint8_t data_cnt,uint8_t result,\
                              int32_t red_rawdata,int32_t ir_rawdata,\
                              int16_t gsensor_x, int16_t gsensor_y, int16_t gsensor_z,\
                              uint8_t red_cur, uint8_t ir_cur);

#endif
uint32_t ble_rawdata_send_handler(void);

extern uint32_t hx3690_timers_start(void);
extern uint32_t hx3690_timers_stop(void);
extern uint32_t hx3690_gpioint_init(void);

extern uint32_t hx3690_gpioint_enable(void);
extern uint32_t hx3690_gpioint_disable(void);
extern uint32_t gsen_read_timers_start(void);
extern uint32_t gsen_read_timers_stop(void);

extern hx3690_results_t hx3690_alg_get_results(void);
extern hx3690_spo2_results_t hx3690_spo2_alg_get_results(void);

extern HRS_CAL_SET_T PPG_hrs_agc(void);
extern SPO2_CAL_SET_T PPG_spo2_agc(void);

#ifdef HRS_ALG_LIB
void hx3690l_hrs_cal_init(void);
void hx3690l_hrs_cal_off(uint8_t enable_50_hz);
uint8_t hx3690l_read_fifo_size(void);
void hx3690l_read_fifo_data(uint8_t read_fifo_size,int32_t *buf);
void read_hrs_data_packet(int32_t *buf);
void read_hrs_ir_packet(int32_t *buf);
void hx3690l_hrs_low_power(void);
void hx3690l_hrs_normal_power(void);
void hx3690l_hrs_updata_reg(void);
void hx3690l_hrs_set_mode(uint8_t mode_cmd);
SENSOR_ERROR_T hx3690l_hrs_enable(void);
void hx3690l_hrs_disable(void);
hx3690_hrs_wear_msg_code_t hx3690_hrs_get_wear_status(void);
uint8_t hx3690l_hrs_read(hrs_sensor_data_t * s_dat);
bool hx3690l_hrs_init(void);      
hx3690_hrs_wear_msg_code_t hx3690_hrs_wear_mode_check(WORK_MODE_T mode,int32_t infrared_data);

bool hx3690_alg_open(void);
bool hx3690_alg_open_deep(void);
void hx3690_alg_close(void);
bool hx3690_alg_send_data(int32_t *new_raw_data, uint8_t dat_len, uint32_t green_data_als, int16_t *gsen_data_x, int16_t *gsen_data_y, int16_t *gsen_data_z);
HRS_CAL_SET_T get_hrs_agc_status(void);
#endif

#ifdef SPO2_ALG_LIB
void hx3690l_spo2_cal_init(void);
void hx3690l_spo2_cal_off(void); 
void read_spo2_data_packet(int32_t *buf);
void read_spo2_ir_packet(int32_t *buf);  
void hx3690l_spo2_low_power(void);
void hx3690l_spo2_normal_power(void); 
void hx3690l_spo2_updata_reg(void);
void hx3690l_spo2_set_mode(uint8_t mode_cmd);
SENSOR_ERROR_T hx3690l_spo2_enable(void);
void hx3690l_spo2_disable(void);
hx3690_spo2_wear_msg_code_t hx3690_spo2_get_wear_status(void);
uint8_t hx3690l_spo2_read(spo2_sensor_data_t * s_dat);
hx3690_spo2_wear_msg_code_t hx3690_spo2_check_unwear(WORK_MODE_T mode,int32_t infrared_data);
hx3690_spo2_wear_msg_code_t hx3690_spo2_check_wear(WORK_MODE_T mode,int32_t infrared_data);

bool hx3690_spo2_alg_send_data(int32_t *red_new_raw_data,int32_t *ir_new_raw_data, uint8_t dat_len,volatile int16_t *gsen_data_x,volatile int16_t *gsen_data_y,volatile int16_t *gsen_data_z);
void hx3690_spo2_alg_close(void);
bool hx3690_spo2_alg_open(void);
bool hx3690_spo2_alg_open_deep(void);
SPO2_CAL_SET_T get_spo2_agc_status(void);
SPO2_CAL_SET_T PPG_spo2_agc(void);
bool hx3690_spo2_alg_send_data(int32_t *red_new_raw_data,int32_t *ir_new_raw_data, uint8_t dat_len,volatile int16_t *gsen_data_x,volatile int16_t *gsen_data_y,volatile int16_t *gsen_data_z);
void heart_rate_meas_timeout_handler(void * p_context);
#endif 

#endif
