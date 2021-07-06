/**************************************************************************************************
 
  QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to QST Corporation ("QST"). 
  Your use of this Software is limited to those 
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

#ifndef _QMA7981_H
#define _QMA7981_H

#define     QMA7981_STEPCOUNTER
#define     QMA7981_HAND_UP_DOWN
//#define     SLEEP_AlGORITHM
//#define     QMA7981_ANY_MOTION
#define     QMA7981_INT_LATCH

#define     QMA7981_SLAVE_ADDR              0x12
#define     QMA7981_LAYOUT                  0
#define     GRAVITY_EARTH_1000              9807	// about (9.80665f)*1000   mm/s2

#define     QMA7981_RAISE_CFG5              0x3F
#define     QMA7981_RAISE_CFG4              0x3E
#define     QMA7981_S_RESET                 0x36
#define     QMA7981_RAISE_CFG3              0x35
#define     QMA7981_RAISE_CFG2              0x34
#define     QMA7981_NVM_CFG                 0x33
#define     QMA7981_ST                      0x32
#define     QMA7981_RST_MOT                 0x30
#define     QMA7981_MOT_CFG3                0x2F
#define     QMA7981_MOT_CFG2                0x2E
#define     QMA7981_MOT_CFG1                0x2D
#define     QMA7981_MOT_CFG0                0x2C
#define     QMA7981_RAISE_CFG1              0x2B
#define     QMA7981_RAISE_CFG0              0x2A
#define     QMA7981_OS_CUST_Z               0x29
#define     QMA7981_OS_CUST_Y               0x28
#define     QMA7981_OS_CUST_X               0x27
#define     QMA7981_INT_CFG                 0x21
#define     QMA7981_INT_PIN_CFG             0x20
#define     QMA7981_STEP_CFG6               0x1F
#define     QMA7981_STEP_CFG5               0x1E
#define     QMA7981_STEP_CFG4               0x1D
#define     QMA7981_INT_MAP3                0x1C
#define     QMA7981_INT_MAP2                0x1B
#define     QMA7981_INT_MAP1                0x1A
#define     QMA7981_INT_MAP0                0x19
#define     QMA7981_INT_EN2                 0x18
#define     QMA7981_INT_EN1                 0x17
#define     QMA7981_INT_EN0                 0x16
#define     QMA7981_STEP_CFG3               0x15
#define     QMA7981_STEP_CFG2               0x14
#define     QMA7981_STEP_CFG1               0x13
#define     QMA7981_STEP_CFG0               0x12
#define     QMA7981_PM                      0x11
#define     QMA7981_BW                      0x10
#define     QMA7981_REG_RANGE               0x0F
#define     QMA7981_STEPCNT_HIGHBYTE        0x0E
#define     QMA7981_INT_ST4                 0x0D
#define     QMA7981_INT_ST3                 0x0C
#define     QMA7981_INT_ST2                 0x0B
#define     QMA7981_INT_ST1                 0x0A
#define     QMA7981_INT_ST0                 0x09
#define     QMA7981_STEPCNT_MIDDLEBYTE      0x08
#define     QMA7981_STEPCNT_LOWBYTE         0x07
#define     QMA7981_DATA_HIGHBYTE_Z         0x06
#define     QMA7981_DATA_LOWBYTE_Z          0x05
#define     QMA7981_DATA_HIGHBYTE_Y         0x04
#define     QMA7981_DATA_LOWBYTE_Y          0x03
#define     QMA7981_DATA_HIGHBYTE_X         0x02
#define     QMA7981_DATA_LOWBYTE_X          0x01
#define     QMA7981_CHIP_ID                 0x00

#define     RAISE_WAKE_PERIOD_HIGH          (0x07 << 4)
#define     RAISE_WAKE_PERIOD_LOW           0xFF
#define     RAISE_WAKE_TIMEOUT_TH_HIGH      (0x0F << 0)
#define     RAISE_WAKE_TIMEOUT_TH_LOW       0xFF
#define     YZ_TH_SEL                       (0x07 << 5)
#define     Y_TH                            (0x1F << 0)
#define     NVM_LOAD                        (0x01 << 3)
#define     NVM_RDY                         (0x01 << 2)
#define     NVM_PROG                        (0x01 << 1)
#define     NVM_LOAD_DONE                   (0x01 << 0)
#define     SELFTEST_BIT                    (0x01 << 7)
#define     SELFTEST_SIGN                   (0x01 << 2)
#define     BP_AXIS_STEP                    (0x03 << 0)
#define     MO_BP_CO                        (0x01 << 7)
#define     STEP_BP_CO                      (0x01 << 6)
#define     LOW_RST_N                       (0x01 << 4)
#define     HIGH_RST_N                      (0x01 << 3)
#define     NO_MOT_RST_N                    (0x01 << 2)
#define     SIG_MOT_RST_N                   (0x01 << 1)
#define     ANY_MOT_RST_N                   0x01
#define     SIG_MOT_TPROOF                  (0x03 << 4)
#define     SIG_MOT_TSKIP                   (0x03 << 2)
#define     SIG_MOT_SEL                     0x01
#define     ANY_MOT_TH                      0xFF
#define     NO_MOT_TH                       0xFF
#define     NO_MOT_DUR                      (0x3F << 2)
#define     ANY_MOT_DUR                     0x03
#define     HD_Z_TH                         (0x07 << 5)
#define     HD_X_TH                         (0x07 << 2)
#define     RAISE_WAKE_DIFF_TH_HIGH         0x03
#define     RAISE_WAKE_DIFF_TH_LOW          (0x03 << 6)
#define     RAISE_WAKE_SUM_TH               0x3F
#define     OS_CUST_Z                       0xFF
#define     OS_CUST_Y                       0xFF
#define     OS_CUST_X                       0xFF
#define     INT_RD_CLR                      (0x01 << 7)
#define     SHADOW_DIS                      (0x01 << 6)
#define     DIS_I2C                         (0x01 << 5)
#define     LATCH_INT_STEP                  (0x01 << 1)
#define     LATCH_INT                       0x01
#define     DIS_PU_SENB                     (0x01 << 7)
#define     DIS_IE_AD0                      (0x01 << 6)
#define     EN_SPI3W                        (0x01 << 5)
#define     STEP_COUNT_PEAK_HIGH            (0x01 << 4)
#define     INT2_OD                         (0x01 << 3)
#define     INT2_LVL                        (0x01 << 2)
#define     INT1_OD                         (0x01 << 1)
#define     INT1_LVL                        (0x01 << 0)
#define     STEP_START_CNT                  (0x07 << 5)
#define     STEP_COUNT_PEAK_LOW             (0x03 << 3)
#define     STEP_COUNT_P2P                  (0x07 << 0)
#define     Z_TH                            (0x0F << 4)
#define     X_TH                            (0x0F << 0)
#define     STEP_INTERVAL                   (0x7F << 1)
#define     EN_RESET_DC                     (0x01 << 0)
#define     INT2_NO_MOT                     (0x01 << 7)
#define     INT2_DATA                       (0x01 << 4)
#define     INT2_LOW                        (0x01 << 3)
#define     INT2_HIGH                       (0x01 << 2)
#define     INT2_ANY_MOT                    (0x01 << 0)
#define     INT2_SIG_STEP                   (0x01 << 6)
#define     INT2_STEP                       (0x01 << 3)
#define     INT2_HD                         (0x01 << 2)
#define     INT2_RAISE                      (0x01 << 1)
#define     INT2_SIG_MOT                    (0x01 << 0)
#define     INT1_NO_MOT                     (0x01 << 7)
#define     INT1_DATA                       (0x01 << 4)
#define     INT1_LOW                        (0x01 << 3)
#define     INT1_HIGH                       (0x01 << 2)
#define     INT1_ANY_MOT                    (0x01 << 0)
#define     INT1_SIG_STEP                   (0x01 << 6)
#define     INT1_STEP                       (0x01 << 3)
#define     INT1_HD                         (0x01 << 2)
#define     INT1_RAISE                      (0x01 << 1)
#define     INT1_SIG_MOT                    (0x01 << 0)
#define     NO_MOT_EN_Z                     (0x01 << 7)
#define     NO_MOT_EN_Y                     (0x01 << 6)
#define     NO_MOT_EN_X                     (0x01 << 5)
#define     ANY_MOT_EN_Z                    (0x01 << 2)
#define     ANY_MOT_EN_Y                    (0x01 << 1)
#define     ANY_MOT_EN_X                    (0x01 << 0)
#define     INT_DATA_EN                     (0x01 << 4)
#define     LOW_EN                          (0x01 << 3)
#define     HIGH_EN_Z                       (0x01 << 2)
#define     HIGH_EN_Y                       (0x01 << 1)
#define     HIGH_EN_X                       (0x01 << 0)
#define     SIG_STEP_IEN                    (0x01 << 6)
#define     STEP_IEN                        (0x01 << 3)
#define     HD_EN                           (0x01 << 2)
#define     RAISE_EN                        (0x01 << 1)
#define     STEP_TIME_UP                    0xFF
#define     STEP_TIME_LOW                   0xFF
#define     STEP_CLR                        (0x01 << 7)
#define     STEP_PRECISION                  (0x7F << 0)
#define     STEP_EN                         (0x01 << 7)
#define     STEP_SAMPLE_CNT                 (0x7F << 0)
#define     MODE_BIT                        (0x01 << 7)
#define     T_RSTB_SINC_SEL                 (0x03 << 4)
#define     MCLK_SEL                        (0x0F << 0)
#define     BW                              (0x1F << 0)
#define     RANGE                           (0x0F << 0)
#define     HIGH_INT                        (0x01 << 4)
#define     HIGH_SIGN                       (0x01 << 3)
#define     HIGH_FIRST_Z                    (0x01 << 2)
#define     HIGH_FIRST_Y                    (0x01 << 1)
#define     HIGH_FIRST_X                    (0x01 << 0)
#define     DATA_INT                        (0x01 << 4)
#define     LOW_INT                         (0x01 << 3)
#define     SIG_STEP                        (0x01 << 6)
#define     STEP_INT                        (0x01 << 3)
#define     HD_INT                          (0x01 << 2)
#define     RAISE_INT                       (0x01 << 1)
#define     SIG_MOT_INT                     (0x01 << 0)
#define     NO_MOT                          (0x01 << 7)
#define     ANY_MOT_SIGN                    (0x01 << 3)
#define     ANY_MOT_FIRST_Z                 (0x01 << 2)
#define     ANY_MOT_FIRST_Y                 (0x01 << 1)
#define     ANY_MOT_FIRST_X                 (0x01 << 0)
#define     STEP_W_TIME_L                   300
#define     STEP_W_TIME_H                   250
#define     QMA7981_DELAY                   0xFF

/* Accelerometer Sensor Full Scale */
#define QMA7981_RANGE_2G      0x01
#define QMA7981_RANGE_4G      0x02
#define QMA7981_RANGE_8G      0x04
#define QMA7981_RANGE_16G     0x08
#define QMA7981_RANGE_32G     0x0f

#ifdef SLEEP_AlGORITHM
#define QST_SLEEP_TIMER_RATIO		2			// timer 1 second
#define QST_SLEEP_TIMER_CYC			60			// timer cycle one minute
#define QST_SLEEP_FIFO_LEN                  32
#define QST_SLEEP_FIFO_SET_INTERVAL         62	// 2s to sample 32 group data
#define QST_SLEEP_LIGHTSLEEP_CONTINUE_NUM			3	// 6  锟斤拷锟窖碉拷浅睡3锟斤拷锟斤拷 6~8
#define QST_SLEEP_DEEPSLEEP_CONTINUE_NUM			10	// 20  浅睡锟斤拷锟斤拷睡锟斤拷时锟斤拷20~30

#define QST_SLEEP_RECORD_LEN						1000
#define QST_SLEEP_RECORD_COUNT_MIN					8

#define QST_SLEEP_MONTION_THRESHOLD					9.0f
#define QST_SLEEP_BIGMONTION_THRESHOLD				200.0f
//small motion
#define QST_SLEEP_MOTION_LIGHT_THRESHOLD			5.0
#define QST_SLEEP_MOTION_AWAKE_THRESHOLD			17.0	//15.0 
//small motion
//big motion
#define QST_SLEEP_BIGMOTION_LIGHT_THRESHOLD			2.0 
#define QST_SLEEP_BIGMOTION_AWAKE_THRESHOLD			8.0 
// big motion  threshol
#define  STEPCOUNTER_THRESHOLD_REG_ENTRY_SLEEP      Step_SENSITIVITY //锟狡诧拷锟斤拷锟斤拷锟斤拷锟斤拷  0x13锟侥达拷锟斤拷锟斤拷值
#define  STEPCOUNTER_THRESHOLD_REG_QUIT_SLEEP       Step_SENSITIVITY //锟狡诧拷锟斤拷锟斤拷锟斤拷锟斤拷  0x13锟侥达拷锟斤拷锟斤拷值
typedef enum
{
	QST_SLEEP_NONE,
	QST_SLEEP_AWAKE = 0x01,
	QST_SLEEP_LIGHT = 0x02,
	QST_SLEEP_DEEP = 0x03,
	QST_SLEEP_UNKNOW = 0xff,

	QST_SLEEP_TOTAL
}qst_sleep_status;

#endif


enum{
	handUp_event   = 0x01,	//hand raise
	handDown_event = 0x02,	//hand down
	stap_event     = 0x04,	//single tap
	dtap_event     = 0x08,  //double tap
	wmi_event      = 0x10,	//acceleration data
	acc_event      = 0x20,	//tilt event
  step_event     = 0x40   //step counter
};

typedef struct _QMA7981_ev_t{
  uint8_t ev;
  uint8_t flg;
  uint8_t size;
  void* data;
}QMA7981_ev_t;

typedef void (*QMA7981_evt_hdl_t)	(QMA7981_ev_t* pev);

uint8_t QMA7981_read_acc(float *accData);
void QMA7981_acc_report_start(uint32_t report_intval_ms);
void QMA7981_acc_report_stop(void);
uint8_t QMA7981_report_acc(void);
void QMA7981_report_handup(void);
void QMA7981_deep_sleep(void);
void QMA7981_wake_up(void);
void QMA7981_disable(void);
int QMA7981_init(QMA7981_evt_hdl_t evt_hdl);
#if defined(QMA7981_STEPCOUNTER)
void QMA7981_clear_step(void);
uint32_t QMA7981_read_stepcounter(void);
void QMA7981_step_report_start(uint32_t report_intval_ms);
void QMA7981_step_report_stop(void);
uint8_t QMA7981_report_stepcounter(void);
#endif
#ifdef SLEEP_AlGORITHM
void qma7981_read_fifo(void);
uint8_t qst_sleep_set(char enable);
void qst_sleep_data_process(void);
qst_sleep_status qst_sleep_get_status(void);
#endif

#endif   /* _QMA7981_H */
