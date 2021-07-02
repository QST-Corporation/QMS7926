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
#define     QMA7981_FSR                     0x0F
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
#define     RAISE_WAKE_DIFF_TH              0x03
#define     RAISE_WAKE_DIFF_TH              (0x03 << 6)
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
#define     STEP_COUNT_PEAK                 (0x01 << 4)
#define     INT2_OD                         (0x01 << 3)
#define     INT2_LVL                        (0x01 << 2)
#define     INT1_OD                         (0x01 << 1)
#define     INT1_LVL                        (0x01 << 0)
#define     STEP_START_CNT                  (0x07 << 5)
#define     STEP_COUNT_PEAK                 (0x03 << 3)
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
#define     STEP_CNT_HIGH                   0xFF
#define     STEP_CNT_MIDDLE                 0xFF
#define     STEP_CNT_LOW                    0xFF
#define     ACC_DATA_Z_HIGH                 0xFF
#define     ACC_DATA_Z_LOW                  0xFC
#define     ACC_DATA_Y_HIGH                 0xFF
#define     ACC_DATA_Y_LOW                  0xFC
#define     ACC_DATA_X_HIGH                 0xFF
#define     ACC_DATA_X_LOW                  0xFC