#ifndef _HX3690L_FACTORY_TEST_H_
#define _HX3690L_FACTORY_TEST_H_

#include <stdint.h>

#define FT_OPEN_G_LED_LTH     4000 +524288
#define FT_OPEN_R_LED_LTH     4000
#define FT_OPEN_IR_LED_LTH    4000

#define FT_CARD_G_LED_HTH     100000 +524288
#define FT_CARD_R_LED_HTH     80000
#define FT_CARD_IR_LED_HTH    100000

typedef enum {
	HR_LEAK_LIGHT_TEST = 1,
	HR_GRAY_CARD_TEST = 2,
	SPO2_LEAK_LIGHT_TEST = 3,
	SPO2_GRAY_CARD_TEST = 4
} HX3693_TEST_MODE;


void hx3693_hrs_factory_test_config(void);
void hx3693_spo2_factory_test_config(void);

SENSOR_ERROR_T hx3690l_hrs_factory_test_cfg(void);
SENSOR_ERROR_T hx3690l_spo2_factory_test_cfg(void);

uint8_t hx3690l_ft_hrs_read(hrs_sensor_data_t * s_dat);
uint8_t hx3690l_ft_spo2_read(spo2_sensor_data_t * s_dat);


bool hx3693_factory_test_read(int32_t *phase1_data,int32_t *phase2_data,int32_t *phase3_data,int32_t *phase4_data);
bool hx3693_factroy_test_old(uint32_t  test_mode);

bool hx3693_factroy_test(HX3693_TEST_MODE test_mode,uint8_t len,int32_t *ppg_buf,
    int32_t *ir_buf);

#endif // _HX3690L_FACTORY_TEST_H_




