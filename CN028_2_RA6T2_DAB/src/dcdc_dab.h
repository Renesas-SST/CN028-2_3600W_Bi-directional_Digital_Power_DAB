/*
 * dcdc_dab.h
 *
 *  Created on: 2022年11月24日
 *      Author: a5073522
 */

#ifndef DCDC_DAB_H_
#define DCDC_DAB_H_

#include <dab_lib.h>

#define M_PI                3.14159265358979323846
#define N_turn_ratio        8                       //砸比
#define Fs                  66000                   //开关管频率
#define L                   (62e-6f)                //电感值
#define K1                  (1/(8*Fs*L))

#define ISEC_EnterBurst     1.5 //A
#define ISEC_ExitBurst      1.8 //A
#define VSEC_BurstTh        2   //V
#define IPRI_EnterBurst     0.2 //A
#define IPRI_ExitBurst      0.4 //A
#define VPRI_BurstTh        10  //V

typedef enum
{
    APP_P2S_TargetS = 0,
    APP_S2P_TargetS = 1,
    APP_S2P_TargetP = 2,
}dab_app;

typedef enum
{
    STANDBY_STATE = 0,
    DAB_RUN_STATE = 1,
    FAULT_STATE   = 2,
}dab_state;

typedef struct
{

}t_cnt;

typedef struct
{
    _Bool sps_en;
}t_flag;

typedef struct
{
    dab_state state;
    dab_app app;
    oprt_mode oprt_mode;
    t_cnt cnt;
    t_flag flag;
}t_dab_sys;

typedef struct dab_para
{
    float Vsec_tgt, Vbus_tgt, Isec_tgt, Ibus_tgt;

    uint32_t adc_Vsec_raw, adc_Isec_raw, adc_Vbus_raw, adc_Ibus_raw, adc_Temp_raw, adc_Temp_HV_raw;
    int32_t adc_Isec_offset, adc_Ibus_offset;
    float adc_Vsec, adc_Isec, adc_Vbus, adc_Ibus, adc_Temp, adc_Temp_HV;
    float adc_Vsec_flt, adc_Isec_flt, adc_Vbus_flt, adc_Ibus_flt;
    float adc_Vsec_old, adc_Isec_old, adc_Vbus_old, adc_Ibus_old;

    float Pdc;
}t_dab_para;

void dab_para_init(void);
void dab_high_fre_routine(void);
void dab_stop(void);
void dab_high_protection(void);
void dab_low_protection(void);
void dab_routine(void);
void burst_function(void);

extern t_dab_para g_dab_para;
extern t_dab_sys g_dab_sys;

#endif /* DCDC_DAB_H_ */
