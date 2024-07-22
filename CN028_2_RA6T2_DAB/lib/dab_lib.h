/*
 * dab_lib.h
 *
 *  Created on: Aug 22, 2023
 *      Author: a5073522
 */

#ifndef DAB_LIB_H_
#define DAB_LIB_H_

typedef enum
{
    SecCV = 0,      /* 副边恒压控制 */
    SecCC = 1,      /* 副边恒流控制 */
    PriCV = 2,      /* 原边恒压控制 */
    PriCC = 3,      /* 原边恒流控制 */
}oprt_mode;

typedef enum
{
    UPPER,
    DOWN,
}end;

typedef enum
{
    LOW,
    HIGH,
}level;

typedef struct
{
    float pAngle;
    uint8_t end_level[2];
    uint32_t PWM_value[8];
}t_pwm_shift;

typedef struct
{
    uint32_t PWM_period;
    uint32_t PWM_over_period;
    uint32_t PWM_deadtime;
}t_pwm_value;

/* PID structure */
typedef struct
{
    float  Ref;             /* reference value */
    float  Fdb;             /* feedback value */
    float  Err;             /* error */
    float  SatErr;          /* integration saturation error */
    float  Kp;              /* proportion gain */
    float  Ki;              /* integration gain */
    float  Kd;              /* differential gain */
    float  P_Out;           /* output value of proportion part */
    float  I_Out;           /* output value of integration part */
    float  D_Out;           /* output value of differential part */
    float  OutMax;          /* maximum threshold */
    float  OutMin;          /* minimum threshold */
    float  OutPreSat;       /* output value before threshold comparison */
    float  Out;             /* PID output value */
    void  (*calc)();        /* Point to PID calculation function */
}t_AntiSatPID;
typedef t_AntiSatPID *t_AntiSatPID_handle;

typedef struct
{
    uint8_t Enable;         /* Enable bit */
    float   Input;          /* Input value */
    float   Out;            /* Output value */
    float   OutLst;         /* Previous output value */
    float   MaxOut;         /* Maximum threshold */
    float   MinOut;         /* Minimum threshold */
    float   ACC_MaxStp;     /* Accelerate ramp step */
    float   DEC_MaxStp;     /* Decelerate ramp step */
    float   Init;           /* Initial value */
    void (*calc)();         /* Point to ramp calculation function */
} t_RAMP2;
typedef t_RAMP2 *t_RAMP2_handle;
#define RAMP2_DEFAULTS          \
{   0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    0,                          \
    (void (*)(u32))RAMP2_calc   \
}

typedef struct goc_para
{
    float Dss;              /* 外移相比 */
    float Dp0;              /* 原边内移相比 */
    float Ds0;              /* 副边内移相比 */
    float M;                /* 电压传输比 */

    uint8_t ModulationMode; /* 1:默认为TPS调制, 0:SPS */
    int8_t Dir;             /* 功率方向 */
    uint8_t Boost;          /* 0:buck, 1:boost */
    uint8_t PwrLvl;         /* 1:轻载, 2:中载, 3:重载 */
    float L_load, M_load, H_load;
    float Pbase;            /* 基准功率 */
    float Ppu;              /* 功率标幺值 */
    float Pout_pu;          /* 补偿功率标幺值 */

    float Vin;
    float Vout;
    float Iin;
    float Iout;
    float N;
    float K;
    oprt_mode mode;
    float Vsec_tgt;
    float Isec_tgt;
    float Vbus_tgt;
    float Ibus_tgt;
    t_RAMP2 * RVout;
    t_RAMP2 * RVin;
    t_RAMP2 * RIout;
    t_RAMP2 * RIin;
    t_AntiSatPID * pri_vloop;
    t_AntiSatPID * sec_vloop;
    t_AntiSatPID * sec_iloop;
    t_AntiSatPID * pri_iloop;

    float M_reciprocal;
    float D1;
    float D2;
    float Dpsi;
}t_goc_para;

void PWM_para_init(t_pwm_shift * ch0, t_pwm_shift * ch1, t_pwm_shift * ch2, t_pwm_shift * ch3, t_pwm_value pwm);
void PWM_value_refresh(t_pwm_shift * ch, t_pwm_value pwm);
void dab_global_optimized_control_pw(t_goc_para * Opt);

#endif /* DAB_LIB_H_ */
