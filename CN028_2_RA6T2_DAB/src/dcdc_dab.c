/*
 * dcdc_dab.c
 *
 *  Created on: 2022年11月24日
 *      Author: a5073522
 */
#include "hal_data.h"
#include "dcdc_dab.h"
#include "math.h"
#include "ra6t2.h"
#include "console.h"
#include "err_handler.h"

typedef enum
{
    BURST_NOT_WORK = 0,
    BURST_PWM_ON = 1,
    BURST_PWM_OFF   = 2,
}burst_state;

void dab_para_reset(void);
void dab_console_set(void);
void dab_PWM_refresh(void);
void dab_start(void);
void pwm_start(void);
void pwm_stop(void);

t_dab_sys g_dab_sys;
t_dab_para g_dab_para;
extern volatile uint32_t g_key_signal;
t_RAMP2 ramp_for_vout, ramp_for_iout, ramp_for_vin, ramp_for_iin;
t_AntiSatPID g_dab_vloop_pi, g_dab_iloop_pi, g_dab_pri_vloop_pi, g_dab_pri_iloop_pi;
t_goc_para g_goc_para;

float Ts_step;

uint8_t burst_code = BURST_NOT_WORK;
bool f_burst = false;
bool f_pwm_output = false;

void dab_para_init(void)
{
    g_dab_para.Vsec_tgt = 48;
    g_dab_para.Isec_tgt = 5;
    g_dab_para.Ibus_tgt = -1;
    g_dab_para.Vbus_tgt = 400;

    g_dab_sys.state = STANDBY_STATE;
    g_dab_sys.app = APP_P2S_TargetS;
    g_dab_sys.oprt_mode = SecCV;

    g_dab_vloop_pi.Kp = (float)0.065;       //控制副边
    g_dab_vloop_pi.Ki = (float)0.00001;
    g_dab_vloop_pi.OutMax = 1;
    g_dab_vloop_pi.OutMin = -1;
    g_dab_vloop_pi.I_Out = 0;
    g_dab_vloop_pi.SatErr = 0;

    g_dab_iloop_pi.Kp = 0.005;
    g_dab_iloop_pi.Ki = (float)0.00001;
    g_dab_iloop_pi.OutMax = 1;
    g_dab_iloop_pi.OutMin = -1;
    g_dab_iloop_pi.I_Out = 0;
    g_dab_iloop_pi.SatErr = 0;

    g_dab_pri_vloop_pi.Kp = (float)0.00195;   //控制原边
    g_dab_pri_vloop_pi.Ki = (float)0.000001;
    g_dab_pri_vloop_pi.OutMax = 1;
    g_dab_pri_vloop_pi.OutMin = -1;
    g_dab_pri_vloop_pi.I_Out = 0;
    g_dab_pri_vloop_pi.SatErr = 0;

    g_dab_pri_iloop_pi.Kp = (float)0.005;
    g_dab_pri_iloop_pi.Ki = (float)0.00001;
    g_dab_pri_iloop_pi.OutMax = 1;
    g_dab_pri_iloop_pi.OutMin = -1;
    g_dab_pri_iloop_pi.I_Out = 0;
    g_dab_pri_iloop_pi.SatErr = 0;

    ramp_for_vout.Enable = 1;
    ramp_for_vout.ACC_MaxStp = 0.00015;
    ramp_for_vout.DEC_MaxStp = 0.00015;
    ramp_for_vout.MaxOut = 60;
    ramp_for_vout.MinOut = 0;

    ramp_for_vin.Enable = 1;
    ramp_for_vin.ACC_MaxStp = 0.0015;
    ramp_for_vin.DEC_MaxStp = 0.0015;
    ramp_for_vin.MaxOut = 420;
    ramp_for_vin.MinOut = 0;

    ramp_for_iout.Enable = 1;
    ramp_for_iout.ACC_MaxStp = 0.00015;
    ramp_for_iout.DEC_MaxStp = 0.00015;
    ramp_for_iout.MaxOut = 75;
    ramp_for_iout.MinOut = -75;

    ramp_for_iin.Enable = 1;
    ramp_for_iin.ACC_MaxStp = 0.00015;
    ramp_for_iin.DEC_MaxStp = 0.00015;
    ramp_for_iin.MaxOut = 9;
    ramp_for_iin.MinOut = -9;

    con.pi_iloop_kp = g_dab_iloop_pi.Kp;
    con.pi_iloop_ki = g_dab_iloop_pi.Ki;
    con.pi_vloop_kp = g_dab_vloop_pi.Kp;
    con.pi_vloop_ki = g_dab_vloop_pi.Ki;
    con.pri_pi_vloop_kp = g_dab_pri_vloop_pi.Kp;
    con.pri_pi_vloop_ki = g_dab_pri_vloop_pi.Ki;
    con.pri_pi_iloop_kp = g_dab_pri_iloop_pi.Kp;
    con.pri_pi_iloop_ki = g_dab_pri_iloop_pi.Ki;
    con.sys_app = g_dab_sys.app;
    con.oprt_mode = g_dab_sys.oprt_mode;
    con.Vsec_ramp_step = ramp_for_vout.ACC_MaxStp * Fs;
    con.Isec_ramp_step = ramp_for_iout.ACC_MaxStp * Fs;
    con.Vpri_ramp_step = ramp_for_vin.ACC_MaxStp * Fs;
    con.Ipri_ramp_step = ramp_for_iin.ACC_MaxStp * Fs;

    Ts_step = (float)1/Fs;
}

void dab_para_reset(void)
{
    switch (g_dab_sys.oprt_mode)
    {
        case PriCV:
        case PriCC:
            g_goc_para.M = 1.49;//N_turn_ratio * Vout / Vin;
            break;
        case SecCV:
        case SecCC:
        default:
            g_goc_para.M = 0.67;
            break;
    }

    ramp_for_vout.OutLst = g_dab_para.adc_Vsec_flt;
    ramp_for_iout.OutLst = g_dab_para.adc_Isec_flt;
    ramp_for_vin.OutLst = g_dab_para.adc_Vbus_flt;
    ramp_for_iin.OutLst = g_dab_para.adc_Ibus_flt;

    g_dab_iloop_pi.I_Out = 0;
    g_dab_iloop_pi.SatErr = 0;

    g_dab_vloop_pi.I_Out = 0;
    g_dab_vloop_pi.SatErr = 0;

    g_dab_pri_vloop_pi.I_Out = 0;
    g_dab_pri_vloop_pi.SatErr = 0;

    g_dab_pri_iloop_pi.I_Out = 0;
    g_dab_pri_iloop_pi.SatErr = 0;

    burst_code = BURST_NOT_WORK;
}

void dab_console_set(void)
{
    g_dab_sys.app = con.sys_app;
    g_dab_sys.oprt_mode = con.oprt_mode;
    g_dab_iloop_pi.Kp = con.pi_iloop_kp;
    g_dab_iloop_pi.Ki = con.pi_iloop_ki;
    g_dab_vloop_pi.Kp = con.pi_vloop_kp;
    g_dab_vloop_pi.Ki = con.pi_vloop_ki;
    g_dab_pri_vloop_pi.Kp = con.pri_pi_vloop_kp;
    g_dab_pri_vloop_pi.Ki = con.pri_pi_vloop_ki;
    g_dab_pri_iloop_pi.Kp = con.pri_pi_iloop_kp;
    g_dab_pri_iloop_pi.Ki = con.pri_pi_iloop_ki;
    ramp_for_vout.ACC_MaxStp = con.Vsec_ramp_step * Ts_step;
    ramp_for_vout.DEC_MaxStp = con.Vsec_ramp_step * Ts_step;
    ramp_for_iout.ACC_MaxStp = con.Isec_ramp_step * Ts_step;
    ramp_for_iout.DEC_MaxStp = con.Isec_ramp_step * Ts_step;
    ramp_for_vin.ACC_MaxStp = con.Vpri_ramp_step * Ts_step;
    ramp_for_vin.DEC_MaxStp = con.Vpri_ramp_step * Ts_step;
    ramp_for_iin.ACC_MaxStp = con.Ipri_ramp_step * Ts_step;
    ramp_for_iin.DEC_MaxStp = con.Ipri_ramp_step * Ts_step;
    g_dab_sys.flag.sps_en = con.sps_en;
}

void burst_function(void)
{
    if (g_dab_sys.oprt_mode == SecCV && g_dab_sys.state == DAB_RUN_STATE)
    {
        if (g_dab_para.adc_Isec_flt < ISEC_EnterBurst && g_dab_para.adc_Isec_flt > (-ISEC_EnterBurst))
        {
            f_burst = true;
        }
        else if (g_dab_para.adc_Isec_flt > ISEC_ExitBurst || g_dab_para.adc_Isec_flt < (-ISEC_ExitBurst))
        {
            f_burst = false;
        }

        if (f_burst)
        {
            float gap;
            gap = g_dab_para.adc_Vsec_flt - g_dab_para.Vsec_tgt;
            if (((gap > (VSEC_BurstTh + 0.1)) || (gap < (-VSEC_BurstTh - 0.1))) && (f_pwm_output == false))
            {
                dab_para_reset();
                pwm_start();
                f_pwm_output = true;
                burst_code = BURST_PWM_ON;
            }
            else if ((gap > (-VSEC_BurstTh + 0.1)) && (gap < (VSEC_BurstTh - 0.1)))
            {
                if (f_pwm_output == true)
                {
                    pwm_stop();
                    f_pwm_output = false;
                }
                burst_code = BURST_PWM_OFF;
            }
        }
        else
        {
            if (f_pwm_output == false)
            {
                dab_para_reset();
                pwm_start();
                f_pwm_output = true;
            }
            burst_code = BURST_NOT_WORK;
        }
    }
    else if (g_dab_sys.oprt_mode == PriCV && g_dab_sys.state == DAB_RUN_STATE)
    {
        if (g_dab_para.adc_Ibus_flt < IPRI_EnterBurst && g_dab_para.adc_Ibus_flt > (-IPRI_EnterBurst))
        {
            f_burst = true;
        }
        else if (g_dab_para.adc_Ibus_flt > IPRI_ExitBurst || g_dab_para.adc_Ibus_flt < (-IPRI_ExitBurst))
        {
            f_burst = false;
        }

        if (f_burst)
        {
            float gap;
            gap = g_dab_para.adc_Vbus_flt - g_dab_para.Vbus_tgt;
            if (((gap > (VPRI_BurstTh + 1)) || (gap < (-VPRI_BurstTh - 1))) && (f_pwm_output == false))
            {
                dab_para_reset();
                pwm_start();
                f_pwm_output = true;
                burst_code = BURST_PWM_ON;
            }
            else if ((gap > (-VPRI_BurstTh + 1)) && (gap < (VPRI_BurstTh - 1)))
            {
                if (f_pwm_output == true)
                {
                    pwm_stop();
                    f_pwm_output = false;
                }
                burst_code = BURST_PWM_OFF;
            }
        }
        else
        {
            if (f_pwm_output == false)
            {
                dab_para_reset();
                pwm_start();
                f_pwm_output = true;
            }
            burst_code = BURST_NOT_WORK;
        }
    }
}

void dab_high_fre_routine(void)
{
    if (burst_code == BURST_PWM_OFF)
    {
        return;
    }

    g_goc_para.Vout = g_dab_para.adc_Vsec_flt;
    g_goc_para.Vin = g_dab_para.adc_Vbus_flt;
    g_goc_para.Iout = g_dab_para.adc_Isec_flt;
    g_goc_para.Iin = g_dab_para.adc_Ibus_flt;
    g_goc_para.N = N_turn_ratio;
    g_goc_para.K = K1;
    g_goc_para.mode = g_dab_sys.oprt_mode;
    g_goc_para.Vbus_tgt = g_dab_para.Vbus_tgt;
    g_goc_para.Vsec_tgt = g_dab_para.Vsec_tgt;
    g_goc_para.Isec_tgt = g_dab_para.Isec_tgt;
    g_goc_para.Ibus_tgt = g_dab_para.Ibus_tgt;
    g_goc_para.RIout = &ramp_for_iout;
    g_goc_para.RVin = &ramp_for_vin;
    g_goc_para.RVout = &ramp_for_vout;
    g_goc_para.RIin = &ramp_for_iin;
    g_goc_para.pri_vloop = &g_dab_pri_vloop_pi;
    g_goc_para.pri_iloop = &g_dab_pri_iloop_pi;
    g_goc_para.sec_iloop = &g_dab_iloop_pi;
    g_goc_para.sec_vloop = &g_dab_vloop_pi;
    dab_global_optimized_control_pw(&g_goc_para);

    dab_PWM_refresh();
}

void dab_PWM_refresh(void)
{
    float tmp;

	//开环SPS
    if (g_dab_sys.flag.sps_en)
    {
        g_goc_para.Dp0 = 0;
        g_goc_para.Ds0 = 0;
        g_goc_para.Dss = con.sps;
    }

    /* pAngle表示原边BH,副边AH,副边BH对原边AH的移相角
     * Dp0表示原边BL对原边AH的移相角
     * Dss表示副边AH对原边AH的移相角
     * Ds0表示副边BL对副边AH的移相角 */
    if (g_goc_para.Dp0 < 0)
    {
        g_pwm_shift_ch1.pAngle = g_goc_para.Dp0 + 1;
    }
    else
    {
        g_pwm_shift_ch1.pAngle = g_goc_para.Dp0 - 1;
    }

    g_pwm_shift_ch2.pAngle = g_goc_para.Dss;
    tmp = g_goc_para.Dss + g_goc_para.Ds0;
    while (tmp > 1)
    {
        tmp -= 2;
    }
    while (tmp < -1)
    {
        tmp += 2;
    }
    if (tmp < 0)
    {
        g_pwm_shift_ch3.pAngle = tmp + 1;
    }
    else
    {
        g_pwm_shift_ch3.pAngle = tmp - 1;
    }

    PWM_value_refresh(&g_pwm_shift_ch1, g_pwm_value);
    PWM_value_refresh(&g_pwm_shift_ch2, g_pwm_value);
    PWM_value_refresh(&g_pwm_shift_ch3, g_pwm_value);
}

void dab_start(void)
{
    /* Charge the bootstrap capacitors of the upper transistors */
    R_BSP_PinAccessEnable();

    /* as general port */
    R_PFS->PORT[11].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[14].PmnPFS = 0x00000004;    /* low output */
    R_PFS->PORT[11].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[15].PmnPFS = 0x00000007;    /* high output */
    R_PFS->PORT[12].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[14].PmnPFS = 0x00000004;
    R_PFS->PORT[12].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[15].PmnPFS = 0x00000007;
    R_PFS->PORT[13].PIN[8].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[8].PmnPFS = 0x00000004;
    R_PFS->PORT[13].PIN[9].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[9].PmnPFS = 0x00000007;
    R_PFS->PORT[14].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[14].PmnPFS = 0x00000004;
    R_PFS->PORT[14].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[15].PmnPFS = 0x00000007;
    DRV_SD_OUTPUT = 0;

    R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);

    /* PB14 as GTIOC1A */
    R_PFS->PORT[11].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[14].PmnPFS = 0x03010000;
    /* PB15 as GTIOC1B */
    R_PFS->PORT[11].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[15].PmnPFS = 0x03010000;;
    /* PC14 as GTIOC3A */
    R_PFS->PORT[12].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[14].PmnPFS = 0x03010000;;
    /* PC15 as GTIOC3B */
    R_PFS->PORT[12].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[15].PmnPFS = 0x03010000;;
    /* PD08 as GTIOC2A */
    R_PFS->PORT[13].PIN[8].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[8].PmnPFS = 0x03010000;;
    /* PD09 as GTIOC2B */
    R_PFS->PORT[13].PIN[9].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[9].PmnPFS = 0x03010000;;
    /* PE14 as GTIOC0A */
    R_PFS->PORT[14].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[14].PmnPFS = 0x03010000;;
    /* PE15 as GTIOC0B */
    R_PFS->PORT[14].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[15].PmnPFS = 0x03010000;;

    R_BSP_PinAccessDisable();
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    DRV_SD_OUTPUT = 0;
    dab_console_set();
    dab_para_reset();

    pwm_start();
    f_pwm_output = true;
    f_burst = false;
}

void pwm_start(void)
{
    R_GPT0->GTSTP = 0x10F;

    PWM_para_init(&g_pwm_shift_ch0, &g_pwm_shift_ch1, &g_pwm_shift_ch2, &g_pwm_shift_ch3, g_pwm_value);

    R_GPT0->GTCCR[0] = g_pwm_shift_ch0.PWM_value[0];    //GTCCRA
    R_GPT0->GTCCR[2] = g_pwm_shift_ch0.PWM_value[0];    //GTCCRC
    R_GPT0->GTCCR[4] = g_pwm_shift_ch0.PWM_value[1];    //GTCCRD
    R_GPT0->GTCCR[1] = g_pwm_shift_ch0.PWM_value[2];    //GTCCRB
    R_GPT0->GTCCR[3] = g_pwm_shift_ch0.PWM_value[2];    //GTCCRE
    R_GPT0->GTCCR[5] = g_pwm_shift_ch0.PWM_value[3];    //GTCCRF

    R_GPT1->GTCCR[0] = g_pwm_shift_ch1.PWM_value[0];    //GTCCRA
    R_GPT1->GTCCR[2] = g_pwm_shift_ch1.PWM_value[0];    //GTCCRC
    R_GPT1->GTCCR[4] = g_pwm_shift_ch1.PWM_value[1];    //GTCCRD
    R_GPT1->GTCCR[1] = g_pwm_shift_ch1.PWM_value[2];    //GTCCRB
    R_GPT1->GTCCR[3] = g_pwm_shift_ch1.PWM_value[2];    //GTCCRE
    R_GPT1->GTCCR[5] = g_pwm_shift_ch1.PWM_value[3];    //GTCCRF

    R_GPT2->GTCCR[0] = g_pwm_shift_ch2.PWM_value[0];    //GTCCRA
    R_GPT2->GTCCR[2] = g_pwm_shift_ch2.PWM_value[0];    //GTCCRC
    R_GPT2->GTCCR[4] = g_pwm_shift_ch2.PWM_value[1];    //GTCCRD
    R_GPT2->GTCCR[1] = g_pwm_shift_ch2.PWM_value[2];    //GTCCRB
    R_GPT2->GTCCR[3] = g_pwm_shift_ch2.PWM_value[2];    //GTCCRE
    R_GPT2->GTCCR[5] = g_pwm_shift_ch2.PWM_value[3];    //GTCCRF

    R_GPT3->GTCCR[0] = g_pwm_shift_ch3.PWM_value[0];    //GTCCRA
    R_GPT3->GTCCR[2] = g_pwm_shift_ch3.PWM_value[0];    //GTCCRC
    R_GPT3->GTCCR[4] = g_pwm_shift_ch3.PWM_value[1];    //GTCCRD
    R_GPT3->GTCCR[1] = g_pwm_shift_ch3.PWM_value[2];    //GTCCRB
    R_GPT3->GTCCR[3] = g_pwm_shift_ch3.PWM_value[2];    //GTCCRE
    R_GPT3->GTCCR[5] = g_pwm_shift_ch3.PWM_value[3];    //GTCCRF

    R_GPT0->GTCNT = 0;
    R_GPT1->GTCNT = 0;
    R_GPT2->GTCNT = 0;
    R_GPT3->GTCNT = 0;
    R_GPT8->GTCNT = 0;

    R_BSP_PinAccessEnable();
    R_PFS->PORT[11].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[14].PmnPFS = 0x03010000;
    R_PFS->PORT[11].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[15].PmnPFS = 0x03010000;;
    R_PFS->PORT[12].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[14].PmnPFS = 0x03010000;;
    R_PFS->PORT[12].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[15].PmnPFS = 0x03010000;;
    R_PFS->PORT[13].PIN[8].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[8].PmnPFS = 0x03010000;;
    R_PFS->PORT[13].PIN[9].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[9].PmnPFS = 0x03010000;;
    R_PFS->PORT[14].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[14].PmnPFS = 0x03010000;;
    R_PFS->PORT[14].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[15].PmnPFS = 0x03010000;;
    R_BSP_PinAccessDisable();

    PWM_OUTPUT_ENABLE;

    gf_pwm_half = 1;

    R_GPT0->GTSTR = 0x10F;
}

void pwm_stop(void)
{
    R_GPT0->GTSTP = 0x010f;
    PWM_OUTPUT_DISABLE;
    R_GPT0->GTSTR = 0x010f;

    R_BSP_PinAccessEnable();
    R_PFS->PORT[11].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[14].PmnPFS = 0x00000004;
    R_PFS->PORT[11].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[11].PIN[15].PmnPFS = 0x00000004;
    R_PFS->PORT[12].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[14].PmnPFS = 0x00000004;
    R_PFS->PORT[12].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[12].PIN[15].PmnPFS = 0x00000004;
    R_PFS->PORT[13].PIN[8].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[8].PmnPFS = 0x00000004;
    R_PFS->PORT[13].PIN[9].PmnPFS_b.PMR = 0;
    R_PFS->PORT[13].PIN[9].PmnPFS = 0x00000004;
    R_PFS->PORT[14].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[14].PmnPFS = 0x00000004;
    R_PFS->PORT[14].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[14].PIN[15].PmnPFS = 0x00000004;
    R_BSP_PinAccessDisable();
}

void dab_stop(void)
{
    R_GPT0->GTSTP = 0x010f;
    PWM_OUTPUT_DISABLE;
    DRV_SD_OUTPUT = 1;
    R_GPT0->GTSTR = 0x010f;
}

void dab_routine(void)
{
    switch (con.cmd)
    {
        case CMD_ON:
            if (g_dab_sys.state == STANDBY_STATE)
            {
                dab_start();
                g_dab_sys.state = DAB_RUN_STATE;
            }
            break;
        case CMD_OFF:
            if (g_dab_sys.state != FAULT_STATE)
            {
                dab_stop();
                g_dab_sys.state = STANDBY_STATE;
            }
            break;
        case CMD_ERR_RST:
            if (g_dab_sys.state == FAULT_STATE)
            {
                dab_err_rst();
                if (g_dab_err.byte == 0)
                {
                    g_dab_sys.state = STANDBY_STATE;
                }
            }
            break;
        default:
            break;
    }
    con.cmd = CMD_NONE;

    switch (g_key_signal)
    {
        case KEY_ON:
            if (g_dab_sys.state == STANDBY_STATE)
            {
                dab_start();
                g_dab_sys.state = DAB_RUN_STATE;
            }
            else if (g_dab_sys.state != FAULT_STATE)
            {
                dab_stop();
                g_dab_sys.state = STANDBY_STATE;
            }
            break;
        case KEY_ERR_RST:
            if (g_dab_sys.state == FAULT_STATE)
            {
                dab_err_rst();
                if (g_dab_err.byte == 0)
                {
                    g_dab_sys.state = STANDBY_STATE;
                }
            }
            break;
        default:
            break;
    }
    g_key_signal = 0;
}
