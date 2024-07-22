/*
 * err_handler.c
 *
 *  Created on: Dec 4, 2023
 *      Author: a5073522
 */
#include "hal_data.h"
#include "err_handler.h"
#include "ra6t2.h"
#include "dcdc_dab.h"

void R_POE_clear(void);

t_dab_err g_dab_err;

void dab_high_protection(void)
{
    /* over current */
    if (g_dab_para.adc_Isec > ERR_ISEC_MAX || g_dab_para.adc_Isec < ERR_ISEC_MIN)
    {
        g_dab_err.bit.Isec_OCP = 1;
    }
    if (g_dab_para.adc_Ibus > ERR_IBUS_MAX || g_dab_para.adc_Ibus < ERR_IBUS_MIN)
    {
        g_dab_err.bit.Ibus_OCP = 1;
    }
    /* over voltage */
    if (g_dab_para.adc_Vbus > ERR_VBUS_Tri_H)
    {
        g_dab_err.bit.Vbus_OVP = 1;
    }
    if (g_dab_para.adc_Vsec > ERR_VSEC_Tri_H)
    {
        g_dab_err.bit.INNER_ERR = 1;
    }

    if (g_dab_err.byte != 0)
    {
        if (g_dab_sys.state == DAB_RUN_STATE)
        {
            dab_stop();
        }
        g_dab_sys.state = FAULT_STATE;
    }
}

void dab_low_protection(void)
{
    static uint16_t count = 0;
    /* under voltage */
    if (g_dab_sys.app != APP_P2S_TargetS)
    {
        if (g_dab_para.adc_Vsec < ERR_VSEC_Tri_L)
        {
            //g_dab_err.bit.Vout_UVP = 1;
        }
    }
    else
    {
        if (g_dab_sys.state == DAB_RUN_STATE)
        {
            if (g_dab_para.adc_Vbus < ERR_VBUS_Tri_L)
            {
                //g_dab_err.bit.Vbus_UVP = 1;
            }
        }

    }
    /* over temperature */
    if (g_dab_para.adc_Temp > ERR_TEMP_Tri_H)
    {
        g_dab_err.bit.OTP = 1;
    }
    /* over load */
    if (g_dab_para.Pdc > ERR_PAC_MAX)
    {
        count++;
        if (count > ERR_PAC_CNT)
        {
            g_dab_err.bit.OLP = 1;
        }
    }
    else
    {
        count = 0;
    }

    if (g_dab_err.byte != 0)
    {
        if (g_dab_sys.state == DAB_RUN_STATE)
        {
            dab_stop();
        }
        g_dab_sys.state = FAULT_STATE;
    }
}

void g_poeg1_callback(poeg_callback_args_t *p_args)
{
    if (R_GPT_POEG1->POEGG_b.PIDF == 1)
    {
        g_dab_err.bit.I_POE = 1;
        NVIC->ICER[(((uint32_t) VECTOR_NUMBER_POEG1_EVENT) >> 5UL)] = (uint32_t) (1UL << (VECTOR_NUMBER_POEG1_EVENT & 0x1FUL));
    }
    if (R_GPT_POEG1->POEGG_b.IOCF == 1)
    {
        g_dab_err.bit.Vout_OVP = 1;
        NVIC->ICER[(((uint32_t) VECTOR_NUMBER_POEG1_EVENT) >> 5UL)] = (uint32_t) (1UL << (VECTOR_NUMBER_POEG1_EVENT & 0x1FUL));
    }
    UNUSED(p_args);
}

void R_POE_clear(void)
{
    if (R_GPT_POEG1->POEGG_b.PIDF == 1 && PORT_INPUT_POEB == 1)
    {
        R_GPT_POEG1->POEGG_b.PIDF = 0;
        g_dab_err.bit.I_POE = 0;
    }
    if (R_GPT_POEG1->POEGG_b.IOCF == 1 && R_ACMPHS2->CMPMON_b.CMPMON == 0)
    {
        R_GPT_POEG1->POEGG_b.IOCF = 0;
        g_dab_err.bit.Vout_OVP = 0;
    }
    NVIC->ISER[(((uint32_t) VECTOR_NUMBER_POEG1_EVENT) >> 5UL)] = (uint32_t) (1UL << (VECTOR_NUMBER_POEG1_EVENT & 0x1FUL));
}

void g_irq0_callback(external_irq_callback_args_t *p_args)
{
    UNUSED(p_args);
//    g_dab_err.bit.Vout_OVP = 1;
//    if (g_dab_sys.state == DAB_RUN_STATE)
//    {
//        dab_stop();
//    }
//    g_dab_sys.state = FAULT_STATE;
}

//void g_comparator2_callback(comparator_callback_args_t *p_args)
//{
//    g_dab_err.bit.Vout_OVP = 1;
//    if (g_dab_sys.state == DAB_RUN_STATE)
//    {
//        dab_stop();
//    }
//    g_dab_sys.state = FAULT_STATE;
//    UNUSED(p_args);
//}

void dab_err_rst(void)
{
    if (g_dab_para.adc_Isec <= ERR_ISEC_MAX && g_dab_para.adc_Isec >= ERR_ISEC_MIN)
    {
        g_dab_err.bit.Isec_OCP = 0;
    }
    if (g_dab_para.adc_Ibus <= ERR_IBUS_MAX && g_dab_para.adc_Ibus >= ERR_IBUS_MIN)
    {
        g_dab_err.bit.Ibus_OCP = 0;
    }
    if (g_dab_para.adc_Vbus < ERR_VBUS_Rcy_H)
    {
        g_dab_err.bit.Vbus_OVP = 0;
    }
    if (g_dab_para.adc_Vsec < ERR_VSEC_Rcy_H)
    {
        g_dab_err.bit.INNER_ERR = 0;
    }
    R_POE_clear();
//    if (PORT_INPUT_IRQ0 == 1)
//    {
//        g_dab_err.bit.Vout_OVP = 0;
//    }
    if (g_dab_sys.app != APP_P2S_TargetS)
    {
        if (g_dab_para.adc_Vsec >= ERR_VSEC_Rcy_L)
        {
            g_dab_err.bit.Vout_UVP = 0;
        }
        g_dab_err.bit.Vbus_UVP = 0;
    }
    else
    {
        g_dab_err.bit.Vout_UVP = 0;
        if (g_dab_para.adc_Vbus >= ERR_VBUS_Rcy_L)
        {
            g_dab_err.bit.Vbus_UVP = 0;
        }
    }
    if (g_dab_para.adc_Temp < ERR_TEMP_Rcy_H)
    {
        g_dab_err.bit.OTP = 0;
    }
    g_dab_err.bit.OLP = 0;
}


