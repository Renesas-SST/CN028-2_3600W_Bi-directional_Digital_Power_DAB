/*
 * ra6t2.c
 *
 *  Created on: 2023年2月10日
 *      Author: a5073522
 */
#include "hal_data.h"
#include "ra6t2.h"
#include "dcdc_dab.h"
#include "console.h"
#include "can_fd.h"

void R_ELC_init(void);
void R_Timer_10kHz_init(void);
fsp_err_t R_ADC_B_Calibrate_user(void);
void R_ADC_init(void);
void R_POE_init(void);
void R_IRQ_init(void);
void R_GPT0_1_2_3_init(void);
void R_GPT8_init(void);
void R_CMP_DA_init(void);

fsp_err_t err = FSP_SUCCESS;
uint8_t gf_sci9_tx_complete = 1;
uint8_t gf_sci9_rx_complete = 0;
t_pwm_shift g_pwm_shift_ch0, g_pwm_shift_ch1, g_pwm_shift_ch2, g_pwm_shift_ch3;
uint8_t gf_pwm_half;//first half: 0; second half: 1;
t_pwm_value g_pwm_value;

void hardware_init(void)
{
    g_pwm_value.PWM_period = (uint32_t)(PWM_CLK * 1000 / (DAB_PWM_FRE * 2));
    g_pwm_value.PWM_deadtime = DAB_DEAD_T * PWM_CLK / 1000;
    g_pwm_value.PWM_over_period = g_pwm_value.PWM_period + g_pwm_value.PWM_deadtime;

    gf_pwm_half = 1;
    PWM_para_init(&g_pwm_shift_ch0, &g_pwm_shift_ch1, &g_pwm_shift_ch2, &g_pwm_shift_ch3, g_pwm_value);
    R_GPT0_1_2_3_init();
    R_GPT8_init();
    R_GPT2->GTIOR_b.PSYE = 1;   /* Set to enable the GTCPP02 pin output */\

    R_Timer_10kHz_init();
    R_ELC_init();
    R_ADC_init();
    R_CMP_DA_init();
    R_POE_init();
    R_IRQ_init();
    R_UART_init();
    R_CAN_init();
}

void startup(void)
{
    R_GPT0->GTSTR = 0x010F;
    g_agt1_10kHz.p_api->start(&g_agt1_10kHz_ctrl);
}

void R_ELC_init(void)
{
    R_ELC_Open(&g_elc_ctrl, &g_elc_cfg);
    R_ELC_LinkSet (&g_elc_ctrl, ELC_PERIPHERAL_ADC0, ELC_EVENT_AGT1_INT);
    R_ELC_Enable (&g_elc_ctrl);
}

void R_Timer_10kHz_init(void)
{
    g_agt1_10kHz.p_api->open(&g_agt1_10kHz_ctrl, &g_agt1_10kHz_cfg);
}

fsp_err_t R_ADC_B_Calibrate_user(void)
{
    /* Store and Clear ADC Start Trigger Enable */
    uint32_t adtrgenr = R_ADC_B->ADTRGENR;
    R_ADC_B->ADTRGENR = 0;

    /* Wait for converter to stop
     *  - See table 36.26 of the RA6T2 User Manual, R01UH0951EJ0100 */
    FSP_HARDWARE_REGISTER_WAIT(R_ADC_B->ADSR_b.ADACT0, 0)
    FSP_HARDWARE_REGISTER_WAIT(R_ADC_B->ADSR_b.ADACT1, 0)

    /* Clear the error status flags */
    R_ADC_B->ADERSCR     = R_ADC_B0_ADERSCR_ADERCLR0_Msk | R_ADC_B0_ADERSCR_ADERCLR1_Msk;
    R_ADC_B->ADOVFERSCR  = R_ADC_B0_ADOVFERSCR_ADOVFEC0_Msk | R_ADC_B0_ADOVFERSCR_ADOVFEC1_Msk;
    R_ADC_B->ADOVFCHSCR0 = R_ADC_B0_ADOVFCHSCR0_OVFCHCn_Msk;
    R_ADC_B->ADOVFEXSCR  = R_ADC_B0_ADOVFEXSCR_OVFEXC0_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC1_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC2_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC5_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC6_Msk | R_ADC_B0_ADOVFEXSCR_OVFEXC7_Msk |
                           R_ADC_B0_ADOVFEXSCR_OVFEXC8_Msk;

    R_ADC_B->ADCALSTR = 0x303;  /* start the internal circuit calibration & start the gain and offset calibration */

    /* Error Status Check */
    uint32_t read_err = R_ADC_B->ADERSCR ||
                        R_ADC_B->ADOVFERSR ||
                        R_ADC_B->ADOVFCHSR0 ||
                        R_ADC_B->ADOVFEXSR;

    /* Reset ADC Start Trigger Enable */
    R_ADC_B->ADTRGENR = adtrgenr;

    fsp_err_t fsp_err = (read_err ? FSP_ERR_INVALID_HW_CONDITION : FSP_SUCCESS);
    FSP_ERROR_LOG(fsp_err);

    return fsp_err;
}

void R_ADC_init(void)
{
    g_adc0.p_api->open(&g_adc0_ctrl, &g_adc0_cfg);
    g_adc0.p_api->scanCfg(&g_adc0_ctrl, &g_adc0_scan_cfg);
    err = R_ADC_B_Calibrate_user();
    if (FSP_SUCCESS != err)
    {
        APP_ERR_TRAP(err);
    }
    R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MICROSECONDS);
    /* A/D group scan priority control */
    R_ADC_B->ADGSPCR = 0x0007;

    R_ADC_B->ADTRGENR = 0x03;
}

void R_POE_init(void)
{
    g_poeg1.p_api->open(&g_poeg1_ctrl, &g_poeg1_cfg);
}

void R_UART_init(void)
{
    R_SCI_B_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    g_dtc0_sci9_txi_cfg.p_info->transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE;
#if 1   /* 115200bps */
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcse = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcs = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.bgdm = 1;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.cks = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brr = 64;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.mddr = (uint8_t) 256;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brme = false;
#endif
#if 0    /* 921600bps */
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcse = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.abcs = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.bgdm = 1;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.cks = 0;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brr = 6;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.mddr = (uint8_t) 220;
    g_uart9_cfg_extend.p_baud_setting->baudrate_bits_b.brme = true;
#endif
    R_SCI_B_UART_BaudSet (&g_uart9_ctrl, g_uart9_cfg_extend.p_baud_setting);
    R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, HEADER_SIZE);
}

void g_uart9_callback(uart_callback_args_t *p_args)
{
    if(NULL != p_args)
    {
        if (UART_EVENT_TX_COMPLETE  == p_args->event)
        {
            gf_sci9_tx_complete = 1;
        }
        if (UART_EVENT_RX_COMPLETE  == p_args->event)
        {
            gf_sci9_rx_complete = 1;
        }
    }
}

void R_IRQ_init(void)
{
    R_ICU_ExternalIrqOpen(&g_irq0_ctrl, &g_irq0_cfg);
    R_ICU_ExternalIrqEnable(&g_irq0_ctrl);
}

void R_GPT0_1_2_3_init(void)
{
    g_timer0.p_api->open(&g_timer0_ctrl, &g_timer0_cfg);
    /* Mode selection */
    R_GPT0->GTCR_b.MD = 1;// Saw-wave one-shot pulse mode
    /* Buffer operation */
    R_GPT0->GTBER_b.CCRSWT = 1;
    /* Start source selection */
    //R_GPT0->GTSSR_b.SSELCA = 1;
    //R_GPT0->GTSSR_b.CSTRT = 1;

    /* Compare value */
    R_GPT0->GTCCR[0] = g_pwm_shift_ch0.PWM_value[0];   //GTCCRA
    /* Buffer value */
    R_GPT0->GTCCR[2] = g_pwm_shift_ch0.PWM_value[0];   //GTCCRC
    R_GPT0->GTCCR[4] = g_pwm_shift_ch0.PWM_value[1];   //GTCCRD
    /* Compare value */
    R_GPT0->GTCCR[1] = g_pwm_shift_ch0.PWM_value[2];   //GTCCRB
    /* Buffer value */
    R_GPT0->GTCCR[3] = g_pwm_shift_ch0.PWM_value[2];   //GTCCRE
    R_GPT0->GTCCR[5] = g_pwm_shift_ch0.PWM_value[3];   //GTCCRF
    /* Period value */
    R_GPT0->GTPR = g_pwm_value.PWM_period;
    R_GPT0->GTPBR = g_pwm_value.PWM_period;
    R_GPT0->GTPDBR = g_pwm_value.PWM_period;


    g_timer1.p_api->open(&g_timer1_ctrl, &g_timer1_cfg);
    /* Mode selection */
    R_GPT1->GTCR_b.MD = 1;// Saw-wave one-shot pulse mode
    /* Buffer operation */
    R_GPT1->GTBER_b.CCRSWT = 1;

    /* Period value */
    R_GPT1->GTPR = g_pwm_value.PWM_period;
    R_GPT1->GTPBR = g_pwm_value.PWM_period;
    R_GPT1->GTPDBR = g_pwm_value.PWM_period;

    /* Compare value */
    R_GPT1->GTCCR[0] = g_pwm_shift_ch1.PWM_value[0];    //GTCCRA
    /* Buffer value */
    R_GPT1->GTCCR[2] = g_pwm_shift_ch1.PWM_value[0];    //GTCCRC
    R_GPT1->GTCCR[4] = g_pwm_shift_ch1.PWM_value[1];    //GTCCRD
    /* Compare value */
    R_GPT1->GTCCR[1] = g_pwm_shift_ch1.PWM_value[2];    //GTCCRB
    /* Buffer value */
    R_GPT1->GTCCR[3] = g_pwm_shift_ch1.PWM_value[2];    //GTCCRE
    R_GPT1->GTCCR[5] = g_pwm_shift_ch1.PWM_value[3];    //GTCCRF


    g_timer2.p_api->open(&g_timer2_ctrl, &g_timer2_cfg);
    /* Mode selection */
    R_GPT2->GTCR_b.MD = 1;// Saw-wave one-shot pulse mode
    /* Buffer operation */
    R_GPT2->GTBER_b.CCRSWT = 1;

    /* Period value */
    R_GPT2->GTPR = g_pwm_value.PWM_period;
    R_GPT2->GTPBR = g_pwm_value.PWM_period;
    R_GPT2->GTPDBR = g_pwm_value.PWM_period;

    /* Compare value */
    R_GPT2->GTCCR[0] = g_pwm_shift_ch2.PWM_value[0];    //GTCCRA
    /* Buffer value */
    R_GPT2->GTCCR[2] = g_pwm_shift_ch2.PWM_value[0];    //GTCCRC
    R_GPT2->GTCCR[4] = g_pwm_shift_ch2.PWM_value[1];    //GTCCRD
    /* Compare value */
    R_GPT2->GTCCR[1] = g_pwm_shift_ch2.PWM_value[2];    //GTCCRB
    /* Buffer value */
    R_GPT2->GTCCR[3] = g_pwm_shift_ch2.PWM_value[2];    //GTCCRE
    R_GPT2->GTCCR[5] = g_pwm_shift_ch2.PWM_value[3];    //GTCCRF


    g_timer3.p_api->open(&g_timer3_ctrl, &g_timer3_cfg);
    /* Mode selection */
    R_GPT3->GTCR_b.MD = 1;// Saw-wave one-shot pulse mode
    /* Buffer operation */
    R_GPT3->GTBER_b.CCRSWT = 1;

    /* Period value */
    R_GPT3->GTPR = g_pwm_value.PWM_period;
    R_GPT3->GTPBR = g_pwm_value.PWM_period;
    R_GPT3->GTPDBR = g_pwm_value.PWM_period;

    /* Compare value */
    R_GPT3->GTCCR[0] = g_pwm_shift_ch3.PWM_value[0];    //GTCCRA
    /* Buffer value */
    R_GPT3->GTCCR[2] = g_pwm_shift_ch3.PWM_value[0];    //GTCCRC
    R_GPT3->GTCCR[4] = g_pwm_shift_ch3.PWM_value[1];    //GTCCRD
    /* Compare value */
    R_GPT3->GTCCR[1] = g_pwm_shift_ch3.PWM_value[2];    //GTCCRB
    /* Buffer value */
    R_GPT3->GTCCR[3] = g_pwm_shift_ch3.PWM_value[2];    //GTCCRE
    R_GPT3->GTCCR[5] = g_pwm_shift_ch3.PWM_value[3];    //GTCCRF

    R_GPT0->GTIOR_b.GTIOA = 0x3;
    R_GPT0->GTIOR_b.GTIOB = 0x3;
    R_GPT1->GTIOR_b.GTIOA = 0x3;
    R_GPT1->GTIOR_b.GTIOB = 0x3;
    R_GPT2->GTIOR_b.GTIOA = 0x3;
    R_GPT2->GTIOR_b.GTIOB = 0x3;
    R_GPT3->GTIOR_b.GTIOA = 0x3;
    R_GPT3->GTIOR_b.GTIOB = 0x3;

    PWM_OUTPUT_DISABLE;
}

void R_GPT8_init(void)
{
    g_timer8.p_api->open(&g_timer8_ctrl, &g_timer8_cfg);
    /* Mode selection */
    R_GPT8->GTCR_b.MD = 0;      // Saw-wave PWM mode 1
    /* buffer operation */
    R_GPT8->GTBER = 0x00100001;

    uint32_t half_period;
    half_period = (g_pwm_value.PWM_period + 1) / 2;
    /* Compare value */
    R_GPT8->GTCCR[0] = 0x001;           //GTCCRA
    R_GPT8->GTCCR[1] = half_period;     //GTCCRB
    R_GPT8->GTCCR[2] = half_period;     //GTCCRC
    R_GPT8->GTCCR[3] = half_period;     //GTCCRE
    R_GPT8->GTCCR[4] = half_period;     //GTCCRD
    R_GPT8->GTCCR[5] = half_period;     //GTCCRF
    /* Period value */
    R_GPT8->GTPR = g_pwm_value.PWM_period;
    R_GPT8->GTPBR = g_pwm_value.PWM_period;
    R_GPT8->GTPDBR = g_pwm_value.PWM_period;

    /* Configure interrupts */
    R_BSP_IrqDisable(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A);
    R_BSP_IrqCfg(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A, 9, NULL);
    R_BSP_IrqEnable(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A);
}

/* CMP & DA*/
void R_CMP_DA_init(void)
{
    //DA2
    g_dac2.p_api->open(&g_dac2_ctrl, &g_dac2_cfg);
    g_dac2.p_api->write(&g_dac2_ctrl, 3664);//Setting of DADR0  * 0.01692 = 60V;3546-60;3664-62
    g_dac2.p_api->start(&g_dac2_ctrl);
    //CMP2 init
    g_comparator2.p_api->open(&g_comparator2_ctrl, &g_comparator2_cfg);
    g_comparator2.p_api->outputEnable(&g_comparator2_ctrl);

    g_dac0.p_api->open(&g_dac0_ctrl, &g_dac0_cfg);
    g_dac0.p_api->write(&g_dac0_ctrl, 0);
    g_dac0.p_api->start(&g_dac0_ctrl);
    g_dac1.p_api->open(&g_dac1_ctrl, &g_dac1_cfg);
    g_dac1.p_api->write(&g_dac1_ctrl, 0);
    g_dac1.p_api->start(&g_dac1_ctrl);
    g_dac3.p_api->open(&g_dac3_ctrl, &g_dac3_cfg);
    g_dac3.p_api->write(&g_dac3_ctrl, 0);
    g_dac3.p_api->start(&g_dac3_ctrl);
}

void gpt8_ccmpa_isr(void)
{
    R_BSP_IrqStatusClear(VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A);

    if (gf_pwm_half == 1)
    {
        /* AD START */
        R_ADC_B->ADSYSTR = 0x01;
        gf_pwm_half = 0;

        /* GPT0 buffer refresh */
        R_GPT0->GTCCR[2] = g_pwm_shift_ch0.PWM_value[6];     //GTCCRC
        R_GPT0->GTCCR[4] = g_pwm_shift_ch0.PWM_value[7];     //GTCCRD
        R_GPT0->GTCCR[3] = g_pwm_shift_ch0.PWM_value[6];     //GTCCRE
        R_GPT0->GTCCR[5] = g_pwm_shift_ch0.PWM_value[7];     //GTCCRF
        /* GPT1 buffer refresh */
        R_GPT1->GTCCR[2] = g_pwm_shift_ch1.PWM_value[6];     //GTCCRC
        R_GPT1->GTCCR[4] = g_pwm_shift_ch1.PWM_value[7];     //GTCCRD
        R_GPT1->GTCCR[3] = g_pwm_shift_ch1.PWM_value[6];     //GTCCRE
        R_GPT1->GTCCR[5] = g_pwm_shift_ch1.PWM_value[7];     //GTCCRF
        /* GPT2 buffer refresh */
        R_GPT2->GTCCR[2] = g_pwm_shift_ch2.PWM_value[6];     //GTCCRC
        R_GPT2->GTCCR[4] = g_pwm_shift_ch2.PWM_value[7];     //GTCCRD
        R_GPT2->GTCCR[3] = g_pwm_shift_ch2.PWM_value[6];     //GTCCRE
        R_GPT2->GTCCR[5] = g_pwm_shift_ch2.PWM_value[7];     //GTCCRF
        /* GPT3 buffer refresh */
        R_GPT3->GTCCR[2] = g_pwm_shift_ch3.PWM_value[6];     //GTCCRC
        R_GPT3->GTCCR[4] = g_pwm_shift_ch3.PWM_value[7];     //GTCCRD
        R_GPT3->GTCCR[3] = g_pwm_shift_ch3.PWM_value[6];     //GTCCRE
        R_GPT3->GTCCR[5] = g_pwm_shift_ch3.PWM_value[7];     //GTCCRF
    }
    else
    {
        gf_pwm_half = 1;

        /* GPT1 buffer refresh */
        R_GPT1->GTCCR[2] = g_pwm_shift_ch1.PWM_value[4];      //GTCCRC
        R_GPT1->GTCCR[4] = g_pwm_shift_ch1.PWM_value[5];      //GTCCRD
        R_GPT1->GTCCR[3] = g_pwm_shift_ch1.PWM_value[4];      //GTCCRE
        R_GPT1->GTCCR[5] = g_pwm_shift_ch1.PWM_value[5];      //GTCCRF
        /* GPT2 buffer refresh */
        R_GPT2->GTCCR[2] = g_pwm_shift_ch2.PWM_value[4];      //GTCCRC
        R_GPT2->GTCCR[4] = g_pwm_shift_ch2.PWM_value[5];      //GTCCRD
        R_GPT2->GTCCR[3] = g_pwm_shift_ch2.PWM_value[4];      //GTCCRE
        R_GPT2->GTCCR[5] = g_pwm_shift_ch2.PWM_value[5];      //GTCCRF
        /* GPT3 buffer refresh */
        R_GPT3->GTCCR[2] = g_pwm_shift_ch3.PWM_value[4];      //GTCCRC
        R_GPT3->GTCCR[4] = g_pwm_shift_ch3.PWM_value[5];      //GTCCRD
        R_GPT3->GTCCR[3] = g_pwm_shift_ch3.PWM_value[4];      //GTCCRE
        R_GPT3->GTCCR[5] = g_pwm_shift_ch3.PWM_value[5];      //GTCCRF
    }
}
