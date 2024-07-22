/*
 * ra6t2.h
 *
 *  Created on: 2023年2月10日
 *      Author: a5073522
 */

#ifndef RA6T2_RA6T2_H_
#define RA6T2_RA6T2_H_

#include <dab_lib.h>

#define Delay_us(us)            R_BSP_SoftwareDelay(us,BSP_DELAY_UNITS_MICROSECONDS)
#define Delay_ms(ms)            R_BSP_SoftwareDelay(ms,BSP_DELAY_UNITS_MILLISECONDS)
#define Delay_s(s)              R_BSP_SoftwareDelay(s,BSP_DELAY_UNITS_SECONDS)

#define PWM_CLK                 200     //MHz
#define DAB_PWM_FRE             66      //kHz
#define DAB_DEAD_T              1000    //ns

#define APP_ERR_TRAP(err)       if(err) {__asm("BKPT #0\n");} /* trap upon the error  */
#define UNUSED(x)               (void)(x)

#define PWM_OUTPUT_ENABLE      {R_GPT0->GTIOR_b.OAE = 1;\
                                R_GPT0->GTIOR_b.OBE = 1;\
                                R_GPT1->GTIOR_b.OAE = 1;\
                                R_GPT1->GTIOR_b.OBE = 1;\
                                R_GPT2->GTIOR_b.OAE = 1;\
                                R_GPT2->GTIOR_b.OBE = 1;\
                                R_GPT3->GTIOR_b.OAE = 1;\
                                R_GPT3->GTIOR_b.OBE = 1;}
#define PWM_OUTPUT_DISABLE     {R_GPT0->GTIOR_b.OAE = 0;\
                                R_GPT0->GTIOR_b.OBE = 0;\
                                R_GPT1->GTIOR_b.OAE = 0;\
                                R_GPT1->GTIOR_b.OBE = 0;\
                                R_GPT2->GTIOR_b.OAE = 0;\
                                R_GPT2->GTIOR_b.OBE = 0;\
                                R_GPT3->GTIOR_b.OAE = 0;\
                                R_GPT3->GTIOR_b.OBE = 0;}
#define PWM_PIN_FUN_RESET      {R_GPT0->GTIOR_b.GTIOA = 0x3;\
                                R_GPT0->GTIOR_b.GTIOB = 0x3;\
                                R_GPT1->GTIOR_b.GTIOA = 0x3;\
                                R_GPT1->GTIOR_b.GTIOB = 0x3;\
                                R_GPT2->GTIOR_b.GTIOA = 0x3;\
                                R_GPT2->GTIOR_b.GTIOB = 0x3;\
                                R_GPT3->GTIOR_b.GTIOA = 0x3;\
                                R_GPT3->GTIOR_b.GTIOB = 0x3;}

#define PORT_INPUT_POEB         R_PORT10->PIDR_b.PIDR12     //PA12
#define PORT_INPUT_IRQ0         R_PORT0->PIDR_b.PIDR0       //P000
#define LED_RED                 R_PORT14->P0DR_b.PODR13     //PE13
#define LED_BLUE                R_PORT11->P0DR_b.PODR10     //PB10
#define DRV_SD_OUTPUT           R_PORT13->P0DR_b.PODR10     //PD10
#define KEY_ENB                 R_PORT13->PIDR_b.PIDR12     //PD12
#define KEY_RST                 R_PORT13->PIDR_b.PIDR13     //PD13
#define PORT_OUTPUT_FAN         R_PORT13->P0DR_b.PODR11     //PD11
#define PORT_OUTPUT_PD04        R_PORT13->P0DR_b.PODR4      //PD04
#define PORT_OUTPUT_PD03        R_PORT13->P0DR_b.PODR3      //PD03

typedef enum
{
    KEY_ON = 1,
    KEY_ERR_RST = 2,
}key;

void R_UART_init(void);
void hardware_init(void);
void startup(void);

extern t_pwm_value g_pwm_value;
extern fsp_err_t err;
extern t_pwm_shift g_pwm_shift_ch0, g_pwm_shift_ch1, g_pwm_shift_ch2, g_pwm_shift_ch3;
extern uint8_t gf_pwm_half;
extern uint8_t gf_sci9_tx_complete;
extern uint8_t gf_sci9_rx_complete;

#endif /* RA6T2_RA6T2_H_ */
