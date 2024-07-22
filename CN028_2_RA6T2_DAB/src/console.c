/*
 * console.c
 *
 *  Created on: 2022年8月24日
 *      Author: a5073522
 */

#include "hal_data.h"
#include "console.h"
#include "dcdc_dab.h"
#include "ra6t2.h"
#include "err_handler.h"

void serial_para_read1_refresh(void);
void serial_para_monitor_refresh(void);
_Bool r_serial_checksum(uint8_t * const buf, uint16_t num);
uint8_t r_serial_getsum(uint8_t * const buf, uint16_t num);
void serial_para_write1(void);
void serial_para_write2(void);

t_console con;
uint8_t rx_num;

uint8_t g_sci9_rx_buf[RX_BUF_SIZE];
uint8_t g_sci9_tx_OK[HEADER_SIZE] = {0x55, 0xF0, 0x00, 0x00, 0x45};
uint8_t g_sci9_tx_ERROR[HEADER_SIZE] = {0x55, 0xF1, 0x00, 0x00, 0x46};
uint8_t g_sci9_tx_Read1[READ1_NUM];
uint8_t g_sci9_tx_Read2[READ2_NUM];
uint8_t g_sci9_tx_Monitor[MONITOR_NUM];

void serial_para_init(void)
{
    con.cmd = CMD_NONE;
    rx_num = HEADER_SIZE;

    g_sci9_tx_Read1[HEAD] = 0x55;
    g_sci9_tx_Read1[CMD] = RET_OK;
    g_sci9_tx_Read1[LEN0] = READ1_LEN / 256;
    g_sci9_tx_Read1[LEN1] = READ1_LEN % 256;
    serial_para_read1_refresh();

    g_sci9_tx_Read2[HEAD] = 0x55;
    g_sci9_tx_Read2[CMD] = RET_OK;
    g_sci9_tx_Read2[LEN0] = READ2_LEN / 256;
    g_sci9_tx_Read2[LEN1] = READ2_LEN % 256;

    FloatTo2Byte(&g_sci9_tx_Read2[5], con.pi_iloop_kp, 0, 10000);
    FloatTo2Byte(&g_sci9_tx_Read2[7], con.pi_iloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[9], con.pi_vloop_kp, 0, 100000);
    FloatTo2Byte(&g_sci9_tx_Read2[11], con.pi_vloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[13], con.pri_pi_iloop_kp, 0, 10000);
    FloatTo2Byte(&g_sci9_tx_Read2[15], con.pri_pi_iloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[17], con.pri_pi_vloop_kp, 0, 100000);
    FloatTo2Byte(&g_sci9_tx_Read2[19], con.pri_pi_vloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[21], con.sps, 328, 100);
    FloatTo2Byte(&g_sci9_tx_Read2[23], con.Vsec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_sci9_tx_Read2[25], con.Isec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_sci9_tx_Read2[27], con.Vpri_ramp_step, 0, 100);
    FloatTo2Byte(&g_sci9_tx_Read2[29], con.Ipri_ramp_step, 0, 1000);

    g_sci9_tx_Read2[SUM] = r_serial_getsum(g_sci9_tx_Read2, READ2_NUM);

    g_sci9_tx_Monitor[HEAD] = 0x55;
    g_sci9_tx_Monitor[CMD] = RET_OK;
    g_sci9_tx_Monitor[LEN0] = MONITOR_LEN / 256;
    g_sci9_tx_Monitor[LEN1] = MONITOR_LEN % 256;
    serial_para_monitor_refresh();
}

/* amp: 1 / resolution */
void FloatTo2Byte(uint8_t * dest, float src, uint16_t offset, float amp)
{
    uint16_t value = (uint16_t)((src + offset) * amp);
    * dest = (uint8_t)(value / 256);
    dest++;
    * dest = (uint8_t)(value % 256);
}

float TwoByte2Float(uint8_t * src, uint16_t offset, float resolution)
{
    float value;
    value = (float)* src * 256;
    src++;
    value = (value + * src) * resolution - offset;
    return value;
}

uint8_t r_serial_getsum(uint8_t * const buf, uint16_t num)
{
    uint8_t result = 0;
    uint16_t i;
    for (i = 0; i < num; i++)
    {
        if (i != SUM)
        {
            result = (uint8_t)(result + buf[i]);
        }
    }
    return result;
}

_Bool r_serial_checksum(uint8_t * const buf, uint16_t num)
{
    _Bool result = 0;
    uint8_t sum;
    sum = r_serial_getsum(buf, num);
    if (buf[SUM] == sum)
    {
        result = 1;
    }
    return result;
}

void uart_operation(void)
{
    if (gf_sci9_rx_complete)
    {
    	con.f.uart_rx_timeout = 0;

        R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, rx_num);
        gf_sci9_rx_complete = 0;
        if (con.cmd != CMD_NONE) return;

        if (gf_sci9_tx_complete)
        {
            gf_sci9_tx_complete = 0;
            if (r_serial_checksum(g_sci9_rx_buf, rx_num))
            {
                switch (g_sci9_rx_buf[CMD])
                {
                    case CONNECT:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.f.uart_connect = true;
                        con.f.uart_rx_timeout = 0;
                        break;
                    case DISCONNECT:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.f.uart_connect = false;
                        break;
                    case RUN:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.cmd = CMD_ON;
                        break;
                    case STOP:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.cmd = CMD_OFF;
                        break;
                    case MONITOR:
                        serial_para_monitor_refresh();
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_Monitor, MONITOR_NUM);
                        break;
                    case READ1:
                        serial_para_read1_refresh();
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_Read1, READ1_NUM);
                        break;
                    case READ2:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_Read2, READ2_NUM);
                        break;
                    case WRITEA:
                        rx_num = HEADER_SIZE + 1;
                        R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, rx_num);
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        break;
                    case WRITEB:
                        rx_num = (uint8_t)(HEADER_SIZE + g_sci9_rx_buf[5]);
                        R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, rx_num);
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        break;
                    case WRITE1:
                        serial_para_write1();
                        rx_num = HEADER_SIZE;
                        R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, rx_num);
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        break;
                    case WRITE2:
                        serial_para_write2();
                        rx_num = HEADER_SIZE;
                        R_SCI_B_UART_Read(&g_uart9_ctrl, g_sci9_rx_buf, rx_num);
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.cmd = CMD_PARA_WRT;
                        break;
                    case ERR_RST:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.cmd = CMD_ERR_RST;
                        break;
                    case SPS_EN:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.sps_en = true;
                        break;
                    case SPS_DIS:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_OK, HEADER_SIZE);
                        con.sps_en = false;
                        break;
                    default:
                        R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_ERROR, HEADER_SIZE);
                        con.f.uart_rx_err = 1;
                        break;
                }
            }
            else
            {
                R_SCI_B_UART_Write(&g_uart9_ctrl, g_sci9_tx_ERROR, HEADER_SIZE);
                con.f.uart_rx_err = 1;
            }
        }
    }
    if ((gf_sci9_tx_complete && con.f.uart_rx_err) || con.f.uart_rx_timeout >= 6000)//600ms
    {
        R_SCI_B_UART_Close(&g_uart9_ctrl);
        R_UART_init();
        con.f.uart_rx_err = 0;
        con.f.uart_rx_timeout = 0;
    }
}

void serial_para_read1_refresh(void)
{
    g_sci9_tx_Read1[5] = con.sys_app;
    FloatTo2Byte(&g_sci9_tx_Read1[6], g_dab_para.Vsec_tgt, 0, 50);
    FloatTo2Byte(&g_sci9_tx_Read1[8], g_dab_para.Vbus_tgt, 0, 50);
    FloatTo2Byte(&g_sci9_tx_Read1[10], g_dab_para.Isec_tgt, 328, 100);
    FloatTo2Byte(&g_sci9_tx_Read1[12], g_dab_para.Ibus_tgt, 328, 100);
    g_sci9_tx_Read1[14] = con.oprt_mode;
    g_sci9_tx_Read1[SUM] = r_serial_getsum(g_sci9_tx_Read1, READ1_NUM);
}

void serial_para_write1(void)
{
    switch (g_sci9_rx_buf[5])
    {
        case 0:
            con.sys_app = APP_P2S_TargetS;
            if (g_sci9_rx_buf[14] == 0)
            {
                con.oprt_mode = SecCC;
            }
            else
            {
                con.oprt_mode = SecCV;
            }
            break;
        case 1:
            con.sys_app = APP_S2P_TargetS;
            if (g_sci9_rx_buf[14] == 0)
            {
                con.oprt_mode = SecCC;
            }
            else
            {
                con.oprt_mode = SecCV;
            }
            break;
        case 2:
            con.sys_app = APP_S2P_TargetP;
            if (g_sci9_rx_buf[14] == 1)
            {
                con.oprt_mode = PriCV;
            }
            else
            {
                con.oprt_mode = PriCC;
            }
            break;
        default:
            break;
    }

    //Limiting
    float tmp;
    tmp = TwoByte2Float(&g_sci9_rx_buf[6], 0, 0.02);
    if (tmp < ERR_VSEC_Tri_H && tmp > ERR_VSEC_Tri_L)
    	g_dab_para.Vsec_tgt = tmp;

    tmp = TwoByte2Float(&g_sci9_rx_buf[8], 0, 0.02);
    if (tmp < ERR_VBUS_Tri_H && tmp > ERR_VBUS_Tri_L)
    	g_dab_para.Vbus_tgt = tmp;

    tmp = TwoByte2Float(&g_sci9_rx_buf[10], 328, 0.01);
    if (tmp < ERR_ISEC_MAX && tmp > ERR_ISEC_MIN)
        g_dab_para.Isec_tgt = tmp;

    tmp = TwoByte2Float(&g_sci9_rx_buf[12], 328, 0.01);
    if (tmp < ERR_IBUS_MAX && tmp > ERR_IBUS_MIN)
        g_dab_para.Ibus_tgt = tmp;
}

void serial_para_write2(void)
{
    con.pi_iloop_kp = TwoByte2Float(&g_sci9_rx_buf[5], 0, 0.0001);
    con.pi_iloop_ki = TwoByte2Float(&g_sci9_rx_buf[7], 0, 0.000001);
    con.pi_vloop_kp = TwoByte2Float(&g_sci9_rx_buf[9], 0, 0.00001);
    con.pi_vloop_ki = TwoByte2Float(&g_sci9_rx_buf[11], 0, 0.000001);
    con.pri_pi_iloop_kp = TwoByte2Float(&g_sci9_rx_buf[13], 0, 0.0001);
    con.pri_pi_iloop_ki = TwoByte2Float(&g_sci9_rx_buf[15], 0, 0.000001);
    con.pri_pi_vloop_kp = TwoByte2Float(&g_sci9_rx_buf[17], 0, 0.00001);
    con.pri_pi_vloop_ki = TwoByte2Float(&g_sci9_rx_buf[19], 0, 0.000001);
    con.sps = TwoByte2Float(&g_sci9_rx_buf[21], 328, 0.01);
    con.Vsec_ramp_step = TwoByte2Float(&g_sci9_rx_buf[23], 0, 0.001);
    con.Isec_ramp_step = TwoByte2Float(&g_sci9_rx_buf[25], 0, 0.001);
    con.Vpri_ramp_step = TwoByte2Float(&g_sci9_rx_buf[27], 0, 0.01);
    con.Ipri_ramp_step = TwoByte2Float(&g_sci9_rx_buf[29], 0, 0.001);

    FloatTo2Byte(&g_sci9_tx_Read2[5], con.pi_iloop_kp, 0, 10000);
    FloatTo2Byte(&g_sci9_tx_Read2[7], con.pi_iloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[9], con.pi_vloop_kp, 0, 100000);
    FloatTo2Byte(&g_sci9_tx_Read2[11], con.pi_vloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[13], con.pri_pi_iloop_kp, 0, 10000);
    FloatTo2Byte(&g_sci9_tx_Read2[15], con.pri_pi_iloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[17], con.pri_pi_vloop_kp, 0, 100000);
    FloatTo2Byte(&g_sci9_tx_Read2[19], con.pri_pi_vloop_ki, 0, 1000000);
    FloatTo2Byte(&g_sci9_tx_Read2[21], con.sps, 328, 100);
    FloatTo2Byte(&g_sci9_tx_Read2[23], con.Vsec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_sci9_tx_Read2[25], con.Isec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_sci9_tx_Read2[27], con.Vpri_ramp_step, 0, 100);
    FloatTo2Byte(&g_sci9_tx_Read2[29], con.Ipri_ramp_step, 0, 1000);

    g_sci9_tx_Read2[SUM] = r_serial_getsum(g_sci9_tx_Read2, READ2_NUM);
}

void serial_para_monitor_refresh(void)
{
    FloatTo2Byte(&g_sci9_tx_Monitor[5], g_dab_para.adc_Vbus_flt, 0, 50);

    FloatTo2Byte(&g_sci9_tx_Monitor[7], g_dab_para.adc_Ibus_flt, 328, 100);

    FloatTo2Byte(&g_sci9_tx_Monitor[9], g_dab_para.adc_Vsec_flt, 0, 50);

    FloatTo2Byte(&g_sci9_tx_Monitor[11], g_dab_para.adc_Isec_flt, 328, 100);

    FloatTo2Byte(&g_sci9_tx_Monitor[13], g_dab_para.Pdc, 32768, 1);

    FloatTo2Byte(&g_sci9_tx_Monitor[15], g_dab_para.adc_Temp, 0, 1);

    g_sci9_tx_Monitor[17] = g_dab_sys.state;

    FloatTo2Byte(&g_sci9_tx_Monitor[18], g_dab_err.byte, 0, 1);

    FloatTo2Byte(&g_sci9_tx_Monitor[20], g_dab_para.adc_Temp_HV, 0, 1);

    g_sci9_tx_Monitor[SUM] = r_serial_getsum(g_sci9_tx_Monitor, MONITOR_NUM);
}

