/*
 * can_fd.c
 *
 *  Created on: 2023年2月1日
 *      Author: a5073522
 */

#include "hal_data.h"
#include "can_fd.h"
#include "r_canfd_cfg.h"
#include "ra6t2.h"
#include "console.h"
#include "err_handler.h"
#include "dcdc_dab.h"

void can_read1_1(void);
void can_read1_2(void);
void can_write1_1(uint8_t * can_rx);
void can_write1_2(uint8_t * can_rx);
void can_read2_1(void);
void can_read2_2(void);
void can_read2_3(void);
void can_read2_4(void);
void can_write2_1(uint8_t * can_rx);
void can_write2_2(uint8_t * can_rx);
void can_write2_3(uint8_t * can_rx);
void can_write2_4(uint8_t * can_rx);
void can_monitor_1(void);
void can_monitor_2(void);
void can_monitor_3(void);

/* Acceptance filter array parameters */
const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
 { /* Accept all messages with Extended ID 0x1000-0x1FFF */
   .id =
   {
    /* Specify the ID, ID type and frame type to accept. */
    .id         = CANFD_FILTER_ID,
    .frame_type = CAN_FRAME_TYPE_DATA,
    .id_mode    = CAN_ID_MODE_EXTENDED
   },

   .mask =
   {
    /* These values mask which ID/mode bits to compare when filtering messages. */
    .mask_id         = MASK_ID,
    .mask_frame_type = ZERO,
    .mask_id_mode    = MASK_ID_MODE
   },

   .destination =
   {
    /* If DLC checking is enabled any messages shorter than the below setting will be rejected. */
    .minimum_dlc = (canfd_minimum_dlc_t)ZERO,

    /* Optionally specify a Receive Message Buffer (RX MB) to store accepted frames. RX MBs do not have an
     * interrupt or overwrite protection and must be checked with R_CANFD_InfoGet and R_CANFD_Read. */
    .rx_buffer   = CANFD_RX_MB_0,

    /* Specify which FIFO(s) to send filtered messages to. Multiple FIFOs can be OR'd together. */
    .fifo_select_flags = CANFD_RX_FIFO_0,
   }
 },
};

can_frame_t g_canfd_tx_frame;
can_frame_t g_canfd_rx_frame;
bool b_canfd_tx_complete = true;
bool b_canfd_rx_complete = false;
bool b_canfd_err_status;

uint8_t f_read = 0;

void R_CAN_init(void)
{
#if CAN_SPEED == 500
    g_canfd0.p_cfg->p_bit_timing->baud_rate_prescaler = 4;
    g_canfd0.p_cfg->p_bit_timing->time_segment_1 = 14;
    g_canfd0.p_cfg->p_bit_timing->time_segment_2 = 5;
    g_canfd0.p_cfg->p_bit_timing->synchronization_jump_width = 5;
#elif CAN_SPEED == 250
    g_canfd0.p_cfg->p_bit_timing->baud_rate_prescaler = 4;
    g_canfd0.p_cfg->p_bit_timing->time_segment_1 = 29;
    g_canfd0.p_cfg->p_bit_timing->time_segment_2 = 10;
    g_canfd0.p_cfg->p_bit_timing->synchronization_jump_width = 5;
#endif
	err = R_CANFD_Open(&g_canfd0_ctrl, &g_canfd0_cfg);
	if (FSP_SUCCESS != err)
	{
		APP_ERR_TRAP(err);
	}

    g_canfd_tx_frame.id_mode = CAN_ID_MODE_STANDARD;
    g_canfd_tx_frame.type = CAN_FRAME_TYPE_DATA;
    g_canfd_tx_frame.data_length_code = CAN_CLASSIC_FRAME_DATA_BYTES;
    g_canfd_tx_frame.options = ZERO;    //CAN frame, not CAN-FD frame
}

void canfd0_callback(can_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
        {
            b_canfd_tx_complete = true;        //set flag bit
            break;
        }
        case CAN_EVENT_RX_COMPLETE: // Currently driver don't support this. This is unreachable code for now.
        {
            b_canfd_rx_complete = true;
            break;
        }
        case CAN_EVENT_ERR_WARNING:             //error warning event
        case CAN_EVENT_ERR_PASSIVE:             //error passive event
        case CAN_EVENT_ERR_BUS_OFF:             //error Bus Off event
        case CAN_EVENT_BUS_RECOVERY:            //Bus recovery error event
        case CAN_EVENT_MAILBOX_MESSAGE_LOST:    //overwrite/overrun error event
        case CAN_EVENT_ERR_BUS_LOCK:            // Bus lock detected (32 consecutive dominant bits).
        case CAN_EVENT_ERR_CHANNEL:             // Channel error has occurred.
        case CAN_EVENT_TX_ABORTED:              // Transmit abort event.
        case CAN_EVENT_ERR_GLOBAL:              // Global error has occurred.
        case CAN_EVENT_TX_FIFO_EMPTY:           // Transmit FIFO is empty.
        {
            break;
        }
    }
}

void canfd_operation(void)
{
	if (b_canfd_rx_complete)
	{
		b_canfd_rx_complete = 0;
        R_CANFD_Read(&g_canfd0_ctrl, 0, &g_canfd_rx_frame);

		if (con.cmd != CMD_NONE) return;
        switch (g_canfd_rx_frame.id)
        {
        	case CAN_ID_CONNECT:
        		con.f.can_connect = true;
        		f_read |= 0x3f;
        		break;
        	case CAN_ID_DISCONNECT:
        		con.f.can_connect = false;
        		break;
        	case CAN_ID_RUN:
        		con.cmd = CMD_ON;
        		break;
        	case CAN_ID_STOP:
        		con.cmd = CMD_OFF;
        		break;
        	case CAN_ID_ERR_RST:
        		con.cmd = CMD_ERR_RST;
        		break;
            case CAN_ID_SPS_EN:
                con.sps_en = true;
                break;
            case CAN_ID_SPS_DIS:
                con.sps_en = false;
                break;
            case CAN_ID_READ1_1:
                f_read |= 0x10;
                break;
            case CAN_ID_READ1_2:
                f_read |= 0x20;
                break;
            case CAN_ID_WRITE1_1:
                can_write1_1(g_canfd_rx_frame.data);
                f_read |= 0x10;
                break;
            case CAN_ID_WRITE1_2:
                can_write1_2(g_canfd_rx_frame.data);
                f_read |= 0x20;
                break;
            case CAN_ID_READ2_1:
                f_read |= 0x01;
                break;
            case CAN_ID_READ2_2:
                f_read |= 0x02;
                break;
            case CAN_ID_READ2_3:
                f_read |= 0x04;
                break;
            case CAN_ID_READ2_4:
                f_read |= 0x08;
                break;
            case CAN_ID_WRITE2_1:
                can_write2_1(g_canfd_rx_frame.data);
                f_read |= 0x01;
                break;
            case CAN_ID_WRITE2_2:
                can_write2_2(g_canfd_rx_frame.data);
                f_read |= 0x02;
                break;
            case CAN_ID_WRITE2_3:
                can_write2_3(g_canfd_rx_frame.data);
                f_read |= 0x04;
                break;
            case CAN_ID_WRITE2_4:
                can_write2_4(g_canfd_rx_frame.data);
                f_read |= 0x08;
                break;
        	default:
        		break;
        }
	}

	if (con.f.can_tx_en)
	{
		con.f.can_tx_en = false;
		if (b_canfd_tx_complete)
		{
			con.f.can_tx_num++;
			switch(con.f.can_tx_num)
			{
				case 1:
					if ((f_read & 0x10) == 0x10)
					{
						b_canfd_tx_complete = false;
						can_read1_1();
						f_read &= 0xEF;
					}
					break;
				case 2:
					if ((f_read & 0x20) == 0x20)
					{
						b_canfd_tx_complete = false;
						can_read1_2();
						f_read &= 0xDF;
					}
					break;
                case 3:
                    if ((f_read & 0x01) == 0x01)
                    {
                        b_canfd_tx_complete = false;
                        can_read2_1();
                        f_read &= 0xFE;
                    }
                    break;
                case 4:
                    if ((f_read & 0x02) == 0x02)
                    {
                        b_canfd_tx_complete = false;
                        can_read2_2();
                        f_read &= 0xFD;
                    }
                    break;
				case 5:
					if ((f_read & 0x04) == 0x04)
					{
						b_canfd_tx_complete = false;
						can_read2_3();
						f_read &= 0xFB;
					}
					break;
                case 6:
                    if ((f_read & 0x08) == 0x08)
                    {
                        b_canfd_tx_complete = false;
                        can_read2_4();
                        f_read &= 0xF7;
                    }
                    break;
				case 7:
					b_canfd_tx_complete = false;
					can_monitor_1();
					break;
				case 8:
					b_canfd_tx_complete = false;
					can_monitor_2();
					break;
				case 9:
                    b_canfd_tx_complete = false;
                    can_monitor_3();
                    con.f.can_tx_num = 0;
				    break;
				default:
					break;
			}
		}
	}
}

void can_read1_1(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ1_1;

    g_canfd_tx_frame.data[0] = con.sys_app;
    FloatTo2Byte(&g_canfd_tx_frame.data[1], g_dab_para.Vsec_tgt, 0, 50);
    FloatTo2Byte(&g_canfd_tx_frame.data[3], g_dab_para.Vbus_tgt, 0, 50);
    FloatTo2Byte(&g_canfd_tx_frame.data[5], g_dab_para.Isec_tgt, 328, 100);
    g_canfd_tx_frame.data[7] = con.oprt_mode;
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_read1_2(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ1_2;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], g_dab_para.Ibus_tgt, 328, 100);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_read2_1(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ2_1;

	FloatTo2Byte(&g_canfd_tx_frame.data[0], con.pi_iloop_kp, 0, 10000);
	FloatTo2Byte(&g_canfd_tx_frame.data[2], con.pi_iloop_ki, 0, 1000000);
	FloatTo2Byte(&g_canfd_tx_frame.data[4], con.pi_vloop_kp, 0, 100000);
	FloatTo2Byte(&g_canfd_tx_frame.data[6], con.pi_vloop_ki, 0, 1000000);
	R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_read2_2(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ2_2;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], con.pri_pi_iloop_kp, 0, 10000);
    FloatTo2Byte(&g_canfd_tx_frame.data[2], con.pri_pi_iloop_ki, 0, 1000000);
    FloatTo2Byte(&g_canfd_tx_frame.data[4], con.pri_pi_vloop_kp, 0, 100000);
    FloatTo2Byte(&g_canfd_tx_frame.data[6], con.pri_pi_vloop_ki, 0, 1000000);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_read2_3(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ2_3;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], con.Vsec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_canfd_tx_frame.data[2], con.Isec_ramp_step, 0, 1000);
    FloatTo2Byte(&g_canfd_tx_frame.data[4], con.Vpri_ramp_step, 0, 100);
    FloatTo2Byte(&g_canfd_tx_frame.data[6], con.Ipri_ramp_step, 0, 1000);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_read2_4(void)
{
    g_canfd_tx_frame.id = CAN_ID_READ2_4;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], con.sps, 328, 100);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_monitor_1(void)
{
    g_canfd_tx_frame.id = CAN_ID_MONITOR_1;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], g_dab_para.adc_Vbus_flt, 0, 50);
    FloatTo2Byte(&g_canfd_tx_frame.data[2], g_dab_para.adc_Ibus_flt, 328, 100);
    FloatTo2Byte(&g_canfd_tx_frame.data[4], g_dab_para.adc_Vsec_flt, 0, 50);
    FloatTo2Byte(&g_canfd_tx_frame.data[6], g_dab_para.adc_Isec_flt, 328, 100);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}
void can_monitor_2(void)
{
    g_canfd_tx_frame.id = CAN_ID_MONITOR_2;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], g_dab_para.Pdc, 32768, 1);
    FloatTo2Byte(&g_canfd_tx_frame.data[2], g_dab_para.adc_Temp, 0, 1);
    g_canfd_tx_frame.data[4] = g_dab_sys.state;
    FloatTo2Byte(&g_canfd_tx_frame.data[5], g_dab_err.byte, 0, 1);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}
void can_monitor_3(void)
{
    g_canfd_tx_frame.id = CAN_ID_MONITOR_3;

    FloatTo2Byte(&g_canfd_tx_frame.data[0], g_dab_para.adc_Temp_HV, 0, 1);
    R_CANFD_Write(&g_canfd0_ctrl, CAN_MAILBOX_NUMBER_0, &g_canfd_tx_frame);
}

void can_write1_1(uint8_t * can_rx)
{
    switch (can_rx[0])
    {
        case 0:
            con.sys_app = APP_P2S_TargetS;
            if (can_rx[7] == 0)
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
            if (can_rx[7] == 0)
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
            if (can_rx[7] == 1)
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
    tmp = TwoByte2Float(&can_rx[1], 0, 0.02);
    if (tmp < ERR_VSEC_Tri_H && tmp > ERR_VSEC_Tri_L)
        g_dab_para.Vsec_tgt = tmp;

    tmp = TwoByte2Float(&can_rx[3], 0, 0.02);
    if (tmp < ERR_VBUS_Tri_H && tmp > ERR_VBUS_Tri_L)
        g_dab_para.Vbus_tgt = tmp;

    tmp = TwoByte2Float(&can_rx[5], 328, 0.01);
    if (tmp < ERR_ISEC_MAX && tmp > ERR_ISEC_MIN)
        g_dab_para.Isec_tgt = tmp;
}

void can_write1_2(uint8_t * can_rx)
{
    float tmp;
    tmp = TwoByte2Float(&can_rx[0], 328, 0.01);
    if (tmp < ERR_IBUS_MAX && tmp > ERR_IBUS_MIN)
        g_dab_para.Ibus_tgt = tmp;
}

void can_write2_1(uint8_t * can_rx)
{
    con.pi_iloop_kp = TwoByte2Float(&can_rx[0], 0, 0.0001);
    con.pi_iloop_ki = TwoByte2Float(&can_rx[2], 0, 0.000001);
    con.pi_vloop_kp = TwoByte2Float(&can_rx[4], 0, 0.00001);
    con.pi_vloop_ki = TwoByte2Float(&can_rx[6], 0, 0.000001);
}

void can_write2_2(uint8_t * can_rx)
{
    con.pri_pi_iloop_kp = TwoByte2Float(&can_rx[0], 0, 0.0001);
    con.pri_pi_iloop_ki = TwoByte2Float(&can_rx[2], 0, 0.000001);
    con.pri_pi_vloop_kp = TwoByte2Float(&can_rx[4], 0, 0.00001);
    con.pri_pi_vloop_ki = TwoByte2Float(&can_rx[6], 0, 0.000001);
}

void can_write2_3(uint8_t * can_rx)
{
    con.Vsec_ramp_step = TwoByte2Float(&can_rx[0], 0, 0.001);
    con.Isec_ramp_step = TwoByte2Float(&can_rx[2], 0, 0.001);
    con.Vpri_ramp_step = TwoByte2Float(&can_rx[4], 0, 0.01);
    con.Ipri_ramp_step = TwoByte2Float(&can_rx[6], 0, 0.001);
}

void can_write2_4(uint8_t * can_rx)
{
    con.sps = TwoByte2Float(&can_rx[0], 328, 0.01);
}
