/*
 * can_fd.h
 *
 *  Created on: 2023年2月7日
 *      Author: a5073522
 */

#ifndef CAN_FD_H_
#define CAN_FD_H_

#define CAN_SPEED                       500                //kbps

#define CAN_MAILBOX_NUMBER_0            (0U)               //mail box number
#define CAN_CLASSIC_FRAME_DATA_BYTES    (8U)               //Data Length code for classic frame
#define CAN_FD_DATA_LENGTH_CODE         (64)               //Data Length code for FD frame
#define CAN_ID                          (0x1100)           //ID to be updated in transmit frame
/* Acceptance filter array parameters */
#define CANFD_FILTER_ID                 (0x00001000)
#define MASK_ID                         (0x10000000)
#define MASK_ID_MODE                    (0)

#define ZERO                            (0U)               //Array Index value
#define NULL_CHAR                       ('\0')

typedef enum e_can_id
{
    CAN_ID_CONNECT = 0x01,
    CAN_ID_DISCONNECT = 0x02,
	CAN_ID_RUN = 0x03,
	CAN_ID_STOP = 0x04,
	CAN_ID_ERR_RST = 0x06,
    CAN_ID_SPS_EN = 0x0D,
    CAN_ID_SPS_DIS = 0x0E,

	CAN_ID_READ1_1 = 0x10,
	CAN_ID_WRITE1_1 = 0x11,
	CAN_ID_READ1_2 = 0x12,
	CAN_ID_WRITE1_2 = 0x13,

	CAN_ID_READ2_1 = 0x20,
	CAN_ID_WRITE2_1 = 0x21,
	CAN_ID_READ2_2 = 0x22,
	CAN_ID_WRITE2_2 = 0x23,
    CAN_ID_READ2_3 = 0x24,
    CAN_ID_WRITE2_3 = 0x25,
    CAN_ID_READ2_4 = 0x26,
    CAN_ID_WRITE2_4 = 0x27,

    CAN_ID_MONITOR_1 = 0x30,
    CAN_ID_MONITOR_2 = 0x31,
    CAN_ID_MONITOR_3 = 0x32,
}t_can_id;

void R_CAN_init(void);
void canfd_operation(void);

extern bool b_canfd_tx_complete;
extern bool b_canfd_rx_complete;

#endif /* CAN_FD_H_ */
