/*
 * err_handler.h
 *
 *  Created on: Dec 4, 2023
 *      Author: a5073522
 */

#ifndef ERR_HANDLER_H_
#define ERR_HANDLER_H_

#define Ts                  0.0001 //the interval time of 10k timer

#define ERR_ISEC_MAX        90
#define ERR_ISEC_MIN        -90

#define ERR_IBUS_MAX        12
#define ERR_IBUS_MIN        -12

#define ERR_VBUS_Tri_H      420
#define ERR_VBUS_Rcy_H      385
#define ERR_VBUS_Tri_L      370
#define ERR_VBUS_Rcy_L      385

#define ERR_VSEC_Tri_H      62
#define ERR_VSEC_Rcy_H      42
#define ERR_VSEC_Tri_L      38
#define ERR_VSEC_Rcy_L      42

#define ERR_TEMP_Tri_H      70
#define ERR_TEMP_Rcy_H      55
#define ERR_PAC_MAX         4000
#define ERR_PAC_CNT         (3/Ts)  //3s

typedef struct
{
    union
    {
        uint16_t byte;
        struct
        {
            uint8_t I_POE    : 1;    //bit-0
            uint8_t Isec_OCP : 1;
            uint8_t Ibus_OCP : 1;
            uint8_t Vout_OVP : 1;
            uint8_t Vout_UVP : 1;
            uint8_t Vbus_OVP : 1;
            uint8_t Vbus_UVP : 1;
            uint8_t OTP      : 1;
            uint8_t OLP      : 1;
            uint8_t INNER_ERR: 1;
            uint8_t          : 6;
        }bit;
    };
}t_dab_err;

extern t_dab_err g_dab_err;

void dab_high_protection(void);
void dab_low_protection(void);
void dab_err_rst(void);


#endif /* ERR_HANDLER_H_ */
