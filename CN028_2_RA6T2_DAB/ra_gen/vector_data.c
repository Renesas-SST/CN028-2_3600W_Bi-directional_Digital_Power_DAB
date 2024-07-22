/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = agt_int_isr, /* AGT1 INT (AGT interrupt) */
            [1] = adc_b_adi0_isr_user, /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = adc_b_adi1_isr_user, /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
            [3] = adc_b_fiforeq5678_isr, /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
            [4] = poeg_event_isr, /* POEG1 EVENT (Port Output disable interrupt B) */
            [5] = sci_b_uart_rxi_isr, /* SCI9 RXI (Received data full) */
            [6] = sci_b_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [7] = sci_b_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [8] = sci_b_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [9] = r_icu_isr, /* ICU IRQ0 (External pin interrupt 0) */
            [10] = canfd_error_isr, /* CAN0 CHERR (Channel error) */
            [11] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [12] = canfd_error_isr, /* CAN GLERR (Global error) */
            [13] = canfd_rx_fifo_isr, /* CAN RXF (Global recieve FIFO interrupt) */
            [14] = gpt8_ccmpa_isr, /* GPT8 CAPTURE COMPARE A (Compare match A) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_AGT1_INT), /* AGT1 INT (AGT interrupt) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI0), /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI1), /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_ADC12_FIFOREQ5678), /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_POEG1_EVENT), /* POEG1 EVENT (Port Output disable interrupt B) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI9_RXI), /* SCI9 RXI (Received data full) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI), /* SCI9 TXI (Transmit data empty) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI), /* SCI9 TEI (Transmit end) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_SCI9_ERI), /* SCI9 ERI (Receive error) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ0), /* ICU IRQ0 (External pin interrupt 0) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_CAN0_CHERR), /* CAN0 CHERR (Channel error) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_CAN0_TX), /* CAN0 TX (Transmit interrupt) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_CAN_GLERR), /* CAN GLERR (Global error) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_CAN_RXF), /* CAN RXF (Global recieve FIFO interrupt) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_GPT8_CAPTURE_COMPARE_A), /* GPT8 CAPTURE COMPARE A (Compare match A) */
        };
        #endif
