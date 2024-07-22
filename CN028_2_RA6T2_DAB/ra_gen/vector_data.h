/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (15)
#endif
/* ISR prototypes */
void agt_int_isr(void);
void adc_b_adi0_isr_user(void);
void adc_b_adi1_isr_user(void);
void adc_b_fiforeq5678_isr(void);
void poeg_event_isr(void);
void sci_b_uart_rxi_isr(void);
void sci_b_uart_txi_isr(void);
void sci_b_uart_tei_isr(void);
void sci_b_uart_eri_isr(void);
void r_icu_isr(void);
void canfd_error_isr(void);
void canfd_channel_tx_isr(void);
void canfd_rx_fifo_isr(void);
void gpt8_ccmpa_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_AGT1_INT ((IRQn_Type) 0) /* AGT1 INT (AGT interrupt) */
#define AGT1_INT_IRQn          ((IRQn_Type) 0) /* AGT1 INT (AGT interrupt) */
#define VECTOR_NUMBER_ADC12_ADI0 ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
#define ADC12_ADI0_IRQn          ((IRQn_Type) 1) /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
#define VECTOR_NUMBER_ADC12_ADI1 ((IRQn_Type) 2) /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
#define ADC12_ADI1_IRQn          ((IRQn_Type) 2) /* ADC0 ADI1 (End of A/D scanning operation(Gr.1)) */
#define VECTOR_NUMBER_ADC12_FIFOREQ5678 ((IRQn_Type) 3) /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
#define ADC12_FIFOREQ5678_IRQn          ((IRQn_Type) 3) /* ADC0 FIFOREQ5678 (FIFO data read request interrupt(Gr.5 to 8)) */
#define VECTOR_NUMBER_POEG1_EVENT ((IRQn_Type) 4) /* POEG1 EVENT (Port Output disable interrupt B) */
#define POEG1_EVENT_IRQn          ((IRQn_Type) 4) /* POEG1 EVENT (Port Output disable interrupt B) */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 5) /* SCI9 RXI (Received data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 5) /* SCI9 RXI (Received data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 6) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 6) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 7) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 7) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 8) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 8) /* SCI9 ERI (Receive error) */
#define VECTOR_NUMBER_ICU_IRQ0 ((IRQn_Type) 9) /* ICU IRQ0 (External pin interrupt 0) */
#define ICU_IRQ0_IRQn          ((IRQn_Type) 9) /* ICU IRQ0 (External pin interrupt 0) */
#define VECTOR_NUMBER_CAN0_CHERR ((IRQn_Type) 10) /* CAN0 CHERR (Channel error) */
#define CAN0_CHERR_IRQn          ((IRQn_Type) 10) /* CAN0 CHERR (Channel error) */
#define VECTOR_NUMBER_CAN0_TX ((IRQn_Type) 11) /* CAN0 TX (Transmit interrupt) */
#define CAN0_TX_IRQn          ((IRQn_Type) 11) /* CAN0 TX (Transmit interrupt) */
#define VECTOR_NUMBER_CAN_GLERR ((IRQn_Type) 12) /* CAN GLERR (Global error) */
#define CAN_GLERR_IRQn          ((IRQn_Type) 12) /* CAN GLERR (Global error) */
#define VECTOR_NUMBER_CAN_RXF ((IRQn_Type) 13) /* CAN RXF (Global recieve FIFO interrupt) */
#define CAN_RXF_IRQn          ((IRQn_Type) 13) /* CAN RXF (Global recieve FIFO interrupt) */
#define VECTOR_NUMBER_GPT8_CAPTURE_COMPARE_A ((IRQn_Type) 14) /* GPT8 CAPTURE COMPARE A (Compare match A) */
#define GPT8_CAPTURE_COMPARE_A_IRQn          ((IRQn_Type) 14) /* GPT8 CAPTURE COMPARE A (Compare match A) */
#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
