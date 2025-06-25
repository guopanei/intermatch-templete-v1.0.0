#include "xunxian.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
volatile int8_t openmv_turn_value = 0;     // �洢OpenMV��ת��ֵ (-100~100)
volatile uint8_t openmv_line_detected = 0; // OpenMV��⵽�ߵı�־
uint8_t openmv_rx_buffer[1] = {0};         // OpenMV���ڽ��ջ�����


//UART5�жϷ�����
void UART5__IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        openmv_rx_buffer[0] = USART_ReceiveData(UART5);
                     
        // ����OpenMV���� (0: δ��⵽��, 1-255: ��⵽��)
        if(openmv_rx_buffer[0] == 0) {
            openmv_line_detected = 0;
            openmv_turn_value = 0;
        } else {
            openmv_line_detected = 1;
            openmv_turn_value = (int8_t)(openmv_rx_buffer[0] - 128);
        }
        
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }
}


