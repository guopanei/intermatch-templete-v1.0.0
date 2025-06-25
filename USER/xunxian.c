#include "xunxian.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
volatile int8_t openmv_turn_value = 0;     // 存储OpenMV的转向值 (-100~100)
volatile uint8_t openmv_line_detected = 0; // OpenMV检测到线的标志
uint8_t openmv_rx_buffer[1] = {0};         // OpenMV串口接收缓冲区


//UART5中断服务函数
void UART5__IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        openmv_rx_buffer[0] = USART_ReceiveData(UART5);
                     
        // 解析OpenMV数据 (0: 未检测到线, 1-255: 检测到线)
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


