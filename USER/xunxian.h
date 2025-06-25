#ifndef __XUNXIAN_H
#define __XUNXIAN_H	
#include "sys.h"
#include "system.h"

extern volatile int8_t openmv_turn_value;
extern volatile uint8_t openmv_line_detected;
extern uint8_t openmv_rx_buffer[1];
void UART5__IRQHandler(void);


#endif 





