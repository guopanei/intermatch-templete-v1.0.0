/**
 * @file dma.h
 * @author TuxMonkey (nqx_2004@qq.com)
 * @brief DMA data reading
 * @version 0.1
 * @date 2025-05-17
 * 
 * Copyright (c) 2025 DLMU-CONE
 * 
 */
#ifndef __DMA_H
#define __DMA_H

#include "stm32f10x_dma.h"
#include "sys.h"
#include "system.h"

void MYDMA_Init2(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);
void MYDMA_Enable2(DMA_Channel_TypeDef*DMA_CHx);

void DMA_printf(const char *format,...);
u32 USART_SendBuffer(const char* buffer, u32 length);

#endif



