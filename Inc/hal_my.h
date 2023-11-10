#ifndef __HAL_MY_H
#define __HAL_MY_H

#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
void HAL_MY_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

//HAL_StatusTypeDef HAL_MY_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,uint16_t Size);


#endif







