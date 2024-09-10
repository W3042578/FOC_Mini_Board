
//Hal库底层配置修改，涉及库的细节修改单独另用自己的函数

#include "stm32f1xx_hal.h"
#include "main.h"
#include "hal_my.h"
#include "stm32f1xx_hal_spi.h"
/**
  * @brief  Handles DMA interrupt request.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.  
  * @retval None
  */
void HAL_MY_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
  uint32_t flag_it = hdma->DmaBaseAddress->ISR;
  uint32_t source_it = hdma->Instance->CCR;
  
  /* Half Transfer Complete Interrupt management ******************************/
  if (((flag_it & (DMA_FLAG_HT1 << hdma->ChannelIndex)) != RESET) && ((source_it & DMA_IT_HT) != RESET))
  {
    /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
    if((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
    {
      /* Disable the half transfer interrupt */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
    }
    /* Clear the half transfer complete flag */
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));

    /* DMA peripheral state is not updated in Half Transfer */
    /* but in Transfer Complete case */

    if(hdma->XferHalfCpltCallback != NULL)
    {
      /* Half transfer callback */
      hdma->XferHalfCpltCallback(hdma);
    }
  }

  /* Transfer Complete Interrupt management ***********************************/
  else if (((flag_it & (DMA_FLAG_TC1 << hdma->ChannelIndex)) != RESET) && ((source_it & DMA_IT_TC) != RESET))
  {
    if((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
    {
      /* Disable the transfer complete and error interrupt */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE | DMA_IT_TC);  

      /* Change the DMA state */
      hdma->State = HAL_DMA_STATE_READY;
    }
    /* Clear the transfer complete flag */
      __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);//拉高片选引脚信号，结束传输
		
    if(hdma->XferCpltCallback != NULL)
    {
      /* Transfer complete callback */
      hdma->XferCpltCallback(hdma);
    }
  }

  /* Transfer Error Interrupt management **************************************/
  else if (( RESET != (flag_it & (DMA_FLAG_TE1 << hdma->ChannelIndex))) && (RESET != (source_it & DMA_IT_TE)))
  {
    /* When a DMA transfer error occurs */
    /* A hardware clear of its EN bits is performed */
    /* Disable ALL DMA IT */
    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << hdma->ChannelIndex);

    /* Update error code */
    hdma->ErrorCode = HAL_DMA_ERROR_TE;

    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    if (hdma->XferErrorCallback != NULL)
    {
      /* Transfer error callback */
      hdma->XferErrorCallback(hdma);
    }
  }
  return;
}


/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
//因为回调函数为staic修饰，只能在本身文件生效，故更改HAL库细节只能放进对应文件,以下为复制内容，实际写在 stm32f1xx_hal_spi.c和对应.h文件中
//需添加main.h头文件确保引脚控制函数生效
//HAL_StatusTypeDef HAL_MY_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
//                                              uint16_t Size)
//{
//  uint32_t             tmp_mode;
//  HAL_SPI_StateTypeDef tmp_state;
//  HAL_StatusTypeDef errorcode = HAL_OK;

//  /* Check rx & tx dma handles */
//  assert_param(IS_SPI_DMA_HANDLE(hspi->hdmarx));
//  assert_param(IS_SPI_DMA_HANDLE(hspi->hdmatx));

//  /* Check Direction parameter */
//  assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

//  /* Process locked */
//  __HAL_LOCK(hspi);

//  /* Init temporary variables */
//  tmp_state           = hspi->State;
//  tmp_mode            = hspi->Init.Mode;

//  if (!((tmp_state == HAL_SPI_STATE_READY) ||
//        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
//  {
//    errorcode = HAL_BUSY;
//    goto error;
//  }

//  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
//  {
//    errorcode = HAL_ERROR;
//    goto error;
//  }

//  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
//  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
//  {
//    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
//  }

//  /* Set the transaction information */
//  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
//  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
//  hspi->TxXferSize  = Size;
//  hspi->TxXferCount = Size;
//  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
//  hspi->RxXferSize  = Size;
//  hspi->RxXferCount = Size;

//  /* Init field not used in handle to zero */
//  hspi->RxISR       = NULL;
//  hspi->TxISR       = NULL;

//#if (USE_SPI_CRC != 0U)
//  /* Reset CRC Calculation */
//  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
//  {
//    SPI_RESET_CRC(hspi);
//  }
//#endif /* USE_SPI_CRC */

//  /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
//  if (hspi->State == HAL_SPI_STATE_BUSY_RX)
//  {
//    /* Set the SPI Rx DMA Half transfer complete callback */
//    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt);
//    hspi->hdmarx->XferCpltCallback     = SPI_DMAReceiveCplt;
//  }
//  else
//  {
//    /* Set the SPI Tx/Rx DMA Half transfer complete callback */
//    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfTransmitReceiveCplt;
//    hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
//  }

//  /* Set the DMA error callback */
//  hspi->hdmarx->XferErrorCallback = SPI_DMAError;

//  /* Set the DMA AbortCpltCallback */
//  hspi->hdmarx->XferAbortCallback = NULL;

//  /* Enable the Rx DMA Stream/Channel  */
//  if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr,
//                                 hspi->RxXferCount))
//  {
//    /* Update SPI error code */
//    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
//    errorcode = HAL_ERROR;

//    hspi->State = HAL_SPI_STATE_READY;
//    goto error;
//  }

//  /* Enable Rx DMA Request */
//  SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

//  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
//  is performed in DMA reception complete callback  */
//  hspi->hdmatx->XferHalfCpltCallback = NULL;
//  hspi->hdmatx->XferCpltCallback     = NULL;
//  hspi->hdmatx->XferErrorCallback    = NULL;
//  hspi->hdmatx->XferAbortCallback    = NULL;

//  /* Enable the Tx DMA Stream/Channel  */
//  if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR,
//                                 hspi->TxXferCount))
//  {
//    /* Update SPI error code */
//    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
//    errorcode = HAL_ERROR;

//    hspi->State = HAL_SPI_STATE_READY;
//    goto error;
//  }

//  /* Check if the SPI is already enabled */
//  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
//  {
//    /* Enable SPI peripheral */
//    __HAL_SPI_ENABLE(hspi);
//  }
//  /* Enable the SPI Error Interrupt Bit */
//  __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));
//	
//	//开始DMA发送前数据时拉低NSS片选脚，尽可能减短发送拉低片选信号到实际发送数据的时间
//	//参考https://blog.csdn.net/chenyuanlidejiyi/article/details/121639160
//	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
//  /* Enable Tx DMA Request */
//  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

//error :
//  /* Process Unlocked */
//  __HAL_UNLOCK(hspi);
//  return errorcode;
//}
	

