/*
 * uart_drv.h
 *
 *  HAL-IDLE+DMA UART reception with per-module callback.
 *  Override __weak HAL_UARTEx_RxEventCallback → UartDma_OnRxEvent.
 */

#ifndef INC_UART_DRV_H_
#define INC_UART_DRV_H_

#include<stdint.h>
#include"main.h"

/* Called from ISR with a complete frame. Keep short. */
typedef void (*UartDma_Callback)(uint8_t *data, uint16_t len);

typedef struct UartDma_Rx {
	UART_HandleTypeDef *huart;
	uint8_t            *dma_buf;
	uint8_t            *frame_buf;
	uint16_t            buf_size;
	UartDma_Callback    cb;   /* module's parser, registered at init */
} UartDma_Rx;

void UartDma_Init(UartDma_Rx *u, UART_HandleTypeDef *huart,
                  uint8_t *dma_buf, uint8_t *frame_buf, uint16_t buf_size,
                  UartDma_Callback on_frame);


#endif /* INC_UART_DRV_H_ */
