#include"remote_ctrl.h"
#include"uart_drv.h"
#include"usart.h"
#include"global_def.h"

#define HEADER  0xAA
#define END  0x55
#define TYPE_MOVE 0x01
#define TYPE_MODE_SHIFT 0x02

static uint8_t     dma_buf[WIFI_BUF_SIZE];
static uint8_t     frame_buf[WIFI_BUF_SIZE];
static UartDma_Rx  g_wifi_uart;

static void wifi_on_frame(uint8_t *data, uint16_t len)
{
	/* MOVE: HEADER | TYPE | X | Y | CHK | END  (6 bytes) */
	/* MODE_SHIFT: HEADER | TYPE | MODE | CHK | END (5 bytes)*/
	if(len!=6 && len!=5)
		return;
	if (data[0] != HEADER || data[len - 1] != END)
		return;
	switch(data[1]){
		case TYPE_MOVE: {
			uint8_t chk = TYPE_MOVE ^ data[2] ^ data[3];
			if(data[4]!=chk)
				return;
			g_wifi_cmd.turn = (int32_t)data[2] * RC_SPEED_SCALE;
			g_wifi_cmd.v    = (int32_t)data[3] * RC_SPEED_SCALE;
			break;
		}
		case TYPE_MODE_SHIFT: {
			uint8_t chk = TYPE_MODE_SHIFT ^ data[2];
			if(data[3]!=chk)
				return;
			g_wifi_cmd.mode_request = (int32_t)data[2];
			break;
		}
	}

}

void Wifi_Init(void)
{
	/*
	 * Pull-up PC11 (GPIOC PUPDR[23:22] = 01).
	 * HAL_UART_MspInit already set MODER/OSPEEDR/AFR; only PUPDR needs
	 * changing.  Direct register write avoids re-initialising MODER on a
	 * live USART3 pin.
	 */
	GPIOC->PUPDR = (GPIOC->PUPDR & ~(3UL << 22)) | (1UL << 22);

	/* Boot sequence: IO(PC9) HIGH = normal run, RST(PC12) release to boot */
	HAL_GPIO_WritePin(ESP_IO_GPIO_Port, ESP_IO_Pin, GPIO_PIN_SET);              /* GPIO0 high */
	HAL_GPIO_WritePin(ESP8266_RST_GPIO_Port, ESP8266_RST_Pin, GPIO_PIN_RESET);  /* RST low  */
	HAL_Delay(100);
	HAL_GPIO_WritePin(ESP8266_RST_GPIO_Port, ESP8266_RST_Pin, GPIO_PIN_SET);    /* RST high */
	HAL_Delay(3000);   /* ESP8266 cold boot: ROM (74880 baud) + fw + WiFi + DHCP */

	/*
	 * Start DMA AFTER ESP is booted and quiet.
	 *
	 * ESP ROM bootloader outputs at 74880 baud; if DMA were running, USART3
	 * would misread this as 115200 -> massive FE/NE -> HAL error ISR kills
	 * DMA before IDLE can fire (HAL_UART_IRQHandler line 2376 vs 2482).
	 *
	 * UartDma_Init clears any stale FE/NE before enabling DMA (uart_drv.c).
	 */
	UartDma_Init(&g_wifi_uart, &huart3,
	             dma_buf, frame_buf, WIFI_BUF_SIZE,
	             wifi_on_frame);
}
