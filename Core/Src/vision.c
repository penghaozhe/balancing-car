#include"remote_ctrl.h"
#include"uart_drv.h"
#include"usart.h"
#include"global_def.h"

#define HEADER     0xAA
#define END        0x55
#define TYPE_TRACK 0x01

static uint8_t     dma_buf[VISION_BUF_SIZE];
static uint8_t     frame_buf[VISION_BUF_SIZE];
static UartDma_Rx  g_vision_uart;

static void vision_on_frame(uint8_t *data, uint16_t len)
{
	/* TRACK: HEADER | TYPE | X | Y | CHK | END  (6 bytes) */
	if (len != 6)
		return;
	if (data[0] != HEADER || data[5] != END)
		return;
	if (data[1] != TYPE_TRACK)
		return;

	uint8_t chk = TYPE_TRACK ^ data[2] ^ data[3];
	if (data[4] != chk)
		return;

	int8_t x = (int8_t)data[2];
	g_vision_cmd.turn = (int32_t)x * RC_SPEED_SCALE;
}

void Vision_Init(void)
{
	UartDma_Init(&g_vision_uart, &huart6,
	             dma_buf, frame_buf, VISION_BUF_SIZE,
	             vision_on_frame);
}
