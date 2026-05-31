#include "remote_ctrl.h"
#include "uart_drv.h"
#include "usart.h"
#include "global_def.h"

#define HEADER     0xAA
#define END        0x55
#define TYPE_TRACK 0x01

static uint8_t     dma_buf[VISION_BUF_SIZE];
static uint8_t     frame_buf[VISION_BUF_SIZE];
static UartDma_Rx  g_vision_uart;

static void vision_on_frame(uint8_t *data, uint16_t len)
{
    /*
     * OpenMV track packet:
     * HEADER | TYPE | X | Y | CHK | END
     * 0xAA   | 0x01 | X | Y | CHK | 0x55
     *
     * X/Y are int8 command values.
     * CHK = TYPE ^ X ^ Y
     */
    for (uint16_t i = 0; i + 5 < len; i++)
    {
        if (data[i] != HEADER)
            continue;

        if (data[i + 1] != TYPE_TRACK)
            continue;

        if (data[i + 5] != END)
            continue;

        uint8_t chk = TYPE_TRACK ^ data[i + 2] ^ data[i + 3];
        if (data[i + 4] != chk)
            continue;

        int8_t x = (int8_t)data[i + 2];

        g_vision_cmd.turn = (int32_t)x * RC_SPEED_SCALE;
        return;
    }
}

void Vision_Init(void)
{
    UartDma_Init(&g_vision_uart, &huart6,
                 dma_buf, frame_buf, VISION_BUF_SIZE,
                 vision_on_frame);
}