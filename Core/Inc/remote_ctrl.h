/*
 * remote_ctrl.h
 *
 *  Application-layer UART receivers for OpenMV (vision) and ESP8266 (wifi).
 *  Each uses uart_drv + HAL_UARTEx_ReceiveToIdle_DMA underneath.
 */

#ifndef INC_REMOTE_CTRL_H_
#define INC_REMOTE_CTRL_H_

#include<stdint.h>
#include"motor.h"                      /* Move_Cmd */

/* ===== Common config ===== */
#define VISION_BUF_SIZE   256
#define WIFI_BUF_SIZE     512

/* int8 XY → speed scale: cmd = raw * SCALE  (e.g. ±127 → ±381) */
#define RC_SPEED_SCALE    3

/* ===== Public init ===== */

void Vision_Init(void);
void Wifi_Init(void);

#endif /* INC_REMOTE_CTRL_H_ */
