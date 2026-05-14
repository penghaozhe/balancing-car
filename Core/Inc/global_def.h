/*
 * global_def.h
 *
 *  Created on: 2026年4月22日
 *      Author: 35973
 */

#ifndef INC_GLOBAL_DEF_H_
#define INC_GLOBAL_DEF_H_
#include"sensor.h"
#include"motor.h"

extern SensorData_t g_sensor_data;
extern volatile Wifi_Cmd   g_wifi_cmd;
extern volatile Move_Cmd   g_vision_cmd;

#endif /* INC_GLOBAL_DEF_H_ */
