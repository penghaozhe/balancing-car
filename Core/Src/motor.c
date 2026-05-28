#include"cmsis_os.h"
#include"motor.h"
#include<stdint.h>
#include<math.h>
#include"global_def.h"
#include"pidConfig.h"

#define DEADZONE_DUTY    5    /* min |duty|% to overcome static friction */
#define COAST_THRESHOLD  3    /* TEST: 0 = coast disabled */

/*==========Global var Define===========*/
static volatile uint8_t g_enable=0;

static Robot_Ctx g_robot;
static Pid_t    g_angle_pid;
/*==========Global var End==============*/


void Robot_Init(void)
{
	g_robot.state = BALANCE;

	g_robot.left.hw.htim_pwm   = &htim1;
	g_robot.left.hw.pwm_ch     = TIM_CHANNEL_1;
	g_robot.left.hw.pwm_period = PWM_PERIOD;
	Motor1Pid_Init(&g_robot.left.ctrl.pid);

	g_robot.right.hw.htim_pwm   = &htim1;
	g_robot.right.hw.pwm_ch     = TIM_CHANNEL_2;
	g_robot.right.hw.pwm_period = PWM_PERIOD;
	Motor2Pid_Init(&g_robot.right.ctrl.pid);

	AnglePid_Init(&g_angle_pid);
}



/*==========State Machine Begin=========*/
static void state_change(Robot_State_t next){
	if (g_robot.state == next) return;

	Pid_Reset(&g_robot.left.ctrl.pid);
	Pid_Reset(&g_robot.right.ctrl.pid);

	g_robot.state = next;
}
/*==========State Machine End===========*/




/*==========Motor Driver Begin==========*/

static inline int32_t clamp(int32_t target,int32_t prev){
		int32_t delta = target - prev;
		if (delta > DU_MAX)
				return prev + DU_MAX;
		if (delta < -DU_MAX)
				return prev - DU_MAX;
		return target;
}

/* Map duty (-100..+100) to locked-antiphase compare value:
    CHx  = duty%,  CHxN = 100% - duty%  (hardware automatic)
   duty  -100 (full reverse) → compare   0 → CHx=0%,   CHxN=100%
   duty     0 (stop)         → compare 50% → CHx=50%,  CHxN=50%
   duty  +100 (full forward) → compare 100% → CHx=100%, CHxN=0%
*/
static uint32_t duty_to_compare(int32_t duty, int32_t period)
{
	int32_t zero = period / 2;
	int32_t span = zero;
	return (uint32_t)(zero + (duty * span) / 100);
}

static void PWM_Write(const Motor_Unit *m, int32_t duty)
{
	uint32_t compare = duty_to_compare(duty, m->hw.pwm_period);
	__HAL_TIM_SET_COMPARE(m->hw.htim_pwm, m->hw.pwm_ch, compare);
}

static void Motor_Update(Robot_Ctx *r,
                  float spL, float measL,
                  float spR, float measR,
                  float dt)
{
	Motor_Ctrl *cL = &r->left.ctrl;
	Motor_Ctrl *cR = &r->right.ctrl;

	int32_t targetL = (int32_t)lroundf(Pid_Update(&cL->pid, spL, measL, dt));
	int32_t targetR = (int32_t)lroundf(Pid_Update(&cR->pid, spR, measR, dt));

	int32_t outL = clamp(targetL, cL->prev_out);
	int32_t outR = clamp(targetR, cR->prev_out);

	cL->prev_out = outL;
	cR->prev_out = outR;

	/*=== Coast: below threshold → cut MOE, motor floats ===*/
	static uint8_t coasting = 0;
	int thresh = coasting ? COAST_THRESHOLD + 2 : COAST_THRESHOLD;
	int should_coast = (outL > -thresh && outL < thresh)
	                && (outR > -thresh && outR < thresh);

	if (should_coast) {
		htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
		coasting = 1;
		return;
	}
	coasting = 0;

	/* restore MOE before driving */
	if (!(htim1.Instance->BDTR & TIM_BDTR_MOE)) {
		htim1.Instance->BDTR |= TIM_BDTR_MOE;
	}

	/* dead-zone compensation: bump sub-threshold duty to min start voltage */
	if (outL > 0 && outL < DEADZONE_DUTY) outL = DEADZONE_DUTY;
	if (outL < 0 && outL > -DEADZONE_DUTY) outL = -DEADZONE_DUTY;
	if (outR > 0 && outR < DEADZONE_DUTY) outR = DEADZONE_DUTY;
	if (outR < 0 && outR > -DEADZONE_DUTY) outR = -DEADZONE_DUTY;

	PWM_Write(&r->left,outL);
	PWM_Write(&r->right,outR);
}
/*==========Motor Driver End============*/






/*==========Task Begin==================*/
static void Motor_Task(void* arg){
	static uint32_t last_tick = 0;
	static uint8_t cs100a_tick = 0;
	static uint16_t prev_enc_L = 0, prev_enc_R = 0;



	for (;;)
	    {
			uint32_t now = osKernelGetTickCount();
			float dt = (float)(now - last_tick) * 0.001f;
			if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
			last_tick = now;

			Robot_State_t state = g_robot.state;
			SensorData_t data;
			Move_Cmd cmd = {0};
			float distance_mm;

			taskENTER_CRITICAL();
			data = g_sensor_data;
			taskEXIT_CRITICAL();

			/* MPU6050 + filter in thread context (I2C is blocking, not safe in ISR) */
			Sensor_Update(&data, dt);

			/* encoder delta with wrap handling (collected by TIM2 ISR) */
			int16_t dL = (int16_t)(data.enc.enc_L - prev_enc_L);
			int16_t dR = (int16_t)(data.enc.enc_R - prev_enc_R);
			float measL = -(float)dL / dt;
			float measR = (float)dR / dt;
			prev_enc_L = data.enc.enc_L;
			prev_enc_R = data.enc.enc_R;

			/* CS100A trigger: fire every 60ms */
			if (++cs100a_tick >= 6) {
				cs100a_tick = 0;
				CS100A_Trigger();
			}

			/* CS100A result: convert and consume */
			if (data.csa.echo_ready) {
				 distance_mm = CS100A_EchoWidth_to_mm(data.csa.echo_width);
			}

		/* Mode switch from wifi (payload: 'I','B','R','T') */
			if (g_wifi_cmd.mode_request) {
				Robot_State_t next = g_robot.state;
				switch (g_wifi_cmd.mode_request) {
				case 'I': next = IDLE;     break;
				case 'B': next = BALANCE;  break;
				case 'R': next = REMOTE;   break;
				case 'T': next = TRACKING; break;
				}
				state_change(next);
				g_wifi_cmd.mode_request = 0;
			}

			/* Per-mode command logic */
			switch(state){
			case IDLE:
				break;
			case BALANCE:
				cmd.v = -Pid_Update(&g_angle_pid, 0.0f, data.pitch, dt);
				break;
			case REMOTE:
				cmd.v    = Pid_Update(&g_angle_pid, 0.0f, data.pitch, dt)
				         + g_wifi_cmd.v;
				cmd.turn = g_wifi_cmd.turn;
				break;
			case TRACKING:
				cmd.v    = Pid_Update(&g_angle_pid, 1.0f, data.pitch, dt);
				cmd.turn = g_vision_cmd.turn;
				if (distance_mm > 0 && distance_mm < 200.0f) {
					cmd.v = 0;   /* obstacle brake: 20cm */
				}
				break;
			}

			/* translate to per-wheel speed and drive */
			float spL = (float)(cmd.v - cmd.turn)*SPEED_SP_GAIN;
			float spR = (float)(cmd.v + cmd.turn)*SPEED_SP_GAIN;

			Motor_Update(&g_robot, spL, measL, spR, measR, dt);

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		    osDelay(10);
	    }
}

//static void Motor_Task(void* arg){
//	//Test only dont touch it
//	PWM_Write(&g_robot.left,30);
//	PWM_Write(&g_robot.right,30);
//	osDelay(5000);
//}
/*============Task End================*/


void MotorTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name = "MotorTask",
        .priority = osPriorityNormal,
        .stack_size = 1024
    };

    osThreadNew(Motor_Task, NULL, &attr);
}
