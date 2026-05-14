#include"cmsis_os.h"
#include"motor.h"
#include<stdint.h>
#include"global_def.h"


/*==========Global var Define===========*/
static volatile uint8_t g_enable=0;

static Robot_Ctx g_robot;
static Pid_t    g_angle_pid;
/*==========Global var End==============*/

//TODO: config TIM9 in cubemx
//	also config GPIO for direction ctl
void Robot_Init(void)
{
	g_robot.state = IDLE;

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
   duty   0 (full reverse) → compare    0 → CHx=0%,  CHxN=100%
   duty   0 (stop)         → compare 50%  → CHx=50%, CHxN=50%, motor idle
   duty 100 (full forward) → compare 100% → CHx=100%,CHxN=0%
*/
static uint32_t duty_to_compare(int32_t duty, int32_t period)
{
	int32_t zero = period / 2;                        /* 50% = stop      */
	int32_t span = zero;                              /* ±50% range      */
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

	int32_t targetL = (int32_t)Pid_Update(&cL->pid, spL, measL, dt);
	int32_t targetR = (int32_t)Pid_Update(&cR->pid, spR, measR, dt);

	int32_t outL = clamp(targetL, cL->prev_out);
	int32_t outR = clamp(targetR, cR->prev_out);

	cL->prev_out = outL;
	cR->prev_out = outR;

	PWM_Write(&r->left,outL);
	PWM_Write(&r->right,outR);
}
/*==========Motor Driver End============*/






/*==========Task Begin==================*/
static void Motor_Task(void* arg){
	const float dt = 0.01f;
	static uint8_t cs100a_tick = 0;

	for (;;)
	    {
			Robot_State_t state = g_robot.state;
			SensorData_t data;
			Move_Cmd cmd = {0};
			float distance_mm;

			taskENTER_CRITICAL();
			data = g_sensor_data;
			taskEXIT_CRITICAL();

			/* meas from encoder: raw counts → speed (counts/s) */
			float measL = (float)data.enc.enc_L / dt;
			float measR = (float)data.enc.enc_R / dt;

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
				cmd.v = (int32_t)Pid_Update(&g_angle_pid, 0.0f, data.pitch, dt);
				break;
			case REMOTE:
				cmd.v    = (int32_t)Pid_Update(&g_angle_pid, 0.0f, data.pitch, dt)
				         + g_wifi_cmd.v;
				cmd.turn = g_wifi_cmd.turn;
				break;
			case TRACKING:
				cmd.v    = (int32_t)Pid_Update(&g_angle_pid, 1.0f, data.pitch, dt);
				cmd.turn = g_vision_cmd.turn;
				if (distance_mm > 0 && distance_mm < 200.0f) {
					cmd.v = 0;   /* obstacle brake: 20cm */
				}
				break;
			}

			/* translate to per-wheel speed and drive */
			float spL = (float)(cmd.v - cmd.turn);
			float spR = (float)(cmd.v + cmd.turn);
			Motor_Update(&g_robot, spL, measL, spR, measR, dt);

		    osDelay(10);
	    }
}
/*============Task End================*/


void MotorTask_Create(void)
{
    const osThreadAttr_t attr = {
        .name = "MotorTask",
        .priority = osPriorityNormal,
        .stack_size = 512
    };

    osThreadNew(Motor_Task, NULL, &attr);
}
