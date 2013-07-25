#include "Balance_control.h"

//タスクの宣言
DeclareTask(ActionTask);

/* バランスコントロールへ渡すコマンド用変数 */
S8  cmd_forward=0, cmd_turn=0;
/* バランスコントロールから返されるモータ制御用変数 */
S8	pwm_l, pwm_r;

int flg=0;

static U32	gyro_offset = 614;    /* gyro sensor offset value */

typedef enum{
	RN_SET,
	RN_RUN
}RN_MODE;

typedef enum{
	BC_ONE,
	BC_TWO,
	BC_THREE,
	BC_FOUR,
} BC_MODE;

RN_MODE run_mode = RN_SET;
BC_MODE boostCheckMode = BC_ONE;


void RN_mode();
void boost();

#define BOOSTTIME 500	//[ms]
#define BT_RCV_BUF_SIZE (32) /* it must be 32bytes with NXT GamePad */
static U8 bt_receive_buf[BT_RCV_BUF_SIZE]; /* Bluetooth receive buffer(32bytes) */


//カウンタの宣言
DeclareCounter(SysTimerCnt);

//初期処理関数（プログラムの最初に呼び出し）
void ecrobot_device_initialize(void)
{
	ecrobot_init_bt_slave("LEJOS-OSEK");			//Bluetooth起動

	//モータリセット
	ecrobot_set_motor_rev(NXT_PORT_A,0);
	ecrobot_set_motor_rev(NXT_PORT_B,0);
	ecrobot_set_motor_rev(NXT_PORT_C,0);
	ecrobot_set_motor_speed(NXT_PORT_A,0);
	ecrobot_set_motor_speed(NXT_PORT_B,0);
	ecrobot_set_motor_speed(NXT_PORT_C,0);
}

//後始末処理関数（プログラム終了時呼び出し）
void ecrobot_device_terminate(void)
{
	ecrobot_term_bt_connection();					//Bluetooth終了

	//モータリセット
	ecrobot_set_motor_rev(NXT_PORT_A,0);
	ecrobot_set_motor_rev(NXT_PORT_B,0);
	ecrobot_set_motor_rev(NXT_PORT_C,0);
	ecrobot_set_motor_speed(NXT_PORT_A, 0);			
	ecrobot_set_motor_speed(NXT_PORT_B, 0);
	ecrobot_set_motor_speed(NXT_PORT_C, 0);
}

//OSEKフック関数
void StartupHook(void){}
void ShutdownHook(StatusType ercd){}
void PreTaskHook(void){}
void PostTaskHook(void){}
void ErrorHook(StatusType ercd){}


//フック関数
void user_1ms_isr_type2(void){
	StatusType ercd;
	ercd = SignalCounter( SysTimerCnt );
	if( ercd != E_OK ){
		ShutdownOS( ercd );
	}
}

TASK(ActionTask)
{

	RN_mode();

	boost();

	TerminateTask();
}


void RN_mode(){

	switch (run_mode){

	case (RN_SET):
		balance_init();	
		//モータリセット
		ecrobot_set_motor_rev(NXT_PORT_A,0);
		ecrobot_set_motor_rev(NXT_PORT_B,0);
		ecrobot_set_motor_rev(NXT_PORT_C,0);
		ecrobot_set_motor_speed(NXT_PORT_A,0);
		ecrobot_set_motor_speed(NXT_PORT_B,0);
		ecrobot_set_motor_speed(NXT_PORT_C,0);
		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == TRUE){
			run_mode=RN_RUN;
			systick_wait_ms(200);
		}
		break;

	case (RN_RUN):

		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == TRUE){
			run_mode=RN_SET;
			systick_wait_ms(200);
		}
		
		(void)ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);	/* スティック入力 */
			
		cmd_forward = -((S8)bt_receive_buf[0]);	/* 前進量 */
		cmd_turn = ((S8)bt_receive_buf[1]);			/* 旋回量 */
		//flg = ((int)bt_receive_buf[2]);	

		//if(flg =! 0)run_mode=RN_SET;

		balance_control(
			(F32)cmd_forward,
			(F32)cmd_turn,
			(F32)ecrobot_get_gyro_sensor(NXT_PORT_S1),
			(F32)gyro_offset,
			(F32)nxt_motor_get_count(NXT_PORT_C),
 			(F32)nxt_motor_get_count(NXT_PORT_B),
			(F32)ecrobot_get_battery_voltage(),
			&pwm_l,
			&pwm_r);
		nxt_motor_set_speed(NXT_PORT_C, pwm_l, 1);
		nxt_motor_set_speed(NXT_PORT_B, pwm_r, 1);


		ecrobot_debug1((int)gyro_offset,(int)ecrobot_get_gyro_sensor(NXT_PORT_S1),0);
		break;
	}
}

void boost(){

	static int counter = 0;

	switch(boostCheckMode){
		case (BC_ONE):	/* スティックが中央（1回目） */
			if(cmd_forward == 0)
			{
				boostCheckMode = BC_TWO;
				counter = 0;
			}
			break;
		case(BC_TWO):	/* スティックが最上（1回目） */
			if(cmd_forward == 50)
			{
				boostCheckMode = BC_THREE;
//				ecrobot_sound_tone(982,512,10);
				counter = 0;
			}
			else if(counter++ > BOOSTTIME/20)
				boostCheckMode = BC_ONE;
			break;
		case (BC_THREE):	/* スティックが中央（2回目） */
			if(cmd_forward == 0)
			{
				boostCheckMode = BC_FOUR;
//				ecrobot_sound_tone(982,512,10);
				counter = 0;
			}
			else if(counter++ > BOOSTTIME/20)
				boostCheckMode = BC_ONE;
			break;
		case (BC_FOUR):		/* スティックが最上（2回目）（成功でターボフラグON） */
			if(cmd_forward == 50)
			{
				boostCheckMode = BC_ONE;
//				ecrobot_sound_tone(982,512,10);
				counter = 0;
			}
			else if(counter++ > BOOSTTIME/20)
				boostCheckMode = BC_ONE;
			break;
		default:
			break;
	}

}