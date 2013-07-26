/*
 * RadioControl.c
 * ETバトルプログラム
 */

#include "ETBattle.h"

/*
 *	各種定義
*/

/*
 *	グローバル変数
 */

/* バランスコントロールへ渡すコマンド用変数 */
S8  cmd_forward, cmd_turn;
/* バランスコントロールから返されるモータ制御用変数 */
S8	pwm_l, pwm_r;

static U32	gyro_offset = 0;    /* gyro sensor offset value */

/*
 *	各種状態定義
 */

//システム全体の状態
typedef enum{
	RN_MODE_INIT, 					//初期状態
	RN_MODE_BALANCE,				//倒立制御ON
	RN_MODE_BALANCEOFF				//倒立制御OF
} RN_MODE;


//キャリブレーションの状態
typedef enum{
	RN_START,					//開始待ちモード
	RN_RUN,						//対戦モード
	RN_BOOST,					//ターボ
	RN_PUSHBUTTON				//ボタン押下モード
} RN_SETTINGMODE;


typedef enum{
	BC_ONE,
	BC_TWO,
	BC_THREE,
	BC_FOUR,
} BC_MODE;

//初期状態
RN_MODE runner_mode = RN_MODE_INIT;
RN_SETTINGMODE setting_mode = RN_START;
BC_MODE boostCheckMode = BC_ONE;

/*
 *	各種プライベート関数定義
 */

//各種プライベート関数
void RN_calibrate();
void RN_setting();
void RN_modesetting();
void runner_mode_change(int flag);
int boost();

#define BT_RCV_BUF_SIZE (32) /* it must be 32bytes with NXT GamePad */
static U8 bt_receive_buf[BT_RCV_BUF_SIZE]; /* Bluetooth receive buffer(32bytes) */

//カウンタの宣言
DeclareCounter(SysTimerCnt);

//液晶ディスプレイに表示するシステム名設定
const char target_subsystem_name[] = "ETBattle";

/*
 *	関数実装
 */


//初期処理関数（プログラムの最初に呼び出し）
void ecrobot_device_initialize(void)
{
	ecrobot_set_light_sensor_active(NXT_PORT_S3);	//光センサ起動
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
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3);	//光センサ終了
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

//走行状態設定関数
void RN_setting()
{

	static int wait_count = 0;

	switch (setting_mode){

			//キャリブレーション状態
		case (RN_START):
			RN_calibrate();
			break;
		
			//対戦中
		case (RN_RUN):
			(void)ecrobot_read_bt_packet(bt_receive_buf, BT_RCV_BUF_SIZE);	/* スティック入力 */
			
			cmd_forward = -((S8)bt_receive_buf[0])/2;	/* 前進量(そのままでは早すぎるので値を半分） */
			cmd_turn = ((S8)bt_receive_buf[1]);			/* 旋回量 */
			
			//ターボチェック
			if(boost() == 1)
			{
				setting_mode = RN_BOOST;
				ecrobot_sound_tone(980,512,100);
			}

			else /* ターボ無し、スティック操作 */
			{
				nxt_motor_set_speed(NXT_PORT_C, cmd_forward + cmd_turn/2, 1);
				nxt_motor_set_speed(NXT_PORT_B, cmd_forward - cmd_turn/2, 1);
			}

			if(ecrobot_get_touch_sensor(NXT_PORT_S4) == 1)	/* ヒットチェック */
			{
				ecrobot_sound_tone(980,512,100);
				nxt_motor_set_speed(NXT_PORT_C,100,1);
				nxt_motor_set_speed(NXT_PORT_B,-100,1);
				wait_count = 0;
				setting_mode = RN_PUSHBUTTON;
			}
			
			break;

			//ターボ動作(ver 2.0新機能）
		case (RN_BOOST):
			wait_count++;

			/* 両車輪にモータ操作量の最大値を送信（スティック操作不可） */
			nxt_motor_set_speed(NXT_PORT_C,127,1);
			nxt_motor_set_speed(NXT_PORT_B,127,1);

			/* 1秒後に通常モードに復帰 */
			if(wait_count > 125)
			{
				setting_mode = RN_RUN;
				wait_count = 0;
			}
			break;

			//敗北動作
		case (RN_PUSHBUTTON):
			wait_count++;

			if(wait_count >= 150)
			{
				TailAngleChange(ANGLEOFZERO);
				nxt_motor_set_speed(NXT_PORT_C,0,1);
				nxt_motor_set_speed(NXT_PORT_B,0,1);
			}

			if(wait_count == 1000)
				setting_mode = RN_START;

			break;

		default:
			break;
	}
}


/*
 *	ターボ入力関数
 *	素早く左スティックを2回上に入力することでターボ（各入力の間隔は0.2秒以内）
 *	引数:無し、返り値:ターボフラグ
 */
int boost(){

	static int counter = 0;
	int boostflag = 0;

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
				boostflag = 1;
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

	return boostflag;	//ターボフラグ（0:無し、1:ターボON）
}

//キャリブレーション関数
void RN_calibrate()
{
	/*バランサーON用*/
	/*
	while(1){
		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == TRUE)
		{
			ecrobot_sound_tone(932, 512, 10);
			gyro_offset += (U32)ecrobot_get_gyro_sensor(NXT_PORT_S1);
			systick_wait_ms(500);
			break;
		}
	}
	*/
	while(1){
		if(ecrobot_get_touch_sensor(NXT_PORT_S4) == TRUE)
		{
			ecrobot_sound_tone(982,512,10);
			TailAngleChange(ANGLEOFDOWN);
			setting_mode = RN_RUN;
			runner_mode = RN_MODE_BALANCEOFF;
			systick_wait_ms(500);
			break;
		}
	}

}

//走行体状態設定関数
void RN_modesetting()
{
	switch (runner_mode){
			//走行体初期状態
		case (RN_MODE_INIT):
			cmd_forward = 0;
			cmd_turn = 0;
			break;

			//バランサー
		case (RN_MODE_BALANCE):
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
			break;

			//バランサー無し
		case (RN_MODE_BALANCEOFF):
			break;

		default:
			nxt_motor_set_speed(NXT_PORT_C, 0, 1);
			nxt_motor_set_speed(NXT_PORT_B, 0, 1);
			break;
	}
}


/*
 *	各種タスク
 */

//走行体管理タスク(4ms)
TASK(ActionTask)
{
	RN_modesetting();	//走行体状態設定
	TailControl();
	TerminateTask();
}

//走行状態管理タスク(5ms)
TASK(ActionTask2)
{
	RN_setting();		//走行状態設定
	TerminateTask();
}

//状態表示管理タスク(20ms)
TASK(DisplayTask)
{
	ecrobot_status_monitor(target_subsystem_name);	//モニタに状態表示
	TerminateTask();
}

/******************************** END OF FILE ********************************/
