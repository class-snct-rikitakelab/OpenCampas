#include "TailControl.h"

//K”öŠp“x’²ßŠÖ”

void TailControl(){
	
	TargetTailAngleControl();

	static float t_hensa = 0;
	static float t_speed = 0;

	t_hensa = result_angle - ecrobot_get_motor_rev(NXT_PORT_A);		//K”ö‚ğ‰º‚°‚é

	t_speed = t_Kp*t_hensa;

	if (t_speed < -100)
		t_speed = -100;
	else if (t_speed > 100)
		t_speed = 100;

	ecrobot_set_motor_speed(NXT_PORT_A, t_speed);
}

void TargetTailAngleControl(){
	static int counter = 0;

	counter++;

	if(target_angle == ANGLEOFZERO)
		result_angle = ANGLEOFZERO;

	if(counter >= TAIL_ANGLE_COUNT)
	{
		if(target_angle - result_angle >= 0){
			result_angle++;

			if(result_angle > target_angle)
				result_angle = target_angle;
		}
		else{
			result_angle--;

			if(result_angle < target_angle)
				result_angle = target_angle;
		}
		counter = 0;
	}
}

void TailAngleChange(int angle){
	target_angle = angle;	
}

