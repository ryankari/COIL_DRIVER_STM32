


#include "motorFunctions.h"
int32_t slope;
uint32_t maxSpeedStatic;


int16_t randomArray[100] = { -15423, 17361, -25204, -3661, 24449, -26753, -10906, 32953, -11104,
		5111, -19970, 14897, 22759, -14485, 18826, -26582, -32605, 20775, 9896, -11489, 20874,
		29943, 21308, 9570, -29919, 1476, 6719, 18871, 87, 21602, 8407, -18974, -26717, -30910,
		30392, -21937, 106, 32602, -6135, 8376, 25504, -13392, 8874, 30590, 14378, 14239, -16838,
		-20838, 24332, -21600, -13654, 19299, 30325, 9222, 16012, 26348, -16449, 8713, -26207, 3964,
		12406, 1834, 14959, -13001, 5146, 516, -6739, -21283, -26393, -6210, -30158, 9273, 24086, 9701,
		-11402, -3592, 9079, 12713, -6874, 7464, -6514, 6795, 26721, 331, -375, 14634, -3239, 29733, -21558, -
		14904, -27320, 28855, 27608, -29791, 20746, 3782, -21487, -11583, 181, 21168 };

void turnMotorOn (void) {
	//extern int16_t pulseCount;
	extern TIM_HandleTypeDef htim2;
	if (STATE.VALUES.BITS.motorDirForward == 0) {
		ROTATE_FORWARD
	} else {
		ROTATE_REVERSE
	}
	STATE.VALUES.BITS.motorOn = 1;
	ENABLE_MOTOR
	 HAL_TIM_Base_Start_IT(&htim2);  // Enable the interrupt
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void setMotorDir(uint32_t motorDIR)
{
	extern STATE_typedef STATE;
	if (motorDIR == 1) {
		STATE.VALUES.BITS.motorDirForward = 0;
	} else {
		STATE.VALUES.BITS.motorDirForward = 1;
	}

}

void turnMotorOff (void) {

	extern TIM_HandleTypeDef htim2;
	STATE.VALUES.BITS.motorOn = 0;
	DISABLE_MOTOR
	HAL_GPIO_WritePin(MOS4_GPIO_Port, MOS4_Pin, 1);
	HAL_TIM_Base_Stop_IT(&htim2);

}


void updateSpeedandDuty (uint32_t prescalar,uint32_t dutyCycle) {

	extern TIM_HandleTypeDef htim2;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	htim2.Init.Prescaler = prescalar;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) 	{ 		Error_Handler(); }
		//duty cycle expressed as 0 to 100%

	//if ((dutyCycle > 0) && (dutyCycle < 100)) {
	//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, htim2.Init.Period*(100-dutyCycle)/100);
	//}

}

void motorSequence (void) {

	extern uint32_t tim3Counts;
	tim3Counts = 0;
	turnMotorOn();

}


uint32_t executemotorSequence (void) {
	extern uint32_t tim3Counts;
	extern STATE_typedef STATE;
	extern TIM_HandleTypeDef htim2;
	uint32_t output;
	static uint32_t output_prev;
	if (tim3Counts <= CountUp) {
		output = MinSpeed + (int32_t) tim3Counts * Slope_Pos;
	}
	else if ( (tim3Counts > CountUp) & (tim3Counts < (CountUp+CountMax))) {
		output = MaxSpeed;
	}
	else if ((tim3Counts >= (CountUp+CountMax)) & (tim3Counts < (CountUp+CountMax+CountDown))) {
		output = MaxSpeed + (int32_t) (tim3Counts- (CountUp+CountMax)) * Slope_Neg;
	}
	else {
		turnMotorOff();
		STATE.VALUES.BITS.motorRamp = 0;
		output = MinSpeed;
	}

	if (output != output_prev) {
		htim2.Init.Prescaler = output;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK) 	{ 		Error_Handler(); }
	}
	output_prev = output;

	return(output);

}

void executemotorRampOn(int32_t maxSpeed) {
	extern uint32_t tim3Counts;
	tim3Counts = 0;
	maxSpeedStatic = maxSpeed;
	slope = (maxSpeed-MinSpeed)/CountUp;
	turnMotorOn();
}


void rampMotorOn (void) {


	extern uint32_t tim3Counts;
	extern STATE_typedef STATE;
	extern TIM_HandleTypeDef htim2;
	static uint32_t output;
	static uint32_t output_prev;
	if (tim3Counts <= CountUp) {
		output = MinSpeed + (int32_t) tim3Counts * slope;
	}
	if ( (tim3Counts > CountUp) & (tim3Counts < (CountUp+CountMax))) {
		output = maxSpeedStatic;
		STATE.VALUES.BITS.rampMotorOn = 0;
		STATE.VALUES.BITS.motorOn = 1;
	}
	if (output != output_prev) {
		htim2.Init.Prescaler = output;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK) 	{ 		Error_Handler(); }
	}
	output_prev = output;


}




uint16_t PIDLoop (int16_t pulseCount,int16_t target,uint16_t reset) {
	float Kp,Ki,Kd,error, proportional, derivative, PID_output;
	static float integral = 0;
	static float previous_error = 0;
#define slowest 450
#define fastest 75
	uint16_t output;
	// Define PID constants
	Kp = 0.0;
	Ki = 0.000001;
	Kd = 0.00001;

	// Variables for PID control
	if (reset == 1) {
		integral = 0;
		return(slowest);
	} else {


		// Compute error
		error = target - pulseCount;

		// Compute PID terms
		proportional = Kp * error;
		integral += Ki * error;
		derivative = Kd * (error - previous_error);
		previous_error = error;
		// Compute PID output
		PID_output = proportional + integral + derivative;
		if (PID_output < 0) {
			PID_output = PID_output*-1;
		}
		output = -PID_output + slowest;
		// Scale PID output to [100, 400]
		if (output > slowest) {
			output = slowest;
		} else if (output < fastest) {
			output = fastest;
		}
		//output = PID_output;
		return(output);
	}


}
