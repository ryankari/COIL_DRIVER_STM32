#ifndef motorFunctions_h
#define motorFunctions_h


	#include "main.h"
	#include "stdlib.h"

	void turnMotorOn(void);
	void turnMotorOff(void);
	void setMotorDir(uint32_t);
	void updateSpeedandDuty(uint32_t,uint32_t);
	void motorSequence (void);
	uint32_t executemotorSequence (void);
	void executemotorRampOn(int32_t);
	void rampMotorOn(void);

	uint16_t PIDLoop (int16_t pulseCount,int16_t target,uint16_t reset);


	#define MaxSpeed 100
	#define MinSpeed 300

	#define CountUp 40 //Based on 10Hz 0.1sec count
	#define CountMax 50
	#define CountDown 40

	#define Slope_Pos (MaxSpeed-MinSpeed)/CountUp
	#define Slope_Neg (MinSpeed-MaxSpeed)/CountDown

	#define CMD0 0
	#define CMD1 1
	#define DataStartBit 1
	#define DataPrescalarBit 2
	#define MotorFunctionReg 1
	#define MotorDirBit 2
	#define MotorSpeedBit 3
	#define MotorDutyBit 4

	enum motorCMDarray {OFF,MOTOR_ON,RAMP_UP_DOWN, RAMP_UP};
   #define PWM_TIM_BASE_START  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { Error_Handler(); }
#define ENABLE_MOTOR HAL_GPIO_WritePin(MOS1_GPIO_Port, MOS1_Pin, 0); //Enable Motor
#define DISABLE_MOTOR 	HAL_GPIO_WritePin(MOS1_GPIO_Port, MOS1_Pin, 1); //Disable Motor
#define ROTATE_FORWARD HAL_GPIO_WritePin(MOS2_GPIO_Port, MOS2_Pin, 1);
#define ROTATE_REVERSE	HAL_GPIO_WritePin(MOS2_GPIO_Port, MOS2_Pin, 0);
#endif
