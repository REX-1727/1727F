/*
 * 1727B.c
 *
 *  Created on: Jan 29, 2016
 *      Author: Anton
 */
#include "1727F.h"


void flywheelInit(flywheel aFlywheel,float (*input)(),
		float (*target)(),
		float kP,
		float kI,
		float kD,
		int outputs[4])
{
	aFlywheel.parameters.input = input;
	aFlywheel.parameters.target = target;
	aFlywheel.parameters.timeOut = -1;
	aFlywheel.parameters.kP = kP;
	aFlywheel.parameters.kI = kI;
	aFlywheel.parameters.kD = kD;
}

void velocityReader(void *ignore)
{
	encoderReset(leftFlywheelEncoder);
	encoderReset(rightFlywheelEncoder);
	unsigned long startTime = 0;
	while(true)
	{
		startTime = millis();
		leftFlywheel.variables.velocityRaw = -encoderGet(leftFlywheelEncoder)*1000/20;
		rightFlywheel.variables.velocityRaw = -encoderGet(rightFlywheelEncoder)*1000/20;
		leftFlywheel.variables.velocity = (leftFlywheel.variables.velocityRaw*(FLYWHEEL_CIRCUMFERENCE/12)/360.0);
		rightFlywheel.variables.velocity = (rightFlywheel.variables.velocityRaw*(FLYWHEEL_CIRCUMFERENCE/12)/360.0);
		encoderReset(leftFlywheelEncoder);
		encoderReset(rightFlywheelEncoder);

		printf("%f\n\r",leftFlywheel.variables.velocity);
		taskDelayUntil(&startTime, MOTOR_REFRESH_TIME);
	}
}



void powerListener(void *params)
{

	if(true)
	{
		while(true)
		{
			if(main.rightDpad.axisValue == JOY_UP)
			{
				leftFlywheel.variables.power++;
				rightFlywheel.variables.power++;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_DOWN)
			{
				leftFlywheel.variables.power--;
				rightFlywheel.variables.power--;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_RIGHT)
			{
				leftFlywheel.variables.power += .5;
				rightFlywheel.variables.power += .5;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_LEFT)
			{
				leftFlywheel.variables.power -= .5;
				rightFlywheel.variables.power -= .5;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_UP)
			{
				leftFlywheel.variables.power =26.5;
				rightFlywheel.variables.power =26.5;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_RIGHT)
			{
				leftFlywheel.variables.power =22;
				rightFlywheel.variables.power =22;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_LEFT)
			{
				leftFlywheel.variables.power =20;
				rightFlywheel.variables.power =20;
				printf("debug");
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_DOWN)
			{
				leftFlywheel.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(200);
			}
			if(leftFlywheel.variables.power<0)
			{
				leftFlywheel.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(200);
			}

			rightFlywheel.variables.powerRaw = (rightFlywheel.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
			leftFlywheel.variables.powerRaw = (leftFlywheel.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

			lcdPrint(uart1,1,"%f",leftFlywheel.variables.power);
			//printf("%f /n /r", leftFlywheel.variables.power);
			taskDelay(20);

		}
	}
}

void driveControl(void *params)
{
	int rightBack;
	int leftBack;
	int rightFront;
	int leftFront;
	while(true)
	{
		rightBack = main.leftVertical.axisValue + main.leftHorizontal.axisValue - main.rightHorizontal.axisValue;
		leftBack = -main.leftVertical.axisValue + main.leftHorizontal.axisValue - main.rightHorizontal.axisValue;
		rightFront = main.leftVertical.axisValue - main.leftHorizontal.axisValue - main.rightHorizontal.axisValue;
		leftFront = main.leftVertical.axisValue + main.leftHorizontal.axisValue + main.rightHorizontal.axisValue;
		motorSet(RB, -rightBack);
		motorSet(LB, leftBack);
		motorSet(RF, -rightFront);
		motorSet(LF, leftFront);

		if(main.rightBumper.axisValue == JOY_UP)
		{
			motorSet(LOWER_INTAKE, -127);
		}
		else if(main.rightBumper.axisValue == JOY_DOWN)
		{
			motorSet(LOWER_INTAKE, 127);
		}
		else
		{
			motorSet(LOWER_INTAKE, 0);
		}
		if(main.leftBumper.axisValue == JOY_UP)
		{
			motorSet(UPPER_INTAKE, -127);
		}
		else if(main.leftBumper.axisValue == JOY_DOWN)
		{
			motorSet(UPPER_INTAKE, 127);
		}
		else
		{
			motorSet(UPPER_INTAKE, 0);
		}


		taskDelay(20);
	}
}


//PID HELPERS


float getRPower()
{
	return rightFlywheel.variables.powerRaw;
}


float gyroTarget = 0;

float getLPower()
{
	return (leftFlywheel.variables.powerRaw);
}

float getRVel()
{
	return (rightFlywheel.variables.velocityRaw);
}

float getLVel()
{
	return (leftFlywheel.variables.velocityRaw);
}

float getGyro()
{
	return (float)(gyroGet(gyro));
}

float getGyroTarget()
{
	return gyroTarget;
}

void setGyroTarget(float target)
{
	gyroTarget = target;
}

//Auton Functions

void driveStraight(unsigned long time, float equilibriumSpeed)
{
	float differential=0;
	float speed = equilibriumSpeed;
	float rightSpeed;
	float leftSpeed;

	int RB_count, RF_count, LB_count, LF_count;

	gyroReset(gyro);

	pidParams_raw differentialParams = {getGyro,getGyroTarget,-1,5,0,1,&differential};
	TaskHandle differentialPID = taskCreate(positionPIDControl_raw, TASK_DEFAULT_STACK_SIZE,&differentialParams , TASK_PRIORITY_DEFAULT);
	unsigned long startTime = millis();
	while(millis() - startTime < time)
	{
		rightSpeed = speed + differential*.5;
		leftSpeed = speed - differential*.5;

		//imeGet(RB,&RB_count);
		//imeGet(RF,&RF_count);
		//imeGet(LB,&LB_count);
		//imeGet(LF,&LF_count);


		motorSet(RB,-rightSpeed);
		motorSet(RF,-rightSpeed);
		motorSet(LB,-leftSpeed);
		motorSet(LF,leftSpeed);
		printf("%f     ",leftSpeed);
		printf("%f\r\n",rightSpeed);
		delay(20);
	}
	taskDelete(differentialPID);
}

void strafeStraight(unsigned long time, float equilibriumSpeed)
{
	float differential;
	float speed = equilibriumSpeed;
	float frontSpeed;
	float backSpeed;

	gyroReset(gyro);

	pidParams_raw differentialParams = {getGyro,getGyroTarget,-1,0,0,0,&differential};
	TaskHandle differentialPID = taskCreate(positionPIDControl_raw, TASK_DEFAULT_STACK_SIZE,&differentialParams , TASK_PRIORITY_DEFAULT);
	unsigned long startTime = millis();
	while(millis()-startTime <time)
	{
		frontSpeed = speed - differential;
		backSpeed = speed + differential;

		motorSet(RB,backSpeed);
		motorSet(RF,frontSpeed);
		motorSet(LB,-backSpeed);
		motorSet(LF,-frontSpeed);
		delay(20);
	}
	taskDelete(differentialPID);
}

void strafe(unsigned long time, int speed)
{
	unsigned long startTime = millis();
	while(millis()-startTime <time)
	{
		motorSet(RB,-speed);
		motorSet(RF,speed);
		motorSet(LB,speed);
		motorSet(LF,speed);
		delay(20);
	}
}
