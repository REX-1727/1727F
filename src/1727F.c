/*
 * 1727B.c
 *
 *  Created on: Jan 29, 2016
 *      Author: Anton
 */
#include "1727F.h"


void flywheelInit(flywheel aFlywheel,int (*input)(),
		int (*target)(),
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
				taskDelay(100);
			}
			else if(main.rightDpad.axisValue == JOY_DOWN)
			{
				leftFlywheel.variables.power--;
				rightFlywheel.variables.power--;
				taskDelay(100);
			}
			else if(main.rightDpad.axisValue == JOY_RIGHT)
			{
				leftFlywheel.variables.power += .5;
				rightFlywheel.variables.power += .5;
				taskDelay(100);
			}
			else if(main.rightDpad.axisValue == JOY_LEFT)
			{
				leftFlywheel.variables.power -= .5;
				rightFlywheel.variables.power -= .5;
				taskDelay(100);
			}
			else if(main.leftDpad.axisValue == JOY_UP)
			{
				leftFlywheel.variables.power =26.5;
				rightFlywheel.variables.power =26.5;
				taskDelay(100);
			}
			else if(main.leftDpad.axisValue == JOY_RIGHT)
			{
				leftFlywheel.variables.power =22;
				rightFlywheel.variables.power =22;
				taskDelay(100);
			}
			else if(main.leftDpad.axisValue == JOY_LEFT)
			{
				leftFlywheel.variables.power =20;
				rightFlywheel.variables.power =20;
				printf("debug");
				taskDelay(100);
			}
			else if(main.leftDpad.axisValue == JOY_DOWN)
			{
				leftFlywheel.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(100);
			}
			if(leftFlywheel.variables.power<0)
			{
				leftFlywheel.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(100);
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

int getRPower()
{
	return (int)(rightFlywheel.variables.powerRaw);
}

int getLPower()
{
	return (int)(leftFlywheel.variables.powerRaw);
}

int getRVel()
{
	return (int)(rightFlywheel.variables.velocityRaw);
}
int getLVel()
{
	return (int)(leftFlywheel.variables.velocityRaw);
}
