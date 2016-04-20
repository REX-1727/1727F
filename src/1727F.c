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
	encoderReset(shooterEncoder);
	unsigned long startTime = 0;
	while(true)
	{
		startTime = millis();
		shooter.variables.velocityRaw = encoderGet(shooterEncoder)*1000/20;
		shooter.variables.velocity = (shooter.variables.velocityRaw*(FLYWHEEL_CIRCUMFERENCE/12)/360.0);
		rightFlywheel.variables.velocity = (rightFlywheel.variables.velocityRaw*(FLYWHEEL_CIRCUMFERENCE/12)/360.0);
		encoderReset(shooterEncoder);

		//printf("%f\n\r",shooter.variables.velocity);

		taskDelayUntil(&startTime, MOTOR_REFRESH_TIME);
	}
}



void powerListener(void *params)
{

	if(true)
	{
		while(true)
		{
			if(partner.rightDpad.axisValue == JOY_UP)
			{
				shooter.variables.power++;
				rightFlywheel.variables.power++;
				taskDelay(200);
			}
			else if(partner.rightDpad.axisValue == JOY_DOWN)
			{
				shooter.variables.power--;
				rightFlywheel.variables.power--;
				taskDelay(200);
			}
			else if(partner.rightDpad.axisValue == JOY_RIGHT)
			{
				shooter.variables.power += .5;
				rightFlywheel.variables.power += .5;
				taskDelay(200);
			}
			else if(partner.rightDpad.axisValue == JOY_LEFT)
			{
				shooter.variables.power -= .5;
				rightFlywheel.variables.power -= .5;
				taskDelay(200);
			}
			else if(partner.leftDpad.axisValue == JOY_UP)
			{
				shooter.variables.power =48;
				rightFlywheel.variables.power =26.5;
				taskDelay(200);
			}
			else if(partner.leftDpad.axisValue == JOY_RIGHT)
			{
				shooter.variables.power =38.5;
				rightFlywheel.variables.power =22;
				taskDelay(200);
			}
			else if(partner.leftDpad.axisValue == JOY_LEFT)
			{
				shooter.variables.power =31.5;
				printf("debug");
				taskDelay(200);
			}
			else if(partner.leftDpad.axisValue == JOY_DOWN)
			{
				shooter.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(200);
			}
			if(shooter.variables.power<0)
			{
				shooter.variables.power =0;
				rightFlywheel.variables.power =0;
				taskDelay(200);
			}

			shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

			lcdPrint(uart1,1,"%f",shooter.variables.power);
			//printf("%f /n /r", leftFlywheel.variables.power);
			taskDelay(20);

		}
	}
}

void intakeControl(void *params)
{
	bool intakeFlag;
	while(true)
	{
		if(main.rightDpad.axisValue == JOY_UP)
		{
			intakeFlag = true;
		}
		if(main.rightDpad.axisValue == JOY_DOWN)
		{
			intakeFlag = false;
		}
		if(main.rightBumper.axisValue == JOY_UP || intakeFlag)
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
		if(main.leftBumper.axisValue == JOY_UP || intakeFlag)
		{
			motorSet(UPPER_INTAKE, 127);
		}
		else if(main.leftBumper.axisValue == JOY_DOWN)
		{
			motorSet(UPPER_INTAKE, -127);
		}
		else
		{
			motorSet(UPPER_INTAKE, 0);
		}
	}
}

void driveControl(void *params)
{


	while(true)
	{
		motorSet(RB, -main.rightVertical.axisValue);
		motorSet(LB, -main.leftVertical.axisValue);
		motorSet(RF, -main.rightVertical.axisValue);
		motorSet(LF, main.leftVertical.axisValue);
		motorSet(RM, main.rightVertical.axisValue);
		motorSet(LM, main.leftVertical.axisValue);


		taskDelay(20);
	}
}


//PID HELPERS


float gyroTarget = 0;

float getPower()
{
	return (shooter.variables.powerRaw);
}

float getVel()
{
	return (shooter.variables.velocityRaw);
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

void setTargetForward(int inches)
{
	leftDriveTarget += (inches*360)/(M_PI*3.75);
	rightDriveTarget += (inches*360)/(M_PI*3.75);
}

void setTargetRotate(int degrees)
{
	leftDriveTarget += degrees*4.3;
	rightDriveTarget -= degrees*4.3;
}

float getLeftDriveTarget()
{
	return leftDriveTarget;
}

float getRightDriveTarget()
{
	return rightDriveTarget;
}

float getLeftDrive()
{
	return encoderGet(leftDriveEncoder);
}

float  getRightDrive()
{
	return encoderGet(rightDriveEncoder);
}

void resetDriveTargets()
{
	leftDriveTarget=0;
	rightDriveTarget=0;
}

int selectAuton()
{
	int currentAuton = 0;

	while(true)
	{
		switch(currentAuton)
		{
		case BLUE_FAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"BLUE far side");
			break;
		case BLUE_NEAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"BLUE near side");
			break;
		case RED_FAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"RED far side");
			break;
		case RED_NEAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"RED near side");
			break;
		case NO_AUTONOMOUS:
			lcdPrint(uart1,1,"NO AUTON");
			break;
		}

		if(lcdReadButtons(uart1) == 1)
		{
			currentAuton -= 1;
			delay(100);
		}

		else if(lcdReadButtons(uart1) == 2)
		{
			return currentAuton;
		}

		else if(lcdReadButtons(uart1) == 4)
		{
			currentAuton +=1;
			delay(100);
		}

		if(currentAuton < 0)
			currentAuton = 0;

		else if(currentAuton > 4)
			currentAuton = 4;
	}
	return 4;
}

