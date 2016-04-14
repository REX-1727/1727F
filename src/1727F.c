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
		int *outputs,
		int outputNumber)
{
	aFlywheel.parameters.input = input;
	aFlywheel.parameters.target = target;
	aFlywheel.parameters.timeOut = -1;
	aFlywheel.parameters.kP = kP;
	aFlywheel.parameters.kI = kI;
	aFlywheel.parameters.kD = kD;
	aFlywheel.parameters.outputs = malloc(outputNumber*sizeof(int));
	aFlywheel.parameters.outputs = outputs;
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
			if(main.rightDpad.axisValue == JOY_UP)
			{
				shooter.variables.power++;
				rightFlywheel.variables.power++;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_DOWN)
			{
				shooter.variables.power--;
				rightFlywheel.variables.power--;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_RIGHT)
			{
				shooter.variables.power += .5;
				rightFlywheel.variables.power += .5;
				taskDelay(200);
			}
			else if(main.rightDpad.axisValue == JOY_LEFT)
			{
				shooter.variables.power -= .5;
				rightFlywheel.variables.power -= .5;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_UP)
			{
				shooter.variables.power =40;
				rightFlywheel.variables.power =26.5;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_RIGHT)
			{
				shooter.variables.power =22;
				rightFlywheel.variables.power =22;
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_LEFT)
			{
				shooter.variables.power =20;
				rightFlywheel.variables.power =20;
				printf("debug");
				taskDelay(200);
			}
			else if(main.leftDpad.axisValue == JOY_DOWN)
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

			rightFlywheel.variables.powerRaw = (rightFlywheel.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
			shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

			lcdPrint(uart1,1,"%f",shooter.variables.power);
			//printf("%f /n /r", leftFlywheel.variables.power);
			taskDelay(20);

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
