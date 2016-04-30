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

		printf("%f\n\r",shooter.variables.velocity);

		taskDelayUntil(&startTime, MOTOR_REFRESH_TIME);
	}
}



void powerListener(void *params)
{

	driveMultiplier = 1;
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
				taskDelay(200);
			}
			else if(partner.leftDpad.axisValue == JOY_RIGHT)
			{
				shooter.variables.power =40;
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

			if(main.rightDpad.axisValue == JOY_RIGHT)
			{
				driveMultiplier = .8;
				delay(200);
			}
			else if(main.rightDpad.axisValue == JOY_LEFT)
			{
				driveMultiplier = 1;
				delay(200);
			}

			shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

			//lcdPrint(uart1,1,"%f",shooter.variables.power);
			//printf("%f /n /r", leftFlywheel.variables.power);
			taskDelay(20);

		}
	}
}

void intakeControl(void *params)
{
	bool intakeFlag = false;
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
		//printf("%d\r\n",ultrasonicGet(loadChecker));
		delay(20);
	}
}

void driveControl(void *params)
{
	float leftPower;
	float rightPower;

	while(true)
	{
		leftPower = ((main.leftVertical.axisValue*(abs(main.leftVertical.axisValue)))*driveMultiplier/(127.0*127.0))*127;
		rightPower = (main.rightVertical.axisValue*(abs(main.rightVertical.axisValue))*driveMultiplier/(127.0*127.0))*127;
		//		printf("%f    %f \r\n",leftPower,rightPower);
		motorSet(RB, -rightPower);
		motorSet(LB, -leftPower);
		motorSet(RF, -rightPower);
		motorSet(LF, leftPower);
		motorSet(RM, rightPower);
		motorSet(LM, leftPower);


		taskDelay(20);
	}
}

bool isLoaded()
{
	int temp = ultrasonicGet(loadChecker);
	return (temp<10 && temp!= 0);
}

void loadBall(int maxLoadTime)
{
	unsigned long startTime = millis();
	ballLoaded = isLoaded();
	while(!ballLoaded && millis()< startTime+maxLoadTime)
	{
		motorSet(LOWER_INTAKE, -127);
		motorSet(UPPER_INTAKE, 127);
		ballLoaded = isLoaded();
		delay(20);
	}
	motorSet(LOWER_INTAKE, 0);
	motorSet(UPPER_INTAKE, 0);
}

void fireBall(int maxLoadTime, int maxFireTime)
{
	unsigned long startTime = millis();
	while(ballLoaded && millis()<startTime+maxFireTime)
	{
		motorSet(LOWER_INTAKE, -127);
		motorSet(UPPER_INTAKE, 127);
		ballLoaded = isLoaded();
		delay(20);
	}
	loadBall(maxLoadTime);
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

void setTargetForward(float inches)
{
	leftDriveTarget += (inches*360)/(M_PI*3.25);
	rightDriveTarget += (inches*360)/(M_PI*3.25);
}

void setTargetRotate(float degrees)
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
			lcdPrint(uart1,1,"BLUE Far Side");
			break;
		case BLUE_NEAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"BLUE Near Side");
			break;
		case RED_FAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"RED Far Side");
			break;
		case RED_NEAR_SIDE_AUTONOMOUS:
			lcdPrint(uart1,1,"RED Near Side");
			break;
		case NO_AUTONOMOUS:
			lcdPrint(uart1,1,"NO AUTON");
			break;
		case PROGRAMMING_SKILLS:
			lcdPrint(uart1,1,"PROG Skills");
			break;
		}

		if(lcdReadButtons(uart1) == 1)
		{
			currentAuton -= 1;
			delay(500);
		}

		else if(lcdReadButtons(uart1) == 4)
		{
			currentAuton +=1;
			delay(500);
		}

		else if(lcdReadButtons(uart1) == 2)
		{
			printf("debug");
			return currentAuton;
		}

		if(currentAuton < 0)
			currentAuton = 0;

		else if(currentAuton > 5)
			currentAuton = 5;
		delay(20);
	}
	return 4;
}

void blueFarSide()
{
	// FIRING AUTON
	//	setTargetRotate(41);
	//	delay(700);
	//	setTargetForward(70);
	//	loadBall(3000);
	//	shooter.variables.power =35;
	//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	//	setTargetRotate(32.5);
	//	delay(2000);
	//	shooter.variables.power =0;
	//	shooter.variables.powerRaw = 0;
	//	setTargetRotate(-18.5);
	//	delay(700);
	//	setTargetForward(34);
	//	loadBall(3000);
	//	setTargetRotate(26.5);
	//	shooter.variables.power =33;
	//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	//	delay(2000);
	//	fireBall(500,500);
	//	fireBall(500,500);
	//	fireBall(500,500);
	//	fireBall(500,500);
	//	shooter.variables.power =0;
	//	shooter.variables.powerRaw = 0;
	//	setTargetRotate(-71.5);
	//	delay(1000);
	//	setTargetForward(24);
	//	loadBall(1000);


	//  HOARDING AUTON
	//	setTargetRotate(41);
	//	delay(700);
	//	setTargetForward(70);
	//	motorSet(LOWER_INTAKE, 127);
	//	delay(3000);
	//	setTargetRotate(32.5);
	//	delay(2000);
	//	setTargetRotate(-18.5);
	//	delay(700);
	//	setTargetForward(34);
	//	delay(2000);
	//	setTargetRotate(26.5);
	//	delay(2000);
	//	setTargetRotate(-71.5);
	//	delay(1000);
	//	setTargetForward(24);
	//	motorSet(LOWER_INTAKE, -127);
	//	motorSet(UPPER_INTAKE, 127);
	//	delay(2000);

//	setTargetRotate(27);
//	delay(1000);
//	setTargetForward(53.6);
//	shooter.variables.power =41;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	delay(2000);
//	setTargetRotate(34);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	setTargetRotate(-14.2);
//	delay(500);
//	shooter.variables.power =38;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	setTargetForward(17);
//	loadBall(3000);
//	delay(250);
//	setTargetRotate(21.1);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);

	shooter.variables.power =48;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	delay(2000);
	loadBall(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	shooter.variables.power =0;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	setTargetRotate(7.69);
	delay(500);
	setTargetForward(70);
	loadBall(3000);
	setTargetRotate(-31);
	delay(500);
	setTargetForward(12);
	delay(1000);
	setTargetRotate(90);
	delay(500);
	setTargetForward(36);

}

void blueNearSide()
{
//	setTargetRotate(50);
//	delay(1000);
//	setTargetForward(60);
//	delay(2000);
//	setTargetRotate(127);
//	delay(750);
//	setTargetForward(12);
//	delay(750);
//	setTargetRotate(45);
//	delay(500);
//	motorSet(LOWER_INTAKE, 127);
//	setTargetForward(34);
//	delay(1000);
//	motorSet(UPPER_INTAKE,127);
//	motorSet(LOWER_INTAKE, -127);
//	setTargetRotate(-45);
//	delay(500);
//	setTargetForward(24);
//	delay(3000);
//	motorSet(UPPER_INTAKE,0);
//	motorSet(LOWER_INTAKE, 0);

	shooter.variables.power =48;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	delay(4000);
	loadBall(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	shooter.variables.power =0;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

	setTargetRotate(6);
	delay(300);
	setTargetForward(51);
	delay(2500);
	setTargetRotate(90);
	delay(750);
	setTargetForward(35);
	delay(1250);
	setTargetRotate(135);
	delay(1000);
	motorSet(LOWER_INTAKE,127);
	setTargetForward(36);
	delay(2000);
	setTargetForward(-12);
	delay(500);
	setTargetRotate(-90);
	delay(500);
//	delay(2000);
//	setTargetRotate(90);
//	delay(500);
//	setTargetForward(-12);
//	delay(750);
//	setTargetForward(18);
//	delay(1000);
}

void redFarSide()
{
//	setTargetRotate(-41);
//	delay(1000);
//	setTargetForward(70);
//	loadBall(3000);
//	shooter.variables.power =35;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	setTargetRotate(-34);
//	delay(2000);
//	fireBall(500,500);
//	fireBall(500,500);
//	fireBall(500,500);
//	fireBall(500,500);
//	shooter.variables.power =0;
//	shooter.variables.powerRaw = 0;
//	setTargetRotate(14.2);
//	delay(700);
//	setTargetForward(34);
//	loadBall(3000);
//	setTargetRotate(-26.5);
//	shooter.variables.power =33;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	delay(2000);
//	fireBall(500,500);
//	fireBall(500,500);
//	fireBall(500,500);
//	fireBall(500,500);
//	shooter.variables.power =0;
//	shooter.variables.powerRaw = 0;
//	setTargetRotate(71.5);
//	delay(1000);
//	setTargetForward(24);
//	loadBall(1000);

//	setTargetRotate(-27);
//	delay(1000);
//	setTargetForward(53.6);
//	shooter.variables.power =41;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	delay(2000);
//	setTargetRotate(-34);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	setTargetRotate(14.2);
//	delay(500);
//	shooter.variables.power =38;
//	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	setTargetForward(17);
//	loadBall(3000);
//	delay(250);
//	setTargetRotate(-21.1);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);
//	fireBall(2000,2000);
//	delay(500);


	shooter.variables.power =48;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	delay(2000);
	loadBall(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	shooter.variables.power =0;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	setTargetRotate(-7.69);
	delay(500);
	setTargetForward(70);
	loadBall(3000);
	setTargetRotate(31);
	delay(500);
	setTargetForward(12);
	delay(1000);
	setTargetRotate(-90);
	delay(500);
	setTargetForward(36);


}

void redNearSide()
{
//	setTargetRotate(-50);
//	delay(500);
//	setTargetForward(60);
//	delay(2000);
//	setTargetRotate(-127);
//	delay(750);
//	setTargetForward(12);
//	delay(750);
//	setTargetRotate(-45);
//	delay(500);
//	motorSet(LOWER_INTAKE, 127);
//	setTargetForward(34);
//	delay(1000);
//	motorSet(LOWER_INTAKE, -127);
//	setTargetRotate(45);
//	delay(500);
//	setTargetForward(24);
//	delay(2000);
	shooter.variables.power =48;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	delay(4000);
	loadBall(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	fireBall(1000,1000);
	delay(1000);
	shooter.variables.power =0;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;

	setTargetRotate(-6);
	delay(300);
	setTargetForward(51);
	delay(2500);
	setTargetRotate(-90);
	delay(750);
	setTargetForward(35);
	delay(1250);
	setTargetRotate(-135);
	delay(1000);
	motorSet(LOWER_INTAKE,127);
	setTargetForward(36);
	delay(2000);
	setTargetForward(-12);
	delay(500);
	setTargetRotate(90);
}

void programmingSkills()
{

	shooter.variables.power = 39;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	delay(2000);
	motorSet(LOWER_INTAKE, -127);
	motorSet(UPPER_INTAKE, 127);
	delay(22000);
	shooter.variables.power = 0;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	setTargetRotate(95);
	delay(1000);
	setTargetForward(120);
	delay(5000);
	shooter.variables.power = 39;
	shooter.variables.powerRaw = (shooter.variables.power)*(12/FLYWHEEL_CIRCUMFERENCE)*360;
	setTargetRotate(-85);
	delay(30000);





}
