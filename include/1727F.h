/*
 * 1727B.h
 *
 *  Created on: Jan 29, 2016
 *      Author: Anton
 */

#ifndef _1727B_H_
#define _1727B_H_

#include <API.h>
#include "rexAPI.h"
#include "math.h"

#define RF				2
#define LF				8
#define RB				2
#define LB				10
#define RM				1
#define LM				8
#define LOWER_INTAKE	7
#define UPPER_INTAKE	4

#define BLUE_FAR_SIDE_AUTONOMOUS	0
#define BLUE_NEAR_SIDE_AUTONOMOUS	1
#define RED_FAR_SIDE_AUTONOMOUS		2
#define RED_NEAR_SIDE_AUTONOMOUS	3
#define NO_AUTONOMOUS				4


#define FLYWHEEL_CIRCUMFERENCE (4*3.1415926535)


typedef struct pidVars
{
	float velocity;
	float velocityRaw;
	float power;
	float powerRaw;
}pidVars;

typedef struct flywheel
{
	pidVars variables;
	pidParams parameters;
}flywheel;

void flywheelInit(flywheel aFlywheel,float (*input)(), float (*target)(), float kP, float kI, float kD, int outputs[4]);

void velocityReader(void *ignore);

void powerListener(void *params);

void driveControl(void *params);

float gyroTarget;

float getPower();

float getVel();

float getGyro();

float getGyroTarget();

void setGyroTarget(float target);

void setTargetForward(float inches);

void setTargetRotate(float degrees);

float getLeftDriveTarget();

float getRightDriveTarget();

float getLeftDrive();

float getRightDrive();

void resetDriveTargets();

void intakeControl(void *params);

int selectAuton();

void blueFarSide();

void blueNearSide();

void redFarSide();

void redNearSide();

bool isLoaded();

void loadBall(int maxLoadTime);

void fireBall(int maxLoadTime, int maxFireTime);

bool twoJoysticks;

flywheel rightFlywheel;

flywheel shooter;

TaskHandle shooter_task;

TaskHandle powerListener_task;

TaskHandle velocity_task;

TaskHandle drive_task;

TaskHandle joystick_task;

TaskHandle leftDrive_autonomous_task;

TaskHandle rightDrive_autonomous_task;

TaskHandle intake_task;

Encoder shooterEncoder;

Encoder leftDriveEncoder;

Encoder rightDriveEncoder;

Gyro gyro;

int leftDriveTarget;

int rightDriveTarget;

int selectedAuton;

bool ballLoaded;

#endif /* _1727B_H_ */
