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

void driveStraight(unsigned long time, float equilibriumSpeed);

void strafeStraight(unsigned long time, float equilibriumSpeed);

bool twoJoysticks;

flywheel rightFlywheel;

flywheel shooter;

TaskHandle shooter_task;

TaskHandle powerListener_task;

TaskHandle velocity_task;

TaskHandle drive_task;

TaskHandle joystick_task;

Encoder shooterEncoder;

Gyro gyro;
#endif /* _1727B_H_ */
