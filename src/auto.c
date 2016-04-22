/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "1727F.h"


/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */

void autonomous() {

	encoderReset(leftDriveEncoder);
	encoderReset(rightDriveEncoder);
	resetDriveTargets();
	taskResume(leftDrive_autonomous_task);
	taskResume(rightDrive_autonomous_task);
	taskResume(shooter_task);
	taskResume(velocity_task);
	taskSuspend(powerListener_task);
	taskSuspend(joystick_task);
	taskSuspend(drive_task);
	taskSuspend(intake_task);

//	taskResume(velocity_task);
//	taskResume(shooter_task);
//	shooter.variables.power =26.5;
//	rightFlywheel.variables.power =26.5;
//	rightFlywheel.variables.powerRaw = ((rightFlywheel.variables.power))*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	shooter.variables.powerRaw = ((shooter.variables.power))*(12/FLYWHEEL_CIRCUMFERENCE)*360;
//	delay(3000);
//	motorSet(LOWER_INTAKE,-127);
//	motorSet(UPPER_INTAKE,-127);
//	delay(4000);
//	taskSuspend(velocity_task);
//	taskSuspend(shooter_task);
//	shooter.variables.power =0;
//	rightFlywheel.variables.power =0;
//	rightFlywheel.variables.powerRaw =0;
//	shooter.variables.powerRaw = 0;
	lcdSetBacklight(uart1,false);

	switch(selectedAuton)
	{
	case BLUE_FAR_SIDE_AUTONOMOUS:
		blueFarSide();
		break;
	case BLUE_NEAR_SIDE_AUTONOMOUS:
		blueNearSide();
		break;
	case RED_FAR_SIDE_AUTONOMOUS:
		redFarSide();
		break;
	case RED_NEAR_SIDE_AUTONOMOUS:
		redNearSide();
		break;
	case NO_AUTONOMOUS:
		break;
	}

	taskSuspend(leftDrive_autonomous_task);
	taskSuspend(rightDrive_autonomous_task);


}
