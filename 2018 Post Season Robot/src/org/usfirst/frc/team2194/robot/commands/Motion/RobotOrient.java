/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 * @author John
 */
public class RobotOrient extends Command {
	private double mySpeed;
	private double myAngle;
	private double myTimeout;
	private int passCount;
	private boolean myAccuracy;
	private boolean inPosition;

	public RobotOrient(double angle, double speed, boolean accuracy, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.robotRotate);
		requires(Robot.driveTrainCanBus);

		mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
		myAccuracy = accuracy;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.robotRotate.setPIDF(Robot.prefs.getDouble("RobotRotateKp", DriveTrainCanBus.drivePrefsDefaults[10]), 0,
				Robot.prefs.getDouble("RobotRotateKd", DriveTrainCanBus.drivePrefsDefaults[22]), 0);
		Robot.robotRotate.setMaxOut(mySpeed);
		Robot.robotRotate.setSetpoint(myAngle);
		Robot.robotRotate.enablePID();
		Robot.orientRunning = true;
		setTimeout(myTimeout);
		passCount = 0;
		RobotMap.driveLeftMotorA.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 2, 0);
		RobotMap.driveRightMotorA.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 2, 0);
		Robot.closeDriveSpeedLoop = true;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		passCount++;
		if (passCount > 5 && Math.abs(Robot.robotRotate.getError()) < Robot.prefs.getDouble("RobotRotateIzone",
				DriveTrainCanBus.drivePrefsDefaults[12]))
			Robot.robotRotate.setPIDF(Robot.robotRotate.getKp(),
					Robot.prefs.getDouble("RobotRotateKi", DriveTrainCanBus.drivePrefsDefaults[11]),
					Robot.prefs.getDouble("RobotRotateKd", DriveTrainCanBus.drivePrefsDefaults[22]), 0);
		if (myAccuracy)
			inPosition = Robot.robotRotate.inPosition();
		else
			inPosition = Robot.robotRotate.closeToPosition();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return passCount > 15 && (isTimedOut() || inPosition);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.robotRotate.disable();
		Robot.orientRunning = false;
		RobotMap.driveRightMotorA.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 0);
		Robot.driveTrainCanBus.configOpenLoopAcceleration(.5);
		Robot.closeDriveSpeedLoop = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
