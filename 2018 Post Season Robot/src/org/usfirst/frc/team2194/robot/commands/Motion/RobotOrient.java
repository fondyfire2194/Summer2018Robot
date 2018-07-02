/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.LogDriveData;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;
	private double startTime;
	private double rampTime;

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
		if (Robot.closeDriveSpeedLoop) {
			RobotMap.driveLeftMotorA.selectProfileSlot(0, 0);
			RobotMap.driveRightMotorA.selectProfileSlot(0, 0);
		}
		Robot.driveTrainCanBus.configOpenLoopAcceleration(0);
		rampIncrement = mySpeed / 10;
		Robot.robotRotate.setPIDF(Robot.prefs.getDouble("RobotRotateKp", DriveTrainCanBus.drivePrefsDefaults[10]), 0,
				Robot.prefs.getDouble("RobotRotateKd", DriveTrainCanBus.drivePrefsDefaults[22]), 0);
		Robot.robotRotate.setMaxOut(DriveTrainCanBus.MINIMUM_START_PCT);
		Robot.robotRotate.setSetpoint(myAngle);
		Robot.robotRotate.enablePID();
		Robot.orientRunning = true;
		setTimeout(myTimeout);
		currentMaxSpeed = DriveTrainCanBus.MINIMUM_START_PCT;
		passCount = 0;
		// Robot.closeDriveSpeedLoop = true;
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		passCount++;
		Robot.robotRotate.setMaxOut(currentMaxSpeed);
		if (!doneAccelerating) {
			currentMaxSpeed = currentMaxSpeed + rampIncrement;
			if (currentMaxSpeed >= mySpeed) {
				currentMaxSpeed = mySpeed;
				doneAccelerating = true;
				rampTime = Timer.getFPGATimestamp() - startTime;
				SmartDashboard.putNumber("Ramptime", rampTime);
			}
		}

		if (passCount > 5 && Math.abs(Robot.robotRotate.getError()) < Robot.prefs.getDouble("RobotRotateIzone",
				DriveTrainCanBus.drivePrefsDefaults[12]))
			Robot.robotRotate.getPIDController()
					.setI(Robot.prefs.getDouble("RobotRotateKi", DriveTrainCanBus.drivePrefsDefaults[11]));
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
		Robot.robotRotate.setMaxOut(DriveTrainCanBus.MINIMUM_START_PCT);
		Robot.robotRotate.disable();
		Robot.orientRunning = false;
		Robot.driveTrainCanBus.configOpenLoopAcceleration(.5);
		// Robot.closeDriveSpeedLoop = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
