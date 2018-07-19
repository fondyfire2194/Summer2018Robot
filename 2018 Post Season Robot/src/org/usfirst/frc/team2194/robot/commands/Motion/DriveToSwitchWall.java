package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToSwitchWall extends Command {
	private double myTargetFt;
	private double myFtPerSec;
	private double myTimeout;
	private motionType myType;
	private int passCount;

	public DriveToSwitchWall(double targetPositionFt, double ftPerSecond, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myTargetFt = targetPositionFt;
		myFtPerSec = ftPerSecond;
		myTimeout = timeout;

	}

	// Called just before this Command runs the first time
	protected void initialize() {

		// drive Kf calculations from measured values with robot on blocks max speed 20
		// ft/sec max enc cts per 100 ms = 9700
		//
		// reverse check 12 X ft/sec/10 = in/100ms then x enc cts per in gives enc cts
		// per 100ms
		//
		// 20 x 12 /10 = 24 then * 403 = 9672
		//
		// so Kf for 100% feed forward is 1023/9700 = .105 80% ff = .08 use as default
		//
		// for Kp if we want a 2% drive increase for 1" position error
		// 2% x 1023/403 = .05

		Robot.driveTrainCanBus.leftPositionTargetFt = myTargetFt;
		Robot.driveTrainCanBus.rightPositionTargetFt = myTargetFt;
		Robot.driveTrainCanBus.runStalledDetect = true;
		setTimeout(myTimeout);

		passCount = 0;
		Robot.magicMotionRunning = true;
		Robot.driveTrainCanBus.magicMotionDrive(myTargetFt, myFtPerSec, driveSide.both);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		passCount++;
		// get past motion starting before looking for stalled
		if (passCount > 50)
			Robot.driveTrainCanBus.runStalledDetect = true;// turn on checks in Drive Monitor task
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.driveTrainCanBus.leftSideStopped || Robot.driveTrainCanBus.rightSideStopped || isTimedOut()
				|| (Robot.driveTrainCanBus.leftSideInPosition() && Robot.driveTrainCanBus.rightSideInPosition()
						&& passCount > 3);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.runStalledDetect = false;
		Robot.driveTrainCanBus.leftDriveOut(0);
		Robot.driveTrainCanBus.rightDriveOut(0);
		Robot.magicMotionRunning = false;
		RobotMap.driveLeftMotorA.selectProfileSlot(0, 0);
		RobotMap.driveLeftMotorB.selectProfileSlot(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}