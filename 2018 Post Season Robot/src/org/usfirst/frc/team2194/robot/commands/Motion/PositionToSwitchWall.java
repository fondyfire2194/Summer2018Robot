package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PositionToSwitchWall extends Command {
	private double myTargetFt;
	private double leftTargetFt;
	private double rightTargetFt;
	private double myFtPerSec;
	private double myTimeout;
	private int passCount;
	private boolean inPosition;

	public PositionToSwitchWall(double targetPositionFt, double ftPerSecond, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myTargetFt = targetPositionFt;
		myFtPerSec = ftPerSecond;
		myTimeout = timeout;
		passCount = 0;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		rightTargetFt = myTargetFt;
		leftTargetFt = myTargetFt;
		Robot.driveTrainCanBus.setPosition(rightTargetFt, driveSide.right, myFtPerSec);
		Robot.driveTrainCanBus.setPosition(leftTargetFt, driveSide.left, myFtPerSec);
		Robot.positionRunning = true;
		Robot.driveTrainCanBus.setLeftBrakeMode(false);
		Robot.driveTrainCanBus.setRightBrakeMode(false);
		setTimeout(myTimeout);
		passCount = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (RobotMap.driveLeftMotorA.getClosedLoopError(0) > 0) {

			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec - Robot.sensors.getGyroPositionStraightComp(),
					driveSide.left);
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec + Robot.sensors.getGyroPositionStraightComp(),
					driveSide.right);
		} else {
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec + Robot.sensors.getGyroPositionStraightComp(),
					driveSide.left);
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec - Robot.sensors.getGyroPositionStraightComp(),
					driveSide.right);
		}
		passCount++;
		if (passCount > 15)
			Robot.driveTrainCanBus.runStalledDetect = true;// turn on checks in Drive Monitor task

		inPosition = Robot.driveTrainCanBus.leftSideStopped || Robot.driveTrainCanBus.rightSideStopped
				|| Robot.driveTrainCanBus.rightSideInPosition() && Robot.driveTrainCanBus.leftSideInPosition();

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || (inPosition && passCount > 3);

	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.stopMotor(driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		Robot.positionRunning = false;
		Robot.driveTrainCanBus.runStalledDetect = false;
		Robot.driveTrainCanBus.setLeftBrakeMode(true);
		Robot.driveTrainCanBus.setRightBrakeMode(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}