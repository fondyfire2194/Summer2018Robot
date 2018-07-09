package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPosition extends Command {
	private double myTargetFt;
	private double leftTargetFt;
	private double rightTargetFt;
	private double myFtPerSec;
	private double myTimeout;
	private boolean myStop;
	private motionType myType;
	private int passCount;
	private boolean myHighAccuracy;
	private boolean inPosition;
	private int inPositionCount;
	private boolean inPositionSeen;

	public DriveToPosition(double targetPositionFt, motionType type, double ftPerSecond, boolean highAccuracy,
			boolean stop, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myTargetFt = targetPositionFt;
		myFtPerSec = ftPerSecond;
		myTimeout = timeout;
		myType = type;
		myStop = stop;
		passCount = 0;
		myHighAccuracy = highAccuracy;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		rightTargetFt = myTargetFt;
		leftTargetFt = myTargetFt;
		if (myType == motionType.incremental) {
			leftTargetFt += Robot.driveTrainCanBus.getLeftFeet();
			rightTargetFt += Robot.driveTrainCanBus.getRightFeet();
		}
		Robot.driveTrainCanBus.setPosition(rightTargetFt, driveSide.right, myFtPerSec);
		Robot.driveTrainCanBus.setPosition(leftTargetFt, driveSide.left, myFtPerSec);
		Robot.positionRunning = true;
		setTimeout(myTimeout);
		passCount = 0;
		inPositionCount = 0;
		inPosition = false;
		inPositionSeen = false;
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

		if (myHighAccuracy)
			inPosition = Robot.driveTrainCanBus.rightSideInPosition() && Robot.driveTrainCanBus.leftSideInPosition();
		else
			inPosition = Robot.driveTrainCanBus.closeToPosition();

		if (inPosition && passCount > 10)
			inPositionSeen = true;
		if (inPositionSeen)
			inPositionCount++;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || myStop || inPosition && inPositionCount > 10;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.stopMotor(driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		Robot.positionRunning = false;
		RobotMap.driveLeftMotorA.selectProfileSlot(0, 0);
		RobotMap.driveLeftMotorB.selectProfileSlot(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}