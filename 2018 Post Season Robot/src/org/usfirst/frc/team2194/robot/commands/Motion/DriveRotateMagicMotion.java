package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Rotates angle amount using Magic Motion
 */
public class DriveRotateMagicMotion extends Command {
	private double myTargetAngle;
	private double myFtPerSec;
	private double myTimeout;

	public DriveRotateMagicMotion(double targetAngle, double ftPerSecond, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myTargetAngle = targetAngle;
		myFtPerSec = ftPerSecond;
		myTimeout = timeout;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainCanBus.leftPositionTargetFt = myTargetAngle * Robot.driveTrainCanBus.MM_FT_PER_DEGREE;
		Robot.driveTrainCanBus.rightPositionTargetFt = -myTargetAngle * Robot.driveTrainCanBus.MM_FT_PER_DEGREE;

		setTimeout(myTimeout);

		Robot.magicMotionRunning = true;
		Robot.driveTrainCanBus.magicMotionDrive(Robot.driveTrainCanBus.leftPositionTargetFt, myFtPerSec,
				driveSide.left);
		Robot.driveTrainCanBus.magicMotionDrive(Robot.driveTrainCanBus.rightPositionTargetFt, myFtPerSec,
				driveSide.right);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		Robot.allCameras.visionTargetNotFound = isTimedOut();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.allCameras.targetsPresent() || Robot.allCameras.visionTargetNotFound;

	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.stopMotor(driveSide.both);
		Robot.magicMotionRunning = false;
		Robot.driveTrainCanBus.driveStraightAngle = Robot.sensors.getGyroYaw();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}