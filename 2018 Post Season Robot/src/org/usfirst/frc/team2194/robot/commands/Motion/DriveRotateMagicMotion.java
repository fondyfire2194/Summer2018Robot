package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.SD;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveRotateMagicMotion extends Command {
	private double myTargetFt;
	private double myFtPerSec;
	private double myTimeout;
	private motionType myType;

	public DriveRotateMagicMotion(double targetPositionFt, motionType type, double ftPerSecond, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myTargetFt = targetPositionFt;
		myFtPerSec = ftPerSecond;
		myTimeout = timeout;
		myType = type;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (myType == motionType.incremental) {
			myTargetFt += Robot.driveTrainCanBus.getLeftFeet();
		}
		Robot.driveTrainCanBus.leftPositionTargetFt = myTargetFt;
		Robot.driveTrainCanBus.rightPositionTargetFt = -myTargetFt;

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