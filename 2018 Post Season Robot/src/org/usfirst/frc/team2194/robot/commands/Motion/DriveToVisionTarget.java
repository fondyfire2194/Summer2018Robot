package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToVisionTarget extends Command {
	private double myFtPerSec;
	private int myTargetOffsetMultiplier;
	private double myTimeout;
	private double activeComp;
	private double initialTargetAngle;
	private int activeError;

	public DriveToVisionTarget(int targetOffsetMultiplier, double ftPerSecond, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);

		myFtPerSec = ftPerSecond;
		myTimeout = timeout;
		myTargetOffsetMultiplier = targetOffsetMultiplier;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainCanBus.setVBus(myFtPerSec * DriveTrainCanBus.FT_PER_SEC_TO_PCT_OUT, driveSide.both);
		Robot.allCameras.targetOffsetMultiplier = myTargetOffsetMultiplier;
		Robot.visionMotionRunning = true;
		Robot.cubeHandler.cubePickedUp = false;
		initialTargetAngle = Robot.driveTrainCanBus.driveStraightAngle;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.allCameras.targetsPresent()) {
			activeComp = Robot.allCameras.getVisionPositionStraightComp();
			activeError = Robot.allCameras.getVisionError();

		} else {
			activeComp = Robot.sensors.getGyroDriveStraightComp();
			Robot.driveTrainCanBus.driveStraightAngle = Robot.sensors.getGyroYaw();
			activeError = (int) Robot.sensors.getGyroAngle();
		}
		if (RobotMap.driveLeftMotorA.getClosedLoopError(0) > 0) {

			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec - activeComp, driveSide.left);
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec + activeComp, driveSide.right);
		} else {
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec + activeComp, driveSide.left);
			Robot.driveTrainCanBus.configDrivePeakout(myFtPerSec - activeComp, driveSide.right);
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || Robot.cubeHandler.cubePickedUp;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.stopMotor(driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		Robot.visionMotionRunning = false;
		Robot.driveTrainCanBus.driveStraightAngle = initialTargetAngle;
		Robot.cubeHandler.cubePickedUp = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}