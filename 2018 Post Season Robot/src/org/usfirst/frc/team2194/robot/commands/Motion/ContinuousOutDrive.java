package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ContinuousOutDrive extends Command {
	private double mySpeed;

	public ContinuousOutDrive(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);
		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainCanBus.leftDriveOut(mySpeed);
		Robot.driveTrainCanBus.rightDriveOut(mySpeed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainCanBus.leftDriveOut(0);
		Robot.driveTrainCanBus.rightDriveOut(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
