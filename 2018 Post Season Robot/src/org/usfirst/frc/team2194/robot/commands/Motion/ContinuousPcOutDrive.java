package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ContinuousPcOutDrive extends Command {
	private double mySpeed;

	public ContinuousPcOutDrive(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);\
		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(60);
		Robot.driveTrainCanBus.leftDrivePctOut(mySpeed);
		Robot.driveTrainCanBus.rightDrivePctOut(mySpeed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
