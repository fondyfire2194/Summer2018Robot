package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetTrajectoryGains extends Command {
	private double[] myGains = { 0, 0, 0, 0 };

	public SetTrajectoryGains(double[] gains) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myGains = gains;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		for (int i = 0; i < Robot.activeTrajectoryGains.length; i++) 
			Robot.activeTrajectoryGains[i] = myGains[i];
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
