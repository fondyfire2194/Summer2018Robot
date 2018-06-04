package org.usfirst.frc.team2194.robot.commands.Climber;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class StopClimber extends InstantCommand {

	public StopClimber() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.climber.stopClimber();
		Robot.airCompressor.start();
	}

}
