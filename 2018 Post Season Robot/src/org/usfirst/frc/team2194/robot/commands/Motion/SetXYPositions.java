package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetXYPositions extends InstantCommand {
	private double myXPosition;
	private double myYPosition;

	public SetXYPositions(double xPosition, double yposition) {
		super();
		myXPosition = xPosition;
		myYPosition = yposition;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.xPosition = myXPosition;
		Robot.yPosition = myYPosition;
	}

}
