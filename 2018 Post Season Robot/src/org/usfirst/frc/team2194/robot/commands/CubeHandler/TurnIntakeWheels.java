package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class TurnIntakeWheels extends InstantCommand {
	private double mySpeed;

	public TurnIntakeWheels(double speed) {
		super();
		mySpeed = speed;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.cubeHandler.intakeWheelsTurn(mySpeed);
	}

}
