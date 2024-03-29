package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetDriveStraightAngle extends InstantCommand {
	private double myAngle;

	public SetDriveStraightAngle(double angle) {
		super();
		myAngle = angle;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.driveTrainCanBus.driveStraightAngle = myAngle;
	}

}
