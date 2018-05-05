package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetDriveBrakeMode extends InstantCommand {
	private boolean myBrake;

	public SetDriveBrakeMode(boolean brake) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myBrake = brake;
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.driveTrainCanBus.setBrakeMode(myBrake);
	}

}
