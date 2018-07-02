package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.LogDriveData;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DoTeleopReverseOrient extends InstantCommand {

	public DoTeleopReverseOrient() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.doTeleopReverseOrient = true;
		new LogDriveData("ReverseOrient", 5).start();
	}

}
