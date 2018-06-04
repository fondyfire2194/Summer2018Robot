package org.usfirst.frc.team2194.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class AlertDriver extends InstantCommand {
	private String myAlert;

	public AlertDriver(String alert) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myAlert = alert;
	}

	// Called once when the command executes
	protected void initialize() {
		DriverStation.reportWarning(myAlert, false);
	}

}
