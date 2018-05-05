package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class BuildTrajectoryFromFile extends InstantCommand {
	String myName;
	double[] myGains;

	public BuildTrajectoryFromFile(String name, double[] gains) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myName = name;
		myGains = gains;
	}

	// Called once when the command executes
	protected void initialize() {
    	Robot.buildTrajectory.buildFileName(myName, myGains);
    	
    }

}
