package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class TurnVisionPipelineOn extends InstantCommand {
	private boolean myOnOff;

	public TurnVisionPipelineOn(boolean onOff) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myOnOff = onOff;

	}

	// Called once when the command executes
	protected void initialize() {
		Robot.allCameras.cubeVisionTurnedOn = myOnOff;

	}

}
