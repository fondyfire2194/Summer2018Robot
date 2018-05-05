package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler.intakeSide;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SpinCube extends InstantCommand {
	private boolean myDirection;

	public SpinCube(boolean direction) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myDirection = direction;
	}

	// Called once when the command executes
	protected void initialize() {
		if (myDirection) {
			Robot.cubeHandler.turnOneIntakeSide(-.5, intakeSide.left);
			Robot.cubeHandler.turnOneIntakeSide(.5, intakeSide.right);
		}
		else
		{
			Robot.cubeHandler.turnOneIntakeSide(.5, intakeSide.left);
			Robot.cubeHandler.turnOneIntakeSide(-.5, intakeSide.right);
		}
	}

}
