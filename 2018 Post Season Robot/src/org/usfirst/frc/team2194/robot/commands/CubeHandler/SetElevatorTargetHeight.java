package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorTargetHeight extends Command {
	private double myHeight;
	private int scanCtr;;

	public SetElevatorTargetHeight(double height) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myHeight = height;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		scanCtr=0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.cubeHandler.moveIsUp = myHeight > Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.moveIsDown = myHeight < Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.holdPositionInches = myHeight;
		scanCtr++;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return scanCtr>=1;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
