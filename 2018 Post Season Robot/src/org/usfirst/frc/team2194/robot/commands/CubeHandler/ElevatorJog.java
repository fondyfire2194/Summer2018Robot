package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorJog extends Command {
	private double heightTarget;
	private double maxHeightIncrement = 6;

	public ElevatorJog() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double yValue;

		if (Math.abs(Robot.oi.gamepad.getY()) > .1)
			yValue = -Robot.oi.gamepad.getY();
		else
			yValue = 0;

		if (Robot.cubeHandler.holdPositionInches < CubeHandler.ELEVATOR_MAX_HEIGHT) {
			heightTarget = Robot.cubeHandler.holdPositionInches + maxHeightIncrement * yValue;
			if (heightTarget > CubeHandler.ELEVATOR_MAX_HEIGHT)
				Robot.cubeHandler.holdPositionInches = CubeHandler.ELEVATOR_MAX_HEIGHT;
			if (heightTarget < CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES)
				Robot.cubeHandler.holdPositionInches = CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		return Math.abs(Robot.oi.gamepad.getY()) < .1;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
