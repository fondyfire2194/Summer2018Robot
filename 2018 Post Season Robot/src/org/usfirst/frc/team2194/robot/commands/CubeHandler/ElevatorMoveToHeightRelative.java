package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorMoveToHeightRelative extends Command {
	private boolean moveIsUp;
	private double moveIsUpLookAhead = 3;
	private boolean moveIsDown;
	private double moveIsDownLookAhead = 3;
	private double myHeight;
	private boolean atPosition;
	private double atPositionBand = 3;
	private double currentHeight;

	public ElevatorMoveToHeightRelative(double height) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myHeight = height;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		moveIsUp = myHeight > Robot.cubeHandler.holdPositionInches;
		moveIsDown = myHeight < Robot.cubeHandler.holdPositionInches;
		//TODO: Add down to switch command
		//		if(myHeight == Robot.cubeHandler.ELEVATOR_PICKUP_POSITION_INCHES) {
//			DownToSwitch.start;
//		}
//		
		Robot.cubeHandler.holdPositionInches = myHeight;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		atPosition = (!moveIsUp && !moveIsDown)
				|| moveIsUp && Robot.cubeHandler.getElevatorPositionInches() > myHeight - atPositionBand
				|| moveIsDown && Robot.cubeHandler.getElevatorPositionInches() < myHeight + atPositionBand;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return atPosition;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
