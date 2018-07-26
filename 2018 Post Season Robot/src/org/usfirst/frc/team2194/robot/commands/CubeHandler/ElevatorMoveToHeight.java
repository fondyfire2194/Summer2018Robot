
package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ElevatorMoveToHeight extends Command {
	private double myHeight;
	private boolean atPosition;
	private double atPositionBand = 3;
	private int testCtr;
	private int testCtr1;

	public ElevatorMoveToHeight(double height) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myHeight = height;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeHandler.moveIsUp = myHeight > Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.moveIsDown = myHeight < Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.holdPositionInches = myHeight;
		//// SmartDashboard.putBoolean("ElMoveIsUp", moveIsUp);
		// SmartDashboard.putBoolean("ElMoveIsDown", moveIsDown);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		atPosition = (!Robot.cubeHandler.moveIsUp && !Robot.cubeHandler.moveIsDown)
				|| Robot.cubeHandler.moveIsUp
						&& Robot.cubeHandler.getElevatorPositionInches() > myHeight - atPositionBand
				|| Robot.cubeHandler.moveIsDown
						&& Robot.cubeHandler.getElevatorPositionInches() < myHeight + atPositionBand;
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
