package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWheelsToEject extends Command {
	private double mySpeed;
	private boolean currentPeakSeen;;
	private double highCurrentLimit = 10;
	private double lowCurrentLimit = 4;

	public TurnWheelsToEject(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeHandler.intakeWheelsTurn(mySpeed);
		setTimeout(2);
		currentPeakSeen = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (RobotMap.intakeLeftMotor.getOutputCurrent() > highCurrentLimit
				|| RobotMap.intakeRightMotor.getOutputCurrent() > highCurrentLimit) {
			currentPeakSeen = true;
		}
		// SmartDashboard.putBoolean("CLSEEN", currentPeakSeen);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || currentPeakSeen && RobotMap.intakeLeftMotor.getOutputCurrent() < lowCurrentLimit
				&& RobotMap.intakeRightMotor.getOutputCurrent() < lowCurrentLimit;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeHandler.intakeWheelsTurn(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
