package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.AlertDriver;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWheelsToIntake extends Command {
	private double mySpeed;
	private boolean currentPeakSeen;
	private double highCurrentLimit = 10;
	private double highCurrentTimeLimit = .25;
	private double highCurrentTime;

	private double startTime;
	private double waitForMotorStartedTime = 1;
	private double myTimeout;

	public TurnWheelsToIntake(double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		mySpeed = speed;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.cubeHandler.intakeWheelsTurn(mySpeed);
		highCurrentTime = 0;
		currentPeakSeen = false;
		setTimeout(myTimeout);
		new AlertDriver("File Started").start();
		startTime = Timer.getFPGATimestamp();
		Robot.cubeHandler.cubePickedUp = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Timer.getFPGATimestamp() - startTime > waitForMotorStartedTime) {
			if ((highCurrentTime == 0) && (RobotMap.intakeLeftMotor.getOutputCurrent() > highCurrentLimit
					|| RobotMap.intakeRightMotor.getOutputCurrent() > highCurrentLimit))
				highCurrentTime = Timer.getFPGATimestamp();
			if (highCurrentTime != 0 && Timer.getFPGATimestamp() - highCurrentTime > highCurrentTimeLimit) {
				currentPeakSeen = true;
				Robot.cubeHandler.cubePickedUp = true;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() || currentPeakSeen;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.cubeHandler.intakeWheelsTurn(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
