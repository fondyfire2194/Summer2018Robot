package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.AlertDriver;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWheelsToIntake extends Command {
	private double mySpeed;
	private boolean currentPeakSeen;
	private double highCurrentLimit = 10;
	private double highCurrentTimeLimit = .5;
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
		if (Robot.createIntakeRunFile)
			Robot.simpleCSVLogger.init("Intake", Robot.intakeNames, Robot.intakeUnits);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Timer.getFPGATimestamp() - startTime > waitForMotorStartedTime) {
			// if ((highCurrentTime == 0) && (RobotMap.intakeLeftMotor.getOutputCurrent() >
			// highCurrentLimit)
			// || RobotMap.intakeRightMotor.getOutputCurrent() > highCurrentLimit)
			// highCurrentTime = Timer.getFPGATimestamp();
			// if (highCurrentTime != 0 && Timer.getFPGATimestamp() - highCurrentLimit >
			// highCurrentTimeLimit)
			// currentPeakSeen = true;
			if (Robot.createIntakeRunFile) {
				Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime),
						RobotMap.intakeLeftMotor.getOutputCurrent(), RobotMap.intakeLeftMotor.getMotorOutputVoltage(),
						RobotMap.intakeRightMotor.getOutputCurrent(),
						RobotMap.intakeRightMotor.getMotorOutputVoltage());
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
		if (Robot.createIntakeRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
