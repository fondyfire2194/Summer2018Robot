package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class LogIntakeData extends TimedCommand {
	private double startTime;

	public LogIntakeData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.createIntakeRunFile)
			Robot.simpleCSVLogger.init("Intake", Robot.intakeNames, Robot.intakeUnits);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createIntakeRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime),
					RobotMap.intakeLeftMotor.getOutputCurrent(), RobotMap.intakeLeftMotor.getMotorOutputVoltage(),
					RobotMap.intakeRightMotor.getOutputCurrent(), RobotMap.intakeRightMotor.getMotorOutputVoltage());
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createIntakeRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
