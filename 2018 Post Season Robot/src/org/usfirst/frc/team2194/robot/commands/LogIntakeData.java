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
	private String[] names = { "Time", "Left Amps", "Left Volts", "Right Amps", "Right Volts", "Dist", "Battery" };
	private String[] units = { "mS", "Amps", "Volts", "Amps", "Volts", "Ft", "Volts" };
	double leftAmps;
	double leftVolts;
	double rightAmps;
	double rightVolts;
	double batteryVolts;
	boolean oops;

	public LogIntakeData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.createIntakeRunFile)
			Robot.simpleCSVLogger.init("Intake", "Intake", names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.simpleCSVLogger.log_open && Robot.createIntakeRunFile) {
			leftAmps = 999999;
			leftVolts = 999999;
			rightAmps = 999999;
			rightVolts = 999999;
			batteryVolts = 99999;
			try {
				leftAmps = RobotMap.intakeLeftMotor.getOutputCurrent();
				leftVolts = RobotMap.intakeLeftMotor.getMotorOutputVoltage();
				rightAmps = RobotMap.intakeRightMotor.getOutputCurrent();
				rightVolts = RobotMap.intakeRightMotor.getMotorOutputVoltage();
				batteryVolts = RobotMap.pdp.getVoltage();

			} catch (Exception e) {
				oops = true;
			}

			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime), leftAmps, leftVolts, rightAmps,
					rightVolts, Robot.driveTrainCanBus.getLeftFeet(),batteryVolts);
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
