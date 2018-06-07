package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * TUning file for robot orient
 */
public class LogOrientData extends TimedCommand {
	private double startTime;
	private String[] names = { "Time", "Gyro Yaw", "Speed Output", "LeftA Amps", "LeftA Volts", "LeftB Amps",
			"LeftB Volts", "LeftC Amps", "LeftC Volts", "RightA Amps", "RightA Volts", "RightB Amps", "RightB Volts",
			"RightC Amps", "RightC Volts", "Left Ft", "Right Ft" };
	private String[] units = { "mS", "Degrees", "PU", "Amps", "Volts", "Amps", "Volts", "Amps", "Volts", "Amps",
			"Volts", "Amps", "Volts", "Amps", "Volts", "Ft", "Ft", "EncCts", "EncCtsPer100ms", "EncCts",
			"EncCtsPer100ms" };

	public LogOrientData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.createOrientRunFile)
			Robot.simpleCSVLogger.init("Orient", names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createOrientRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime) * 1000, Robot.sensors.getGyroYaw(),
					RobotMap.driveLeftMotorA.getMotorOutputVoltage(), RobotMap.driveLeftMotorB.getOutputCurrent(),
					RobotMap.driveLeftMotorB.getMotorOutputVoltage(), RobotMap.driveLeftMotorC.getOutputCurrent(),
					RobotMap.driveLeftMotorC.getMotorOutputVoltage(), RobotMap.driveRightMotorA.getOutputCurrent(),
					RobotMap.driveRightMotorA.getMotorOutputVoltage(), RobotMap.driveRightMotorB.getOutputCurrent(),
					RobotMap.driveRightMotorB.getMotorOutputVoltage(), RobotMap.driveRightMotorC.getOutputCurrent(),
					RobotMap.driveRightMotorC.getMotorOutputVoltage(), Robot.driveTrainCanBus.getLeftFeet(),
					Robot.driveTrainCanBus.getRightFeet(), RobotMap.driveLeftMotorA.getClosedLoopError(0),
					RobotMap.driveLeftMotorA.getSelectedSensorVelocity(0),
					RobotMap.driveRightMotorA.getClosedLoopError(0),
					RobotMap.driveRightMotorA.getSelectedSensorVelocity(0));
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createOrientRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
