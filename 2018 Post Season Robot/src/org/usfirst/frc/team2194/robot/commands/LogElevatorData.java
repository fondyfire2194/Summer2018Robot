package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class LogElevatorData extends TimedCommand {
	private double startTime;
	private String[] names = { "Time", "TrajPos", "TrajVel", "Tgt Ht", "Act Ht", "Amps", "Volts", "Speed" };
	private String[] units = { "mS", "Inches", "Inches", "Inches", "Inches", "Amps", "Volts", "In/Sec" };

	public LogElevatorData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger.init("Elevator", "Elevator", names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createElevatorRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime),
					RobotMap.elevatorMotor.getActiveTrajectoryPosition(),
					RobotMap.elevatorMotor.getActiveTrajectoryVelocity(), Robot.cubeHandler.holdPositionInches,
					Robot.cubeHandler.getElevatorPositionInches(), RobotMap.elevatorMotor.getOutputCurrent(),
					RobotMap.elevatorMotor.getMotorOutputVoltage(),
					Robot.cubeHandler.getElevatorSpeedInchesPerSecond());
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
