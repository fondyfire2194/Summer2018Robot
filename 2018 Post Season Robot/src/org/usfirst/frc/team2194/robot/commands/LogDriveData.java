package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 */
public class LogDriveData extends Command {
	private double startTime, myTimeout;
	private String myTypeName;

	private String[] names = { "Time", "Gyro Yaw", "LeftSpeed", "RightSpeed", "LeftA Amps", "LeftA Volts", "LeftB Amps",
			"LeftB Volts", "LeftC Amps", "LeftC Volts", "RightA Amps", "RightA Volts", "RightB Amps", "RightB Volts",
			"RightC Amps", "RightC Volts", "Left Ft", "Right Ft", "Left Error", "Left Vel", "Right Error",
			"Right Vel" };
	private String[] units = { "mS", "Degrees", "PU", "PU", "Amps", "Volts", "Amps", "Volts", "Amps", "Volts", "Amps",
			"Volts", "Amps", "Volts", "Amps", "Volts", "Ft", "Ft", "EncCts", "Ft/sec", "EncCts", "Ft/sec" };

	public LogDriveData(String typeName, double timeout) {
		myTimeout = timeout;
		myTypeName = typeName;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(myTimeout);
		Robot.createTrajectoryRunFile = false;
		Robot.createDriveRunFile = true;
		Robot.simpleCSVLogger.init(myTypeName, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime) * 1000, Robot.sensors.getGyroYaw(),
					Robot.driveTrainCanBus.leftSpeedForDebug, Robot.driveTrainCanBus.rightSpeedForDebug,
					RobotMap.driveLeftMotorA.getOutputCurrent(), RobotMap.driveLeftMotorA.getMotorOutputVoltage(),
					RobotMap.driveLeftMotorB.getOutputCurrent(), RobotMap.driveLeftMotorB.getMotorOutputVoltage(),
					RobotMap.driveLeftMotorC.getOutputCurrent(), RobotMap.driveLeftMotorC.getMotorOutputVoltage(),
					RobotMap.driveRightMotorA.getOutputCurrent(), RobotMap.driveRightMotorA.getMotorOutputVoltage(),
					RobotMap.driveRightMotorB.getOutputCurrent(), RobotMap.driveRightMotorB.getMotorOutputVoltage(),
					RobotMap.driveRightMotorC.getOutputCurrent(), RobotMap.driveRightMotorC.getMotorOutputVoltage(),
					Robot.driveTrainCanBus.getLeftFeet(), Robot.driveTrainCanBus.getRightFeet(),
					((double) RobotMap.driveLeftMotorA.getClosedLoopError(0)),
					Robot.driveTrainCanBus.getLeftFeetPerSecond(),
					(double) RobotMap.driveRightMotorA.getClosedLoopError(0),
					Robot.driveTrainCanBus.getRightFeetPerSecond());
		}
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut() || !Robot.createDriveRunFile;
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger.close();
			Robot.createDriveRunFile = false;
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
