package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorMoveToHeight extends Command {
	private double myHeight;
	private boolean atPosition;
	private double atPositionBand = 3;
	private double startTime;

	private String[] names = { "Time", "TargetHt", "ElevCmdSpeed", "Elev Amps", "Elev Volts", "Actual Ht",
			"ElevatorSpeedIPS", "Enc Speed", "Loop Error" };
	private String[] units = { "mS", "In", "PU", "Amps", "Volts", "In", "In/sec", "Cts/100ms", "Counts" };
	private Timer atPositionTimer;

	public ElevatorMoveToHeight(double height) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myHeight = height;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger.init("Elev", "Elev", names, units);
		Robot.cubeHandler.moveIsUp = myHeight > Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.moveIsDown = myHeight < Robot.cubeHandler.holdPositionInches;
		Robot.cubeHandler.holdPositionInches = myHeight;
		atPositionTimer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createElevatorRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime) * 1000, myHeight,
					RobotMap.elevatorMotor.getMotorOutputPercent(), RobotMap.elevatorMotor.getOutputCurrent(),
					RobotMap.elevatorMotor.getMotorOutputVoltage(), Robot.cubeHandler.getElevatorPositionInches(),
					Robot.cubeHandler.getElevatorSpeedInchesPerSecond(),
					RobotMap.elevatorMotor.getSelectedSensorVelocity(0),
					(double) RobotMap.elevatorMotor.getClosedLoopError(0));
		}
		atPosition = (!Robot.cubeHandler.moveIsUp && !Robot.cubeHandler.moveIsDown)
				|| Robot.cubeHandler.moveIsUp
						&& Robot.cubeHandler.getElevatorPositionInches() > myHeight - atPositionBand
				|| Robot.cubeHandler.moveIsDown
						&& Robot.cubeHandler.getElevatorPositionInches() < myHeight + atPositionBand;

		if (!atPosition)
			atPositionTimer.reset();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return atPositionTimer.get() > .5;
	}

	// Called once after isFinished returns true
	protected void end() {
		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger.close();
		atPositionTimer.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
