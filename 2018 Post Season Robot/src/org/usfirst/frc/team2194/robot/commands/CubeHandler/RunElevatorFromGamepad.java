package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunElevatorFromGamepad extends Command {

	public RunElevatorFromGamepad() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.cubeHandler);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		RobotMap.elevatorMotor.configNominalOutputForward(0, 0);
		RobotMap.elevatorMotor.configNominalOutputReverse(-0, 0);
		RobotMap.elevatorMotor.configPeakOutputForward(1, 0);
		RobotMap.elevatorMotor.configPeakOutputReverse(-1, 0);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		RobotMap.elevatorMotor.configNominalOutputForward(0.0, 0);
		RobotMap.elevatorMotor.configNominalOutputReverse(0.0, 0);
		RobotMap.elevatorMotor.configPeakOutputForward(1, 0);
		RobotMap.elevatorMotor.configPeakOutputReverse(-1, 0);

		double yValue;
		double temp;
		if (Math.abs(Robot.oi.gamepad.getY()) > .1)
			yValue = Robot.oi.gamepad.getY();
		else
			yValue = 0;
		if (!RobotMap.elevatorSwitch.get() && yValue > 0)// inhibit down move on bottom switch
			yValue = 0;
		// square joystick and preserve sign
		temp = yValue * yValue;
		if (yValue < 0)
			temp = -temp;
		Robot.cubeHandler.runElevatorMotor(-yValue);// y up gives a negative value
		// if (yValue > 0)
		// Robot.cubeHandler.holdPositionInches =
		// Robot.cubeHandler.getElevatorPositionInches() + 2;
		// else
		// Robot.cubeHandler.holdPositionInches =
		// Robot.cubeHandler.getElevatorPositionInches();

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.cubeHandler.holdPositionInches = Robot.cubeHandler.getElevatorPositionInches();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
