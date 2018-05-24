package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunFromGamepadCanBus extends Command {
	double straightComp = 0;

	public RunFromGamepadCanBus() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		double yValue;
		double xValue;
		if (Math.abs(Robot.driveTrainCanBus.getJoystickY()) > .1)
			yValue = Robot.driveTrainCanBus.getJoystickY();
		else
			yValue = 0;
		// square joystick and preserve sign
		if (yValue > 0)
			yValue *= yValue;
		else
			yValue *= -yValue;

		if (Math.abs(Robot.driveTrainCanBus.getJoystickTwist()) > .1)
			xValue = Robot.driveTrainCanBus.getJoystickTwist();
		else
			xValue = 0;
		// square joystick and preserve sign
		if (xValue > 0)
			xValue *= xValue;
		else
			xValue *= -xValue;

		// arcade drive takes throttle and turn values

		if (Robot.driveStraight) {
			straightComp = Robot.sensors.getGyroDriveStraightComp();
			xValue = 0;
		} else
			straightComp = 0;
		if (!DriverStation.getInstance().isAutonomous())
			Robot.driveTrainCanBus.arcadeDrive(-yValue, xValue, straightComp);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
