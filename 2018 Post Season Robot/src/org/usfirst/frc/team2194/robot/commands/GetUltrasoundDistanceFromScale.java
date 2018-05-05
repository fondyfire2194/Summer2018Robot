package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GetUltrasoundDistanceFromScale extends Command {
	private int passCount;
	private double total;

	public GetUltrasoundDistanceFromScale() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(3);
		passCount = 0;
		total = 0;
		Robot.sensors.scaleReadDistance = 0;
		Robot.sensors.scaleMoveDistance = Robot.sensors.scaleReadDistance - Sensors.SCALE_DISTANCE_CONSTANT;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		boolean scaleInMeasuringHeightRange = true;

		if (scaleInMeasuringHeightRange && Robot.sensors.getForwardUltrasoundFeet() > Sensors.MIN_SCALE_ULTRASOUND_FT
				&& Robot.sensors.getForwardUltrasoundFeet() < Sensors.MAX_SCALE_ULTRASOUND_FT) {
			passCount++;
			total += Robot.sensors.getForwardUltrasoundFeet();

		} else {
			passCount = 0;
			total = 0;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut()||passCount >= 25;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.sensors.scaleReadDistance = total / passCount;
		Robot.sensors.scaleMoveDistance = Robot.sensors.scaleReadDistance - Sensors.SCALE_DISTANCE_CONSTANT;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
