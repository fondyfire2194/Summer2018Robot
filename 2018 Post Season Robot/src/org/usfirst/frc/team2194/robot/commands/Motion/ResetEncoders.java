package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ResetEncoders extends InstantCommand {

	public ResetEncoders() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);
	}

	// Called once when the command executes
	protected void initialize() {
		RobotMap.driveLeftMotorA.setSelectedSensorPosition(0, 0, 0);
		RobotMap.driveRightMotorA.setSelectedSensorPosition(0, 0, 0);
	}

}
