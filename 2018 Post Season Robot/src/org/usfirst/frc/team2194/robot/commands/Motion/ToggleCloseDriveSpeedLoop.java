package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ToggleCloseDriveSpeedLoop extends InstantCommand {

	public ToggleCloseDriveSpeedLoop() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.closeDriveSpeedLoop = !Robot.closeDriveSpeedLoop;
		if (Robot.closeDriveSpeedLoop) {
			RobotMap.driveLeftMotorA.selectProfileSlot(0, 0);
			RobotMap.driveRightMotorA.selectProfileSlot(0, 0);
		}

	}

}
