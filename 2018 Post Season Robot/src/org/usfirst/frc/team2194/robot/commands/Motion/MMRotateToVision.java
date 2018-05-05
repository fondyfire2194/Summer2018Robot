package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MMRotateToVision extends CommandGroup {

	public MMRotateToVision(double angle, double rate, double timeOut) {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1(
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		/*
		 * Used to force rotate in a direction until vision seen A positive angle forces
		 * cw rotation
		 * 
		 */
		addSequential(new ResetEncoders());
		addSequential(new DriveRotateMagicMotion(angle * Robot.driveTrainCanBus.MM_FT_PER_DEGREE, motionType.absolute,
				rate, 5));

	}
}
