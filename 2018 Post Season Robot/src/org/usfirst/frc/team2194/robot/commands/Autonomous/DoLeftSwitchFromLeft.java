package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.DistCon2;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoLeftSwitchFromLeft extends CommandGroup {

	public DoLeftSwitchFromLeft() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		addSequential(new ResetEncoders());
		// away from switch
		addSequential(new DriveMagicMotion(-DistCon2.LRSW_RD1, motionType.absolute, driveSide.both,
				DistCon.SHORT_POSITION_RATE, 3));
		// turn to angle and drop elevator

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));

		addSequential(new RobotOrient(0, DistCon.ORIENT_RATE, driveSide.both, true, 3));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}
