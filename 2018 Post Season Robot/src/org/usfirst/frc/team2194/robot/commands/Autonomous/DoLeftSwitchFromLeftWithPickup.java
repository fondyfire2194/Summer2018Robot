package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.DistCon2;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoLeftSwitchFromLeftWithPickup extends CommandGroup {

	public DoLeftSwitchFromLeftWithPickup() {
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
				DistCon.SHORT_POSITION_RATE, 2));
		// turn to angle and drop elevator

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));
		// turn ready to reverse toward scale
		addSequential(new RobotOrient(DistCon2.LRSW_A1, DistCon.ORIENT_RATE, true, 1.5));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(DistCon2.LRSW_A1));
		// back up to get angle on cube
		addSequential(
				new DriveToPosition(-DistCon2.LRSW_RD2, motionType.absolute, DistCon.LONG_POSITION_RATE, true, false,3));
		// turn to line up with cube
		addSequential(new RobotOrient(DistCon2.LRSW_A2, DistCon.ORIENT_RATE, true, 2));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(DistCon2.LRSW_A2));
		// advance behind cube
		// addSequential(new DriveToPosition(DistCon2.LRSW_TO_CUBE, motionType.absolute,
		// DistCon.LONG_POSITION_RATE, true, 3));
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}
