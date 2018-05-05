package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightScaleFromLeftMove extends CommandGroup {

	public DoRightScaleFromLeftMove() {

		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		addSequential(new ResetEncoders());
		addSequential(new ResetGyro());
		addSequential(new SetDriveStraightAngle(0));
		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));

		addSequential(new DriveToPosition(DistCon.LRSC_RL_1, motionType.absolute, DistCon.LONG_POSITION_RATE, true, 3));

		addSequential(new RobotOrient(90, DistCon.ORIENT_RATE, driveSide.both, true, 2));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(90));

		addSequential(new DriveToPosition(DistCon.LRSC_RL_2, motionType.absolute, DistCon.LONG_POSITION_RATE, true, 4));

		addSequential(new RobotOrient(0, DistCon.ORIENT_RATE, driveSide.both, true, 2));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(0));
//
		addParallel(new DriveToPosition(DistCon.LRSC_RL_3, motionType.absolute, DistCon.SHORT_POSITION_RATE, true, 3));
		addSequential(new RobotOrient(-90, DistCon.ORIENT_RATE, driveSide.both, true, 2));
		addSequential(new ResetEncoders());
////
//		addParallel(new ElevatorToScale());
//		addSequential(new OuttakeCube(.5));

		addSequential(new SetFirstAutoCommandsDone());

	}

}
