package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.PositionToSwitchWall;
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
public class DoLeftSwitchFromCenterMove extends CommandGroup {

	public DoLeftSwitchFromCenterMove() {

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

		addSequential(new ResetGyro());

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new DriveMagicMotion(DistCon.LSWC_1, motionType.absolute, driveSide.both,
				DistCon.SHORT_POSITION_RATE, 3));

		addSequential(new RobotOrient(DistCon.LSW_CA, DistCon.LSW_C_ORIENT_RATE, driveSide.both, true, 1));

		addSequential(new ResetEncoders());

		addSequential(new SetDriveStraightAngle(DistCon.LSW_CA));

		addSequential(new DriveToPosition(DistCon.LSWC_2, motionType.absolute, DistCon.SHORT_POSITION_RATE+1, false, 2));

		addSequential(new RobotOrient(0, DistCon.LSW_C_ORIENT_RATE, driveSide.both, true, 1.5));

		addSequential(new ResetEncoders());

		addSequential(new SetDriveStraightAngle(0));

		addSequential(new PositionToSwitchWall(DistCon.LSWC_3 +1, DistCon.SHORT_POSITION_RATE, 2));

		addSequential(new OuttakeCube(.5));

		addSequential(new SetFirstAutoCommandsDone());

	}

}
