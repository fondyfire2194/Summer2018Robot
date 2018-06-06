package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightSwitchFromLeftMove extends CommandGroup {

	public DoRightSwitchFromLeftMove() {

		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		/*
		 * 
		 * drive forward until between switch and scale then turn 90 degrees cw. Drive
		 * another x ft yhen turn to degrees and drive into switch wall
		 * 
		 */
		addSequential(new ResetEncoders());
		addSequential(new ResetGyro());
		addSequential(new SetDriveStraightAngle(0));
		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new DriveToPosition(DistCon.LRSW_RL_1, motionType.absolute, DistCon.LONG_POSITION_RATE, false,
				false, 3));

		addSequential(new RobotOrient(90, DistCon.ORIENT_RATE, false, 2));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(90));

		addSequential(new DriveToPosition(DistCon.LRSW_RL_2, motionType.absolute, DistCon.SHORT_POSITION_RATE, true,
				false, 3));

		addSequential(new RobotOrient(DistCon.LRSW_RL_A, DistCon.ORIENT_RATE, true, 2));
		addSequential(new ResetEncoders());
		addSequential(new SetDriveStraightAngle(DistCon.LRSW_RL_A));

		addSequential(new DriveToPosition(DistCon.LRSW_RL_3, motionType.absolute, DistCon.SHORT_POSITION_RATE, false,
				false, 3));

		addSequential(new OuttakeCube(.75));

		addSequential(new SetFirstAutoCommandsDone());
	}

}
