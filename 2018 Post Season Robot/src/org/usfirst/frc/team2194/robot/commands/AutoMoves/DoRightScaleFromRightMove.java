package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
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
public class DoRightScaleFromRightMove extends CommandGroup {

	public DoRightScaleFromRightMove() {

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
		addSequential(new SetDriveStraightAngle(2));
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_AUTO_FIRST_POSITION_INCHES));
		addSequential(new TimeDelay(.5));

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new DriveToPosition(DistCon.LR_SC_2 - 1, motionType.absolute, DistCon.LONG_POSITION_RATE, false,
				false, 4.5));

		addSequential(new RobotOrient(-70, DistCon.ORIENT_RATE, true, 4));

		addSequential(new SetDriveStraightAngle(-70));

		addSequential(new ResetEncoders());

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SCALE_POSITION_INCHES));

		addSequential(new OuttakeCube(.8));

		addSequential(new SetFirstAutoCommandsDone());
	}

}
