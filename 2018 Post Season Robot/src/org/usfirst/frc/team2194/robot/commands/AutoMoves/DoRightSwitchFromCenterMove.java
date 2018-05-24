package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.Motion.PositionToSwitchWall;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightSwitchFromCenterMove extends CommandGroup {

	public DoRightSwitchFromCenterMove() {

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
		addSequential(new SetDriveStraightAngle(0));
		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new PositionToSwitchWall(DistCon.RSWC_1, DistCon.SHORT_POSITION_RATE, 3));

		addSequential(new OuttakeCube(.5));

		addSequential(new SetFirstAutoCommandsDone());

	}

}
