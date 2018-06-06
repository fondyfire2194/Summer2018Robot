package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoCrossLineMove extends CommandGroup {

	public DoCrossLineMove() {

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
		addSequential(new SetDriveStraightAngle(0));
		addSequential(new ResetEncoders());
		addParallel(new DriveToPosition(DistCon.CROSS_LINE, motionType.absolute, DistCon.LONG_POSITION_RATE, false,
				false, 8));
		// addSequential(new ElevatorToSwitch());
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));
		addSequential(new TimeDelay(2));
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

		addSequential(new SetFirstAutoCommandsDone());

	}

}
