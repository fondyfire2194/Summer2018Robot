package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.Motion.PathfinderTrajectoryUsingNotifier;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoTrajectoryScale extends CommandGroup {

	public DoTrajectoryScale() {
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
		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));
		addSequential(new PathfinderTrajectoryUsingNotifier());
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SCALE_POSITION_INCHES));

		addSequential(new OuttakeCube(.5));
		addSequential(new SetFirstAutoCommandsDone());
	}
}
