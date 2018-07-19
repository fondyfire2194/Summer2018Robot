package org.usfirst.frc.team2194.robot.commands.AutoMoves;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.SetFirstAutoCommandsDone;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.Motion.PathfinderTrajectoryUsingNotifier;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.SetLeftDriveBrakeOn;
import org.usfirst.frc.team2194.robot.commands.Motion.SetRightDriveBrakeOn;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoTrajectorySwitch extends CommandGroup {

	public DoTrajectorySwitch() {
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
		addSequential(new ResetGyro());
		addSequential(new ResetEncoders());
		addSequential(new SetLeftDriveBrakeOn(Robot.rightSwitchActive));
		addSequential(new SetRightDriveBrakeOn(Robot.leftSwitchActive));
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_AUTO_FIRST_POSITION_INCHES));

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));
		addSequential(new PathfinderTrajectoryUsingNotifier());
		addSequential(new SetLeftDriveBrakeOn(true));
		addSequential(new SetRightDriveBrakeOn(true));

		addSequential(new OuttakeCube(.5));
		addSequential(new SetFirstAutoCommandsDone());
	}
}
