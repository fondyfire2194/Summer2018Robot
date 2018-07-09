package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.SetTrajectoryGains;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.CloseIntakeArms;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.DelayedElevatorMove;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.TurnWheelsToIntake;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToCubePickup;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.PathfinderTrajectoryUsingNotifier;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.RunReverseTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightSwitchFromCenterTrajectories extends CommandGroup {

	public DoRightSwitchFromCenterTrajectories() {
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
		 * This trajectory retracts from the switch and turns to face cube stack at 50
		 * degrees. The trajectory is defined from the switch and moves in reverse.
		 * Reset encoders first as reverse trajectory requires this
		 */
		addSequential(new CloseIntakeArms());

		addSequential(new ResetEncoders());

		addSequential(new ResetGyro());

		addParallel(new DelayedElevatorMove(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

		addSequential(new RunReverseTrajectory("RSW_C1", DriveTrainCanBus.RSW_C1Rev));

		addSequential(new SetDriveStraightAngle(0));

		addSequential(new ResetEncoders());

		addParallel(new DriveToCubePickup(5.5, motionType.absolute, DistCon.SHORT_POSITION_RATE, 2));

		addSequential(new TurnWheelsToIntake(.5, 3));

		addParallel(new DriveToPosition(0, motionType.absolute, DistCon.SHORT_POSITION_RATE, false, false, 2));

		addSequential(new ResetEncoders());

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new SetTrajectoryGains(DriveTrainCanBus.RSW_C1));

		addSequential(new PathfinderTrajectoryUsingNotifier());

		addSequential(new OuttakeCube(.5));

		addSequential(new ResetEncoders());

	}
}
