package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.TurnWheelsToIntake;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.RunReverseTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.RunTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightSwitchFromRightTrajectories extends CommandGroup {

	public DoRightSwitchFromRightTrajectories() {
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
		addSequential(new RunReverseTrajectory("RSW_R1", DriveTrainCanBus.RSW_R));
		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));
		// turn ready to reverse toward scale
		addSequential(new ResetEncoders());
		addSequential(new RunTrajectory("RSW_R2", DriveTrainCanBus.RSW_R));
		addSequential(new SetDriveStraightAngle(50));

		addSequential(new ResetEncoders());

		// addParallel(new DriveToVisionTarget(0, 5, 4));

		// addSequential(new SpinCube(false));

		addSequential(new TimeDelay(1));

		addSequential(new TurnWheelsToIntake(.5, 1));

		// addParallel(new DriveToPosition(0, motionType.absolute,
		// DistCon.SHORT_POSITION_RATE, false, 2));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		// addSequential(new PositionToSwitchWall(3, DistCon.SHORT_POSITION_RATE, 2));

		addSequential(new OuttakeCube(.5));

		addSequential(new ResetEncoders());

		// addSequential(new DriveMagicMotion(-2, motionType.absolute, driveSide.both,
		// DistCon.SHORT_POSITION_RATE, 2.5));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}