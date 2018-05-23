package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.AlertDriver;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.WaitForOperator;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.CloseIntakeArms;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.SpinCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.TurnWheelsToIntake;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.PositionToSwitchWall;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.RunReverseTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoLeftSwitchFromCenterUsingTrajectories extends CommandGroup {

	public DoLeftSwitchFromCenterUsingTrajectories() {
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

		addSequential(new ResetEncoders());
		addSequential(new WaitForOperator());
		addParallel(new AlertDriver("Running Reverse Traj LSWC1"));
		addSequential(new RunReverseTrajectory("LSW_C1", DriveTrainCanBus.LSW_C));

		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));
		addSequential(new CloseIntakeArms());

		addSequential(new SetDriveStraightAngle(50));

		addSequential(new ResetEncoders());
		addSequential(new WaitForOperator());

		addParallel(new AlertDriver("Driving to Position 5.5"));
		addSequential(new DriveToPosition(5.5, motionType.absolute, DistCon.SHORT_POSITION_RATE, false, 2));

		addSequential(new SpinCube(false));
		//
		addSequential(new TimeDelay(1));
		//
		addSequential(new TurnWheelsToIntake(.5, 1));
		addSequential(new WaitForOperator());

		addParallel(new DriveToPosition(0, motionType.absolute, DistCon.SHORT_POSITION_RATE, false, 2));
		/*
		 * This trajectory reverses from the cube stack and turns to line up with the
		 * switch It is defined from its end point and should be run in reverse
		 * 
		 */

		addSequential(new ResetEncoders());
		addSequential(new WaitForOperator());

		addParallel(new AlertDriver("Trajectory LSWC2"));
		addSequential(new RunReverseTrajectory("LSW_C2", DriveTrainCanBus.LSW_C));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new RobotOrient(0, DistCon.ORIENT_RATE, driveSide.both, false, 1.5));

		addSequential(new SetDriveStraightAngle(0));

		addSequential(new ResetEncoders());
		addSequential(new WaitForOperator());

		addParallel(new AlertDriver("Positon to Switch Wall"));

		addSequential(new PositionToSwitchWall(6, DistCon.SHORT_POSITION_RATE, 2));

		addSequential(new OuttakeCube(.5));

		addSequential(new ResetEncoders());
		addSequential(new WaitForOperator());

		addParallel(new AlertDriver("Retracting"));
		addSequential(new DriveMagicMotion(-2, motionType.absolute, driveSide.both, DistCon.SHORT_POSITION_RATE, 2.5));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}
