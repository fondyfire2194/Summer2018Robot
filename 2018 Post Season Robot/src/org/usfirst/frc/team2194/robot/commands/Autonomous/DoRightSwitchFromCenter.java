package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.LogIntakeData;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.CloseIntakeArms;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OuttakeCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.SpinCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.TurnWheelsToIntake;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToCubePickup;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.PositionToSwitchWall;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.SetDriveStraightAngle;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoRightSwitchFromCenter extends CommandGroup {

	public DoRightSwitchFromCenter() {
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

		addParallel(new DriveMagicMotion(-3, motionType.absolute, driveSide.both, DistCon.SHORT_POSITION_RATE, 3));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));
		addSequential(new CloseIntakeArms());
		addSequential(new RobotOrient(-50, DistCon.ORIENT_RATE,  false, 1.5));

		addSequential(new SetDriveStraightAngle(-50));

		addSequential(new ResetEncoders());

		addParallel(new DriveToCubePickup(4, motionType.absolute, DistCon.SHORT_POSITION_RATE, 5));

//		addSequential(new SpinCube(true));
//
//		addSequential(new TimeDelay(1));

		addSequential(new TurnWheelsToIntake(.5, 5));

		addParallel(new DriveToPosition(0, motionType.absolute, DistCon.SHORT_POSITION_RATE,false, false, 2));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));

		addSequential(new RobotOrient(0, DistCon.ORIENT_RATE, false, 1.5));

		addSequential(new SetDriveStraightAngle(0));

		addSequential(new ResetEncoders());

		addSequential(new PositionToSwitchWall(4.5, DistCon.SHORT_POSITION_RATE, 2));

		addSequential(new OuttakeCube(.5));

		addSequential(new ResetEncoders());

		addSequential(new DriveMagicMotion(-2, motionType.absolute, driveSide.both, DistCon.SHORT_POSITION_RATE, 2.5));

		addSequential(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}
